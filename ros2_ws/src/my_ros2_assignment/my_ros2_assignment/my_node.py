#!/usr/bin/env python3
"""
Doosan E0509 MoveIt2 GUI Controller (ROS2 Humble + PyQt5)
"""

import sys
import time
from dataclasses import dataclass
from typing import List, Optional, Tuple

from PyQt5 import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionClient
from rclpy.duration import Duration

import tf2_ros

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


# -----------------------------
# Utils (공용 유틸)
# -----------------------------
def _now_str() -> str:
    """GUI 로그에 찍기 위한 현재 시간(HH:MM:SS)"""
    return time.strftime("%H:%M:%S")


def _clamp(x: float, lo: float, hi: float) -> float:
    """
    입력값을 [lo, hi] 범위로 강제 제한.
    - 속도/가속도 스케일(0~1) 범위 유지
    - relative offset 과도 입력 방지 -> EE 포인트가 워크스페이스 밖으로 튀는 문제 방지
    """
    return max(lo, min(hi, x))


def _quat_identity() -> Quaternion:
    """
    orientation을 단순화하기 위해 회전을 제외한 position만 이동 
    """
    q = Quaternion()
    q.x, q.y, q.z, q.w = 0.0, 0.0, 0.0, 1.0
    return q


# MoveIt error code를 사람이 읽을 수 있게 매핑
MOVEIT_ERR = {
    1: "SUCCESS",
    -1: "PLANNING_FAILED",
    -2: "INVALID_MOTION_PLAN",
    -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
    -4: "CONTROL_FAILED",
    -6: "TIMED_OUT",
    -10: "START_STATE_IN_COLLISION",
    -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
    -12: "GOAL_IN_COLLISION",
    -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
    -14: "GOAL_CONSTRAINTS_VIOLATED",
    -21: "FRAME_TRANSFORM_FAILURE",
    -23: "ROBOT_STATE_STALE",
    -31: "NO_IK_SOLUTION",
}


@dataclass
class TargetPoint:
    """
    GUI 테이블의 한 행(목표점)을 표현하는 데이터 구조.
    - ABS 모드: base_frame 기준 목표 EE xyz
    - REL 모드: 현재 EE xyz에 더할 offset (dx,dy,dz)
    """
    x: float
    y: float
    z: float


# -----------------------------
# Backend ROS2 Node
# -----------------------------
class BackendNode(Node):
    """
    [역할]
    - ROS2 통신 전담(구독/TF/MoveIt 액션)
    
    [핵심 기능]
    1) joint_states 구독(토픽 이름 자동탐지)
    2) TF2로 base_frame -> ee_link 변환 조회(현재 EE 위치)
    3) MoveGroup 액션 목표 전송(plan+execute)
    """

    # joint_states 토픽 후보
    JOINT_STATE_CANDIDATES = [
        "/dsr01/joint_states",
        "/joint_states",
        "/dsr01/gz/joint_states",
        "/dsr01/dynamic_joint_states",
        "/dsr01/gz/joint_states",
    ]

    def __init__(self):
        super().__init__("my_ros2_assignment_backend")

        # 파라미터: launch에서 바꿀 수 있도록 declare
        self.declare_parameter("move_action", "/move_action")
        self.declare_parameter("joint_topic", "")
        self.declare_parameter("group_name", "manipulator")
        self.declare_parameter("ee_link", "link_6")
        self.declare_parameter("base_frame", "base_link")

        # 실제 적용값 로드
        self.move_action_name: str = self.get_parameter("move_action").value
        self.joint_topic: str = (self.get_parameter("joint_topic").value or "").strip()
        self.group_name: str = self.get_parameter("group_name").value
        self.ee_link: str = self.get_parameter("ee_link").value
        self.base_frame: str = self.get_parameter("base_frame").value

        # 상태 캐시(최신 joint / 최신 ee pose)
        self._last_joint_state: Optional[JointState] = None
        self._last_joint_state_time: float = 0.0

        self._last_ee_xyz: Optional[Tuple[float, float, float]] = None
        self._last_ee_time: float = 0.0

        # -----------------------------
        # joint_states 구독은 지연 연결 오류 방지(lazy subscribe)
        # - bringup이 늦게 뜰 수 있으므로 timer로 재시도
        # -----------------------------
        self._joint_sub = None
        self._joint_timer = self.create_timer(0.5, self._ensure_joint_subscription)

        # MoveIt MoveGroup 액션 클라이언트
        self._move_client = ActionClient(self, MoveGroup, self.move_action_name)

        # -----------------------------
        # TF2 리스너: /tf 구독해서 버퍼에 저장
        # - base_frame -> ee_link 현재 변환을 읽어서 EE xyz를 얻는다.
        # -----------------------------
        self._tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info(f"[setup] move_action={self.move_action_name}")
        self.get_logger().info(
            f"[setup] joint_topic={'fixed:'+self.joint_topic if self.joint_topic else 'auto-detect'}"
        )
        self.get_logger().info(f"[setup] tf base_frame={self.base_frame}, ee_link={self.ee_link}")

    # -------- joint state (lazy subscribe) --------
    def _topic_exists(self, topic_name: str) -> bool:
        """ROS graph에 특정 토픽이 존재하는지 확인"""
        topics = dict(self.get_topic_names_and_types())
        return topic_name in topics

    def _pick_joint_state_topic_no_throw(self) -> Optional[str]:
        """
        joint_states 토픽 자동 선택.
        1) 후보 리스트(JOINT_STATE_CANDIDATES)를 우선 순위로 탐색
        2) 없으면 이름이 joint_states로 끝나는 토픽을 fallback
        """
        topics = dict(self.get_topic_names_and_types())
        if not topics:
            return None

        for t in self.JOINT_STATE_CANDIDATES:
            if t in topics:
                return t

        for t in topics.keys():
            if t.endswith("joint_states"):
                return t

        return None

    def _ensure_joint_subscription(self):
        """
        timer(0.5s)마다 호출되며,
        아직 joint_states를 구독하지 않았다면:
        - fixed param이면 해당 토픽이 나타날 때까지 대기
        - auto면 현재 ROS graph에서 joint_states를 찾아 구독
        """
        if self._joint_sub is not None:
            return

        if self.joint_topic:
            if self._topic_exists(self.joint_topic):
                self._joint_sub = self.create_subscription(
                    JointState, self.joint_topic, self._on_joint_state, 10
                )
                self.get_logger().info(f"[joint] subscribed fixed topic: {self.joint_topic}")
            return

        picked = self._pick_joint_state_topic_no_throw()
        if picked:
            self.joint_topic = picked
            self._joint_sub = self.create_subscription(
                JointState, self.joint_topic, self._on_joint_state, 10
            )
            self.get_logger().info(f"[joint] auto subscribed: {self.joint_topic}")

    def _on_joint_state(self, msg: JointState):
        """joint_states 콜백: 최신 관절 상태 캐시"""
        self._last_joint_state = msg
        self._last_joint_state_time = time.time()

    def get_last_joint_positions(self) -> Optional[List[float]]:
        """GUI 표시용: 최신 joint positions(관절각) 리스트 반환"""
        if self._last_joint_state is None:
            return None
        return list(self._last_joint_state.position)

    def has_recent_joint_state(self, timeout_sec: float = 1.0) -> bool:
        """최근 timeout_sec 이내에 joint_state를 받았는지 여부"""
        if self._last_joint_state is None:
            return False
        return (time.time() - self._last_joint_state_time) <= timeout_sec

    # -------- TF: current EE pose (xyz) --------
    def get_current_ee_xyz(self, timeout_sec: float = 0.05) -> Optional[Tuple[float, float, float]]:
        """
        [현재 EE 위치 읽기]
        - TF2 buffer에서 base_frame -> ee_link 변환을 조회
        - translation(x,y,z)를 EE 위치로 사용
        - TF가 아직 준비 안 됐으면 None 반환
        """
        try:
            tf = self._tf_buffer.lookup_transform(
                self.base_frame,  # target
                self.ee_link,     # source
                rclpy.time.Time(),
                timeout=Duration(seconds=float(timeout_sec)),
            )
            t = tf.transform.translation
            xyz = (float(t.x), float(t.y), float(t.z))
            self._last_ee_xyz = xyz
            self._last_ee_time = time.time()
            return xyz
        except Exception:
            return None

    def get_last_ee_xyz_cached(self) -> Optional[Tuple[float, float, float]]:
        """GUI 표시용: TF 조회 실패 시 마지막 캐시값이라도 표시"""
        return self._last_ee_xyz

    # -------- moveit --------
    def wait_moveit_server(self, timeout_sec: float = 10.0) -> bool:
        """
        MoveGroup 액션 서버가 준비될 때까지 대기.
        move_group이 아직 안 떴으면 False
        """
        return self._move_client.wait_for_server(timeout_sec=timeout_sec)

    def plan_and_execute_xyz(
        self,
        xyz: Tuple[float, float, float],
        frame_id: str,
        max_vel_scale: float,
        max_acc_scale: float,
        allowed_planning_time: float = 5.0,
    ) -> Tuple[bool, int]:
        """
        [MoveIt에 목표 전달]
        - 입력: 목표 EE 위치(x,y,z), 기준프레임(frame_id), vel/acc 스케일
        - 내부: MoveGroup action goal 생성 후 send_goal_async → 결과 대기
        - 출력: (성공여부, error_code)

        [제약 방식]
        - PositionConstraint + 작은 sphere(반경 1cm)로 목표 근처를 허용
          → 단일 점 목표보다 플래닝 안정성 증가
        - orientation은 identity로 고정(간단화)

        [예시]
        - ABS의 타겟 좌표 : base_link 기준으로 해당 좌표로 EE를 이동
        - REL의 타겟 좌표: TF로 읽은 현재 EE + Offset
        """
        max_vel_scale = _clamp(max_vel_scale, 0.01, 1.0)
        max_acc_scale = _clamp(max_acc_scale, 0.01, 1.0)

        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = float(allowed_planning_time)
        goal.request.max_velocity_scaling_factor = float(max_vel_scale)
        goal.request.max_acceleration_scaling_factor = float(max_acc_scale)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = float(xyz[0])
        pose.pose.position.y = float(xyz[1])
        pose.pose.position.z = float(xyz[2])
        pose.pose.orientation = _quat_identity()

        pc = PositionConstraint()
        pc.header.frame_id = frame_id
        pc.link_name = self.ee_link
        pc.weight = 1.0

        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.01]  # radius

        bv = BoundingVolume()
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose.pose)
        pc.constraint_region = bv

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        goal.request.goal_constraints = [constraints]

        # 비동기 전송 후 완료까지 기다림
        send_future = self._move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if goal_handle is None or (not goal_handle.accepted):
            return (False, -1)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        res = result_future.result()
        if res is None:
            return (False, -2)

        error_code_val = int(res.result.error_code.val)
        success = (error_code_val == 1)
        return (success, error_code_val)


# -----------------------------
# Worker Thread (Qt)
# -----------------------------
class MoveWorker(QtCore.QThread):

    log_sig = QtCore.pyqtSignal(str)
    done_sig = QtCore.pyqtSignal(bool, str)

    def __init__(
        self,
        backend: BackendNode,
        points: List[TargetPoint],
        is_relative: bool,
        max_vel: float,
        max_acc: float,
        parent=None,
    ):
        super().__init__(parent)
        self.backend = backend
        self.points = points
        self.is_relative = is_relative
        self.max_vel = max_vel
        self.max_acc = max_acc
        self._stop_flag = False

    def request_stop(self):
        """Stop 버튼에서 호출: 다음 루프에서 중단"""
        self._stop_flag = True

    def _log(self, s: str):
        """Worker에서 GUI 로그창으로 메시지 전달(signal)"""
        self.log_sig.emit(f"[{_now_str()}] {s}")

    def run(self):
        """
        [실행 흐름]
        1) MoveIt 서버 준비 확인
        2) 목표점 리스트 순회
           - REL: 현재 EE + offset (clamp로 안전 제한)
           - ABS: 그대로 목표 사용
        3) plan_and_execute_xyz 호출
        4) 성공 시 0.2초 대기(상태 동기화 안정화)
           - allowed_start_tolerance mismatch(-4) 완화 목적
        """
        try:
            self._log("[worker] start")

            if not self.backend.wait_moveit_server(timeout_sec=10.0):
                raise RuntimeError("MoveIt action server not available. Did you launch move_group?")

            self._log("[setup] moveit_server=OK")

            if not self.backend.has_recent_joint_state(timeout_sec=2.0):
                jt = self.backend.joint_topic if self.backend.joint_topic else "(detecting...)"
                self._log(f"[warn] no recent joint_states yet. topic={jt}")

            frame = self.backend.base_frame

            for i, p in enumerate(self.points, start=1):
                if self._stop_flag:
                    self._log("[worker] stop requested")
                    self.done_sig.emit(False, "Stopped by user")
                    return

                # -----------------------------
                # 목표 생성(ABS/REL)
                # -----------------------------
                if self.is_relative:
                    # (진짜 상대좌표) 현재 EE pose를 TF로 읽고 offset을 더한다.
                    cur = self.backend.get_current_ee_xyz(timeout_sec=0.2)
                    if cur is None:
                        raise RuntimeError(
                            f"TF not available: cannot use relative mode (base={self.backend.base_frame}, ee={self.backend.ee_link})"
                        )

                    # 과도한 offset 입력 방지(워크스페이스 밖으로 튀는 것 방지)
                    dx = _clamp(p.x, -0.10, 0.10)
                    dy = _clamp(p.y, -0.10, 0.10)
                    dz = _clamp(p.z, -0.10, 0.10)

                    target = (cur[0] + dx, cur[1] + dy, cur[2] + dz)

                    self._log(
                        f"[move {i}] target REL(TF) cur=({cur[0]:.3f},{cur[1]:.3f},{cur[2]:.3f}) "
                        f"+ d=({dx:.3f},{dy:.3f},{dz:.3f}) -> ({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) frame={frame}"
                    )
                else:
                    target = (p.x, p.y, p.z)
                    self._log(
                        f"[move {i}] target ABS xyz=({target[0]:.3f},{target[1]:.3f},{target[2]:.3f}) frame={frame}"
                    )

                self._log(
                    f"[move {i}] group={self.backend.group_name}, ee_link={self.backend.ee_link}, vel={self.max_vel:.2f}, acc={self.max_acc:.2f}"
                )

                # -----------------------------
                # MoveIt plan+execute
                # -----------------------------
                ok, code = self.backend.plan_and_execute_xyz(
                    xyz=target,
                    frame_id=frame,
                    max_vel_scale=self.max_vel,
                    max_acc_scale=self.max_acc,
                    allowed_planning_time=5.0,
                )

                if ok:
                    self._log(f"[move {i}] MoveGroup SUCCESS (planned+executed)")
                    # 실행 직후 joint_states/CurrentStateMonitor 동기화 지연 완화
                    time.sleep(0.2)
                else:
                    name = MOVEIT_ERR.get(code, "UNKNOWN")
                    self._log(f"[move {i}] MoveGroup FAILED (error_code={code}, {name})")
                    raise RuntimeError(f"MoveGroup FAILED (error_code={code}, {name})")

            self._log("[worker] done")
            self.done_sig.emit(True, "OK")

        except Exception as e:
            self.done_sig.emit(False, str(e))


# -----------------------------
# GUI
# -----------------------------
class MainWindow(QtWidgets.QMainWindow):
    """
    [GUI 구성]
    - 왼쪽(Control): 목표 입력/옵션/실행 버튼
    - 오른쪽(Status): 현재 joint, EE pose, 로그 출력

    [레이아웃 비율]
    - Control : Status = 1 : 2
    """

    def __init__(self, backend: BackendNode):
        super().__init__()
        self.backend = backend
        self.worker: Optional[MoveWorker] = None

        self.setWindowTitle("my_ros2_assignment - Doosan E0509 MoveIt GUI")
        self.resize(1100, 650)

        central = QtWidgets.QWidget()
        self.setCentralWidget(central)

        root = QtWidgets.QHBoxLayout(central)

        # -----------------------------
        # Left: Control
        # -----------------------------
        left = QtWidgets.QGroupBox("Control")
        left_layout = QtWidgets.QVBoxLayout(left)

        self.chk_relative = QtWidgets.QCheckBox("Relative coordinate mode (TF offset)")
        self.chk_relative.setChecked(False)

        form = QtWidgets.QFormLayout()
        self.edt_vel = QtWidgets.QLineEdit("0.5")
        self.edt_acc = QtWidgets.QLineEdit("0.5")
        form.addRow("Max velocity scaling (0~1)", self.edt_vel)
        form.addRow("Max acceleration scaling (0~1)", self.edt_acc)

        self.table = QtWidgets.QTableWidget(0, 3)
        self.table.setHorizontalHeaderLabels(["x", "y", "z"])
        self.table.horizontalHeader().setStretchLastSection(True)
        self.table.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)

        btn_row = QtWidgets.QHBoxLayout()
        self.btn_add = QtWidgets.QPushButton("Add Target")
        self.btn_del = QtWidgets.QPushButton("Remove Target")
        btn_row.addWidget(self.btn_add)
        btn_row.addWidget(self.btn_del)

        self.btn_execute = QtWidgets.QPushButton("Execute")
        self.btn_stop = QtWidgets.QPushButton("Stop")
        self.btn_stop.setEnabled(False)

        left_layout.addWidget(self.chk_relative)
        left_layout.addLayout(form)
        left_layout.addWidget(QtWidgets.QLabel("Target points (x,y,z)"))
        left_layout.addWidget(self.table)
        left_layout.addLayout(btn_row)
        left_layout.addWidget(self.btn_execute)
        left_layout.addWidget(self.btn_stop)
        left_layout.addStretch(1)

        # -----------------------------
        # Right: Status
        # -----------------------------
        right = QtWidgets.QGroupBox("Status")
        right_layout = QtWidgets.QVBoxLayout(right)

        self.lbl_robot = QtWidgets.QLabel("robot: unknown")
        self.lbl_motion = QtWidgets.QLabel("motion: idle")
        self.lbl_joint_topic = QtWidgets.QLabel("joint_topic: (detecting...)")
        self.lbl_joints = QtWidgets.QLabel("joints: -")
        self.lbl_ee = QtWidgets.QLabel("ee_xyz: -")
        self.lbl_moveit = QtWidgets.QLabel("moveit: -")

        self.log = QtWidgets.QPlainTextEdit()
        self.log.setReadOnly(True)

        right_layout.addWidget(self.lbl_robot)
        right_layout.addWidget(self.lbl_motion)
        right_layout.addWidget(self.lbl_joint_topic)
        right_layout.addWidget(self.lbl_joints)
        right_layout.addWidget(self.lbl_ee)
        right_layout.addWidget(self.lbl_moveit)
        right_layout.addWidget(QtWidgets.QLabel("Log"))
        right_layout.addWidget(self.log)

        # Control : Status = 1 : 2  (Control 폭 1/3)
        root.addWidget(left, 1)
        root.addWidget(right, 2)

        # -----------------------------
        # Signals
        # -----------------------------
        self.btn_add.clicked.connect(self.on_add)
        self.btn_del.clicked.connect(self.on_del)
        self.btn_execute.clicked.connect(self.on_execute)
        self.btn_stop.clicked.connect(self.on_stop)

        # 주기적으로 상태 갱신(토픽/TF 캐시를 GUI에 반영)
        self.ui_timer = QtCore.QTimer(self)
        self.ui_timer.setInterval(200)
        self.ui_timer.timeout.connect(self.refresh_status)
        self.ui_timer.start()

        # 초기 테이블 값(예시) 의도적으로 안정적인 값 참고
        for _ in range(3):
            self.on_add()
        self.table.setItem(0, 0, QtWidgets.QTableWidgetItem("0.300"))
        self.table.setItem(0, 1, QtWidgets.QTableWidgetItem("0.000"))
        self.table.setItem(0, 2, QtWidgets.QTableWidgetItem("0.300"))
        self.table.setItem(1, 0, QtWidgets.QTableWidgetItem("0.350"))
        self.table.setItem(1, 1, QtWidgets.QTableWidgetItem("0.100"))
        self.table.setItem(1, 2, QtWidgets.QTableWidgetItem("0.250"))
        self.table.setItem(2, 0, QtWidgets.QTableWidgetItem("0.280"))
        self.table.setItem(2, 1, QtWidgets.QTableWidgetItem("-0.100"))
        self.table.setItem(2, 2, QtWidgets.QTableWidgetItem("0.280"))

        self._append_log(f"[setup] joint_topic={self.backend.joint_topic or '(detecting...)'}")
        self._append_log(f"[setup] move_action={self.backend.move_action_name}")
        self._append_log(f"[setup] tf base={self.backend.base_frame}, ee={self.backend.ee_link}")

    def _append_log(self, s: str):
        """로그창에 문자열 추가"""
        self.log.appendPlainText(s)

    def on_add(self):
        """테이블 행 추가(목표점 추가)"""
        r = self.table.rowCount()
        self.table.insertRow(r)
        for c in range(3):
            self.table.setItem(r, c, QtWidgets.QTableWidgetItem("0.0"))

    def on_del(self):
        """테이블 마지막 행 삭제"""
        r = self.table.rowCount()
        if r > 0:
            self.table.removeRow(r - 1)

    def _read_points(self) -> List[TargetPoint]:
        """
        GUI 테이블의 모든 행을 읽어 TargetPoint 리스트로 변환.
        - 숫자 파싱 실패 시 즉시 예외(잘못된 입력 방지)
        """
        pts: List[TargetPoint] = []
        for r in range(self.table.rowCount()):
            try:
                x = float(self.table.item(r, 0).text())
                y = float(self.table.item(r, 1).text())
                z = float(self.table.item(r, 2).text())
                pts.append(TargetPoint(x, y, z))
            except Exception:
                raise RuntimeError(f"Invalid target at row {r+1}")
        if len(pts) < 1:
            raise RuntimeError("Need at least 1 target point")
        return pts

    def _read_vel_acc(self) -> Tuple[float, float]:
        """
        속도/가속도 스케일 입력값을 읽고 [0.01, 1.0] 범위로 제한.
        - 0은 위험하니 최소 0.01로 클램프
        """
        try:
            v = float(self.edt_vel.text())
            a = float(self.edt_acc.text())
        except Exception:
            raise RuntimeError("Invalid vel/acc input")

        v = _clamp(v, 0.01, 1.0)
        a = _clamp(a, 0.01, 1.0)
        return v, a

    def on_execute(self):
        """
        Execute 버튼 동작:
        - 테이블/옵션 입력값 읽기
        - MoveWorker 생성 후 start()
        - 버튼 enable/disable 및 상태 라벨 변경
        """
        if self.worker is not None and self.worker.isRunning():
            self._append_log("[gui] already running")
            return

        try:
            pts = self._read_points()
            v, a = self._read_vel_acc()
            rel = self.chk_relative.isChecked()

            self._append_log("[gui] execute pressed")

            self.worker = MoveWorker(
                backend=self.backend,
                points=pts,
                is_relative=rel,
                max_vel=v,
                max_acc=a,
            )
            self.worker.log_sig.connect(self._append_log)
            self.worker.done_sig.connect(self.on_worker_done)

            self.btn_execute.setEnabled(False)
            self.btn_stop.setEnabled(True)
            self.lbl_motion.setText("motion: moving")

            self.worker.start()

        except Exception as e:
            self._append_log(f"[gui] error: {e}")

    def on_stop(self):
        """Stop 버튼: Worker에 stop flag 요청(즉시 강제정지 X, 다음 루프에서 중단)"""
        if self.worker is not None and self.worker.isRunning():
            self._append_log("[gui] stop pressed")
            self.worker.request_stop()

    def on_worker_done(self, ok: bool, msg: str):
        """Worker 종료 시 호출: 버튼/상태 라벨 복구 + 결과 로그 출력"""
        if ok:
            self._append_log("[gui] worker finished OK")
        else:
            self._append_log(f"[gui] worker failed: {msg}")

        self.btn_execute.setEnabled(True)
        self.btn_stop.setEnabled(False)
        self.lbl_motion.setText("motion: idle")
        self.worker = None

    def refresh_status(self):
        """
        200ms마다 상태 갱신:
        - joint_topic 표시
        - 최신 joint 6개 표시
        - TF 기반 EE xyz 표시(없으면 캐시/또는 no tf)
        """
        jt = self.backend.joint_topic if self.backend.joint_topic else "(detecting...)"
        self.lbl_joint_topic.setText(f"joint_topic: {jt}")

        jp = self.backend.get_last_joint_positions()
        if jp is None:
            self.lbl_joints.setText("joints: (no data)")
        else:
            vals = ", ".join([f"{x:.3f}" for x in jp[:6]])
            self.lbl_joints.setText(f"joints: [{vals}]")

        ee = self.backend.get_current_ee_xyz(timeout_sec=0.01)
        if ee is None:
            cached = self.backend.get_last_ee_xyz_cached()
            if cached is None:
                self.lbl_ee.setText("ee_xyz: (no tf)")
            else:
                self.lbl_ee.setText(
                    f"ee_xyz: ({cached[0]:.3f}, {cached[1]:.3f}, {cached[2]:.3f}) (cached)"
                )
        else:
            self.lbl_ee.setText(f"ee_xyz: ({ee[0]:.3f}, {ee[1]:.3f}, {ee[2]:.3f})")

        self.lbl_robot.setText(
            f"robot: group={self.backend.group_name}, ee={self.backend.ee_link}, base={self.backend.base_frame}"
        )
        self.lbl_moveit.setText(f"moveit: action={self.backend.move_action_name}")


# -----------------------------
# ROS Spin Thread
# -----------------------------
class RosSpinThread(QtCore.QThread):
    """
    - ROS2 executor.spin()을 별도 스레드에서 돌려
      GUI 이벤트 루프와 분리한다.
    - 이 스레드가 없으면 TF/joint_states 콜백이 안 돌거나, Action future 완료 처리가 늦어져 전체가 꼬일 수 있음.
    """

    def __init__(self, executor: MultiThreadedExecutor, parent=None):
        super().__init__(parent)
        self.executor = executor

    def run(self):
        """ROS executor를 계속 spin"""
        self.executor.spin()


# -----------------------------
# main
# -----------------------------
def main():
    rclpy.init()

    backend = BackendNode()

    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(backend)

    app = QtWidgets.QApplication(sys.argv)
    win = MainWindow(backend)
    win.show()

    spin_thread = RosSpinThread(executor)
    spin_thread.start()

    rc = app.exec_()

    executor.shutdown()
    backend.destroy_node()
    rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()

