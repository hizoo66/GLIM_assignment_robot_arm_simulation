# Doosan E0509 MoveIt2 GUI (ROS2 Humble)

PyQt5 기반 GUI를 통해 **Doosan E0509 로봇팔**을
**ROS2 + MoveIt2 + 시뮬레이션** 환경에서 **XYZ 목표점 기반으로 계획/실행**하기 위한 애플리케이션입니다.

본 프로젝트의 목적은
> **ROS2 노드 + MoveIt2 Action + GUI(Thread 분리)** 구조를 명확히 이해

---

## 1. 개발 환경

### OS
- Ubuntu 22.04

### ROS
- ROS2 Humble

### 주요 프레임워크 / 라이브러리
- **rclpy** (ROS2 Python API)
- **MoveIt2**
- **PyQt5** (GUI)
- **Rviz** (시뮬레이션)
- **MultiThreadedExecutor**

### 사용 메시지 / 액션
- `sensor_msgs/JointState`
- `geometry_msgs/PoseStamped`
- `moveit_msgs/action/MoveGroup`
- `moveit_msgs/msg/Constraints`

---

## 2. 전체 구조 개요

┌──────────────┐

│ 　PyQt5 GUI　　　　 │ ← 사용자 입력 (XYZ, 속도, 모드)

└──────┬───────┘

　　　　　│ (Qt Signal / Thread)

┌──────▼───────┐

│　 MoveWorker　　 │ ← 작업 스레드 (비동기 실행)

└──────┬───────┘

　　　　　　│

┌──────▼──────────────────┐

│ 　BackendNode 　　　　　　　│

│ 　- JointState subscribe 　　　　│

│ 　- MoveGroup Action Client　　│

└──────┬──────────────────┘

　　　　　│

┌──────▼─────────┐

│　 MoveIt2 　　　　　　│

│ (Planning/Exec)　　　 │

└────────────────┘


핵심 설계 포인트:
- **ROS Spin과 GUI 이벤트 루프 분리**
- **MoveIt Action은 별도 Worker Thread에서 실행**
- GUI는 ROS 통신으로 블로킹되지 않음

---

## 3. 실행 방법 (Execution Manual)

### 3.1 사전 준비

1. ROS2 & MoveIt2 설치
**ROS2 설치 : ros-humble-desktop**
    ROS2 저장소 추가
    ```python
    # add-apt-repository 명령어 제공 : PPA / universe / multiverse 추가 기능
    sudo apt install software-properties-common -y
    # Ubuntu가 제공하는 공식 universe 저장소를 활성화
    sudo add-apt-repository universe
    
    # 키 등록
    sudo apt update
    sudo apt install curl -y
    sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
    ```
    
    ROS2 소스 추가
    ```python
    echo "deb [arch=$(dpkg --print-architecture) \
    signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
    http://packages.ros.org/ros2/ubuntu \
    $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
    
    ```
    
    ROS2 Humble 설치
    ```python
    sudo apt update
    sudo apt install ros-humble-desktop -y
    
    # ROS2 환경 활성화
    source /opt/ros/humble/setup.bash
    ```
    
**doosan-robot2 설치 시작 :** https://github.com/DoosanRobotics/doosan-robot2
    
    ROS2 워크스페이스 만들기
    ```python
    cd ~
    mkdir -p ros2_ws/src
    cd ros2_ws
    ```
    
    doosan-robot2 GitHub 클론
    ```python
    cd ~/ros2_ws/src
    git clone https://github.com/DoosanRobotics/doosan-robot2.git
    
    ```
    
    - 두산로보틱스 Required Dependencies
    ```python
    sudo apt-get update
    sudo apt-get install -y libpoco-dev libyaml-cpp-dev wget \
                            ros-humble-control-msgs ros-humble-realtime-tools ros-humble-xacro \
                            ros-humble-joint-state-publisher-gui ros-humble-ros2-control \
                            ros-humble-ros2-controllers ros-humble-gazebo-msgs ros-humble-moveit-msgs \
                            dbus-x11 ros-humble-moveit-configs-utils ros-humble-moveit-ros-move-group \
                            ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ign-ros2-control
    
    ```
    
    - 두산 Install Gazebo 
    ```python
    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install -y libignition-gazebo6-dev ros-humble-gazebo-ros-pkgs ros-humble-ros-gz-sim ros-humble-ros-gz
    ```
    
    ---
    
**rosdep 설치**
    
    ```python
    sudo apt update
    sudo apt install python3-rosdep -y
    
    # 초기화 한 번만 실행, rosdep 데이터베이스 초기화
    sudo rosdep init
    ```
    
    의존성 설치 (자동)

    ```python
    cd ~/ros2_ws
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    ```
    
**colcon 설치**
        
    ```python
    sudo apt update
    sudo apt install python3-colcon-common-extensions -y
    
    ```
2. Doosan E0509 MoveIt 패키지 실행
   (시뮬레이션 툴 또는 실기)

```bash
# 본 프로젝트에서는 moveit2 + Rviz사용 (터미널 A)
ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=virtual model:=e0509
```

### 3.2 JointState 토픽이 publish 중인지 확인
```bash
# 다른 터미널 (터미널 B)
ros2 topic list | grep joint
```

---

### 3.3 GUI 실행
```bash
ros2 run my_ros2_assignment my_node
```
실행 시:

- joint_states 토픽 자동 탐색
- `/move_action` MoveGroup Action 연결 시도

---

## 4. 작동 로직 설명 (핵심)

### 4.1 JointState 자동 탐색 로직
- 환경마다 JointState 토픽 이름이 다르기 때문에 토픽 후보를 우선 탐색한다.
```bash
JOINT_STATE_CANDIDATES = [
    "/dsr01/joint_states",
    "/joint_states",
    "/dsr01/gz/joint_states",
]
```
- 없으면 endswith("joint_states")로 Fallback 하며 발견 시 lazy subscribe (Timer 기반)
-> 이로써 노드 실행 시 joint_states가 늦게 올라와도 문제 없다.
---

### 4.2 MoveIt 제어 방식
- `MoveGroup` **Action Client** 사용
- 목표는 **XYZ 위치만 제약**
- Orientation은 Identity Quaternion 유지
```bash
PositionConstraint
 └─ BoundingVolume (Sphere, radius=0.01m)
 ```
- 해당 방식을 통해 IK 안정성을 높이며, Orientation 문제로 인한 실패를 낮춘다
---

### 4.3 Absolute / Relative 좌표 모드
| 모드 | 설명 |
| --- | --- |
| Absolute | base_frame 기준 절대 좌표 |
| Relative | 이전 목표점 기준 offset 누적 | 

---

### 4.4 Thread 분리 구조

| 역할 | Thread |
| --- | --- |
| GUI | Main Qt Thread |
| ROS Spin | RosSpinThread |
| MoveIt 실행 | MoveWorker |
- `MultiThreadedExecutor` 사용
- MoveIt 실행 중에도 GUI 응답 유지

---

## 5. UI 구성 설명

### 5.1 Control Panel (좌측)

| 항목 | 설명 |
| --- | --- |
| Relative mode | 상대좌표 이동 |
| Max velocity | MoveIt 속도 스케일 (0~1) |
| Max acceleration | 가속도 스케일 (0~1) |
| Target Table | XYZ 목표점 리스트 |
| Execute | 이동 실행 |
| Stop | 현재 작업 중단 |

---

### 5.2 Status Panel (우측)

| 항목 | 설명 |
| --- | --- |
| robot | group / ee_link / base_frame |
| motion | idle / moving |
| joint_topic | 감지된 joint_states 토픽 |
| joints | 현재 관절값 (최대 6개 표시) |
| moveit | 연결된 action 이름 |
| Log | 실행 로그 출력 |

---

## 6. 주요 클래스 설명

### BackendNode

- ROS2 Node
- JointState 구독
- MoveGroup Action Client
- GUI에서 접근 가능한 상태 제공

### MoveWorker (QThread)

- 목표점 순차 실행
- Stop 요청 처리
- GUI로 로그/완료 신호 전달

### MainWindow

- 사용자 입력 처리
- Worker 관리
- 상태 주기적 갱신

### RosSpinThread

- ROS Executor 전용 스레드
- GUI 블로킹 방지

---

## 7. 파라미터

| 파라미터 | 기본값 | 설명 |
| --- | --- | --- |
| move_action | `/move_action` | MoveGroup Action |
| joint_topic | `""` | 비우면 자동 탐색 |
| group_name | `manipulator` | MoveIt planning group |
| ee_link | `link_6` | End-effector link |
| base_frame | `base_link` | 기준 프레임 |

---

## 8. 주의사항

- MoveIt `move_group` 노드가 반드시 실행 중이어야 함
- base_frame / ee_link 이름은 로봇 URDF 기준 확인 필요
- Orientation 제약이 필요한 경우 확장 필요

---
