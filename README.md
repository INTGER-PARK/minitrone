# Minitrone MuJoCo · ROS 2 Workspace

본 워크스페이스는 **서울과학기술대학교 모바일 로봇 연구실(Mobile Robotics Laboratory, MRL)**에서 제작한 드론 시뮬레이션 환경입니다.

**연구실 홈페이지: [https://mrl.seoultech.ac.kr/index.do](https://mrl.seoultech.ac.kr/index.do)**

4개의 모터와 4개의 틸트 서보를 갖는 Minitrone의 비행 및 벽 접촉 실험을 위한 ROS 2 워크스페이스입니다. MuJoCo 물리 시뮬레이션, 위치·자세 제어, 구동기 할당, 외력 관측, 접촉 압력중심(CoP) 추정, 어드미턴스 및 수동 정렬 제어를 포함합니다.

## 1. 실행 환경과 MuJoCo 설치

ROS 2는 이미 설치되어 있다고 가정합니다. 아래 명령은 **Ubuntu 22.04 / ROS 2 Humble / 시스템 Python 3.10** 기준입니다. MuJoCo는 현재 개발 환경에 설치된 **3.3.7**을 기준으로 고정합니다.

### 의존성 설치

```bash
source /opt/ros/humble/setup.bash
sudo apt update
sudo apt install -y \
  build-essential python3-pip python3-colcon-common-extensions \
  python3-numpy python3-pyqt5 python3-pyqtgraph \
  libglfw3 libgl1-mesa-dri libeigen3-dev \
  ros-humble-eigen3-cmake-module ros-humble-rosidl-default-generators \
  ros-humble-rosidl-runtime-py ros-humble-rosbag2

/usr/bin/python3 -m pip install --user "mujoco==3.3.7" "numpy<2"
```

MuJoCo의 공식 Python 패키지에는 엔진 라이브러리가 포함되어 있어 별도 엔진 다운로드가 필요하지 않습니다. 설치 방법은 [MuJoCo 공식 Python 문서](https://mujoco.readthedocs.io/en/3.3.7/python.html#installation)를 참고하세요. ROS 2와 같은 시스템 Python을 사용하도록 Conda 등 다른 Python 환경은 비활성화한 상태에서 설치·빌드합니다.

```bash
/usr/bin/python3 -c "import mujoco, rclpy, pyqtgraph; print('MuJoCo:', mujoco.__version__)"
```

MuJoCo 뷰어와 그래프 창에는 그래픽 데스크톱 및 OpenGL 환경이 필요합니다. 화면 없이 물리 시뮬레이션만 실행하는 방법은 아래 `minitrone_plant` 항목을 참고하세요.

## 2. 빌드 및 터미널 준비

저장소를 `~/ros2_project/minitrone_ws`에 배치한 예시입니다. 다른 경로에 있다면 `cd` 경로를 변경합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_project/minitrone_ws
colcon build --executor sequential
source install/setup.bash
```

**이후 실행 명령을 입력하는 각 터미널마다** 다음을 실행합니다.

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_project/minitrone_ws
source install/setup.bash
```

Python 코드나 XML을 수정한 경우에도 설치 공간에 반영되도록 다시 빌드합니다.

```bash
colcon build --packages-select minitrone_plant --executor sequential
source install/setup.bash
```

## 3. 패키지 구성

| 패키지 | 언어 / 빌드 | 역할 |
| --- | --- | --- |
| `minitrone_plant` | Python / ament_python | MuJoCo 모델 로딩, 물리 계산, 상태·접촉 데이터 발행, 벽 조작 및 그래프 |
| `minitrone_controller` | C++ / ament_cmake | 위치·자세 제어, 모터·서보 할당, 외력 관측기, EKF, 접촉 제어 |
| `minitrone_cmd` | C++·Python / ament_cmake | 위치·자세·외력 명령 생성, 키보드 조작, launch 파일 |
| `minitrone_interfaces` | ROS 메시지 / ament_cmake | 패키지 사이에서 사용하는 사용자 정의 메시지 |

```text
minitrone_ws/
├── src/
│   ├── minitrone_plant/
│   │   ├── minitrone_plant/     # 시뮬레이터와 GUI 노드
│   │   └── xml/                 # 기체·벽·장면 모델
│   ├── minitrone_controller/src/
│   ├── minitrone_cmd/
│   │   ├── src/
│   │   ├── scripts/
│   │   └── launch/
│   └── minitrone_interfaces/msg/
├── matlabscripts/               # CSV 기반 실험 결과 시각화
├── bags/                        # rosbag 기록
├── build/                       # 빌드 산출물
├── install/                     # ROS 2 실행 시 사용하는 설치 공간
└── log/                         # 빌드 로그
```

### 모델 및 제어 흐름

실제로 로딩하는 장면은 [`src/minitrone_plant/xml/Scene.xml`](src/minitrone_plant/xml/Scene.xml)입니다. 파일 이름의 대소문자를 구분합니다.

- `minitrone_430x430x370_armature_walls.xml`: 기체, 틸트 서보, 추력 구동기 및 +X 방향 접촉판.
- `palm.xml`: 위치와 자세를 변경할 수 있는 벽 역할의 `hand_palm`.
- 폴더의 다른 기체 XML은 현재 `Scene.xml`에서 직접 사용하지 않습니다.

기체 본체 질량은 2.5 kg이며, plant는 물리 간격을 **0.0025 s (400 Hz)**로 설정합니다. 입력은 모터 속도 명령 4개와 서보 각도 명령 4개입니다. 모터 추력은 `0.02 × omega²`로 변환되고, allocator의 모터당 최대 추력은 15 N, 서보 명령 제한은 ±65°입니다.

```text
위치·자세 명령 ──> wrench_controller ──> allocator_controller ──> plant
                       ↑                       ↑                 │
                       └────────── /minitrone/state ──────────────┘

plant ── /minitrone/mob_observer_input ──> 외력 관측기
                                              │
                                      외력·모멘트 / CoP 추정
                                              │
                                      선택적 접촉 제어
```

기본 제어기는 `/minitrone/state`를 사용합니다. EKF를 실행해도 제어기 입력이 자동으로 EKF 출력으로 변경되지는 않습니다.

## 4. 빠른 실행

### 벽 접촉 실험: `arm_launch.py`

터미널 1에서 실행합니다.

```bash
ros2 launch minitrone_cmd arm_launch.py
```

다음 프로세스가 실행됩니다.

| 실행 대상 | 기능 |
| --- | --- |
| `minitrone_plant` | MuJoCo 시뮬레이션과 뷰어 |
| `minitrone_wrench_controller` | 위치·자세 명령을 힘·모멘트로 변환 |
| `minitrone_allocator_controller` | 힘·모멘트를 모터·서보 명령으로 변환 |
| `minitrone_second_wrench_observer` | 2차 외력 관측 및 CoP 추정 |
| `ros2 bag record -a` | 전체 토픽 기록 |

**위치·자세 명령 노드와 접촉 제어 노드는 자동 실행되지 않습니다.** 터미널 2에서 위치 명령을 추가합니다.

```bash
ros2 run minitrone_cmd minitrone_position_teleop
```

필요하면 별도 터미널에서 벽 조작과 결과 그래프를 실행합니다.

```bash
ros2 run minitrone_plant minitrone_palm_teleop
```

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot
```

bag 저장 경로는 현재 launch 코드에서 **`~/ros2_project/minitrone_ws/bags/bag_all_YYYYMMDD_HHMMSS`**로 지정되어 있습니다. 저장소를 다른 위치로 옮겨도 이 경로는 자동으로 바뀌지 않으므로 `arm_launch.py`의 `workspace_dir`을 수정해야 합니다.

### 실행 시 공통 사항

- launch에서 이미 시작한 노드를 `ros2 run`으로 중복 실행하지 않습니다.
- `/minitrone/cmd`를 발행하는 `minitrone_position_cmd`와 `minitrone_position_teleop` 중 하나만 활성화합니다.
- 키보드 조작 노드는 별도 터미널에서 `ros2 run`으로 실행하고 해당 터미널에 포커스를 둡니다. 뷰어의 `F` 키는 뷰어 창에서 입력합니다.
- 종료는 각 터미널에서 `Ctrl+C`를 사용합니다. 뷰어 창만 닫아도 ROS 노드와 물리 계산은 계속 실행될 수 있습니다.

## 5. `minitrone_plant` 실행 노드

### `minitrone_plant` — 물리 시뮬레이터

`/minitrone/input`의 구동 명령을 적용하고 기체 상태, 실제 구동 힘·모멘트, 접촉 힘과 CoP를 발행합니다. 외력 명령과 벽 자세 명령도 처리합니다. 단독 실행에는 비행 제어기가 포함되지 않습니다.

```bash
ros2 run minitrone_plant minitrone_plant
```

뷰어 없는 실행:

```bash
ros2 run minitrone_plant minitrone_plant --ros-args -p enable_viewer:=false
```

| 파라미터 | 기본값 | 기능 |
| --- | --- | --- |
| `enable_viewer` | `true` | MuJoCo 뷰어 실행 |
| `viewer_show_propellers` | `true` | 프로펠러 시각화 그룹 표시 |
| `viewer_show_contact_forces` | `true` | 접촉 합력 화살표 표시, 뷰어 `F` 키로 전환 |
| `viewer_contact_force_scale` | `0.03` | 접촉력 화살표 길이 배율 |
| `random_seed` | `1` | 센서 잡음 난수 시드 |
| `contact_test_case` | `B` | 접촉 조건 선택 |
| `isolate_plate_wall_contact` | `true` | 벽과 기체의 접촉을 접촉판 중심으로 분리 |
| `solver_iterations` | `0` | 양수이면 solver 반복 횟수 덮어쓰기 |

접촉 조건은 A: 마찰 없는 `condim=1`, B: 미끄럼 마찰을 포함한 `condim=3`, C: 회전 마찰을 매우 작게 둔 `condim=6`, D: XML의 `condim`과 마찰 유지입니다. 기본 실험은 B입니다.

```bash
ros2 run minitrone_plant minitrone_plant --ros-args \
  -p contact_test_case:=B -p solver_iterations:=100
```

현재 launch 파일에는 `enable_viewer` 등을 전달하는 launch 인자가 없습니다. headless 구성은 plant를 위 명령으로 실행하고 제어기를 별도 실행하거나 launch의 `Node(parameters=...)`를 수정합니다.

### `minitrone_palm_teleop` — 벽 위치·자세 조작

`/minitrone/palm_pose_cmd`로 벽의 위치·자세를 보냅니다. 기본 이동 속도는 0.05 m/s, 회전 속도는 1°/s입니다.

```bash
ros2 run minitrone_plant minitrone_palm_teleop --ros-args \
  -p speed_xyz:=0.05 -p speed_rpy_deg:=1.0
```

| 키 | 기능 |
| --- | --- |
| `W/S`, `A/D`, `Q/E` | 각각 x, y, z 위치 증가/감소 |
| `R/F`, `T/G`, `Y/H` | 각각 yaw, pitch, roll 증가/감소 |
| `Space` / `0` / `X` | 이동 정지 / 초기 명령 복원 / 종료 |

### `minitrone_external_wrench_plot` — 외력 및 CoP 그래프

2차 외력 관측기 출력과 CoP 추정·실제 값을 확인하는 Qt/PyQtGraph GUI입니다. 관측기와 plant가 실행되어 있어야 해당 데이터가 표시됩니다.

```bash
ros2 run minitrone_plant minitrone_external_wrench_plot --ros-args \
  -p window_sec:=15.0 -p refresh_hz:=20.0 \
  -p plot_rows:=3 -p plot_columns:=3
```

`plot_rows`와 `plot_columns`는 1~3입니다. 접촉판 표시 크기는 기본 `plate_size_y=0.40`, `plate_size_z=0.38` m이며, `cop_force_min=0.5` N, `cop_trail_length=100`, `cop_y_sign=1.0`, `cop_z_sign=-1.0`으로 표시를 조정할 수 있습니다.

### `minitrone_topic_plot` — 범용 토픽 그래프

```bash
ros2 run minitrone_plant minitrone_topic_plot
```

토픽 목록에서 숫자 필드를 선택해 그래프에 추가합니다. 가변 길이 배열은 메시지가 들어온 뒤 원소별 필드로 펼쳐집니다.

## 6. `minitrone_cmd` 실행 노드

### `minitrone_position_cmd` — 목표 위치 명령

목표 위치를 `/minitrone/cmd`로 200 Hz 발행합니다. 기본적으로 비활성화되어 있으며 `O` 키 또는 `enabled` 파라미터로 전환합니다. x 이동은 기본 5초, y 이동은 기본 10초 이후 시작하며 축별 속도 제한으로 목표에 접근합니다. 초기 z 명령은 `Z_CMD`로 바로 설정됩니다.

```bash
ros2 run minitrone_cmd minitrone_position_cmd --ros-args \
  -p enabled:=true -p X_CMD:=0.0 -p Y_CMD:=0.0 -p Z_CMD:=1.0
```

실행 중 목표 변경:

```bash
ros2 param set /minitrone_position_cmd X_CMD 0.5
ros2 param set /minitrone_position_cmd Z_CMD 1.2
```

속도 제한 기본값은 `max_speed_x=0.10`, `max_speed_y=0.10`, `max_speed_z=0.30` m/s입니다. 시작 시각은 `approach_start_sec`, `y_start_sec`로 지정합니다.

### `minitrone_position_teleop` — 키보드 위치 명령

키 입력으로 이동 속도를 선택하고 이를 적분한 위치 명령을 발행합니다. 기본 초기 명령은 `[0, 0, 1] m`입니다.

```bash
ros2 run minitrone_cmd minitrone_position_teleop --ros-args \
  -p speed_xy:=0.3 -p speed_z:=0.2
```

| 키 | 기능 |
| --- | --- |
| `W/S`, `A/D`, `R/F` | 각각 x, y, z 증가/감소 |
| 방향키 / `[` / `]` | x·y 이동 / 상승 / 하강 |
| `Space` / `0` / `Q` | 이동 정지 / 초기 명령 복원 / 종료 |

### `minitrone_attitude_sweep_cmd` — 키보드 자세 명령

이름과 달리 자동 주기 스윕이 아니라 키 입력으로 자세 명령을 변경하는 노드입니다. `/minitrone/att_cmd`를 기본 50 Hz로 발행하고, 한 번에 0.5°씩 변경합니다. 기본 제한은 각 축 ±60°입니다.

```bash
ros2 run minitrone_cmd minitrone_attitude_sweep_cmd --ros-args -p max_abs_deg:=30.0
```

| 키 | 기능 |
| --- | --- |
| `q/w`, `e/r`, `t/y` | 각각 roll, pitch, yaw 증가/감소 |
| `z` / `x` | 자세 명령 0으로 초기화 / 종료 |

### `minitrone_external_wrench_cmd` — 외력·외부 모멘트 입력

기체 body 좌표계 외란을 `/minitrone/external_wrench_cmd`로 발행합니다. 기본 증분은 힘 0.5 N, 모멘트 0.1 N·m이며, plant는 명령 수신이 0.2초 이상 끊기면 외란을 해제합니다.

```bash
ros2 run minitrone_cmd minitrone_external_wrench_cmd --ros-args \
  -p force_step:=0.5 -p moment_step:=0.1 \
  -p moment_disturbance_amplitude:=0.3 \
  -p moment_disturbance_frequency_hz:=0.5
```

| 키 | 기능 |
| --- | --- |
| `q/a`, `w/s`, `e/d` | 각각 Mx, My, Mz 증가/감소 |
| `i/k`, `j/l` | 각각 Fx, Fy 증가/감소 |
| `u` | Fz 증가 |
| `o`, `p`, `y` | 각각 Mx, My, Mz 사인파 외란 켜기/끄기 |
| `z` / `x` | 모든 외란 초기화 / 종료 |

현재 코드에서 `o`는 Fz 감소가 아니라 Mx 사인파 외란 전환입니다. Fz 감소 키는 구현되어 있지 않습니다.

## 7. `minitrone_controller` 실행 노드

### `minitrone_wrench_controller` — 위치·자세 제어

위치 PID, 자세 제어 및 중력 보상으로 body 좌표계 힘·모멘트 명령 `/minitrone/wrench_cmd`를 계산합니다. 새 상태 메시지 수신 시 제어를 수행하며, 어드미턴스 활성 상태에서는 해당 노드의 위치·자세 명령을 사용합니다.

```bash
ros2 run minitrone_controller minitrone_wrench_controller
```

`mass` 기본값은 2.5 kg, `gravity`는 9.81 m/s²입니다.

### `minitrone_allocator_controller` — 구동기 할당

힘·모멘트 명령과 측정 서보 각도를 이용해 모터 속도 4개와 서보 각도 4개를 계산하고 `/minitrone/input`으로 발행합니다.

```bash
ros2 run minitrone_controller minitrone_allocator_controller
```

포화 진단 로그 활성화:

```bash
ros2 param set /minitrone_allocator_controller enable_saturation_debug true
```

입력 토픽은 `input_wrench_topic`으로 변경할 수 있으며 기본값은 `/minitrone/wrench_cmd`입니다.

### `minitrone_first_wrench_observer` — 1차 외력 관측기

plant가 발행하는 상태와 실제 구동 힘·모멘트 묶음인 `/minitrone/mob_observer_input`을 사용해 외력을 추정합니다. 출력은 `/minitrone/external_wrench_hat`입니다.

```bash
ros2 run minitrone_controller minitrone_first_wrench_observer
```

### `minitrone_second_wrench_observer` — 2차 외력 관측기 및 CoP 추정

같은 observer 입력으로 외력·외부 모멘트를 추정하고 `/minitrone/external_wrench_hat_second_order`, `/minitrone/cop_hat`을 발행합니다. 어드미턴스와 수동 정렬 제어에서 사용하는 관측기입니다.

```bash
ros2 run minitrone_controller minitrone_second_wrench_observer
```

축별 관측기 설정은 `omega_n_force_x/y/z`, `omega_n_moment_x/y/z`, `zeta_force_x/y/z`, `zeta_moment_x/y/z`입니다. CoP 계산에는 `plate_size_y`, `plate_size_z`, `cop_force_min`, `r_com_to_contact_x/y/z` 등을 사용합니다. 기체나 접촉판 모델 변경 시 이 설정도 확인합니다.

### `minitrone_ekf_state_estimator` — 상태 추정

위치·속도·자세와 IMU 바이어스를 포함한 15차원 상태를 추정합니다. 기본 입력은 `/minitrone/state`, 출력은 `/minitrone/state_ekf`이며 추가 진단 출력은 `/ekf/state_vector`, `/ekf/bias`입니다.

```bash
ros2 run minitrone_controller minitrone_ekf_state_estimator
```

**현재 가속도 인터페이스에 불일치가 있습니다.** EKF와 메시지 주석은 `acc`를 body 좌표계 IMU specific force로 가정하지만, 현재 plant는 world 속도를 차분한 가속도를 넣습니다. EKF 결과를 제어 피드백으로 사용하기 전에 이 정의를 일치시켜야 합니다. 기본 제어기는 EKF 출력을 사용하지 않습니다.

### `minitrone_admittance_controller` — 6자유도 어드미턴스 제어

2차 관측기의 외력 추정을 이용해 위치·자세 참조를 조정합니다. `/minitrone/cmd_admittance`, `/minitrone/att_cmd_admittance`, `/minitrone/admittance_active`를 통해 wrench controller와 연동합니다.

`arm_launch.py`와 위치 명령 노드가 실행 중인 상태에서 별도 터미널에 추가합니다.

```bash
ros2 run minitrone_controller minitrone_admittance_controller --ros-args \
  -p f_normal_des:=5.0
```

| 키 | 기능 |
| --- | --- |
| `O` | 어드미턴스 활성화 / 해제 후 현재 자세 유지(HOLD) |
| `P` | 일반 위치·자세 명령 통과 모드로 복귀 |
| `U/J` | 목표 법선 힘 증가/감소, 기본 0.1 N |

토픽으로 활성화할 수도 있습니다.

```bash
ros2 topic pub --once /minitrone/admittance_enable std_msgs/msg/Bool "{data: true}"
```

기본값은 `enabled=false`, `f_normal_des=5.0` N입니다. 축별 가상 질량·감쇠·강성은 `adm_m_*`, `adm_d_*`, `adm_k_*` 파라미터로 설정합니다.

### `passive_aligning_controller` — 접촉면 수동 정렬 제어

`/minitrone/wrench_cmd`를 받아 접촉 법선 힘과 접선 방향 운동을 제어하고, 접촉면 정렬에 필요한 회전축의 제어 모멘트를 완화합니다. 결과는 `/minitrone/wrench_passive_align`로 발행합니다.

```bash
ros2 run minitrone_controller passive_aligning_controller --ros-args \
  -p f_normal_des:=5.0
```

`O`로 활성화/해제하고 `U/J`로 목표 법선 힘을 조절합니다. 토픽으로는 `/minitrone/passive_align_enable`에 `std_msgs/msg/Bool`을 보냅니다.

**이 노드만 추가하면 allocator는 여전히 원래 wrench를 사용합니다.** 수동 정렬 실험에서는 기존 launch를 종료하고, 아래 명령을 각각 별도 터미널에서 실행해 전체 경로를 연결합니다.

```bash
# 터미널 1
ros2 run minitrone_plant minitrone_plant
# 터미널 2
ros2 run minitrone_controller minitrone_wrench_controller
# 터미널 3
ros2 run minitrone_controller minitrone_second_wrench_observer
# 터미널 4: 필터 출력을 allocator에 연결
ros2 run minitrone_controller minitrone_allocator_controller --ros-args \
  -p input_wrench_topic:=/minitrone/wrench_passive_align
# 터미널 5: 실행 후 O 키로 활성화
ros2 run minitrone_controller passive_aligning_controller
# 터미널 6
ros2 run minitrone_cmd minitrone_position_teleop
```

## 8. 메시지 및 주요 토픽

`minitrone_interfaces`는 메시지만 제공하며 `ros2 run` 실행 노드는 없습니다.

| 메시지 | 주요 필드와 단위 |
| --- | --- |
| `Cmd` | `pos_cmd[3]`: world 위치 [m] |
| `AttitudeCmd` | `roll_ref`, `pitch_ref`, `yaw_ref`: 자세 명령 [deg] |
| `Wrench` | `force[3]` [N], `moment[3]` [N·m]; 좌표계·기준점은 토픽별로 구분 |
| `Input` | `u[0:4]`: 모터 속도 명령, `u[4:8]`: 서보 명령 [rad] |
| `MinitroneState` | `pos`, `vel`: world; `rpy`: rad; `w_rpy`: body 각속도 [rad/s]; `servo`: deg. `acc`는 위 EKF 설명 참고 |
| `MobObserverInput` | `step`, `sim_time`, 상태, body 좌표계 실제 구동 힘·모멘트 |
| `CenterOfPressure` | 접촉면 좌표계 `y`, `z` [m], `normal_force` [N], `valid`, `corner_forces[4]` |

| 토픽 | 용도 |
| --- | --- |
| `/minitrone/cmd`, `/minitrone/att_cmd` | 위치·자세 목표 |
| `/minitrone/wrench_cmd` | 제어 힘·모멘트 명령 |
| `/minitrone/input` | 모터·서보 명령 |
| `/minitrone/state` | plant 상태 |
| `/minitrone/mob_observer_input` | 외력 관측기에 전달하는 상태·구동 입력 묶음 |
| `/minitrone/actuation_wrench_body` | 실제 구동 힘·모멘트 |
| `/minitrone/external_wrench_cmd` | 사용자가 부여하는 외란 |
| `/minitrone/external_wrench_hat_second_order` | 2차 관측기 외력 추정 |
| `/minitrone/cop_real`, `/minitrone/cop_hat` | 접촉 데이터 기반 CoP / 관측기 기반 CoP |
| `/minitrone/palm_pose_cmd`, `/minitrone/palm_pose_state` | 벽 위치·자세 명령 / 상태 |
| `/contact_method1/contact_wrench_ground_truth_com` | CoM 기준 접촉 wrench |
| `/contact_method1/contact_wrench_ground_truth_C` | 접촉면 중심 기준 접촉 wrench |
| `/contact_method1/contact_count`, `/contact_method1/debug/*` | 접촉 수, 힘 분해, 접촉 영역 등 진단 |

CoP의 접촉면 좌표계 C는 기체 +X 접촉면 중심에 놓이며 +x는 기체에서 벽을 향하는 법선입니다. `valid=false`인 CoP는 유효한 접촉 위치로 해석하지 않습니다.

```bash
ros2 node list
ros2 topic list
ros2 topic hz /minitrone/state
ros2 topic echo /minitrone/cop_hat
ros2 interface show minitrone_interfaces/msg/MobObserverInput
```

## 9. 기록 및 결과 분석

`arm_launch.py`는 자동으로 전체 토픽을 기록합니다. 다른 구성에서 수동 기록하려면:

```bash
mkdir -p bags
ros2 bag record -a -o bags/manual_contact_test
```

출력 디렉터리 이름은 기존 기록과 겹치지 않게 지정하고, 기록 종료 시 `Ctrl+C`로 정상 종료합니다.

```bash
ros2 bag info bags/manual_contact_test
ros2 bag play bags/manual_contact_test
```

재생 데이터만 분석할 때는 실시간 시뮬레이션을 종료한 뒤 plot 노드를 실행합니다.

[`matlabscripts/minitrone_plot_all_topics.m`](matlabscripts/minitrone_plot_all_topics.m)은 **bag 파일이 아닌 CSV**를 읽어 위치, 자세, 서보, 추력, 외력 등을 표시합니다. 스크립트의 `csv_path`를 실제 파일로 변경해야 하며, bag→CSV 변환 도구는 이 저장소에 포함되어 있지 않습니다. 입력 열과 데이터 형식은 스크립트의 `normalize_csv_columns` 및 토픽 변환 함수를 참고하세요.

## 10. 실행 문제 확인

| 증상 | 확인 사항 |
| --- | --- |
| `Package not found` / 실행 파일을 찾지 못함 | 빌드 성공 여부 및 해당 터미널의 `source install/setup.bash` 확인 |
| `No module named mujoco` | ROS 노드를 실행하는 시스템 Python에 MuJoCo를 설치했는지 확인 |
| GLFW 초기화 실패 / 뷰어가 열리지 않음 | 그래픽 세션과 OpenGL 환경 확인. 화면이 필요 없으면 plant의 `enable_viewer:=false` 사용 |
| Qt/PyQtGraph import 오류 | `python3-pyqt5`, `python3-pyqtgraph` 설치 및 Python 환경 확인 |
| 위치 명령이 나오지 않음 | position command의 `enabled` 상태 확인. `arm_launch.py`에는 위치 명령 노드가 없어 별도 실행 필요 |
| 수동 정렬이 제어에 반영되지 않음 | allocator의 `input_wrench_topic`이 `/minitrone/wrench_passive_align`인지 확인 |
| `mj_copyDataVisual: ... stack is in use` | 최신 `plant.py`를 빌드했는지 확인. 뷰어 초기화·동기화와 물리 계산은 같은 잠금으로 보호되어야 함 |

새 환경의 전체 조합과 장시간 GUI 실행은 환경별 확인이 필요합니다. 이 문서는 현재 소스의 실행 파일, 기본 파라미터 및 토픽 연결을 기준으로 작성했습니다.
