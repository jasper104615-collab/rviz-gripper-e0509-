# RViz Real Gripper Bundle

Doosan **공식 GitHub** + RH-P12-RNA **공식 GitHub**만 clone한 PC에 **추가로** 넣을 파일 모음입니다.

RViz에서 **실로봇 팔 + 실그리퍼 상태**를 보거나, Isaac sim2real 그리퍼 명령을 쓸 때 필요합니다.

---

## 전제 (다른 PC에서 미리 받아 둘 것)

| Repo | 용도 | 예시 clone |
|------|------|------------|
| **doosan-robot2** | `dsr_bringup2`, `dsr_description2`, mesh | `~/doosan-robot2` |
| **rh_p12_rna_controller** | `gripper_service_node`, Modbus/TCP | `~/rh_p12_rna_controller` |

- ROS 2 Humble
- Doosan ws 빌드·source 완료 (`dsr_bringup2`, `dsr_description2` 등)

---

## 이 번들 폴더 구조

```
rviz_real_gripper_bundle/
├── README.md                          ← 이 파일
├── scripts/
│   └── install_overlay.sh             ← 공식 repo에 URDF·gripper_node 덮어쓰기
├── overlay/                           ← 공식 repo용 패치 파일
│   ├── README.md
│   ├── doosan/dsr_description2/urdf/e0509_with_gripper.urdf
│   └── rh_p12/.../gripper_node.py
└── ros2_ws/src/                       ← colcon에 넣을 **추가** 패키지
    ├── isaac_gripper_sim2real/        ← RViz 병합, sim2real 브리지, launch
    └── rh_p12_rna_controller_interfaces/  ← msg/srv/action (공식에 없으면 필수)
```

### 공식 repo에 이미 있는 것 (번들에 **없음**)

- `dsr_bringup2`, `dsr_controller2`, `dsr_description2` (mesh 포함)
- `rh_p12_rna_controller` 본체 (`gripper_service_node` 등) — **overlay로 `gripper_node.py`만 교체**

---

## 설치 (1회)

### 1) 이 번들 복사

```bash
# sim2real_deploy 안에 있거나, USB/zip으로 다른 PC에 복사
cd ~/sim2real_deploy/rviz_real_gripper_bundle   # 경로는 자유
```

### 2) Overlay 적용 (URDF + gripper_node)

```bash
chmod +x scripts/install_overlay.sh

# 기본 경로: ~/doosan-robot2, ~/rh_p12_rna_controller
./scripts/install_overlay.sh

# 경로가 다르면
DOOSAN_SRC=~/doosan-robot2/src/doosan-robot2 \
RH12_SRC=~/src/rh_p12_rna_controller \
  ./scripts/install_overlay.sh
```

### 3) ROS ws에 번들 패키지 링크/복사

```bash
mkdir -p ~/sim2real_ros2_ws/src
cp -r ros2_ws/src/isaac_gripper_sim2real ~/sim2real_ros2_ws/src/
cp -r ros2_ws/src/rh_p12_rna_controller_interfaces ~/sim2real_ros2_ws/src/

# RH-P12 공식 repo도 같은 ws에 (overlay 적용된 상태)
cp -r ~/rh_p12_rna_controller ~/sim2real_ros2_ws/src/
# 또는 symlink: ln -s ~/rh_p12_rna_controller ~/sim2real_ros2_ws/src/
```

### 4) 빌드

```bash
source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash

cd ~/sim2real_ros2_ws
colcon build --packages-select \
  rh_p12_rna_controller_interfaces \
  rh_p12_rna_controller \
  isaac_gripper_sim2real

# Doosan URDF 갱신
cd ~/doosan-robot2
colcon build --packages-select dsr_description2

source ~/doosan-robot2/install/setup.bash
source ~/sim2real_ros2_ws/install/setup.bash
```

---

## 실행

### A) RViz — 실로봇 팔 + 그리퍼 **상태만** (명령 X)

```bash
source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash
source ~/sim2real_ros2_ws/install/setup.bash

ros2 launch isaac_gripper_sim2real real_rviz_with_gripper.launch.py \
  mode:=real host:=110.120.1.40 port:=12345 model:=e0509 robot_ip:=110.120.1.40
```

- 팔: `/dsr01/joint_states`
- 그리퍼 pulse: `/gripper/state` → rad 변환 → `/dsr01/joint_states_rviz`
- URDF: `isaac_gripper_sim2real/urdf/e0509_with_gripper.urdf` (그리퍼 Z -90° 반영)

### B) 그리퍼 **수동** 명령 (RViz launch 켠 상태)

```bash
ros2 service call /gripper/set_position \
  rh_p12_rna_controller_interfaces/srv/SetPosition \
  "{position: 420, current: 300, timeout_sec: 5.0}"
```

### C) Isaac sim2real — 그리퍼 **자동** 명령 (T4)

```bash
# T2: Isaac Play + /isaac/joint_states
ros2 launch isaac_gripper_sim2real sim2real_gripper.launch.py \
  robot_ip:=110.120.1.40 enable_command:=true
```

명령 경로: `/isaac/joint_states` → `gripper_bridge` → `/gripper/cmd_direct` → `gripper_service_node`

---

## 토픽 / 서비스 요약

| 이름 | 타입 | 방향 | 용도 |
|------|------|------|------|
| `/gripper/state` | `sensor_msgs/JointState` | pub | pulse 피드백 (`gripper_joint`) |
| `/gripper/cmd_direct` | `std_msgs/String` | sub | `"custom 420 300"` (sim2real, yaml에서 enable 필요) |
| `/gripper/set_position` | `SetPosition` srv | srv | 수동 위치 명령 |
| `/dsr01/joint_states_rviz` | `JointState` | pub | RViz용 팔+그리퍼 병합 |

---

## 설정 파일 (isaac_gripper_sim2real)

| 파일 | 용도 |
|------|------|
| `config/rviz_gripper_service.yaml` | RViz용 gripper TCP **상태 읽기** (`direct_cmd: false`) |
| `config/rviz_gripper.yaml` | joint_states 병합 (pulse→rad) |
| `config/sim2real_gripper.yaml` | Isaac sim2real **명령** (`direct_cmd: true`) |
| `urdf/e0509_with_gripper.urdf` | RViz robot_description |

---

## 확인

```bash
ros2 topic echo /gripper/state --once
ros2 topic echo /dsr01/joint_states_rviz --once
ros2 service call /gripper/get_state rh_p12_rna_controller_interfaces/srv/GetState "{}"
```

정상 로그 (`gripper_service_node`):

```
TCP(9105) 접속 성공!
TCP state 수신 시작
```

---

## 문제 해결

| 증상 | 확인 |
|------|------|
| 그리퍼 RViz 안 움직임 | `/gripper/state` pulse 변하는지 |
| `DRL 서비스 연결 실패` | bringup 먼저, overlay `gripper_node.py` 적용·재빌드 |
| 그리퍼/팔 90° 틀어짐 | `install_overlay.sh` URDF 적용 + launch 재시작 |
| 명령 안 됨 (RViz launch만) | 정상 — `set_position` srv 또는 `sim2real_gripper.launch.py` 추가 |

---

## upstream (이 번들 제외)

- [Doosan Robotics ROS 2](https://github.com/DoosanRobotics/doosan-robot2) — `doosan-robot2`
- RH-P12-RNA controller — 팀에서 쓰는 공식 `rh_p12_rna_controller` repo URL

이 번들은 위 repo **위에 얹는 add-on** 입니다.
