# rviz-gripper-e0509

**실로봇 팔 + 그리퍼**를 RViz에 띄우고, **real → RViz** 로 joint 값을 연동합니다.

- 팔: `/dsr01/joint_states` (실로봇 rad)
- 그리퍼: `/gripper/state` (pulse) → rad 변환
- URDF: `e0509_with_gripper.urdf` (팔 6축 + RH-P12)

Repo: [jasper104615-collab/rviz-gripper-e0509-](https://github.com/jasper104615-collab/rviz-gripper-e0509-)

---

## 사전 준비 (공식 GitHub clone)

| Repo | 용도 |
|------|------|
| [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) | bringup, mesh, controller |
| `rh_p12_rna_controller` | 그리퍼 TCP / `/gripper/state` |

---

## 이 repo 구조

```
rviz-gripper-e0509/
├── README.md
├── scripts/install_overlay.sh
├── overlay/
│   ├── doosan/.../e0509_with_gripper.urdf   # 팔+그리퍼, Z-90° 장착
│   └── rh_p12/.../gripper_node.py
└── ros2_ws/src/
    ├── isaac_gripper_sim2real/    ← RViz real-to-sim 연동 패키지
    └── rh_p12_rna_controller_interfaces/
```

---

## 설치

```bash
git clone https://github.com/jasper104615-collab/rviz-gripper-e0509-.git
cd rviz-gripper-e0509-

chmod +x scripts/install_overlay.sh
DOOSAN_SRC=~/doosan-robot2/src/doosan-robot2 \
RH12_SRC=~/rh_p12_rna_controller \
  ./scripts/install_overlay.sh

mkdir -p ~/gripper_rviz_ws/src
cp -r ros2_ws/src/* ~/gripper_rviz_ws/src/
ln -sf ~/rh_p12_rna_controller ~/gripper_rviz_ws/src/

source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash

cd ~/doosan-robot2 && colcon build --packages-select dsr_description2
cd ~/gripper_rviz_ws
colcon build --packages-select \
  rh_p12_rna_controller_interfaces rh_p12_rna_controller isaac_gripper_sim2real

source ~/doosan-robot2/install/setup.bash
source ~/gripper_rviz_ws/install/setup.bash
```

---

## 실행 (한 launch)

```bash
ros2 launch isaac_gripper_sim2real real_rviz_with_gripper.launch.py \
  mode:=real \
  host:=110.120.1.40 \
  port:=12345 \
  model:=e0509 \
  robot_ip:=110.120.1.40
```

### Real → RViz 데이터 흐름

```
[팔] joint_state_broadcaster → /dsr01/joint_states (joint_1~6)
[그리퍼] gripper_service_node → /gripper/state (pulse)
              ↓ rviz_joint_state_merger (pulse→rad)
         /dsr01/joint_states_rviz (joint_1~6 + rh_r1,l1,r2,l2)
              ↓ robot_state_publisher (e0509_with_gripper.urdf)
                    RViz
```

---

## 확인

```bash
ros2 topic echo /dsr01/joint_states
ros2 topic echo /gripper/state
ros2 topic echo /dsr01/joint_states_rviz
```

---

## 그리퍼 수동 명령 (선택)

RViz launch는 **상태만** 읽습니다. 움직이려면:

```bash
ros2 service call /gripper/set_position \
  rh_p12_rna_controller_interfaces/srv/SetPosition \
  "{position: 420, current: 300, timeout_sec: 5.0}"
```

Isaac sim2real 그리퍼 명령은 별도: `sim2real_gripper.launch.py` (sim2real_deploy 참고)

---

## pulse ↔ rad (그리퍼)

| pulse | rad (rh_r1) |
|-------|-------------|
| 0 | 0.0 (열림) |
| 700 | ~1.101 (닫힘) |

설정: `config/rviz_gripper.yaml`, `config/rviz_gripper_service.yaml`
