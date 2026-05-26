# rviz-gripper-e0509

E0509 실로봇 **그리퍼만** RViz에 띄우고, **실제 pulse 피드백**을 joint angle로 보여줍니다.

Repo: [jasper104615-collab/rviz-gripper-e0509-](https://github.com/jasper104615-collab/rviz-gripper-e0509-)

---

## 필요한 공식 repo (다른 PC에서 clone)

| Repo | 용도 |
|------|------|
| [doosan-robot2](https://github.com/DoosanRobotics/doosan-robot2) | `dsr_description2` mesh, **DRL 서비스** (bringup) |
| `rh_p12_rna_controller` | `gripper_service_node` (공식 RH-P12) |

---

## 이 repo에 있는 것

```
rviz-gripper-e0509/
├── README.md
├── scripts/install_overlay.sh      # gripper_node.py 패치
├── overlay/rh_p12/.../gripper_node.py
└── ros2_ws/src/
    ├── rviz_gripper_e0509/         ← RViz 그리퍼 전용 패키지
    └── rh_p12_rna_controller_interfaces/
```

**없는 것:** 팔 URDF, Isaac sim2real, arm joint_states 병합

---

## 설치 (1회)

```bash
git clone https://github.com/jasper104615-collab/rviz-gripper-e0509-.git
cd rviz-gripper-e0509-

# RH-P12 gripper_node overlay
chmod +x scripts/install_overlay.sh
RH12_SRC=~/rh_p12_rna_controller ./scripts/install_overlay.sh

# ROS ws
mkdir -p ~/gripper_rviz_ws/src
cp -r ros2_ws/src/* ~/gripper_rviz_ws/src/
ln -s ~/rh_p12_rna_controller ~/gripper_rviz_ws/src/   # 또는 copy

source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash

cd ~/gripper_rviz_ws
colcon build --packages-select rh_p12_rna_controller_interfaces rh_p12_rna_controller rviz_gripper_e0509
colcon build --packages-select dsr_description2   # mesh (doosan ws)
source ~/gripper_rviz_ws/install/setup.bash
```

---

## 실행

### T1 — Doosan bringup (DRL/TCP용, RViz 없어도 됨)

`gripper_service`가 `/dsr01/drl/drl_start` 를 씁니다.

```bash
source ~/doosan-robot2/install/setup.bash
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py \
  mode:=real host:=110.120.1.40 port:=12345 model:=e0509
```

(RViz 창은 닫아도 됨. controller만 살아 있으면 OK.)

### T2 — 그리퍼만 RViz

```bash
source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash
source ~/gripper_rviz_ws/install/setup.bash

ros2 launch rviz_gripper_e0509 gripper_rviz_real.launch.py robot_ip:=110.120.1.40
```

---

## 데이터 흐름

```
실 RH-P12 (Modbus/TCP)
  → gripper_service_node → /gripper/state (pulse)
  → gripper_joint_state  → /joint_states (rh_r1,l1,r2,l2 rad)
  → robot_state_publisher + RViz (그리퍼 URDF만)
```

---

## 확인

```bash
ros2 topic echo /gripper/state
ros2 topic echo /joint_states
```

RViz Fixed Frame: **`world`**

---

## 그리퍼 수동 명령 (선택)

```bash
ros2 service call /gripper/set_position \
  rh_p12_rna_controller_interfaces/srv/SetPosition \
  "{position: 420, current: 300, timeout_sec: 5.0}"
```

---

## pulse ↔ angle

| pulse | 의미 | rh_r1 rad |
|-------|------|-----------|
| 0 | 열림 | 0.0 |
| 700 | 닫힘 | ~1.101 |

설정: `config/gripper_rviz_real.yaml`
