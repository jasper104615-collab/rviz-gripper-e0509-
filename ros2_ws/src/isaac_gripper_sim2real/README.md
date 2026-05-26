# isaac_gripper_sim2real

Isaac 그리퍼(`rh_r1`) → 실 RH-P12. **팔 브리지(T3)와 반드시 동시에** 켭니다.

> Clone: [isaacsimgrippersim2realv1](https://github.com/jasper104615-collab/isaacsimgrippersim2realv1)  
> Deploy repo: `docs/ARCHITECTURE_AND_WORKFLOW.md`

---

## 1. 시스템 구조도 (Architecture)

PLC Modbus 노드 구조와 같은 레이어로 표현한 **팔+그리퍼 동시 sim2real** 전체입니다.

```mermaid
flowchart LR
  subgraph T2["Isaac Sim (T2)"]
    IS["sim_to_real_0519.usda\nROS2PublishJointState"]
  end

  subgraph ROS["ROS2 토픽 /isaac/joint_states"]
    JS["sensor_msgs/JointState\njoint_1~6 + rh_r1,l1,r2,l2 [rad]"]
  end

  subgraph T3_ARM["T3 — 팔 sim2real"]
    SJS["sub_joint_state\n(isaac_sim_2_real)"]
    DRFL["move_joint / servoj_rt\n(DRFL, drl_start X)"]
  end

  subgraph T4_GRI["T4 — 그리퍼 sim2real (이 패키지)"]
    GB["gripper_bridge\n(isaac_gripper_sim2real)"]
    GS["gripper_service_node\n(rh_p12_rna_controller)"]
    TCP["TCP JSON Modbus\n:9105"]
  end

  subgraph HW["Real Hardware"]
    E0509["Doosan E0509\n팔 6축"]
    RHP12["RH-P12-RN-A\nModbus RTU\nslave ID=1"]
  end

  IS -->|"JointState 발행"| JS
  JS -->|"joint_1~6 구독\nrad→deg"| SJS
  JS -->|"rh_r1 구독\nrad→pulse"| GB
  SJS --> DRFL --> E0509
  GB -->|"/gripper/cmd_direct\nString custom PULSE CUR"| GS
  GS --> TCP --> RHP12
  GS -->|"/gripper/state\nJointState publish"| MON["T5 모니터"]
```

### 레이어별 상세 (그리퍼 T4)

| 레이어 | 구성요소 | ROS 인터페이스 | 타입 / 단위 |
|--------|----------|----------------|-------------|
| **입력** | Isaac Sim | `/isaac/joint_states` | `JointState`, rad |
| **브리지** | `gripper_bridge` | 구독: `/isaac/joint_states` | `rh_r1` 추출 |
| **명령** | `gripper_bridge` | 발행: `/gripper/cmd_direct` | `String` `"custom 420 300"` |
| **Owner** | `gripper_service_node` | 구독: `/gripper/cmd_direct` | 큐 → TCP Modbus |
| **통신** | DRL TCP Server | `<ROBOT_IP>:9105` | JSON frame + Modbus FC06/FC16 |
| **하드웨어** | Tool Flange Serial | RS-485 Modbus RTU | RH-P12, slave ID **1** |
| **피드백** | `gripper_service_node` | 발행: `/gripper/state` | pulse(실측), effort(current) |

### ASCII 구조 (PLC 다이어그램 스타일)

```
┌─────────────────────────────────────────────────────────────────────────┐
│  Isaac Sim (T2)                                                         │
│  sim_to_real_0519.usda  ──►  /isaac/joint_states  (JointState, rad)    │
└───────────────────────────────┬─────────────────────────────────────────┘
                                │
           ┌────────────────────┴────────────────────┐
           ▼                                         ▼
┌──────────────────────┐                 ┌──────────────────────────────┐
│ T3 sub_joint_state   │                 │ T4 gripper_bridge            │
│ joint_1~6 구독       │                 │ rh_r1 구독                   │
│ rad → deg            │                 │ rad → pulse (100~420)        │
└──────────┬───────────┘                 └──────────────┬───────────────┘
           │                                            │
           ▼                                            ▼
┌──────────────────────┐                 ┌──────────────────────────────┐
│ move_joint           │                 │ /gripper/cmd_direct          │
│ servoj_rt_stream     │                 │ String "custom PULSE CUR"    │
│ (DRFL, drl_start X)  │                 └──────────────┬───────────────┘
└──────────┬───────────┘                                │
           │                                            ▼
           │                              ┌──────────────────────────────┐
           │                              │ gripper_service_node         │
           │                              │ command_transport:=tcp       │
           │                              │ direct_cmd_topic_enabled     │
           │                              └──────────────┬───────────────┘
           │                                            │
           ▼                                            ▼
┌──────────────────────┐                 ┌──────────────────────────────┐
│ Doosan E0509 팔      │                 │ TCP :9105 → Modbus RTU       │
│ (T1 bringup)         │                 │ reg 282 Goal Position        │
└──────────────────────┘                 └──────────────┬───────────────┘
                                                        ▼
                                           ┌──────────────────────────────┐
                                           │ RH-P12-RN-A (slave ID=1)     │
                                           └──────────────────────────────┘
```

---

## 2. 시퀀스 다이어그램 (Workflow)

PLC `read P020` / `write_coils` 흐름과 같은 형식입니다.

```mermaid
sequenceDiagram
  participant Isaac as Isaac Sim (T2)
  participant T3 as sub_joint_state (T3)
  participant T4 as gripper_bridge (T4)
  participant GS as gripper_service_node
  participant TCP as DRL TCP Server :9105
  participant HW as RH-P12 Modbus

  Note over GS,HW: 부팅 시 connect() — drl_start 1회 (TCP 서버 주입)
  GS->>TCP: DRL inject TCP server (async)
  GS->>TCP: socket connect
  TCP-->>GS: connected

  loop [매 max_publish_hz, 기본 10Hz]
    Isaac->>T3: /isaac/joint_states (joint_1~6)
    Isaac->>T4: /isaac/joint_states (rh_r1)
    T3->>T3: rad → deg, filter
    T3->>T3: move_joint / servoj_rt (DRFL)
    Note over T3: drl_start 사용 안 함
    T4->>T4: rh_r1 rad → pulse
    T4->>GS: /gripper/cmd_direct "custom 420 300"
    GS->>TCP: FC06 Goal Current + FC16 Goal Position
    TCP->>HW: Modbus RTU write
    HW-->>TCP: ack
    loop [매 state_hz, 기본 20Hz]
      GS->>TCP: FC03/FC04 read present
      TCP->>HW: Modbus RTU read
      HW-->>TCP: position, current
      GS->>GS: /gripper/state publish
    end
  end
```

### 시퀀스 요약

| 단계 | 주체 | 동작 | drl_start |
|------|------|------|-----------|
| **부팅** | `gripper_service_node` | TCP 서버 DRL 주입 + socket connect | **1회** |
| **루프 (팔)** | `sub_joint_state` | `/isaac/joint_states` → move_joint | 없음 |
| **루프 (그리퍼)** | `gripper_bridge` | `rh_r1` → `/gripper/cmd_direct` | 없음 |
| **쓰기** | `gripper_service_node` | TCP → Modbus FC06/FC16 | 없음 |
| **읽기** | `gripper_service_node` | TCP → Modbus read → `/gripper/state` | 없음 |

→ **T3 + T4 동시 실행 가능** (`command_transport:=drl` 이면 쓰기마다 drl_start → **동시 불가**)

---

## 3. 이 패키지 역할 (T4)

| | T3 `sub_joint_state` | **T4 `gripper_bridge`** |
|--|----------------------|-------------------------|
| Isaac joint | `joint_1`~`6` | `rh_r1` |
| 실로봇 출력 | move_joint / servoj_rt | `/gripper/cmd_direct` |
| drl_start | 없음 | **명령마다 없음** (tcp) |

**T3 + T4 = 팔 + 그리퍼 동시 sim2real**

---

## 4. 동시 실행 명령 (T1~T5)

### 공통 source

```bash
source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash
source ~/sim2real_deploy/ros2_ws/install/setup.bash
```

| 터미널 | 명령 |
|--------|------|
| **T1** | `ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=<ROBOT_IP> model:=e0509` |
| **T2** | `isaac_assets/my_robot/sim_to_real_0519.usda` → Play |
| **T3** | 아래 팔 명령 |
| **T4** | 아래 그리퍼 launch (**T3와 동시**) |
| **T5** | `ros2 topic hz /isaac/joint_states` 등 확인 |

**T3 — 팔**

```bash
ros2 run isaac_sim_2_real sub_joint_state --ros-args \
  -p motion_mode:=move_joint \
  -p enable_motion:=true \
  -p input_topic:=/isaac/joint_states \
  -p max_publish_hz:=10.0 \
  -p move_joint_sync_type:=1
```

**T4 — 그리퍼**

```bash
ros2 launch isaac_gripper_sim2real sim2real_gripper.launch.py \
  robot_ip:=<ROBOT_IP> \
  enable_command:=true
```

**T5 — 확인**

```bash
ros2 topic hz /isaac/joint_states
ros2 topic echo /gripper/cmd_direct --once
ros2 topic echo /gripper/state --field position --once
```

---

## 5. DRL — 동시 가능 조건

| T3 + T4 조합 | 팔+그리퍼 동시 |
|--------------|----------------|
| move_joint + **tcp** + rh_p12_direct | **✅** |
| servoj_rt + **tcp** + rh_p12_direct | **✅** |
| * + 그리퍼 **drl** | **❌** |

---

## 6. 출력 모드

| `output_mode` | 출력 | T3와 동시 |
|---------------|------|-----------|
| **`rh_p12_direct`** (기본) | `/gripper/cmd_direct` | ✅ |
| `rh_p12_service` | `/gripper/set_position` srv | ✅ (느림) |
| `e0509_topic` | `/dsr01/gripper/position_cmd` Int32 | ✅ (e0509 bringup) |

---

## 7. 변환 (rh_r1 → pulse)

```
t = clamp(rh_r1_rad / 1.101, 0, 1)
pulse = round(100 + t * 320)    # 열림 100, 닫힘 420
```

---

## 8. 빌드

```bash
cd ~/sim2real_deploy/ros2_ws
colcon build --packages-select \
  rh_p12_rna_controller_interfaces rh_p12_rna_controller isaac_gripper_sim2real
source install/setup.bash
```

설정: `config/sim2real_gripper.yaml`

---

## 9. RViz — 실로봇 + 그리퍼 시각화 (Real → RViz)

URDF: `urdf/e0509_with_gripper.urdf` (팔 6축 + RH-P12, link_6→gripper Z -90°)

### 9.1 실행

```bash
source /opt/ros/humble/setup.bash
source ~/doosan-robot2/install/setup.bash          # 필수 (dsr_bringup2)
source ~/sim2real_deploy/ros2_ws/install/setup.bash

ros2 launch isaac_gripper_sim2real real_rviz_with_gripper.launch.py \
  mode:=real host:=110.120.1.40 port:=12345 model:=e0509 robot_ip:=110.120.1.40
```

> `ModuleNotFoundError: dsr_bringup2` → Doosan ws `source` 누락

---

### 9.2 노드 구조

`real_rviz_with_gripper.launch.py` 가 띄우는 노드:

```mermaid
flowchart TB
  subgraph Doosan["Doosan (dsr_bringup2 / ros2_control)"]
    CN["ros2_control_node\nnamespace: /dsr01"]
    JSB["joint_state_broadcaster"]
    DSR["dsr_controller2"]
  end

  subgraph Gripper["RH-P12 (rh_p12_rna_controller)"]
    GS["gripper_service_node"]
  end

  subgraph ThisPkg["isaac_gripper_sim2real"]
    MER["rviz_joint_state_merger"]
  end

  subgraph Viz["Visualization"]
    RSP["robot_state_publisher\n(팔+그리퍼 URDF)"]
    RV["rviz2"]
  end

  HW_ARM["실 E0509 컨트롤러"] --> DSR
  DSR --> JSB
  JSB -->|"/dsr01/joint_states"| MER

  HW_GRI["실 RH-P12 Modbus"] --> GS
  GS -->|"/gripper/state"| MER

  MER -->|"/dsr01/joint_states_rviz"| RSP
  RSP -->|"/tf /tf_static"| RV
```

| 노드 | 패키지 | 역할 |
|------|--------|------|
| `ros2_control_node` | controller_manager | 실로봇 연결, joint state 읽기 |
| `joint_state_broadcaster` | controller_manager | `/dsr01/joint_states` 발행 |
| `dsr_controller2` | dsr_controller2 | Doosan 실기 연동 |
| `gripper_service_node` | rh_p12_rna_controller | TCP→Modbus, `/gripper/state` 발행 |
| `rviz_joint_state_merger` | **isaac_gripper_sim2real** | 팔+그리퍼 joint_states 병합 |
| `robot_state_publisher` | robot_state_publisher | URDF→TF (`joint_states_rviz` 구독) |
| `rviz2` | rviz2 | 3D 시각화 |

**기동 순서:** `control_node` → spawner → `dsr_controller2` 완료 후 → `gripper_service` + `merger` + `rviz2`

---

### 9.3 Topic 구조

```mermaid
flowchart LR
  subgraph Input["실로봇 입력"]
    T1["/dsr01/joint_states\nJointState"]
    T2["/gripper/state\nJointState"]
  end

  MER["rviz_joint_state_merger"]

  subgraph Output["RViz 입력"]
    T3["/dsr01/joint_states_rviz\nJointState"]
    TF["/tf /tf_static"]
  end

  T1 -->|"joint_1~6 [rad]"| MER
  T2 -->|"gripper_joint [pulse]"| MER
  MER -->|"joint_1~6 + rh_r1,l1,r2,l2 [rad]"| T3
  T3 --> RSP["robot_state_publisher"] --> TF
```

| Topic | 타입 | Pub | Sub | 내용 |
|-------|------|-----|-----|------|
| `/dsr01/joint_states` | `sensor_msgs/JointState` | joint_state_broadcaster | rviz_joint_state_merger | 팔 6축 **rad** |
| `/gripper/state` | `sensor_msgs/JointState` | gripper_service_node | rviz_joint_state_merger | `gripper_joint` **pulse** (0~700) |
| `/dsr01/joint_states_rviz` | `sensor_msgs/JointState` | rviz_joint_state_merger | robot_state_publisher | 병합 결과 **rad** |
| `/tf` | `tf2_msgs/TFMessage` | robot_state_publisher | rviz2 | 링크 pose |
| `/tf_static` | `tf2_msgs/TFMessage` | robot_state_publisher | rviz2 | 고정 joint |

**Joint 이름 (`/dsr01/joint_states_rviz`):**

| 이름 | 출처 | 단위 |
|------|------|------|
| `joint_1` … `joint_6` | 실 팔 피드백 | rad |
| `rh_r1` | pulse→rad 변환 (마스터) | rad |
| `rh_l1`, `rh_r2`, `rh_l2` | rh_r1 과 동일 (mimic) | rad |

**그리퍼 변환 (`gripper_conversion.py`):**

```
t = (pulse - pulse_open) / (pulse_closed - pulse_open)   # 0~700 → 0~1
rh_r1_rad = open_rad + t * (closed_rad - open_rad)         # 0 ~ 1.101 rad
```

설정: `config/rviz_gripper.yaml` (`pulse_open=0`, `pulse_closed=700`)

---

### 9.4 Service / DRL (그리퍼 통신)

| 인터페이스 | 타입 | 용도 |
|------------|------|------|
| `/dsr01/drl/drl_start` | `dsr_msgs2/srv/DrlStart` | gripper TCP 서버 DRL 주입 (부팅 1회) |
| `/gripper/get_state` | `rh_p12_rna_controller_interfaces/srv/GetState` | merger 폴링 백업 |
| `/gripper/set_position` | `SetPosition` srv | 수동 그리퍼 명령 (RViz launch는 기본 **미사용**) |

RViz launch는 **명령 안 보냄** (`direct_cmd_topic_enabled: false`). 상태 읽기만.

---

### 9.5 시퀀스 (Real → RViz)

```mermaid
sequenceDiagram
  participant HW as Real Robot
  participant DSR as dsr_controller2
  participant JSB as joint_state_broadcaster
  participant GS as gripper_service_node
  participant MER as rviz_joint_state_merger
  participant RSP as robot_state_publisher
  participant RV as rviz2

  Note over DSR,GS: dsr_controller2 기동 후 gripper_service TCP connect

  loop [state_hz ~10Hz]
    HW->>DSR: arm joint feedback
    DSR->>JSB: read joints
    JSB->>MER: /dsr01/joint_states
    HW->>GS: Modbus present position
    GS->>MER: /gripper/state (pulse)
    MER->>MER: pulse → rh_r1 rad, mimic joints
    MER->>RSP: /dsr01/joint_states_rviz
    RSP->>RV: /tf
  end
```

---

### 9.6 URDF / TF

- **robot_description:** `e0509_with_gripper.urdf` (팔 mesh + RH-P12 mesh)
- **Fixed Frame (RViz):** `base_link` 또는 `world`
- **그리퍼 장착:** `link_6` → `rh_p12_rn_base` fixed joint, `rpy="0 0 -1.5708"`

mesh 경로: `package://dsr_description2/meshes/...` → Doosan `dsr_description2` 설치 필요

---

### 9.7 확인 명령

```bash
ros2 topic echo /dsr01/joint_states --once
ros2 topic echo /gripper/state --once
ros2 topic echo /dsr01/joint_states_rviz --once
ros2 run tf2_tools view_frames
```

---

### 9.8 overlay launch (bringup 분리 시)

```bash
# T1: dsr bringup
ros2 launch dsr_bringup2 dsr_bringup2_rviz.launch.py mode:=real host:=110.120.1.40 port:=12345 model:=e0509

# T2: 그리퍼 피드백 + 병합만 (RSP/RViz는 T1 것을 gripper URDF + joint_states_rviz remap으로 교체)
ros2 launch isaac_gripper_sim2real rviz_gripper_overlay.launch.py robot_ip:=110.120.1.40
```
