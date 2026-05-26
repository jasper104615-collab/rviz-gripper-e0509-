# Overlay (공식 repo 위에 덮어쓰는 파일)

| 파일 | 대상 (공식 clone 경로) | 변경 내용 |
|------|----------------------|-----------|
| `doosan/dsr_description2/urdf/e0509_with_gripper.urdf` | `doosan-robot2/.../dsr_description2/urdf/` | RH-P12 장착 Z -90° (`link_6_rh_p12_rn_base_fixed`) |
| `rh_p12/.../gripper_node.py` | `rh_p12_rna_controller/.../gripper_node.py` | DRL 서비스 준비 전 재시도, TCP state stream |

적용:

```bash
./scripts/install_overlay.sh
# 또는 경로 지정
DOOSAN_SRC=~/doosan-robot2/src/doosan-robot2 \
RH12_SRC=~/rh_p12_rna_controller \
  ./scripts/install_overlay.sh
```
