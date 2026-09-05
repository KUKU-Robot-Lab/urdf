# OpenArm End-Effector URDF Workspace

This workspace is for composing OpenArm arms with multiple end-effectors and generating RL-only URDFs with a stable naming and action-order interface.

The important separation is:

- Source/vendor descriptions keep their original names for ROS2, drivers, and hardware bringup.
- RL URDFs are generated artifacts with canonical names for Isaac/RL training.
- The RL agent should use the generated RL URDF plus its manifest, not the hardware/source URDF directly.

## Current Structure

```text
/home/user/rl_ws/urdf
├── assemblies/                 # Local composition xacro files
├── config/                     # Local controller configs
├── docs/                       # Plans and notes
├── eef/                        # Local end-effector xacro wrappers
├── generated/
│   ├── rl/                     # RL-only canonical URDFs and manifests
│   └── source/                 # Stable generated/source URDF inputs
├── launch/                     # Local ROS2 launch files
├── previews/                   # Visual/debug URDF previews
├── scripts/                    # Shell helpers
├── tools/                      # Generation/maintenance tools
├── vendor/                     # Imported/reference robot packages
│   ├── openarm_description/    # OpenArm ROS description package
│   ├── delto_m_ros2/           # Tesollo/Delto ROS2 packages
│   ├── RH56F1/                 # RH56F1 source hand packages and assets
│   └── openarm_real/           # OpenArm real hardware packages
├── build/                      # colcon build output
├── install/                    # colcon install output
└── log/                        # colcon log output
```

### Why Vendor Packages Are Under `vendor/`

Imported/reference packages are grouped under `vendor/` so local composition files and generated training assets are not mixed with upstream source trees. Launch files and helper scripts use explicit `vendor/...` filesystem paths where needed. Xacro files still use ROS package names such as `$(find openarm_description)`, so source `/home/user/rl_ws/urdf/install/setup.bash` or rebuild the workspace before processing those xacros.

## Directory Details

### `assemblies/`

Local robot compositions live here.

- `openarm_modular_dual.xacro` - OpenArm dual-arm body with left stock gripper and right Tesollo hand.
- `openarm_modular_dual_tesollo.xacro` - OpenArm dual-arm body with Tesollo hands on both sides.
- `openarm_left_gripper_bimanual_real.xacro` - Real-hardware OpenArm control description with left stock gripper and right hand controlled separately.

These are composition sources. They are not the preferred RL training inputs.

### `eef/`

Local wrapper xacros for end-effectors.

- `tesollo_left_wrapper.xacro`
- `tesollo_right_wrapper.xacro`
- `gripper_left.xacro`

Wrappers add stable helper frames around vendor end-effector descriptions. They are useful for source composition, but final RL naming is handled by `tools/generate_rl_urdf.py`.

### `generated/source/`

Stable source/generated URDFs used as inputs to the RL canonical generator.

- `openarm_tesollo_bi.urdf` (DG-5F both hands) -> `openarm_dg5f-m_bi`
- `openarm_tesollo_bi_s.urdf` (DG-5F-S both hands) -> `openarm_dg5f-s_bi`
- `openarm_bi_rh56f1.urdf` (RH56F1 both hands) -> `openarm_rh56f1_bi`
- `openarm_gripper_bi.urdf` (stock gripper both arms, `tools/gen_gripper_bi_source.sh`) -> `openarm_gripper_bi`
- `openarm_tesollo_sensor.urdf`, `openarm_modular_dual.urdf`, `openarm_bimanual_no_mount.urdf` (legacy, not generated from any more)

### `generated/rl/`

RL-only canonical outputs (2026-09-05 line-up, one per end-effector, both arms):

| asset | hands | hand drive | vendor gains ported | fabric variants |
|---|---|---|---|---|
| `openarm_dg5f-m_bi_rl` | Tesollo DG-5F x2 (link masses scaled x1.0463 to the measured **1.763 kg** per hand, vendor 1.685) | direct (20/20 per hand) | arm `control_gains.yaml` + `dg5f_driver` PID (p 1.5 / d 0) | `openarm_dg5f-m_bi_right`, `_left` |
| `openarm_dg5f-s_bi_rl` | Tesollo DG-5F-S x2 | direct | same as above (same driver stack) | `openarm_dg5f-s_bi_right`, `_left` |
| `openarm_rh56f1_bi_rl` | Inspire RH56F1 x2 | PhysX mimic (6 driven / 12) | arm only - the vendor stack (`vendor/inspire_ws`, RS-485 registers angleSet / speedSet / forceSet) exposes no PD gains -> hand keeps fallback 100/1 | `openarm_rh56f1_bi` |
| `openarm_gripper_bi_rl` | stock OpenArm gripper x2 | PhysX mimic (jaw 2 follows jaw 1) | arm + `GRIPPER_KP/KD` 5.0/0.1 (`openarm_real` hardware interface, motor-unit values carried verbatim) | `openarm_gripper_bi_right`, `_left` |

Common to all four: head_v1 on `body_link` at z=0.750 (B4 measurement pending) — the head
links/joints/masses come from the URDF, but their geometry is the Fusion USD itself:
`tools/build_usd.py` grafts `hdgp/assets/simulation_setting/head_v1/usd/head_v1.usda` under
each head link (`<root>/head_{base,mid,camera}/head_v1/...`, materials and the D435i optical
frames `head_camera/head_v1/camera_link/{color,depth,left_ir,right_ir,ir_projector}_frame`
included, referenced physics stripped; ⚠ the RGB lens is the `ir_projector_frame` aperture —
hand-eye 2026-09-05, see `HEAD_RGB_LENS_FRAME`) — the
vendor body's head housing / shoulder shell removed and the 60x60 pillar cut at the
head base plate underside (z=0.730, see `tools/crop_body_plate.py`), every massless
frame carrying a token 1e-5 kg in the URDF itself, colliders = convex hull on OpenArm
links (body, arms, head, stock gripper) and convex decomposition on the dexterous hand
links (manifest `collision_approximation`), origin at the mount plate top.

Each `<asset>.urdf` + `<asset>_manifest.yaml` here is the input of `tools/build_usd.py`;
`generated/rl/<asset>/` is the built USD bundle (mirrored to `hdgp/assets/robot/<asset>/`).

The pre-09.05 files (`openarm_tesollo_sensor_rl`, `openarm_tesollo_bi_rl`,
`openarm_tesollo_bi_s_rl`, `openarm_bi_rh56f1_rl`) are frozen on disk for the sim2real
scripts that still read them; the generator, the USD builder and the fabric generator no
longer know them.

## ⚠ URDF 무질량 프레임 — PhysX 유령 1 kg (2026-09-02 실측)

**새 자산을 만들 때 반드시 확인할 것.**

URDF 에서 `<inertial>` 이 없는 링크는 "질량 없는 좌표 프레임"이다. 그런데 USD 변환은
`merge_fixed_joints=False` 로 한다(fingertip 접촉센서와 `palm_body` 이름을 보존해야 하고,
병합하면 바디 20개가 사라져 env 가 부팅에서 죽는다 — `tools/build_usd.py` 주석 참조).
그래서 그 프레임들도 **각각 rigid body 가 되고, 질량이 없으면 PhysX 가 기본값 1.0 kg 을
붙인다.**

실제 피해 (`openarm_tesollo_sensor_rl`):

| 유령 링크 | 런타임 질량 |
|---|---|
| `l_hl_gripper_tcp` | 1.0 kg |
| `r_hl_mount` · `r_hl_palm_alias` · `r_hl_palm_ee` | 각 1.0 kg |

손끝·손바닥이라 중력 모멘트가 통째로 바뀌어, 같은 게인(70/70/70/60/10/10/10)에서
**sim 좌팔 정적 처짐 j7 11.07° vs 실기 4.2°** (2.6배). sim2real 이 몇 주간 어긋났다.
양팔 tesollo 자산(`*_bi_rl`, `*_bi_s_rl`)은 손마다 3개씩 **총 6 kg** 이었다.

**세 가지 함정**

1. **USD 에 질량 0 을 적어도 소용없다.** 이미 `0.0` 이 적혀 있었는데 PhysX 가 강체의
   0 질량을 거부하고 1.0 kg 으로 대체했다. → **아주 작은 양수**(1e-4 kg)를 써야 한다.
2. **USD 값만 보면 속는다.** 반드시 런타임에서 확인할 것:
   `robot.root_physx_view.get_masses()` (러너 `mass` 명령이 URDF 와 나란히 찍는다).
3. **바디를 지우거나 merge 하지 말 것.** `body_names` 순서가 바뀌어 접촉센서·fabric
   바디맵·충돌필터가 조용히 어긋난다.

**도구**

```bash
# 2026-09-05 부터 생성기가 URDF 자체에 토큰 관성(1e-5 kg / 1e-7 kg·m²)을 쓴다 — fabric
# URDF 의 헬퍼 프레임도 같은 1e-5 (PD 모델에서 질량 0 은 발산한다, 사용자 지시).
# ★벤더 URDF 에 **질량 0 으로 적힌** 링크도 같은 취급이다 — RH56F1 은 sensor/tip 20개가
#   mass 0 + 영 텐서로 온다(PhysX 는 이것도 1 kg 으로 바꾼다 = 손끝 20 kg). 생성기가
#   1e-5 로 올리고, build_usd 의 shrink_massless_frames 도 mass<=0 을 잡는다.
# (generate_rl_urdf.give_massless_frames_token_mass) — USD 임포터가 그대로 physics
# 레이어에 적고, 아래 shrink_massless_frames 는 뒷단 안전망으로만 남는다.
IsaacLab/isaaclab.sh -p tools/build_usd.py <asset> --sync-hdgp
# (2026-09-05 이전의 얇은 변형 `_hull/_lgrip/_armhull` 은 폐기 — 링크별 콜라이더
#  정책이 캐노니컬 빌드에 들어갔다. manifest `collision_approximation` 참조.)

# 이미 만들어진 자산을 고칠 때 (재빌드는 임포터 드리프트 위험이 있어 비권장)
IsaacLab/isaaclab.sh -p tools/patch_ghost_masses.py \
    --usd  <hdgp/assets/robot/<dir>/<asset>.usd> \
    --urdf <urdf/generated/rl/<asset>.urdf> [--dry-run]
```

`verify_contract` 는 `<inertial>` 이 **있는** 링크만 검사하므로 정확히 반대 집합인
이 유령들을 못 잡는다 — 그래서 별도 처리가 필요하다.

### `previews/`

Scratch/debug URDF files for visual inspection.

- `link7_material_preview.urdf`
- `link7_mat3_component_preview.urdf`
- `openarm_bimanual_link7_parts_preview.urdf`

Do not use these as training inputs.

## CLI Usage

Run commands from anywhere unless noted.

### Generate all RL URDFs

```bash
# head 를 Fusion 내보내기(hdgp/assets/simulation_setting/head_v1)에서 갱신했을 때만:
python3 /home/user/rl_ws/urdf/tools/import_head_v1.py
# 양팔 스톡 그리퍼 소스(xacro, ROS humble 필요) — 벤더 xacro 가 바뀌었을 때만:
bash /home/user/rl_ws/urdf/tools/gen_gripper_bi_source.sh
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py          # URDF + manifest + 자기충돌 감사
/home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/build_usd.py --sync-hdgp   # USD 4종 -> hdgp/assets/robot/
python3 /home/user/rl_ws/urdf/tools/gen_fabric_urdfs.py --sync-hdgp        # Fabrics IK URDF 7종
```

Expected outputs:

```text
generated/rl/openarm_dg5f-m_bi_rl.urdf      + _manifest.yaml   + openarm_dg5f-m_bi_rl/  (USD)
generated/rl/openarm_dg5f-s_bi_rl.urdf      + _manifest.yaml   + openarm_dg5f-s_bi_rl/
generated/rl/openarm_rh56f1_bi_rl.urdf      + _manifest.yaml   + openarm_rh56f1_bi_rl/
generated/rl/openarm_gripper_bi_rl.urdf     + _manifest.yaml   + openarm_gripper_bi_rl/
generated/fabric/openarm_dg5f-m_bi_{right,left}/ openarm_dg5f-s_bi_{right,left}/
                 openarm_gripper_bi_{right,left}/ openarm_rh56f1_bi/
```

### Generate one RL URDF

```bash
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_dg5f-m_bi
```

Valid source names:

```text
openarm_dg5f-m_bi
openarm_dg5f-s_bi
openarm_rh56f1_bi
openarm_gripper_bi
```

### Validate the generator syntax

```bash
python3 -m py_compile /home/user/rl_ws/urdf/tools/generate_rl_urdf.py
```

### Inspect action order

```bash
sed -n '/^control_joint_order:/,/^kinematic_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_dg5f-m_bi_rl_manifest.yaml
```

### Inspect full kinematic order

```bash
sed -n '/^kinematic_joint_order:/,/^fixed_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_rh56f1_bi_rl_manifest.yaml
```

### Visualize the modular source xacro

```bash
/home/user/rl_ws/urdf/scripts/run_openarm_modular_dual.sh
```

The script uses:

```text
/home/user/rl_ws/urdf/launch/display_openarm_modular_dual.launch.py
```

If `install/setup.bash` is missing, the helper builds packages from:

```text
/home/user/rl_ws/urdf/vendor/openarm_description
/home/user/rl_ws/urdf/vendor/delto_m_ros2/dg_description
```

That launch file points to:

```text
/home/user/rl_ws/urdf/assemblies/openarm_modular_dual.xacro
```

### Real hardware bringup source path

The real-hardware launch path is:

```text
/home/user/rl_ws/urdf/launch/openarm_left_gripper_right_dg5_real.launch.py
```

It uses:

```text
/home/user/rl_ws/urdf/assemblies/openarm_left_gripper_bimanual_real.xacro
```

## RL Naming Scheme

The canonical RL schema uses compact side/type prefixes.

### Side Prefix

```text
r_ = right
l_ = left
```

### OpenArm Body and Arms

```text
body_root      # stage-level root link with no geometry
body_link      # OpenArm physical base link
body_j_base    # body_root/body fixed joint

r_al_0..7      # right arm links
l_al_0..7      # left arm links

r_aj_base      # fixed body -> right arm base joint
l_aj_base      # fixed body -> left arm base joint

r_aj_1..7      # right arm actuated joints
l_aj_1..7      # left arm actuated joints
```

### End-Effector Links and Joints

```text
r_hl_*         # right hand links
l_hl_*         # left hand links

r_hj_*         # right hand joints
l_hj_*         # left hand joints
```

Examples:

```text
r_hj_mount
r_hj_base
r_hj_palm
r_hj_palm_sensor
r_hj_thumb_1
r_hj_thumb_2
r_hj_thumb_sensor
r_hj_thumb_tip
r_hj_pinky_1
r_hj_pinky_tip
```

The generator preserves fixed joints for mount/base/palm/sensor/tip structure. These are important for real sensor interpretation and consistent observation frames even when they are not action-controlled.

## Action Order

The RL action order is defined by each manifest's `control_joint_order`, not by relying on parser-specific URDF ordering.

The order is always:

```text
right arm -> right hand movable joints -> left arm -> left hand movable joints
```

For Tesollo bimanual:

```text
r_aj_1..7
r_hj_thumb_1..4
r_hj_index_1..4
r_hj_middle_1..4
r_hj_ring_1..4
r_hj_pinky_1..4
l_aj_1..7
l_hj_thumb_1..4
l_hj_index_1..4
l_hj_middle_1..4
l_hj_ring_1..4
l_hj_pinky_1..4
```

For RH56F1 bimanual, mimic joints are excluded from `control_joint_order` and retained in `kinematic_joint_order`.
The stock-gripper asset works the same way: `r_hj_gripper_1` / `l_hj_gripper_1` are the
commandable jaws, `*_hj_gripper_2` mimics them (16 actions in total).

Current RH56F1 control order is:

```text
r_aj_1..7
r_hj_thumb_1
r_hj_thumb_2
r_hj_index_1
r_hj_middle_1
r_hj_ring_1
r_hj_pinky_1
l_aj_1..7
l_hj_thumb_1
l_hj_thumb_2
l_hj_index_1
l_hj_middle_1
l_hj_ring_1
l_hj_pinky_1
```

## Generation Principle

The generator follows a source-preserving pipeline.

```text
vendor/source URDF
    -> parse XML
    -> build source-to-canonical link map
    -> build source-to-canonical joint map
    -> rename URDF into RL-only canonical schema
    -> reorder top-level links/joints for readability
    -> validate uniqueness and parent/child references
    -> write RL URDF
    -> write manifest with action and kinematic order
```

### What the Generator Changes

- Robot name gets `_rl` suffix.
- OpenArm body, arm links, and arm joints are renamed to canonical names.
- Tesollo and RH56F1 end-effector links/joints are renamed to canonical hand names.
- Fixed base, palm, sensor, and tip joints are kept and renamed.
- Mimic joints remain in the URDF and kinematic order.
- Mimic joints are excluded from action control order.

### What the Generator Does Not Change

- It does not modify source/vendor URDFs.
- It does not change meshes, inertials, limits, origins, axes, mimic tags, or geometry.
- It does not make hardware/control URDFs use RL names.
- It does not infer a single action size across hands with different actuation models.

## Manifests

Each manifest has these sections.

```yaml
source_urdf: generated/source/openarm_tesollo_bi.urdf
generated_urdf: generated/rl/openarm_tesollo_bi_rl.urdf
control_joint_order:
  - r_aj_1
  - r_aj_2
kinematic_joint_order:
  - body_j_base
  - r_aj_base
fixed_joint_order:
  - body_j_base
source_to_canonical_joints:
  openarm_right_joint1: r_aj_1
source_to_canonical_links:
  openarm_right_link1: r_al_1
```

Use `control_joint_order` for action vector indexing. Use `kinematic_joint_order`, `fixed_joint_order`, and `source_to_canonical_*` for debugging, observation mapping, and sensor/frame interpretation.

## Recommended Training Inputs

Use one of:

```text
/home/user/rl_ws/hdgp/assets/robot/openarm_dg5f-m_bi_rl/openarm_dg5f-m_bi_rl.usd
/home/user/rl_ws/hdgp/assets/robot/openarm_dg5f-s_bi_rl/openarm_dg5f-s_bi_rl.usd
/home/user/rl_ws/hdgp/assets/robot/openarm_rh56f1_bi_rl/openarm_rh56f1_bi_rl.usd
/home/user/rl_ws/hdgp/assets/robot/openarm_gripper_bi_rl/openarm_gripper_bi_rl.usd
```

And always load the matching manifest (`<asset>_manifest.yaml` next to the USD, or
`generated/rl/<asset>_manifest.yaml` here).

## Adding a New End-Effector

1. Add or import the source end-effector URDF/xacro under its vendor/source package.
2. Create a local wrapper under `eef/` if helper frames are needed.
3. Create a composition xacro under `assemblies/`.
4. Generate or save the stable source URDF under `generated/source/`.
5. Add a source entry and mapping rules to `tools/generate_rl_urdf.py`.
6. Run the generator (`python3 tools/generate_rl_urdf.py`). It also runs the
   self-collision audit (`tools/audit_self_collision.py`) - a FAIL means the
   asset interpenetrates at rest and must be fixed, not skipped.
7. Verify the new manifest's `control_joint_order` matches the desired action vector.
8. Build the USD headlessly (no GUI import):
   `/home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/build_usd.py <asset> [--sync-hdgp]`.
   Import settings (convexDecomposition colliders, unmerged fixed joints, fixed
   base) are pinned in the script and contract-checked against the manifest.
9. If Fabrics needs the robot, add a variant to `tools/gen_fabric_urdfs.py`
   and run it (`[--sync-hdgp]`); every variant is FK-gated against its RL URDF.

## Notes

- `pinky` is the canonical name. Source names like `little` are mapped to `pinky`.
- Source typos such as RH56F1 `plam` are mapped to canonical `palm` names.
- XML order is made readable, but the manifest is the authoritative action-order source.
