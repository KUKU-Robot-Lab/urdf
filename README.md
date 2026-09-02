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

- `openarm_tesollo_sensor.urdf`
- `openarm_tesollo_bi.urdf`
- `openarm_bi_rh56f1.urdf`
- `openarm_modular_dual.urdf`
- `openarm_bimanual_no_mount.urdf`

The first three are the currently validated structures and are used by the RL generator.

### `generated/rl/`

RL-only canonical outputs.

- `openarm_tesollo_sensor_rl.urdf`
- `openarm_tesollo_sensor_rl_manifest.yaml`
- `openarm_tesollo_bi_rl.urdf`
- `openarm_tesollo_bi_rl_manifest.yaml`
- `openarm_bi_rh56f1_rl.urdf`
- `openarm_bi_rh56f1_rl_manifest.yaml`

Use these for training.

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
# 새 빌드는 자동 처리됨 (build_usd.py: shrink_massless_frames)
IsaacLab/isaaclab.sh -p tools/build_usd.py <asset> --sync-hdgp

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
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py
```

This generates all validated RL assets under `generated/rl/`.

Expected outputs:

```text
generated/rl/openarm_tesollo_sensor_rl.urdf
generated/rl/openarm_tesollo_sensor_rl_manifest.yaml
generated/rl/openarm_tesollo_bi_rl.urdf
generated/rl/openarm_tesollo_bi_rl_manifest.yaml
generated/rl/openarm_bi_rh56f1_rl.urdf
generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
```

### Generate one RL URDF

```bash
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_tesollo_bi
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_tesollo_sensor
python3 /home/user/rl_ws/urdf/tools/generate_rl_urdf.py openarm_bi_rh56f1
```

Valid source names:

```text
openarm_tesollo_sensor
openarm_tesollo_bi
openarm_bi_rh56f1
```

### Validate the generator syntax

```bash
python3 -m py_compile /home/user/rl_ws/urdf/tools/generate_rl_urdf.py
```

### Inspect action order

```bash
sed -n '/^control_joint_order:/,/^kinematic_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl_manifest.yaml
```

### Inspect full kinematic order

```bash
sed -n '/^kinematic_joint_order:/,/^fixed_joint_order:/p'   /home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
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
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_sensor_rl.urdf
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl.urdf
/home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl.urdf
```

And always load the matching manifest:

```text
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_sensor_rl_manifest.yaml
/home/user/rl_ws/urdf/generated/rl/openarm_tesollo_bi_rl_manifest.yaml
/home/user/rl_ws/urdf/generated/rl/openarm_bi_rh56f1_rl_manifest.yaml
```

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
