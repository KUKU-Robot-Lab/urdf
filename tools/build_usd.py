#!/usr/bin/env python3
"""Headless URDF -> USD build for the generated RL assets.

Replaces the manual Isaac Sim GUI import. The import settings are pinned in
code so they can never drift from what the asset pipeline assumes:

- colliders (2026-09-05 policy, recorded per asset in the manifest's
  `collision_approximation` block by tools/generate_rl_urdf.py): dexterous hand
  links (DG-5F / DG-5F-S / RH56F1) are imported as convexDecomposition - a
  plain hull fills concave pockets (the palm's thumb pocket) and fabricates
  resting penetrations; OpenArm-made links (body, arms, head, stock gripper)
  are then overridden to convexHull. ⚠ PhysX GPU hulls are capped at 64
  vertices and inflate past the exact hull (measured 2026-08: ghost contacts
  across a 9.6 mm arm gap with self-collision ON), which is why the audit's
  hull-only WARN pairs are all authored as PhysX collision filters.
- merge_fixed_joints False: the historical GUI imports did NOT merge. Measured
  on the previous openarm_tesollo_bi_s_rl USD: 77 rigid bodies including every
  `*_hl_palm` and `*_tip`. Merging drops 20 bodies, and hdgp addresses them by
  name - `palm_body` (the pose every fabric/IK/reward term is built on) and the
  fingertip contact sensors. With merge on the env dies at boot:
  "Sensor at path '.../r_hl_thumb_tip' could not find any bodies with contact
  reporter API". The body contract is verified below so this cannot regress
  silently again.
- fix_base True, import-time self_collision False (the training cfg decides
  `enabled_self_collisions` at runtime).
- joint drive: position targets, with each joint's PD gains lifted from the
  controller config that actually drives it (`AssetBuild.gain_sources`) and a
  fallback for joints no controller covers. hdgp actuator configs may still
  overwrite gains at spawn time.
- hand actuation per `AssetBuild.hand_drive`: "mimic" for an underactuated
  hand (RH56F1) authors PhysX mimic joints from the URDF <mimic> tags,
  "direct" (Tesollo) drives every hand joint independently.

Both are declared per asset in `ASSET_BUILDS`; an unlisted asset is a build
error rather than a default, so a new robot cannot silently inherit the wrong
actuation model.

The importer emits the same layered structure the GUI produced (top-level
`<asset>.usd` + `configuration/{base,physics,robot,sensor}.usd`), so the whole
asset directory is the artifact. After conversion the build verifies against
the manifest: every joint in `control_joint_order` must exist in the USD, and
every mesh collider must carry the required collision approximation.

Run (Isaac environment required):

    /home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/build_usd.py [asset...] \
        [--sync-hdgp]

Note: apps/isaaclab.python.kit pins isaacsim.asset.importer.urdf (2.4.31);
changing that pin can change conversion output - keep it in mind when
comparing against GUI-imported USDs.
"""

import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("names", nargs="*",
                    help="RL asset names, e.g. openarm_tesollo_bi_s_rl (default: all).")
parser.add_argument("--sync-hdgp", action="store_true",
                    help="Copy the USD layer stack and manifest into hdgp/assets/robot/<asset>/.")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import math  # noqa: E402
import os as _os
import shutil  # noqa: E402
import sys  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from dataclasses import dataclass, field  # noqa: E402
from pathlib import Path  # noqa: E402

import json  # noqa: E402
import yaml  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"
HDGP_ROBOT_DIR = ROOT.parent / "hdgp" / "assets" / "robot"
_RGB_FRAME: dict[str, str] = {}

# ★2026-09-05 (user): the head must come from the Fusion USD itself, not from the
#   URDF importer's re-meshed copy - head_v1.usda carries the per-face materials and
#   the D435i optical frames (color/depth/left_ir/right_ir/ir_projector, ROS optical
#   orientation) that the importer cannot reproduce. The URDF import still defines
#   the head links/joints/masses (kinematics, control); graft_head_v1() then swaps
#   each head link's imported visuals+collisions for a reference to head_v1.usda.
HEAD_V1_USD = ROOT.parent / "hdgp" / "assets" / "simulation_setting" / "head_v1" / "usd" / "head_v1.usda"
HEAD_V1_DATA = ROOT / "vendor" / "head_v1" / "head_data.json"
HEAD_V1_ROOT_PRIM = "/HeadV1"
HEAD_V1_LINKS = ("base_link", "pan_link", "tilt_link", "camera_link")
HEAD_V1_JOINTS = ("root_joint", "pan_joint", "tilt_joint", "camera_joint")
# robot link -> usda links whose geometry it owns (tilt+camera are one URDF link)
HEAD_V1_GRAFT = {
    "head_base": ("base_link",),
    "head_mid": ("pan_link",),
    "head_camera": ("tilt_link", "camera_link"),
}
HEAD_V1_GRAFT_PRIM = "head_v1"

# Fallback drive gains, for joints no controller config covers (see
# joint_drive_gains). hdgp actuator cfgs may still overwrite them at spawn.
DRIVE_STIFFNESS = 100.0
DRIVE_DAMPING = 1.0

# Mimic compliance, overwriting the importer's defaults (25 / 0.005).
#
# A PhysX mimic joint is a spring, not a rigid linkage, and the natural
# frequency formulation scales its stiffness by the joint's own inertia - which
# is nearly nothing for a distal phalanx. At the importer defaults the coupling
# is authored but does not hold: driving the RH56F1 leader joints and letting
# the followers settle, the follower drifts the WRONG way as the finger curls
# (leader 0.8 rad -> follower -0.299 rad where +0.893 was asked for). Measured
# worst |q_follower - multiplier*q_leader| over all 12 pairs, sim dt 1/120:
#
#     naturalFrequency / dampingRatio     leader 0.2 rad    leader 0.8 rad
#     25 / 0.005 (importer default)         0.360 rad         1.192 rad
#     500 / 1.0 (pinned here)               0.009 rad         0.012 rad
#
# Re-measure with the same sweep if the value is ever changed; "the API is
# applied" is not evidence that the joints actually track. Worst tracking error
# over all 12 pairs, dt 1/120, no contact:
#
#     naturalFrequency   omega*dt   leader 0.2 rad   leader 0.8 rad
#             25 (imp.)      0.21        0.360            1.192
#             50             0.42        0.244            0.876
#            100             0.83        0.122            0.344
#            200             1.67        0.045            0.078
#            500             4.17        0.009            0.012
#
# 200 is the stiffest value inside the omega*dt < 2 bound a downstream scene
# asked for after a solver blow-up. That blow-up was the limit conflict below,
# not integrator instability - the same report measured that shrinking the
# physics dt 8x (omega*dt 4.2 -> 0.52) did NOT stop the divergence. 500 tracks
# 6x tighter and is worth re-testing once the widened limits are in.
MIMIC_NATURAL_FREQUENCY = 200.0
MIMIC_DAMPING_RATIO = 1.0

# How far contact may push a DRIVEN joint past its own limit. The dependent
# joint's limit is widened to cover the mimic demand over that whole excursion.
#
# The importer sizes a dependent limit as leader_range * multiplier plus a 20%
# buffer, which for a short-range leader is almost nothing: r_hj_thumb_2 spans
# 0.475 rad, so r_hj_thumb_3 came out [-0.108, +0.651]. Measured downstream in
# a 1024-env grasp scene, contact pushed r_hj_thumb_2 to -0.247 rad (its lower
# limit is 0); the mimic then demanded -0.282 rad of thumb_3, outside its
# limit, so the limit constraint and the mimic constraint became jointly
# unsatisfiable. The solver injects energy to satisfy both and the scene
# explodes - 400 of 1024 envs were in that state at once, |qd| 292 rad/s on
# the dependent joints while the arm stayed at 3.1.
#
# Widening costs nothing physically: the mimic constraint is what decides where
# a dependent joint sits, and its limit is only a backstop. 0.5 rad covers ~2x
# the worst overshoot measured so far.
MIMIC_LEADER_OVERSHOOT = 0.5
# Same idea for a prismatic leader (stock gripper jaw, 44 mm stroke): metres.
MIMIC_LEADER_OVERSHOOT_PRISMATIC_M = 0.01


@dataclass(frozen=True)
class GainSource:
    """A controller config to lift the real PD gains out of.

    `gains_at` is the key path to descend before reaching a mapping of driver
    joint name -> gains. Driver names are translated to canonical joint names
    through the asset manifest's `source_to_canonical_joints`, so a source can
    be pointed at any asset and only its own joints are touched; `aliases`
    covers keys the manifest cannot resolve because they are not joint names.
    """

    path: str
    stiffness_key: str
    damping_key: str
    gains_at: tuple[str, ...] = ()
    aliases: dict[str, tuple[str, ...]] = field(default_factory=dict)


# The arm's ros2_control gains. Same OpenArm on every asset here, and the file
# is keyed `joint1..joint7` rather than by joint name, so it needs aliases.
ARM_DRIVER_GAINS = GainSource(
    path="vendor/openarm_description/config/arm/v10/control_gains.yaml",
    stiffness_key="kp",
    damping_key="kd",
    aliases={f"joint{i}": (f"r_aj_{i}", f"l_aj_{i}") for i in range(1, 8)},
)

# The DG-5F hand's position-PID controller. `command_interface: effort` with a
# position reference, so `p`/`d` are Nm/rad and Nm/(rad/s) - the same units the
# USD drive wants.
# ⚠ This is the VENDOR default (p=1.5). The deployed robot re-applies a tuned
# p=4.5 / d=0 at every bringup (sim2real `apply_hand_gains.py`, 3x vendor,
# oscillates from p=6). Change the numbers here if the built asset should carry
# the tuned value instead - it moves hand physics for every Tesollo asset, so
# it wants a deliberate rebuild, not a drive-by edit.
DG5F_DRIVER_GAINS = GainSource(
    path="vendor/delto_m_ros2/dg5f_driver/config/dg5f_both_pid_all_controller.yaml",
    stiffness_key="p",
    damping_key="d",
    gains_at=("ros__parameters", "gains"),
)


@dataclass(frozen=True)
class FixedGains:
    """Gains that live in vendor source code rather than a config file."""

    joints: tuple[str, ...]
    stiffness: float
    damping: float
    origin: str


# The stock OpenArm gripper: one Damiao motor in MIT mode, gains hard-coded in
# the vendor hardware interface (vendor/openarm_real/openarm_ros2/openarm_hardware/
# include/openarm_hardware/v10_simple_hardware.hpp GRIPPER_KP / GRIPPER_KD).
# ⚠ Those are MOTOR gains (N·m/rad about the motor axis); the URDF jaw joint is
# prismatic (metres), so the numbers are carried over verbatim per the 2026-09-05
# instruction, not unit-converted. `hj_gripper_2` is the mimic follower - listed
# so a "direct" build would still cover it; under "mimic" it has no drive.
GRIPPER_DRIVER_GAINS = FixedGains(
    joints=("r_hj_gripper_1", "r_hj_gripper_2", "l_hj_gripper_1", "l_hj_gripper_2"),
    stiffness=5.0, damping=0.1,
    origin="openarm_real v10_simple_hardware.hpp GRIPPER_KP/GRIPPER_KD",
)


@dataclass(frozen=True)
class AssetBuild:
    """Per-asset build settings that cannot be derived from the URDF alone.

    `hand_drive` picks the actuation model:

    * "mimic" - underactuated hand (RH56F1: 12 physical DOF, 6 driven). The
      URDF <mimic> tags become PhysX mimic joints and the dependent joints lose
      their drive, so only `control_joint_order` is commandable.
    * "direct" - every hand joint has its own drive (Tesollo). URDF <mimic>
      tags are ignored and the joint stays independently driven.

    ★The IsaacLab flag behind "mimic" is named backwards.
    `UrdfConverterCfg.convert_mimic_joints_to_normal_joints` is passed straight
    to the importer as `set_parse_mimic()` (urdf_converter.py:131), which reads
    it as "parse the mimic tags": True KEEPS the coupling. Leaving it at its
    default False is what silently cost RH56F1 its 12 couplings when the GUI
    import was replaced by this script - the URDF still had every <mimic> tag,
    the USD had none, and the distal joints became free-swinging bodies.

    ★openarm_tesollo_sensor_rl is "direct" even though its `l_hj_gripper_2`
    carries a <mimic> tag: hdgp's left gripper task adapted to the missing
    coupling by commanding both jaws (gripper/left/grasp_sensor/
    grasp_left_preset.py) and its deployed checkpoint was trained that way.
    Switching it to "mimic" changes jaw physics under a trained policy.
    """

    hand_drive: str
    gain_sources: tuple[GainSource | FixedGains, ...]


# ★Every asset must be listed - an unlisted asset is a build error, not a
# default, so a new asset cannot silently inherit the wrong actuation model.
# 2026-09-05 line-up (generate_rl_urdf.SOURCES): vendor gains verbatim -
# OpenArm arm control_gains.yaml, DG-5F(-S) dg5f_driver PID, stock gripper
# GRIPPER_KP/KD. RH56F1 has NO vendor PD: the vendor stack (vendor/inspire_ws,
# RS-485 protocol) only exposes angleSet / speedSet / forceSet / defaultSpeedSet /
# defaultForceSet registers (position servo), so its hand joints keep the
# DRIVE_STIFFNESS/DRIVE_DAMPING fallback (sim2real maps rad<->register in
# isaacsim_bridge/config/rh56f1_hand_calibration.yaml).
ASSET_BUILDS: dict[str, AssetBuild] = {
    "openarm_dg5f-m_bi_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
    # DG-5F short base: the same DG-5F hand with a shortened mount/base, so the
    # same dg5f_driver PID applies verbatim (identical lj_/rj_dg_* joint names;
    # dg5f_ros2's config is byte-identical to the delto_m_ros2 copy read here).
    "openarm_dg5f-m-short_bi_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
    # DG-5F-S shares the dg5f_driver stack (same lj_/rj_dg_* joint names).
    "openarm_dg5f-s_bi_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
    # RH56F1: 12 physical hand DOF, 6 driven -> PhysX mimic joints.
    "openarm_rh56f1_bi_rl": AssetBuild(
        hand_drive="mimic", gain_sources=(ARM_DRIVER_GAINS,)),
    # Stock gripper: one motor drives both jaws (URDF finger_joint2 mimics
    # finger_joint1) -> PhysX mimic; only *_hj_gripper_1 is commandable.
    "openarm_gripper_bi_rl": AssetBuild(
        hand_drive="mimic", gain_sources=(ARM_DRIVER_GAINS, GRIPPER_DRIVER_GAINS)),
}


def descend(node: dict, keys: tuple[str, ...], source: GainSource) -> dict:
    """Follow `keys` into a ros2_control document, ignoring the node name.

    ros2_control files namespace every controller under a node key like
    `/**/lj_dg_pospid`, one per controller, so the gain map is not at a fixed
    path. Every branch whose key path resolves is merged.
    """
    if not keys:
        return node
    merged: dict = {}
    for value in node.values():
        if not isinstance(value, dict):
            continue
        found = value
        for key in keys:
            found = found.get(key) if isinstance(found, dict) else None
            if found is None:
                break
        if isinstance(found, dict):
            merged.update(found)
    if not merged:
        raise SystemExit(f"no '{'.'.join(keys)}' block in {source.path}")
    return merged


def controller_gains(source: GainSource, manifest: dict, asset: str) -> dict[str, tuple[float, float]]:
    """Canonical joint name -> (stiffness, damping) from one controller config."""
    document = yaml.safe_load((ROOT / source.path).read_text(encoding="utf-8"))
    entries = descend(document, source.gains_at, source)
    rename = manifest["source_to_canonical_joints"]

    gains: dict[str, tuple[float, float]] = {}
    for key, values in entries.items():
        targets = source.aliases.get(key) or ((rename[key],) if key in rename else ())
        for joint in targets:
            gains[joint] = (float(values[source.stiffness_key]),
                            float(values[source.damping_key]))
    if not gains:
        raise SystemExit(
            f"[{asset}] {source.path} matched no joint - the manifest's "
            "source_to_canonical_joints has none of its keys and no alias covers them"
        )
    return gains


def joint_drive_gains(asset: str, manifest: dict) -> UrdfConverterCfg.JointDriveCfg.PDGainsCfg:
    """Per-joint PD gains lifted from the asset's controller configs.

    Anything no controller covers keeps the fallback. Keys are regexes the
    importer matches with `re.search` in insertion order, so the catch-all has
    to go first for the per-joint entries to win. Values are SI (Nm/rad); the
    importer converts to the per-degree units USD stores.
    """
    stiffness: dict[str, float] = {".*": DRIVE_STIFFNESS}
    damping: dict[str, float] = {".*": DRIVE_DAMPING}
    present = set(manifest["kinematic_joint_order"])
    for source in ASSET_BUILDS[asset].gain_sources:
        if isinstance(source, FixedGains):
            gains = {j: (source.stiffness, source.damping) for j in source.joints if j in present}
            if not gains:
                raise SystemExit(f"[{asset}] {source.origin}: none of {source.joints} is in the asset")
        else:
            gains = controller_gains(source, manifest, asset)
        for joint, (joint_stiffness, joint_damping) in gains.items():
            stiffness[f"^{joint}$"] = joint_stiffness
            damping[f"^{joint}$"] = joint_damping
    covered = len(stiffness) - 1
    print(f"[{asset}] driver gains ported for {covered} joints; "
          f"the rest keep {DRIVE_STIFFNESS}/{DRIVE_DAMPING}")
    return UrdfConverterCfg.JointDriveCfg.PDGainsCfg(stiffness=stiffness, damping=damping)


def convert(asset: str) -> Path:
    urdf_path = RL_DIR / f"{asset}.urdf"
    manifest_path = RL_DIR / f"{asset}_manifest.yaml"
    if not urdf_path.is_file() or not manifest_path.is_file():
        raise SystemExit(f"missing URDF or manifest for {asset}")
    if asset not in ASSET_BUILDS:
        raise SystemExit(
            f"[{asset}] no entry in ASSET_BUILDS - declare its hand_drive "
            "(\"mimic\" or \"direct\") and gain sources before building it"
        )
    manifest = yaml.safe_load(manifest_path.read_text(encoding="utf-8"))
    policy = manifest.get("collision_approximation")
    if not policy:
        raise SystemExit(
            f"[{asset}] manifest has no collision_approximation block - "
            "regenerate with `python3 tools/generate_rl_urdf.py`"
        )
    collider_type = policy["default"]
    hull_links = list(policy.get("convex_hull_links") or [])

    out_dir = RL_DIR / asset
    converter_cfg = UrdfConverterCfg(
        asset_path=str(urdf_path),
        usd_dir=str(out_dir),
        usd_file_name=f"{asset}.usd",
        force_usd_conversion=True,
        fix_base=True,
        merge_fixed_joints=False,
        self_collision=False,
        collider_type=collider_type,
        # True == "keep the URDF mimic coupling" - see AssetBuild.hand_drive.
        convert_mimic_joints_to_normal_joints=ASSET_BUILDS[asset].hand_drive == "mimic",
        joint_drive=UrdfConverterCfg.JointDriveCfg(
            target_type="position",
            gains=joint_drive_gains(asset, manifest),
        ),
    )
    if "self_collision_filtered_pairs" not in manifest:
        raise SystemExit(
            f"[{asset}] manifest has no self_collision_filtered_pairs - "
            "regenerate with `python3 tools/generate_rl_urdf.py` (audit enabled)"
        )
    usd_path = Path(UrdfConverter(converter_cfg).usd_path)
    graft_head_v1(usd_path, asset)
    if hull_links:
        override_link_approximation(usd_path, asset, hull_links, "convexHull")
    rgb_frame = (manifest.get("camera_view_frame") or {}).get("rgb_lens_usd_frame")
    if not rgb_frame:
        raise SystemExit(f"[{asset}] manifest camera_view_frame has no rgb_lens_usd_frame - regenerate the URDF")
    _RGB_FRAME[asset] = rgb_frame
    # ★head 검증은 sync 후로 옮겼다 — 참조가 상대경로라 hdgp 배치에서만 해석된다.
    verify_contract(usd_path, manifest, asset, urdf_path, want=APPROX_TOKEN[collider_type],
                    per_link_want={l: "convexHull" for l in hull_links})
    patch_and_verify_mimic_joints(usd_path, asset, urdf_path)
    apply_collision_filters(usd_path, urdf_path, asset,
                            [tuple(p) for p in manifest["self_collision_filtered_pairs"]])
    patch_visuals_prims(out_dir, asset, urdf_path)
    shrink_massless_frames(usd_path, urdf_path, asset)
    # make generated/rl/<asset>/ a self-contained bundle: usd layers + the
    # exact urdf/manifest the usd was built from
    for source in (urdf_path, manifest_path):
        shutil.copyfile(source, out_dir / source.name)
    # ★반드시 **마지막** — 이 뒤로는 head 참조가 빌드 디렉터리에서 해석되지 않는다.
    relativize_head_reference(usd_path, asset)
    return usd_path


def head_v1_link_offsets() -> dict[str, tuple[float, float, float]]:
    """robot head link frame origin, expressed in the head_v1 base (Fusion) frame.

    head_v1.usda authors every link's geometry in the base frame (link Xforms are
    identity); the URDF links sit at the joint axis points, so the referenced
    geometry has to be shifted back by that origin.
    """
    data = json.loads(HEAD_V1_DATA.read_text(encoding="utf-8"))
    pan = tuple(float(v) for v in data["pan_axis_point_m"])
    tilt = tuple(float(v) for v in data["tilt_axis_point_m"])
    return {"head_base": (0.0, 0.0, 0.0), "head_mid": pan, "head_camera": tilt}


def graft_head_v1(usd_path: Path, asset: str) -> None:
    """Replace the imported head geometry with references into head_v1.usda.

    Per robot head link: the importer's `visuals`/`collisions` children are
    deactivated and a `head_v1` Xform (translate = -link origin) references the
    whole /HeadV1 prim, so material bindings (/HeadV1/Looks) and the optical
    frames keep resolving; everything that is not this link's geometry is
    deactivated and the referenced physics (articulation root, rigid bodies,
    masses, joints) is stripped so the robot stays ONE articulation whose
    head links come from the URDF. Camera consumers can then use
    <root>/head_camera/head_v1/camera_link/color_frame (etc.).
    """
    if not HEAD_V1_USD.is_file():
        raise SystemExit(f"[{asset}] head_v1 USD missing: {HEAD_V1_USD}")
    stage = Usd.Stage.Open(str(usd_path))
    root_path = stage.GetDefaultPrim().GetPath()
    offsets = head_v1_link_offsets()
    grafted = 0
    for link, keep in HEAD_V1_GRAFT.items():
        link_prim = stage.GetPrimAtPath(root_path.AppendChild(link))
        if not link_prim.IsValid():
            raise SystemExit(f"[{asset}] head link {link} missing - cannot graft head_v1")
        for child in ("visuals", "collisions"):
            imported = link_prim.GetChild(child)
            if imported.IsValid():
                imported.SetActive(False)
        graft = stage.DefinePrim(link_prim.GetPath().AppendChild(HEAD_V1_GRAFT_PRIM), "Xform")
        ox, oy, oz = offsets[link]
        UsdGeom.Xformable(graft).AddTranslateOp().Set(Gf.Vec3d(-ox, -oy, -oz))
        # ★참조는 여기서 **절대경로**로 건다 — 아래에서 참조된 프림을 읽어 물리를 벗겨내야
        #   하므로 저작 시점에 해석돼야 한다. 배포용 상대경로 변환은 convert() 마지막의
        #   `relativize_head_reference()` 가 한다(사유는 그 함수 주석).
        graft.GetReferences().AddReference(str(HEAD_V1_USD), HEAD_V1_ROOT_PRIM)
        # one articulation only: strip the referenced head's own physics
        graft.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        graft.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        for joint in HEAD_V1_JOINTS:
            graft.GetChild(joint).SetActive(False)
        for usda_link in HEAD_V1_LINKS:
            prim = graft.GetChild(usda_link)
            if not prim.IsValid():
                raise SystemExit(f"[{asset}] head_v1.usda has no link {usda_link}")
            if usda_link not in keep:
                prim.SetActive(False)
                continue
            prim.RemoveAPI(UsdPhysics.RigidBodyAPI)
            prim.RemoveAPI(UsdPhysics.MassAPI)
            collision = prim.GetChild("Collision")
            if not collision.HasAPI(UsdPhysics.MeshCollisionAPI):
                raise SystemExit(f"[{asset}] head_v1.usda {usda_link}/Collision has no mesh collider")
            grafted += 1
    stage.GetRootLayer().Save()
    print(f"[{asset}] head_v1.usda grafted onto {len(HEAD_V1_GRAFT)} head links ({grafted} usda links; "
          f"optical frames at <root>/head_camera/{HEAD_V1_GRAFT_PRIM}/camera_link/*_frame)")


def relativize_head_reference(usd_path: Path, asset: str) -> int:
    """head_v1 참조를 **hdgp 배치 기준 상대경로**로 다시 쓴다. convert() 의 마지막 단계.

    ★2026-09-09. 여태 이 참조에 빌드 머신의 절대경로(`/home/user/rl_ws/...`)가 박혀
      학습 서버(`/home/oem/...`)에서 열리지 않았다. 자산은 USD 만 배포되는데 참조 대상
      head_v1 은 **서버에도 존재한다** — 경로만 틀렸다. 치명적이지 않아 경고로만 남았고
      (팔·손 물리엔 안 닿는다) env 하나당 3줄씩 나온다: 4096 env 에 12,288줄,
      24576 env 에 73,728줄이 되어 씬 구축을 늦춘다.
      ⚠**머리 지오메트리가 서버 학습에서 통째로 빠져 있었다**는 뜻이기도 하다.

    상대경로는 hdgp 배치(`assets/robot/<asset>/` → `assets/simulation_setting/`) 기준이라
    빌드 디렉터리에서는 해석되지 않는다. 그래서 이 변환은 **모든 편집이 끝난 뒤**에 하고,
    head graft 검증은 sync 후 hdgp 사본에서 한다(main 참조).
    """
    rel = _os.path.relpath(HEAD_V1_USD, HDGP_ROBOT_DIR / asset)
    stage = Usd.Stage.Open(str(usd_path))
    changed = 0
    for prim in stage.TraverseAll():
        if prim.GetName() != HEAD_V1_GRAFT_PRIM:
            continue
        refs = prim.GetReferences()
        refs.ClearReferences()
        refs.AddReference(rel, HEAD_V1_ROOT_PRIM)
        changed += 1
    if not changed:
        raise SystemExit(f"[{asset}] head_v1 graft 프림을 못 찾았다 — 상대경로 변환 실패")
    stage.GetRootLayer().Save()
    print(f"[{asset}] head_v1 참조 상대경로화 {changed}개 -> {rel}")
    return changed


def verify_head_v1_graft(usd_path: Path, asset: str, rgb_frame: str) -> None:
    """The grafted RGB-lens frame (manifest camera_view_frame.rgb_lens_usd_frame, the
    head_v1.usda label of the aperture that is the RGB lens - hand-eye 2026-09-05) must
    coincide with the URDF's head_cam_view frame."""
    stage = Usd.Stage.Open(str(usd_path))
    root_path = stage.GetDefaultPrim().GetPath()
    cache = UsdGeom.XformCache()
    color = stage.GetPrimAtPath(root_path.AppendPath(f"head_camera/{HEAD_V1_GRAFT_PRIM}/camera_link/{rgb_frame}"))
    view = stage.GetPrimAtPath(root_path.AppendChild("head_cam_view"))
    if not color.IsValid() or not view.IsValid():
        raise SystemExit(f"[{asset}] head_v1 graft verification: {rgb_frame} or head_cam_view missing")
    delta = cache.GetLocalToWorldTransform(color).ExtractTranslation() - cache.GetLocalToWorldTransform(view).ExtractTranslation()
    if delta.GetLength() > 1e-4:
        raise SystemExit(f"[{asset}] head_v1 graft misplaced: color_frame - head_cam_view = {delta} m")
    bodies = [p.GetName() for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
              if p.HasAPI(UsdPhysics.RigidBodyAPI) and p.GetName() in HEAD_V1_LINKS]
    roots = [p.GetPath().pathString for p in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
             if p.HasAPI(UsdPhysics.ArticulationRootAPI)]
    if bodies or len(roots) != 1:
        raise SystemExit(f"[{asset}] head_v1 graft left physics behind: bodies={bodies} articulation roots={roots}")
    print(f"[{asset}] head_v1 graft ok: {rgb_frame} == head_cam_view ({delta.GetLength() * 1e6:.1f} um), 1 articulation root")
    _trace(f"[{asset}] head_v1 graft ok: {rgb_frame} == head_cam_view ({delta.GetLength() * 1e6:.1f} um)")


def shrink_massless_frames(usd_path: Path, urdf_path: Path, asset: str) -> None:
    """URDF 에 <inertial> 이 없는 링크의 질량을 0 으로 눌러 **유령 1 kg** 을 없앤다.

    ★09.02 실측 사고. URDF 에서 `<inertial>` 이 없는 링크는 "질량 없는 좌표 프레임"인데,
    `merge_fixed_joints=False`(fingertip 센서·palm_body 이름 보존을 위해 의도적)로
    임포트하면 그 프레임도 각각 rigid body 가 되고, 질량이 안 적혀 있으면 PhysX 가
    **기본값 1.0 kg** 을 붙인다. 하필 손끝이라 중력 모멘트가 통째로 바뀌었다:

      l_hl_gripper_tcp 1.0 kg · r_hl_mount/palm_alias/palm_ee 각 1.0 kg = 총 4 kg
      → sim 좌팔 정적 처짐 j7 **11.07°** vs 실기 4.2°(같은 게인 70/60/10). 2.6배.

    `verify_contract` 는 <inertial> 이 **있는** 링크만 검사하므로 정확히 반대 집합인
    이 유령들을 못 잡았다.

    ★★질량 **0 은 소용없다**(09.02 실측): USD 에 0.0 이 적혀 있어도 PhysX 는 강체에
    질량 0 을 허용하지 않아 **기본값 1.0 kg** 으로 대체한다. 그래서 아주 작은 양수를
    넣는다. 1e-4 kg 이면 손끝에서 0.0001×9.81×0.12 ≈ 0.00012 N·m — 실기 j7 중력토크
    0.76 N·m 의 0.02% 라 무시할 수 있고, 솔버도 0 질량 강체 문제를 겪지 않는다.
    """
    def _mass(link) -> float:
        node = link.find("inertial/mass")
        return float(node.get("value") or 0.0) if node is not None else 0.0

    # no <inertial>, or a vendor mass of 0 (RH56F1 sensor/tip links) - both are 1 kg ghosts
    massless = [link.get("name") for link in ET.parse(urdf_path).getroot().findall("link")
                if _mass(link) <= 0.0 and link.get("name")]
    if not massless:
        return
    stage = Usd.Stage.Open(str(usd_path))
    fixed, missing = [], []
    for name in massless:
        prim = next((pr for pr in stage.Traverse() if pr.GetName() == name
                     and pr.HasAPI(UsdPhysics.RigidBodyAPI)), None)
        if prim is None:
            missing.append(name)
            continue
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr().Set(1e-4)
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-4, 1e-4, 1e-4))
        fixed.append(name)
    stage.GetRootLayer().Save()
    print(f"[{asset}] 무질량 프레임 {len(fixed)}개 질량 1e-4 kg 로 고정: {fixed}"
          + (f" (강체 아님·건너뜀: {missing})" if missing else ""))
    # 재검증 — 여기서 실패하면 유령이 남은 것이다
    check = Usd.Stage.Open(str(usd_path))
    left = [pr.GetName() for pr in check.Traverse()
            if pr.GetName() in set(fixed) and pr.HasAPI(UsdPhysics.MassAPI)
            and (UsdPhysics.MassAPI(pr).GetMassAttr().Get() or 1.0) > 1e-3]
    if left:
        raise SystemExit(f"[{asset}] 유령 질량이 남았다: {left}")


def patch_visuals_prims(out_dir: Path, asset: str, urdf_path: Path) -> None:
    """Define empty /visuals/<link> prims the importer forgot.

    base.usd references /visuals/<link> in the physics/base layers for every
    link, but links without visual geometry (massless frames like *_hl_mount,
    palm_alias, head_cam_view) never get one - every stage open then spams
    'Unresolved reference' warnings (the old GUI pipeline had the same defect,
    patched ad hoc by IsaacLab scripts/tools/fix_openarm_physics_usd.py).
    """
    link_names = [link.get("name") or "" for link in ET.parse(urdf_path).getroot().findall("link")]
    for layer_name in (f"{asset}_physics.usd", f"{asset}_base.usd"):
        layer_path = out_dir / "configuration" / layer_name
        stage = Usd.Stage.Open(str(layer_path))
        patched = 0
        for link in link_names:
            path = f"/visuals/{link}"
            if not stage.GetPrimAtPath(path).IsValid():
                stage.DefinePrim(path, "Xform")
                patched += 1
        if patched:
            stage.GetRootLayer().Save()
            print(f"[{asset}] patched {patched} missing /visuals prims in {layer_name}")


def urdf_mimic_joints(urdf_path: Path) -> dict[str, tuple[str, float]]:
    """Map every URDF mimic joint to its (leader joint, multiplier)."""
    joints = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        mimic = joint.find("mimic")
        if mimic is not None:
            joints[joint.get("name") or ""] = (
                mimic.get("joint") or "", float(mimic.get("multiplier", 1.0))
            )
    return joints


def widen_dependent_limit(prim, leader_prim, multiplier: float, touched_layers: set) -> None:
    """Grow a dependent joint's limit to cover the mimic demand under overshoot.

    USD revolute limits are in degrees (prismatic: metres), and the mimic
    multiplier is unitless, so the computation stays in the joint's own unit.
    Limits are only ever widened - a joint whose importer limit is already
    generous keeps it.
    """
    overshoot = (MIMIC_LEADER_OVERSHOOT_PRISMATIC_M if leader_prim.IsA(UsdPhysics.PrismaticJoint)
                 else math.degrees(MIMIC_LEADER_OVERSHOOT))
    reach = [
        value * multiplier
        for value in (leader_prim.GetAttribute("physics:lowerLimit").Get() - overshoot,
                      leader_prim.GetAttribute("physics:upperLimit").Get() + overshoot)
    ]
    for attr_name, bound, pick in (("physics:lowerLimit", min(reach), min),
                                   ("physics:upperLimit", max(reach), max)):
        attr = prim.GetAttribute(attr_name)
        attr.Set(pick(attr.Get(), bound))
        touched_layers.update(
            spec.layer for spec in attr.GetPropertyStack(Usd.TimeCode.Default()))


def patch_one_mimic_joint(prim, leader_prim, name: str, leader: str, multiplier: float,
                          touched_layers: set) -> list[str]:
    """Stiffen one mimic joint and return what is wrong with it, if anything."""
    if prim is None:
        return [f"{name}: absent from USD"]
    axes = [s.split(":", 1)[1] for s in prim.GetAppliedSchemas()
            if s.startswith("PhysxMimicJointAPI:")]
    if not axes:
        return [f"{name}: no PhysxMimicJointAPI"]

    prefix = f"physxMimicJoint:{axes[0]}"
    problems = []
    targets = prim.GetRelationship(f"{prefix}:referenceJoint").GetTargets()
    got_leader = targets[0].name if targets else None
    if got_leader != leader:
        problems.append(f"{name}: leader {got_leader} != urdf {leader}")
    gearing = prim.GetAttribute(f"{prefix}:gearing").Get()
    if gearing is None or abs(gearing + multiplier) > 1e-4:
        problems.append(f"{name}: gearing {gearing} != -{multiplier}")

    for suffix, value in (("naturalFrequency", MIMIC_NATURAL_FREQUENCY),
                          ("dampingRatio", MIMIC_DAMPING_RATIO)):
        attr = prim.GetAttribute(f"{prefix}:{suffix}")
        if not attr:
            problems.append(f"{name}: no {suffix} attribute to stiffen")
            continue
        attr.Set(value)
        touched_layers.update(
            spec.layer for spec in attr.GetPropertyStack(Usd.TimeCode.Default()))

    if leader_prim is None:
        problems.append(f"{name}: leader {leader} absent, cannot size its limit")
    else:
        widen_dependent_limit(prim, leader_prim, multiplier, touched_layers)
    return problems


def patch_and_verify_mimic_joints(usd_path: Path, asset: str, urdf_path: Path) -> None:
    """Stiffen the mimic springs, and check every URDF mimic tag survived.

    The verification is not decorative: the coupling can vanish again without a
    single error, because the URDF keeps its <mimic> tags, the USD builds, and
    every other contract passes while the dependent joints quietly become free
    bodies. That is exactly how RH56F1's 12 couplings were lost. PhysX writes
    the constraint as `q_dependent + gearing * q_leader + offset = 0`, so
    gearing is the *negative* of the URDF multiplier.
    """
    mimics = urdf_mimic_joints(urdf_path)
    if not mimics:
        return
    if ASSET_BUILDS[asset].hand_drive != "mimic":
        print(f"[{asset}] hand_drive=direct: {len(mimics)} URDF mimic tag(s) "
              "left as independently driven joints")
        return

    stage = Usd.Stage.Open(str(usd_path))
    stage.Load()
    joints = {
        prim.GetName(): prim
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint)
    }

    problems = []
    touched_layers = set()
    # Mimic chains: RH56F1's thumb_4 follows thumb_3, which itself follows
    # thumb_2. A joint that is someone's leader has to be widened before the
    # joint that is sized from it, or the second one is sized from a stale limit.
    leaders = {leader for leader, _ in mimics.values()}
    for name, (leader, multiplier) in sorted(mimics.items(), key=lambda kv: (kv[0] not in leaders, kv[0])):
        problems.extend(patch_one_mimic_joint(
            joints.get(name), joints.get(leader), name, leader, multiplier, touched_layers))

    for layer in touched_layers:
        layer.Save()

    if problems:
        raise SystemExit(
            f"[{asset}] PhysX mimic contract FAILED for {len(problems)} joint(s): "
            + "; ".join(problems[:6]) + (" ..." if len(problems) > 6 else "")
        )
    print(f"[{asset}] mimic contract ok: {len(mimics)} PhysX mimic joints "
          f"(naturalFrequency={MIMIC_NATURAL_FREQUENCY}, dampingRatio={MIMIC_DAMPING_RATIO})")


def merged_body_resolver(urdf_path: Path, bodies: set[str]):
    """Map a URDF link to the USD rigid body it merged into (fixed-joint walk)."""
    fixed_parent: dict[str, str] = {}
    for joint in ET.parse(urdf_path).getroot().findall("joint"):
        if joint.get("type") == "fixed":
            parent, child = joint.find("parent"), joint.find("child")
            assert parent is not None and child is not None
            fixed_parent[child.get("link") or ""] = parent.get("link") or ""

    def resolve(link: str) -> str | None:
        current = link
        while current not in bodies:
            if current not in fixed_parent:
                return None
            current = fixed_parent[current]
        return current

    return resolve


def apply_collision_filters(usd_path: Path, urdf_path: Path, asset: str,
                            pairs: list[tuple[str, str]]) -> None:
    """Author PhysicsFilteredPairs for the manifest's audited pairs.

    The audit exports WARN pairs (documented in self_collision_allowlist.yaml)
    whose raw clearance is within convex-decomposition cooking inflation, plus
    unverified/nested vendor geometry, into the manifest. Those pairs must
    never generate self-collision contacts; everything else keeps full
    self-collision.
    """
    stage = Usd.Stage.Open(str(usd_path))
    root_path = stage.GetDefaultPrim().GetPath()
    bodies = {
        prim.GetName()
        for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    }
    resolve = merged_body_resolver(urdf_path, bodies)

    filtered = set()
    for link_a, link_b in pairs:
        body_a, body_b = resolve(link_a), resolve(link_b)
        if body_a is None or body_b is None:
            raise SystemExit(f"[{asset}] cannot map filtered pair to bodies: {link_a} <-> {link_b}")
        if body_a == body_b or frozenset((body_a, body_b)) in filtered:
            continue  # merged into one body, or already filtered
        filtered.add(frozenset((body_a, body_b)))
        prim = stage.GetPrimAtPath(root_path.AppendChild(body_a))
        assert prim.IsValid(), body_a
        api = UsdPhysics.FilteredPairsAPI.Apply(prim)
        api.GetFilteredPairsRel().AddTarget(root_path.AppendChild(body_b))
    stage.GetRootLayer().Save()
    print(f"[{asset}] collision filters authored: {len(filtered)} body pairs "
          f"(from {len(pairs)} audited link pairs)")


APPROX_TOKEN = {"convex_decomposition": "convexDecomposition", "convex_hull": "convexHull"}


def override_link_approximation(usd_path: Path, asset: str, links: list[str], token: str) -> None:
    """Author `physics:approximation = token` on every mesh collider under `links`.

    The importer only knows one collider type per asset; the per-link policy
    (hand = decomposition, OpenArm links = hull) is applied here, authored in
    the asset's root layer (same place as the collision filters).
    """
    stage = Usd.Stage.Open(str(usd_path))
    root_path = stage.GetDefaultPrim().GetPath()
    changed = 0
    for link in links:
        link_prim = stage.GetPrimAtPath(root_path.AppendChild(link))
        if not link_prim.IsValid():
            raise SystemExit(f"[{asset}] --decomp-links: no body named {link}")
        targets = [prim.GetPath() for prim in Usd.PrimRange(link_prim, Usd.TraverseInstanceProxies())
                   if prim.HasAPI(UsdPhysics.MeshCollisionAPI)]
        for path in targets:
            prim = stage.GetPrimAtPath(path)
            # The importer instances the collision subtree; instance proxies cannot
            # carry overrides, so de-instance the nearest instanceable ancestor
            # (authored in the variant's ROOT layer, i.e. variant-local) first.
            if prim.IsInstanceProxy():
                anc = prim
                while anc and not anc.IsInstance():
                    anc = anc.GetParent()
                if not anc:
                    raise SystemExit(f"[{asset}] instance proxy without instance ancestor: {path}")
                anc.SetInstanceable(False)
                prim = stage.GetPrimAtPath(path)
            UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Set(token)
            changed += 1
    stage.GetRootLayer().Save()
    print(f"[{asset}] collider approximation overridden to {token} on {changed} colliders under {links}")


def verify_contract(usd_path: Path, manifest: dict, asset: str,
                    urdf_path: Path | None = None, want: str = "convexDecomposition",
                    per_link_want: dict[str, str] | None = None) -> None:
    """Manifest joints and every inertial link must exist; colliders must match collider_type.

    ★The body check is not decorative. hdgp addresses bodies **by name** -
    `palm_body` carries the pose that every fabric/IK/reward term is built on,
    and the fingertip contact sensors point at `*_tip`. A build that merges
    fixed joints silently drops 20 of them (77 -> 57 on bi_s) and this function
    still printed "contract ok", because it only ever looked at joints.
    """
    stage = Usd.Stage.Open(str(usd_path))
    usd_joints = set()
    usd_bodies = set()
    wrong_approximations = []
    counts: dict[str, int] = {}
    # instance proxies included: collision meshes live inside instanceable prims
    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        if prim.IsA(UsdPhysics.RevoluteJoint) or prim.IsA(UsdPhysics.PrismaticJoint):
            usd_joints.add(prim.GetName())
        if prim.HasAPI(UsdPhysics.RigidBodyAPI):
            usd_bodies.add(prim.GetName())
        if prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            got = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
            counts[got] = counts.get(got, 0) + 1
            expected_here = want
            for link, token in (per_link_want or {}).items():
                if f"/{link}/" in prim.GetPath().pathString:
                    expected_here = token
            if got != expected_here:
                wrong_approximations.append((prim.GetPath().pathString, got, expected_here))

    if urdf_path is not None:
        # Every link with an <inertial> is a body downstream code may address.
        want_bodies = {link.get("name")
                       for link in ET.parse(urdf_path).getroot().findall("link")
                       if link.find("inertial") is not None}
        lost = sorted(want_bodies - usd_bodies)
        if lost:
            raise SystemExit(
                f"[{asset}] USD body contract FAILED - {len(lost)} inertial links are not "
                f"rigid bodies (fixed-joint merging?): {', '.join(lost[:8])}"
                f"{' ...' if len(lost) > 8 else ''}"
            )

    expected = set(manifest["control_joint_order"])
    missing = sorted(expected - usd_joints)
    if missing:
        raise SystemExit(
            f"[{asset}] USD joint contract FAILED - {len(missing)} control joints missing: "
            f"{', '.join(missing[:8])}{' ...' if len(missing) > 8 else ''}"
        )
    if wrong_approximations:
        first = wrong_approximations[0]
        raise SystemExit(
            f"[{asset}] collider approximation FAILED for {len(wrong_approximations)} "
            f"colliders, e.g. {first[0]}: got {first[1]}, want {first[2]}"
        )
    extra = sorted(usd_joints - expected)
    print(f"[{asset}] contract ok: {len(expected)} control joints, "
          f"{len(usd_bodies)} rigid bodies, colliders {counts}"
          + (f"; extra articulated joints: {', '.join(extra)}" if extra else ""))


def sync_hdgp(asset: str, usd_path: Path) -> None:
    """Mirror the USD layer stack (+manifest) into hdgp's asset copy."""
    source_dir = usd_path.parent
    destination = HDGP_ROBOT_DIR / asset
    destination.mkdir(parents=True, exist_ok=True)
    (destination / "configuration").mkdir(exist_ok=True)
    targets = [usd_path, RL_DIR / f"{asset}_manifest.yaml", RL_DIR / f"{asset}.urdf"]
    targets += sorted((source_dir / "configuration").glob("*.usd"))
    for source in targets:
        relative = source.relative_to(source_dir) if source.is_relative_to(source_dir) else Path(source.name)
        target = destination / relative
        shutil.copyfile(source, target)
        print(f"[{asset}] synced -> {target}")


class _TracedExit(SystemExit):
    pass


def _trace(message: str) -> None:
    """stdout is lost once the Kit app is up (observed 2026-09-05) - keep a file trail."""
    with (RL_DIR / "build_trace.log").open("a", encoding="utf-8") as f:
        f.write(message + "\n")


def main() -> int:
    # default: the declared line-up only (legacy *_rl.urdf files stay frozen on disk)
    assets = args_cli.names or sorted(ASSET_BUILDS)
    _trace(f"build start: assets={assets} sync={args_cli.sync_hdgp}")
    for asset in assets:
        usd_path = convert(asset)
        print(f"[{asset}] wrote {usd_path}")
        _trace(f"[{asset}] wrote {usd_path}")
        if args_cli.sync_hdgp:
            sync_hdgp(asset, usd_path)
            _trace(f"[{asset}] synced")
            # head_v1 참조는 hdgp 배치 기준 상대경로라 **여기서만** 해석된다.
            verify_head_v1_graft(HDGP_ROBOT_DIR / asset / f"{asset}.usd", asset, _RGB_FRAME[asset])
            _trace(f"[{asset}] head_v1 graft verified (synced copy)")
        else:
            raise SystemExit(
                f"[{asset}] --sync-hdgp 없이 빌드하면 head_v1 참조(상대경로)를 검증할 수 없다 — "
                "빌드는 끝났지만 검증되지 않은 자산이다. --sync-hdgp 로 다시 돌릴 것.")
    _trace("build done")
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    sys.exit(code)
