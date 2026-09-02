#!/usr/bin/env python3
"""Headless URDF -> USD build for the generated RL assets.

Replaces the manual Isaac Sim GUI import. The import settings are pinned in
code so they can never drift from what the asset pipeline assumes:

- collider_type "convex_decomposition": required by the self-collision audit
  (tools/audit_self_collision.py) - a plain convex hull fills concave pockets
  (e.g. the palm's thumb pocket) and fabricates resting penetrations. The
  manifest records this as `requires_collision_approximation`.
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

import shutil  # noqa: E402
import sys  # noqa: E402
import xml.etree.ElementTree as ET  # noqa: E402
from dataclasses import dataclass, field  # noqa: E402
from pathlib import Path  # noqa: E402

import yaml  # noqa: E402
from isaaclab.sim.converters import UrdfConverter, UrdfConverterCfg  # noqa: E402
from pxr import Usd, UsdPhysics  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"
HDGP_ROBOT_DIR = ROOT.parent / "hdgp" / "assets" / "robot"

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
# applied" is not evidence that the joints actually track.
MIMIC_NATURAL_FREQUENCY = 500.0
MIMIC_DAMPING_RATIO = 1.0


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
    gain_sources: tuple[GainSource, ...]


# ★Every asset must be listed - an unlisted asset is a build error, not a
# default, so a new asset cannot silently inherit the wrong actuation model.
ASSET_BUILDS: dict[str, AssetBuild] = {
    # RH56F1: 12 physical hand DOF, 6 driven. No controller PD to port - the
    # hand is a position servo with a register interface and no exposed gains
    # (sim2real/isaacsim_bridge/config/rh56f1_hand_calibration.yaml).
    "openarm_bi_rh56f1_rl": AssetBuild(
        hand_drive="mimic", gain_sources=(ARM_DRIVER_GAINS,)),
    "openarm_tesollo_bi_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
    "openarm_tesollo_bi_s_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
    # Left arm carries the 2-finger gripper (no DG-5F controller covers it),
    # right arm carries the DG-5F hand.
    "openarm_tesollo_sensor_rl": AssetBuild(
        hand_drive="direct", gain_sources=(ARM_DRIVER_GAINS, DG5F_DRIVER_GAINS)),
}

# Collider approximation: convexDecomposition on EVERY mesh collider.
# A convexHull build was attempted for spawn speed (GUI-era assets were 100%
# hull) and REVERTED: PhysX GPU limits convex hulls to 64 vertices, and the
# 64-vertex circumscribed hull of a large mesh inflates tens of millimetres
# past the exact hull - measured ghost contacts across a 9.6mm (l_al_5/l_al_7,
# 427kN) and even a 36mm gap (body/gripper finger). That is incompatible with
# enabled_self_collisions=True and cannot be audited offline. The GUI-era hull
# assets only worked because self-collision was always off. Slow first boot is
# decomposition cooking; it is cached per machine afterwards.


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
    for source in ASSET_BUILDS[asset].gain_sources:
        for joint, (joint_stiffness, joint_damping) in controller_gains(source, manifest, asset).items():
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
    collider_type = manifest.get("requires_collision_approximation", "convex_decomposition")

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
    verify_contract(usd_path, manifest, asset, urdf_path)
    patch_and_verify_mimic_joints(usd_path, asset, urdf_path)
    apply_collision_filters(usd_path, urdf_path, asset,
                            [tuple(p) for p in manifest["self_collision_filtered_pairs"]])
    patch_visuals_prims(out_dir, asset, urdf_path)
    # make generated/rl/<asset>/ a self-contained bundle: usd layers + the
    # exact urdf/manifest the usd was built from
    for source in (urdf_path, manifest_path):
        shutil.copyfile(source, out_dir / source.name)
    return usd_path


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


def patch_one_mimic_joint(prim, name: str, leader: str, multiplier: float,
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
    for name, (leader, multiplier) in sorted(mimics.items()):
        problems.extend(patch_one_mimic_joint(
            joints.get(name), name, leader, multiplier, touched_layers))

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


def verify_contract(usd_path: Path, manifest: dict, asset: str,
                    urdf_path: Path | None = None) -> None:
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
            if got != "convexDecomposition":
                wrong_approximations.append((prim.GetPath().pathString, got, "convexDecomposition"))

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


def main() -> int:
    if args_cli.names:
        assets = args_cli.names
    else:
        assets = sorted(p.stem for p in RL_DIR.glob("*_rl.urdf"))
    for asset in assets:
        usd_path = convert(asset)
        print(f"[{asset}] wrote {usd_path}")
        if args_cli.sync_hdgp:
            sync_hdgp(asset, usd_path)
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    sys.exit(code)
