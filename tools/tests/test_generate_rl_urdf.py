"""Tests for the RL URDF generation pipeline (base origin fix + head attach)."""

from __future__ import annotations

import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import generate_rl_urdf as gen  # noqa: E402

RL_NAMES = list(gen.SOURCES.keys())


@pytest.fixture(scope="session", autouse=True)
def generated_outputs(tmp_path_factory) -> None:
    # The audit is exercised separately in test_audit_self_collision.py;
    # skipping it here keeps the rest of the suite fast. ★Generate into a temp
    # dir: writing --skip-audit outputs into generated/rl/ silently strips the
    # `self_collision_filtered_pairs` block from the real manifests, and the next
    # USD build then refuses them (bit twice on 2026-09-05).
    gen.OUT_DIR = tmp_path_factory.mktemp("generated_rl")
    assert gen.main(["--skip-audit"]) == 0


def load_urdf(name: str) -> ET.Element:
    return ET.parse(gen.OUT_DIR / f"{name}_rl.urdf").getroot()


def load_manifest(name: str) -> dict:
    import yaml

    with open(gen.OUT_DIR / f"{name}_rl_manifest.yaml", encoding="utf-8") as f:
        return yaml.safe_load(f)


def joints_by_name(root: ET.Element) -> dict[str, ET.Element]:
    return {j.attrib["name"]: j for j in root.findall("joint")}


def origin_xyz(elem: ET.Element) -> tuple[float, float, float]:
    origin = elem.find("origin")
    assert origin is not None
    x, y, z = (float(v) for v in origin.attrib["xyz"].split())
    return x, y, z


def stl_z_range(path: Path) -> tuple[float, float]:
    data = path.read_bytes()
    count = struct.unpack("<I", data[80:84])[0]
    zmin, zmax = float("inf"), float("-inf")
    offset = 84
    for _ in range(count):
        record = data[offset : offset + 50]
        for v in range(3):
            z = struct.unpack("<f", record[20 + v * 12 : 24 + v * 12])[0]
            zmin, zmax = min(zmin, z), max(zmax, z)
        offset += 50
    return zmin, zmax


@pytest.mark.parametrize("name", RL_NAMES)
def test_arm_base_lifted_to_mount_top(name: str) -> None:
    joints = joints_by_name(load_urdf(name))
    for joint_name in ("r_aj_base", "l_aj_base"):
        _, _, z = origin_xyz(joints[joint_name])
        assert z == pytest.approx(0.690, abs=1e-9)


@pytest.mark.parametrize("name", RL_NAMES)
def test_body_link_uses_cropped_meshes(name: str) -> None:
    root = load_urdf(name)
    body = next(l for l in root.findall("link") if l.attrib["name"] == "body_link")
    for tag in ("visual", "collision"):
        mesh = body.find(f"{tag}/geometry/mesh")
        assert mesh is not None
        filename = mesh.attrib["filename"]
        assert filename.startswith("file://")
        # plate-cropped, vendor head housing removed, pillar cut at the head base
        # plate underside (mm in the name)
        assert filename.endswith(f"_cut_nohousing_top{round(gen.BODY_TOP_CROP_Z * 1000)}.stl")
        assert Path(filename[len("file://") :]).is_file()
    inertial = body.find("inertial")
    assert inertial is not None
    _, _, z = origin_xyz(inertial)
    assert z == pytest.approx(-0.008, abs=1e-9)


def test_cropped_collision_mesh_has_no_geometry_below_origin() -> None:
    zmin, zmax = stl_z_range(gen.ROOT / "generated" / "rl" / "meshes" / "body_link0_symp_cut.stl")
    assert zmin >= -1e-3
    assert zmax == pytest.approx(765.0, abs=0.5)


def test_body_housing_removed_and_pillar_cut_at_head_base_plate() -> None:
    """The vendor head housing (x up to +65 mm around the pillar) is gone and the
    60x60 pillar ends at the head_v1 base plate underside (mount 0.750 - plate
    0.020 = 0.730), so body_link and the head links cannot interpenetrate at
    rest (the old [body_link, head_mid] allowlist entry)."""
    import trimesh

    assert gen.BODY_TOP_CROP_Z == pytest.approx(0.750 - 0.020)
    top_mm = round(gen.BODY_TOP_CROP_Z * 1000)
    for stem in ("body_link0_symp_cut", "body_link0_visual_cut"):
        mesh = trimesh.load(gen.ROOT / "generated" / "rl" / "meshes" / f"{stem}_nohousing_top{top_mm}.stl",
                            force="mesh")
        (_, _, zmin), (_, _, zmax) = mesh.bounds
        assert zmin >= -1e-3 and zmax == pytest.approx(top_mm, abs=0.01)
        # above the housing band only the pillar footprint (60x60 mm) remains
        upper = mesh.vertices[mesh.vertices[:, 2] > 620.0]
        assert upper.size and abs(upper[:, 0]).max() <= 30.5 and abs(upper[:, 1]).max() <= 30.5


@pytest.mark.parametrize("name", RL_NAMES)
def test_every_link_has_inertial(name: str) -> None:
    """Massless frames get a token 1e-5 kg in the URDF (PhysX would give a
    massless body 1.0 kg, and 0 kg is rejected -> also 1.0 kg; a zero mass also
    breaks the PD models built on the same URDF)."""
    root = load_urdf(name)
    assert gen.TOKEN_MASS_KG == pytest.approx(1e-5)
    for link in root.findall("link"):
        inertial = link.find("inertial")
        assert inertial is not None, link.attrib["name"]
        mass = float(inertial.find("mass").attrib["value"])
        assert mass >= gen.TOKEN_MASS_KG


def hand_mass_kg(root: ET.Element, side: str) -> float:
    return sum(float(l.find("inertial/mass").attrib["value"]) for l in root.findall("link")
               if l.attrib["name"].startswith(f"{side}_hl_"))


def test_dg5f_hand_mass_matches_measurement() -> None:
    """DG-5F: vendor 1.685 kg scaled to the user's 1.763 kg scale measurement."""
    root = load_urdf("openarm_dg5f-m_bi")
    for side in ("r", "l"):
        assert hand_mass_kg(root, side) == pytest.approx(1.763, abs=1e-3)


@pytest.mark.parametrize("name", [n for n in RL_NAMES if n not in gen.HAND_MASS_TARGET_KG])
def test_other_hands_keep_vendor_mass(name: str) -> None:
    root = load_urdf(name)
    vendor = {"openarm_dg5f-m-short_bi": 1.5700, "openarm_dg5f-s_bi": 1.2084,
              "openarm_rh56f1_bi": 0.7077, "openarm_gripper_bi": 0.4222}
    for side in ("r", "l"):
        assert hand_mass_kg(root, side) == pytest.approx(vendor[name], abs=2e-3)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_attached(name: str) -> None:
    root = load_urdf(name)
    link_names = {l.attrib["name"] for l in root.findall("link")}
    assert {"head_base", "head_mid", "head_camera"} <= link_names

    joints = joints_by_name(root)
    mount = joints["head_j_mount"]
    assert mount.attrib["type"] == "fixed"
    parent, child = mount.find("parent"), mount.find("child")
    assert parent is not None and parent.attrib["link"] == "body_link"
    assert child is not None and child.attrib["link"] == "head_base"
    assert origin_xyz(mount) == pytest.approx((0.0, 0.0, gen.HEAD_MOUNT_Z))

    for joint_name in ("head_j_pan", "head_j_tilt"):
        assert joints[joint_name].attrib["type"] == "revolute"


@pytest.mark.parametrize("name", RL_NAMES)
def test_camera_view_frame(name: str) -> None:
    root = load_urdf(name)
    link_names = {l.attrib["name"] for l in root.findall("link")}
    assert "head_cam_view" in link_names

    joints = joints_by_name(root)
    view = joints["head_j_cam_view"]
    assert view.attrib["type"] == "fixed"
    parent, child = view.find("parent"), view.find("child")
    assert parent is not None and parent.attrib["link"] == "head_camera"
    assert child is not None and child.attrib["link"] == "head_cam_view"
    assert origin_xyz(view) == pytest.approx(gen.HEAD_CAM_VIEW_XYZ)

    manifest = load_manifest(name)
    frame = manifest["camera_view_frame"]
    assert frame["link"] == "head_cam_view"
    assert frame["parent_link"] == "head_camera"
    assert tuple(frame["xyz"]) == pytest.approx(gen.HEAD_CAM_VIEW_XYZ)
    assert tuple(frame["rpy"]) == pytest.approx(gen.HEAD_CAM_VIEW_RPY)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_joints_kinematic_only(name: str) -> None:
    manifest = load_manifest(name)
    control = manifest["control_joint_order"]
    kinematic = manifest["kinematic_joint_order"]
    assert not any(j.startswith("head_") for j in control)
    assert {"head_j_mount", "head_j_pan", "head_j_tilt"} <= set(kinematic)
    assert "head_j_mount" in manifest["fixed_joint_order"]


@pytest.mark.parametrize("name", RL_NAMES)
def test_control_joint_order_preserved(name: str) -> None:
    manifest = load_manifest(name)
    control = manifest["control_joint_order"]
    assert control[:7] == [f"r_aj_{i}" for i in range(1, 8)]
    assert all(j.startswith(("r_aj_", "r_hj_", "l_aj_", "l_hj_")) for j in control)


@pytest.mark.parametrize("name", RL_NAMES)
def test_head_mesh_paths_resolve(name: str) -> None:
    root = load_urdf(name)
    for link_name in ("head_base", "head_mid", "head_camera"):
        link = next(l for l in root.findall("link") if l.attrib["name"] == link_name)
        for mesh in link.iter("mesh"):
            filename = mesh.attrib["filename"]
            assert filename.startswith("file://")
            assert Path(filename[len("file://") :]).is_file()


TESOLLO_NAMES = [name for name in RL_NAMES if "dg5f" in name]
# assets whose link7 carries a replacement hand (the stock gripper keeps the stock link7)
HAND_MOUNT_NAMES = [name for name in RL_NAMES if "gripper" not in name]


@pytest.mark.parametrize("name", RL_NAMES)
def test_manifest_collider_policy(name: str) -> None:
    """collision 근사의 **기본값은 벤더 검증값(hull)**, 벗어나면 사유가 있어야 한다.

    09.05 규약은 반대였다(decomposition 기본 + OpenArm 링크만 hull). 09.09 에 뒤집었다 —
    벤더 자신의 Isaac 변환 설정이 `collider_type: convex_hull` 이고
    (vendor/delto_m_ros2/dg_isaacsim/.../config.yaml), decomposition 은 로봇 collision
    shape 을 731개까지 불려 접촉 임펄스가 관절 한계를 뚫는 원인이 됐다(hull 로 75개).
    ⇒ **새 자산은 아무것도 선언하지 않으면 hull 이다.** 근거는 docs/ROBOT_ASSET_SPEC.md §1.
    """
    manifest = load_manifest(name)
    policy = manifest["collision_approximation"]
    assert policy["default"] == gen.collider_default(name)

    deviation = gen.COLLIDER_DEVIATIONS.get(name)
    if deviation is None:
        assert policy["default"] == gen.DEFAULT_COLLIDER == "convex_hull", (
            f"{name}: 선언이 없으면 벤더 검증값(hull)이어야 한다")
    else:
        value, reason = deviation
        assert policy["default"] == value
        assert reason.strip(), f"{name}: 벤더 이탈에는 사유가 필요하다"

    # OpenArm 제작 링크(몸통·팔·머리·스톡 그리퍼)는 어느 자산에서든 hull 이다.
    hull = set(policy["convex_hull_links"])
    assert {"body_link", "r_al_1", "l_al_7", "head_base"} <= hull
    assert all(link.startswith(gen.OPENARM_HULL_LINK_PREFIXES) for link in hull)
    if "gripper" in name:
        assert {"r_hl_gripper_base", "l_hl_gripper_left_finger"} <= hull
    assert manifest["asset"] == f"{name}_rl"


def test_every_asset_declares_a_joint_limit_policy() -> None:
    """자산은 관절한계 방침을 **명시적으로** 선언해야 한다(빈 dict 허용, 미등록은 에러).

    벤더 URDF 한계는 자기관통 없는 가동 범위를 보장하지 않는다 — `_3/_4` 는 두 벤더 사본이
    똑같이 ±1.5708(대칭 ±90°)을 주는 placeholder 이고, `thumb_1` 하한은 손바닥을 0.67 mm
    파고드는 각도다. "아직 안 쟀다"와 "잴 필요가 없다"를 구분해 남기기 위한 강제다.
    """
    missing = [n for n in gen.SOURCES if n not in gen.JOINT_LIMIT_RESTRICTIONS]
    assert not missing, (
        f"JOINT_LIMIT_RESTRICTIONS 미등록: {missing} — 빈 dict 라도 선언할 것 "
        "(docs/ROBOT_ASSET_SPEC.md §2)")


def test_addressing_frames_are_detected_by_rule_not_by_name() -> None:
    """주소용 프레임 제거는 **규칙**이어야 한다 — 새 로봇에도 그대로 적용되도록.

    지오메트리 없음 + 들어오는 고정관절 1개 + 앞뒤 중 하나가 항등변환 → 제거 대상.
    이름으로 참조되는 프레임은 KEEP_ADDRESSING_FRAMES 가 보존한다.
    """
    import xml.etree.ElementTree as ET

    root = ET.parse(gen.OUT_DIR / "openarm_dg5f-m_bi_rl.urdf").getroot()
    links = {l.get("name") for l in root.findall("link")}
    # 제거됐어야 하는 것(항등변환 더미)
    assert not {"r_hl_mount", "l_hl_mount", "r_hl_palm_alias", "l_hl_palm_alias"} & links
    # 보존됐어야 하는 것(이름으로 참조된다)
    assert {"r_hl_palm_ee", "l_hl_palm_ee"} <= links
    # 재실행해도 더 제거할 것이 없어야 한다(멱등)
    assert gen.find_addressing_frames(root) == []
    # 사라진 링크의 고정변환을 자식이 흡수했는지 — 조인트 이름은 살아 있어야 한다
    joints = {j.get("name") for j in root.findall("joint")}
    assert "r_hj_mount" in joints, "변환을 가진 조인트 이름은 남아야 한다(fabric 생성기가 쓴다)"


def test_gripper_asset_action_and_mimic() -> None:
    manifest = load_manifest("openarm_gripper_bi")
    assert manifest["control_joint_order"] == (
        [f"r_aj_{i}" for i in range(1, 8)] + ["r_hj_gripper_1"]
        + [f"l_aj_{i}" for i in range(1, 8)] + ["l_hj_gripper_1"])
    joints = joints_by_name(load_urdf("openarm_gripper_bi"))
    for side in ("r", "l"):
        mimic = joints[f"{side}_hj_gripper_2"].find("mimic")
        assert mimic is not None and mimic.attrib["joint"] == f"{side}_hj_gripper_1"


@pytest.mark.parametrize("name", HAND_MOUNT_NAMES)
def test_arm_link7_meshes_support_hand_mount(name: str) -> None:
    """A link7 carrying a replacement hand must use the cropped visual mesh
    (stock-gripper motor removed) and the bolt-free flange-cut collision mesh
    (bolts insert into the adapter; kept in collision they would penetrate it
    under self-collision). A link7 with the stock gripper keeps stock meshes."""
    root = load_urdf(name)
    hand_links = gen.hand_mount_parent_links(root)
    assert hand_links, name
    for link in root.findall("link"):
        if link.attrib["name"] not in hand_links or link.attrib["name"] not in {"r_al_7", "l_al_7"}:
            continue
        visual_meshes = [
            m.attrib["filename"].rsplit("/", 1)[-1]
            for v in link.findall("visual")
            for m in v.iter("mesh")
        ]
        assert visual_meshes == ["link7_without_mat2_mat3_components00_03.dae"], (name, visual_meshes)
        collision = [
            m.attrib["filename"]
            for c in link.findall("collision")
            for m in c.iter("mesh")
        ]
        assert len(collision) == 1, (name, collision)
        assert collision[0].rsplit("/", 1)[-1] == "link7_flange_cut.stl", collision[0]
        assert Path(collision[0].removeprefix("file://")).is_file(), collision[0]


def test_link7_flange_collision_has_no_bolts() -> None:
    """The flange-cut collision mesh must end at the flange plate plane."""
    import crop_link7_flange as clf

    _, zmax_raw = stl_z_range(clf.OUTPUT)
    assert zmax_raw <= clf.FLANGE_TOP_RAW_MM + 1e-3
    assert abs(zmax_raw * 0.001 - 0.5585 - gen.LINK7_FLANGE_Z) < 1e-4


@pytest.mark.parametrize("name", TESOLLO_NAMES)
def test_adapter_plate_has_no_collision(name: str) -> None:
    """The adapter plate is fully enclosed (flange below, hand mount above);
    its collision geometry only produces resting-pose self-collision contacts."""
    root = load_urdf(name)
    for link in root.findall("link"):
        if not link.attrib["name"].endswith("_hl_adapter"):
            continue
        assert link.findall("visual"), link.attrib["name"]
        assert not link.findall("collision"), link.attrib["name"]


@pytest.mark.parametrize("name", TESOLLO_NAMES)
def test_tesollo_mount_flush_on_link7_flange(name: str) -> None:
    """The hand mount must sit on the link7 flange plane (cropped mesh top),
    with no gap left by the removed stock-gripper motor section."""
    joints = joints_by_name(load_urdf(name))
    mounts = [j for j in ("r_hj_mount", "l_hj_mount") if j in joints]
    assert mounts, name
    for joint_name in mounts:
        _, _, z = origin_xyz(joints[joint_name])
        assert abs(z - gen.LINK7_FLANGE_Z) < 1e-9, (name, joint_name, z)
