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


#: 우리가 추가한 링크 — **벤더 손 질량 합에 섞지 않는다**(손이 아니라 팔↔손 어댑터다).
#:   `*_hl_flange_adapter` (09.10): dg5f-m-short <-> OpenArm 플랜지 어댑터 판, 0.031642 kg.
#:   질량 자체는 벤더값이지만(vendor/dg5f_m_short_adaptor/usd) 벤더가 고시한 **손** 질량
#:   (1.5700 kg)에는 안 들어가므로, 합계에 섞으면 드리프트 검사가 깨진다.
NON_VENDOR_HAND_LINKS = ("_hl_flange_adapter",)


def hand_mass_kg(root: ET.Element, side: str) -> float:
    return sum(float(l.find("inertial/mass").attrib["value"]) for l in root.findall("link")
               if l.attrib["name"].startswith(f"{side}_hl_")
               and not any(l.attrib["name"].endswith(x) for x in NON_VENDOR_HAND_LINKS))


def test_dg5f_hand_mass_matches_measurement() -> None:
    """DG-5F: vendor 1.685 kg scaled to the user's 1.763 kg scale measurement."""
    root = load_urdf("openarm_dg5f-m_bi")
    for side in ("r", "l"):
        assert hand_mass_kg(root, side) == pytest.approx(1.763, abs=1e-3)


@pytest.mark.parametrize("name", [n for n in RL_NAMES if n not in gen.HAND_MASS_TARGET_KG])
def test_other_hands_keep_vendor_mass(name: str) -> None:
    root = load_urdf(name)
    # ★09.10 `-tl` 은 short 와 **같은 손**이고 thumb_1 만 fixed 로 용접했다. 용접은
    #   링크·질량을 옮기지 않으므로 손 질량이 같아야 한다(실측 1.5700 kg 로 확인).
    #   여기 값이 갈리면 용접이 링크를 병합했다는 뜻이라 실패해야 맞다.
    vendor = {"openarm_dg5f-m-short_bi": 1.5700, "openarm_dg5f-m-short-tl_bi": 1.5700,
              "openarm_dg5f-s_bi": 1.2084,
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
    """자산은 손 관절 사양 출처를 **명시적으로** 선언해야 한다(None 허용, 미등록은 에러).

    벤더가 같은 값을 두 곳에서 다르게 준다 — 드라이버 URDF 의 effort 7.5 N·m / velocity π 는
    placeholder 로 보이고, 사용자 매뉴얼 §3.1 은 peak 2.0 N·m / 75 RPM 이다. 가동범위도 7개
    관절이 어긋난다. "매뉴얼과 대조했다"와 "아직 안 했다"를 구분해 남기기 위한 강제다.
    """
    missing = [n for n in gen.SOURCES if n not in gen.HAND_JOINT_SPEC]
    assert not missing, (
        f"HAND_JOINT_SPEC 미등록: {missing} — None 이라도 선언할 것 "
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
    """The hand chain must start on the link7 flange plane (cropped mesh top),
    with no gap left by the removed stock-gripper motor section.

    ★09.10 The joint that sits on the flange is no longer always ``*_hj_mount``:
    dg5f-m-short inserts an adapter plate, so the flange joint is
    ``*_hj_flange_adapter`` and ``*_hj_mount`` carries only the plate thickness.
    Check **the joint whose parent is the arm flange link**, whatever it is named,
    and separately assert that any intervening links only add thickness (z-only,
    no rotation) so the mount stays flush and axially aligned.
    """
    root = load_urdf(name)
    joints = joints_by_name(root)
    parent_of = {j.find("child").get("link"): j for j in root.findall("joint")}
    checked = []
    for side in ("r", "l"):
        mount = joints.get(f"{side}_hj_mount")
        if mount is None:
            continue
        # 마운트에서 팔 플랜지까지 거슬러 올라가며 중간 링크의 변환을 모은다.
        chain, cur = [mount], mount.find("parent").get("link")
        while cur not in gen.ARM_FLANGE_LINKS and cur in parent_of:
            chain.append(parent_of[cur])
            cur = parent_of[cur].find("parent").get("link")
        assert cur in gen.ARM_FLANGE_LINKS, (name, side, "팔 플랜지까지 못 올라갔다")
        flange_joint = chain[-1]
        _, _, z = origin_xyz(flange_joint)
        assert abs(z - gen.LINK7_FLANGE_Z) < 1e-9, (name, flange_joint.get("name"), z)
        # 중간(어댑터) 조인트들은 두께만 더해야 한다 — 회전·횡변위가 있으면 마운트가 틀어진다.
        for j in chain[:-1]:
            x, y, zz = origin_xyz(j)
            rpy = (j.find("origin").get("rpy") or "0 0 0") if j.find("origin") is not None else "0 0 0"
            assert abs(x) < 1e-9 and abs(y) < 1e-9, (name, j.get("name"), "횡변위", x, y)
            assert all(abs(float(v)) < 1e-9 for v in rpy.split()), (name, j.get("name"), "회전", rpy)
            assert zz > 0.0, (name, j.get("name"), "두께가 0 이하", zz)
        checked.append(side)
    assert checked, name


# ══════════════════════════════════════════════════════════════════════════════
# 09.09 손 관절 사양을 **벤더 공식 매뉴얼**로 설정한다 (DG5F_User_Manual v1.1.4)
#   벤더가 같은 값을 두 곳에서 다르게 준다: 드라이버 URDF 와 사용자 매뉴얼 §3.1/§3.3.1 이
#   **7개 관절·effort·velocity 에서 어긋난다**. 매뉴얼이 공식 제품 사양이므로 그쪽이 기준이다.
#     effort   URDF 7.5 N·m  vs  매뉴얼 peak 2.0 N·m      (3.75배 과대)
#     velocity URDF 3.142 rad/s vs 매뉴얼 75 RPM=7.854     (2.5배 과소)
#     thumb_1  URDF -22~+51°  vs  매뉴얼 -22~+77°          (엄지 대향 26° 손실)
# ══════════════════════════════════════════════════════════════════════════════
def test_hand_spec_table_is_declared_per_asset():
    """미등록 자산은 빌드 에러여야 한다 — 조용히 벤더 URDF placeholder 를 쓰면 안 된다."""
    src = (TOOLS_DIR / "generate_rl_urdf.py").read_text(encoding="utf-8")
    assert "HAND_JOINT_SPEC" in src, "손 관절 사양 표가 없다"
    assert "HAND_JOINT_SPEC 에 항목이 없다" in src, "미등록 자산이 빌드 에러가 아니다"


def test_manual_effort_and_velocity_land_in_generated_urdf():
    """생성 URDF 의 손 20관절 effort=7.5 · velocity=pi (벤더 Isaac USD, 09.10 전환)."""
    import math
    import re
    p = TOOLS_DIR.parent / "generated" / "rl" / "openarm_dg5f-m_bi_rl.urdf"
    if not p.is_file():
        import pytest
        pytest.skip(f"자산 미생성: {p}")
    t = p.read_text(encoding="utf-8")
    bad = []
    for m in re.finditer(r'<joint name="([rl]_hj_[^"]+)"[^>]*>(.*?)</joint>', t, re.S):
        a = re.search(r"<limit([^/>]*)", m.group(2))
        if not a:
            continue
        eff = float(re.search(r'effort="([^"]+)"', a.group(1)).group(1))
        vel = float(re.search(r'velocity="([^"]+)"', a.group(1)).group(1))
        if abs(eff - 7.5) > 1e-6 or abs(vel - math.pi) > 1e-3:
            bad.append(f"{m.group(1)} eff={eff} vel={vel}")
    assert not bad, f"벤더 USD effort/velocity 가 안 실렸다: {bad[:5]}"


def test_manual_joint_ranges_land_in_generated_urdf():
    """**벤더 Isaac USD** `dg5f_right` 가동범위가 그대로 실려야 한다(09.10 전환).

    ★`thumb_2` 만 부호가 반대다 — 벤더 우손 USD 는 -155~0, 좌손은 0~+155 다.
      좌우 부호 규약이 다른 관절이 있으므로 표에서 추론하지 말 것(docs §2-1b).
    ★과신전(`_3/_4` 음수)은 **자산에서 허용**한다. 실기가 실제로 되기 때문이다.
      정책이 그걸 명령하지 못하게 막는 것은 프로필의 `hand_action_limit_override` 몫이고,
      물리 한계와 액션 범위는 서로 다른 층이다(09.09 사용자 확정).
    """
    import math
    import re
    p = TOOLS_DIR.parent / "generated" / "rl" / "openarm_dg5f-m_bi_rl.urdf"
    if not p.is_file():
        import pytest
        pytest.skip(f"자산 미생성: {p}")
    expect_deg = {
        "thumb_1": (-22, 77), "thumb_2": (-155, 0), "thumb_3": (-90, 90), "thumb_4": (-90, 90),
        "index_1": (-31, 20), "index_2": (0, 115), "index_3": (-90, 90), "index_4": (-90, 90),
        "middle_1": (-25, 25), "middle_2": (0, 115), "middle_3": (-90, 90), "middle_4": (-90, 90),
        "ring_1": (-15, 32), "ring_2": (0, 110), "ring_3": (-90, 90), "ring_4": (-90, 90),
        "pinky_1": (0, 60), "pinky_2": (-15, 90), "pinky_3": (-90, 90), "pinky_4": (-90, 90),
    }
    t = p.read_text(encoding="utf-8")
    bad = []
    for m in re.finditer(r'<joint name="r_hj_([a-z]+_\d)"[^>]*>(.*?)</joint>', t, re.S):
        key = m.group(1)
        if key not in expect_deg:
            continue
        a = re.search(r"<limit([^/>]*)", m.group(2)).group(1)
        lo = math.degrees(float(re.search(r'lower="([^"]+)"', a).group(1)))
        hi = math.degrees(float(re.search(r'upper="([^"]+)"', a).group(1)))
        el, eh = expect_deg[key]
        if abs(lo - el) > 0.5 or abs(hi - eh) > 0.5:
            bad.append(f"{key}: {lo:+.1f}~{hi:+.1f} (기대 {el:+.0f}~{eh:+.0f})")
    assert len(expect_deg) and not bad, f"매뉴얼 가동범위 불일치: {bad}"


def test_thumb_opposition_range_is_restored():
    """★엄지 대향(`thumb_1`)이 잘려 있으면 인벨롭 파지가 원리적으로 안 된다.

    09.09 까지 ±9.2° 로 잘려 있었다(매뉴얼 -22~+77° 의 19%). 엄지가 손가락 쪽으로
    돌아오지 못하면 감싸 쥘 수 없다 — 이 테스트가 그 회귀를 막는다.
    """
    import math
    import re
    p = TOOLS_DIR.parent / "generated" / "rl" / "openarm_dg5f-m_bi_rl.urdf"
    if not p.is_file():
        import pytest
        pytest.skip(f"자산 미생성: {p}")
    t = p.read_text(encoding="utf-8")
    m = re.search(r'<joint name="r_hj_thumb_1"[^>]*>(.*?)</joint>', t, re.S)
    a = re.search(r"<limit([^/>]*)", m.group(1)).group(1)
    span = math.degrees(float(re.search(r'upper="([^"]+)"', a).group(1))
                        - float(re.search(r'lower="([^"]+)"', a).group(1)))
    assert span > 90.0, f"엄지 대향 가동폭이 {span:.1f}° 뿐이다 — 매뉴얼은 99°"


def test_left_hand_is_the_mirror_of_the_right():
    """★왼손이 오른손 값을 그대로 받으면 조용히 반대로 움직인다.

    사양표는 매뉴얼 §3.3.1 **오른손** 기준이다. 벌림/대향은 좌우 부호가 뒤집히고
    (l = -hi..-lo) 굴곡은 그대로다. 생성기는 그 관계를 원본 URDF 에서 판정해야 한다.
    09.09 에 표를 그대로 양손에 적용해 `l_hj_thumb_2` 가 [0,+3.14]→[-2.71,0] 로
    **부호가 뒤집힌 자산**을 한 번 만들었다 — 이 테스트가 그 회귀를 막는다.
    """
    import math
    import re
    p = TOOLS_DIR.parent / "generated" / "rl" / "openarm_dg5f-m_bi_rl.urdf"
    if not p.is_file():
        pytest.skip(f"자산 미생성: {p}")
    t = p.read_text(encoding="utf-8")
    lim = {}
    for m in re.finditer(r'<joint name="([rl]_hj_[a-z]+_\d)"[^>]*>(.*?)</joint>', t, re.S):
        a = re.search(r"<limit([^/>]*)", m.group(2)).group(1)
        lim[m.group(1)] = (float(re.search(r'lower="([^"]+)"', a).group(1)),
                           float(re.search(r'upper="([^"]+)"', a).group(1)))
    # 벌림/대향은 미러, 굴곡은 동일 — 어느 쪽이든 **부호가 뒤집히지 않았는지**가 핵심이다.
    bad = []
    for name, (llo, lhi) in lim.items():
        if not name.startswith("l_"):
            continue
        rlo, rhi = lim["r_" + name[2:]]
        same = abs(llo - rlo) < 1e-6 and abs(lhi - rhi) < 1e-6
        mirror = abs(llo - (-rhi)) < 1e-6 and abs(lhi - (-rlo)) < 1e-6
        if not (same or mirror):
            bad.append(f"{name} [{math.degrees(llo):+.0f},{math.degrees(lhi):+.0f}] vs "
                       f"r [{math.degrees(rlo):+.0f},{math.degrees(rhi):+.0f}]")
    assert not bad, f"좌우 관계가 동일도 미러도 아니다: {bad}"
    # `thumb_2` 는 굴곡인데 좌우 부호가 반대인 관절이다 — 왼손 굴곡은 **양수** 방향이어야 한다.
    assert lim["l_hj_thumb_2"][1] > 1.0, \
        f"왼손 엄지 굴곡 방향이 뒤집혔다: {lim['l_hj_thumb_2']}"
    assert lim["r_hj_thumb_2"][0] < -1.0, \
        f"오른손 엄지 굴곡 방향이 뒤집혔다: {lim['r_hj_thumb_2']}"


def test_hand_spec_tables_match_the_vendor_isaac_usd():
    """`_DG5F_M_RANGE_DEG` / `_DG5F_S_RANGE_DEG` 가 **벤더 Isaac USD 와 일치**한다.

    표를 손으로 베낀 값이 아니라는 유일한 보증이다. 벤더가 자산을 갱신하면 여기서 깨진다.
    출처: `repo/tesollo/tesollo_model/dg5f*/usd/*_right/configuration/*_physics.usd` (docs §0).
    """
    import math
    import re as _re

    import pytest
    Usd = pytest.importorskip("pxr.Usd", reason="USD 미설치 — 벤더 대조 생략")

    vendor_root = TOOLS_DIR.parents[1] / "repo" / "tesollo" / "tesollo_model"
    if not vendor_root.is_dir():
        pytest.skip(f"벤더 레포 없음: {vendor_root}")

    import importlib.util
    spec = importlib.util.spec_from_file_location("_gen", TOOLS_DIR / "generate_rl_urdf.py")
    gen = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(gen)

    pat = _re.compile(r"^(?:rj_dg|joint)_([1-5])_([1-4])$")
    fingers = {"1": "thumb", "2": "index", "3": "middle", "4": "ring", "5": "pinky"}
    cases = [("dg5f/usd/dg5f_right/configuration/dg5f_right_physics.usd", gen._DG5F_M_RANGE_DEG),
             ("dg5fs/usd/dg5fs_right/configuration/dg5fs_right_physics.usd", gen._DG5F_S_RANGE_DEG)]
    for rel, table in cases:
        usd = vendor_root / rel
        if not usd.is_file():
            pytest.skip(f"벤더 자산 없음: {usd}")
        stage = Usd.Stage.Open(str(usd))
        seen, bad = set(), []
        max_force, max_vel = set(), set()
        for prim in stage.Traverse():
            m = pat.match(prim.GetName())
            if not m or prim.GetName() in seen:
                continue
            g = lambda k: (prim.GetAttribute(k).Get() if prim.GetAttribute(k) else None)
            if g("drive:angular:physics:stiffness") is None:
                continue
            seen.add(prim.GetName())
            key = f"{fingers[m.group(1)]}_{m.group(2)}"
            want = table[key]
            got = (g("physics:lowerLimit"), g("physics:upperLimit"))
            if abs(got[0] - want[0]) > 0.05 or abs(got[1] - want[1]) > 0.05:
                bad.append(f"{key}: 표 {want} vs USD ({got[0]:.2f}, {got[1]:.2f})")
            max_force.add(round(g("drive:angular:physics:maxForce"), 3))
            max_vel.add(round(g("physxJoint:maxJointVelocity"), 1))
        assert len(seen) == 20, f"{rel}: 관절 20개가 아니라 {len(seen)}개"
        assert not bad, f"{rel} 표가 벤더 USD 와 다르다: {bad}"
        assert max_force == {gen.HAND_PEAK_TORQUE_NM}, f"{rel}: maxForce {max_force}"
        assert max_vel == {round(math.degrees(gen.HAND_NO_LOAD_RAD_S), 1)}, f"{rel}: maxJointVelocity {max_vel}"


def test_short_flange_adapter_is_present_and_declared_non_vendor() -> None:
    """dg5f-m-short 만 플랜지 어댑터 판을 갖고, 질량·관성은 **벤더값**이어야 한다.

    출처: `vendor/dg5f_m_short_adaptor/usd/dg5f_m_short_adaptor.usda` (Tesollo, Fusion 덤프)
        physics:mass 0.031642 kg (PLA 1.24 g/cm^3) · centerOfMass (0,0,0.004979)
        diagonalInertia (8.695287e-06, 8.695495e-06, 1.68636e-05)
    ★09.10 첫 판은 재질을 몰라 알루미늄으로 0.06912 kg 을 추정해 넣었다 — 2.18배 과대였다.
      이 테스트가 그 종류의 추정 복귀를 막는다.
    """
    for name in ("openarm_dg5f-m_bi", "openarm_dg5f-s_bi"):
        links = {l.attrib["name"] for l in load_urdf(name).findall("link")}
        assert not any(n.endswith("_hl_flange_adapter") for n in links), \
            f"{name} 은 어댑터가 없어야 한다(short 전용)"
    root = load_urdf("openarm_dg5f-m-short_bi")
    for side in ("r", "l"):
        link = [l for l in root.findall("link") if l.attrib["name"] == f"{side}_hl_flange_adapter"]
        assert link, f"{side}_hl_flange_adapter 가 없다"
        m = float(link[0].find("inertial/mass").attrib["value"])
        assert m == pytest.approx(0.031642, abs=1e-6), m
        ine = link[0].find("inertial/inertia")
        assert float(ine.get("ixx")) == pytest.approx(8.695287e-06, rel=1e-6)
        assert float(ine.get("izz")) == pytest.approx(1.68636e-05, rel=1e-6)
        com = [float(v) for v in link[0].find("inertial/origin").get("xyz").split()]
        assert com == pytest.approx([0.0, 0.0, 0.004979], abs=1e-9)
    # 판 두께 10mm 가 마운트 체인에 정확히 더해져야 한다.
    joints = joints_by_name(root)
    for side in ("r", "l"):
        _, _, z = origin_xyz(joints[f"{side}_hj_mount"])
        assert z == pytest.approx(0.010, abs=1e-9), (side, z)
