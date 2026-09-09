"""Tests for the consolidated Fabrics URDF generator.

Each builder already enforces its FK gate (raises on mismatch), so generation
succeeding is the core assertion; the rest checks the fabric-code contracts.
"""

from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import gen_fabric_urdfs as gen  # noqa: E402

EXPECTED_CSPACE = {
    "openarm_dg5f-m_bi_right": 27,
    "openarm_dg5f-m_bi_left": 27,
    "openarm_dg5f-m-short_bi_right": 27,
    "openarm_dg5f-m-short_bi_left": 27,
    "openarm_dg5f-s_bi_right": 27,
    "openarm_dg5f-s_bi_left": 27,
    "openarm_gripper_bi_right": 7,
    "openarm_gripper_bi_left": 7,
    "openarm_rh56f1_bi": 26,
}
TESOLLO_VARIANTS = [n for n in EXPECTED_CSPACE if "dg5f" in n]
GRIPPER_VARIANTS = [n for n in EXPECTED_CSPACE if "gripper" in n]


@pytest.fixture(scope="module")
def outputs() -> dict[str, Path]:
    return {name: build() for name, build in gen.VARIANTS.items()}


def test_all_variants_generate_with_fk_gate(outputs):
    assert set(outputs) == set(EXPECTED_CSPACE)
    for path in outputs.values():
        assert path.is_file()


@pytest.mark.parametrize("name", list(EXPECTED_CSPACE))
def test_cspace_dimension(outputs, name):
    joints = gen.parse_urdf(outputs[name])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert len(revolute) == EXPECTED_CSPACE[name], revolute


@pytest.mark.parametrize("name", TESOLLO_VARIANTS + GRIPPER_VARIANTS)
def test_fabric_convention_frames_present(outputs, name):
    """palm helpers, palm_link, and fingertip frames are fabric-code contracts."""
    root = ET.parse(outputs[name]).getroot()
    joint_names = {j.get("name") for j in root.iter("joint")}
    link_names = {l.get("name") for l in root.iter("link")}
    assert gen.PALM_HELPER_JOINTS <= joint_names
    assert "palm_link" in link_names
    for index in range(1, 6):
        assert f"rl_dg_{index}_tip" in link_names


@pytest.mark.parametrize("name", GRIPPER_VARIANTS)
def test_gripper_hand_is_frozen(outputs, name):
    joints = gen.parse_urdf(outputs[name])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert all(n.startswith("openarm_right_joint") for n in revolute)


def test_rh56f1_frames(outputs):
    root = ET.parse(outputs["openarm_rh56f1_bi"]).getroot()
    link_names = {l.get("name") for l in root.iter("link")}
    for side in ("r", "l"):
        for key in gen.RH_PALM_AXIS:
            assert f"ps_{side}_{key}" in link_names
        for finger in gen.FINGERS:
            assert f"{side}_hl_{finger}_tip" in link_names


@pytest.mark.parametrize("name", list(EXPECTED_CSPACE))
def test_manifest_matches_urdf(outputs, name):
    import yaml

    manifest = yaml.safe_load((outputs[name].parent / f"{name}_manifest.yaml").read_text())
    joints = gen.parse_urdf(outputs[name])
    revolute = [n for n, j in joints.items() if j["type"] == "revolute"]
    assert manifest["cspace_dim"] == len(revolute)
    assert manifest["cspace_joint_order"] == revolute
    assert manifest["robot_name"] == name


@pytest.mark.parametrize("name", list(EXPECTED_CSPACE))
def test_masses_vendor_or_helper_token(outputs, name):
    """Real links carry the RL asset's inertials, fabric-only frames the 1e-5 kg
    token (never 0: the same URDF feeds PD-controlled models)."""
    root = ET.parse(outputs[name]).getroot()
    masses = {l.get("name"): float(l.find("inertial/mass").get("value")) for l in root.iter("link")}
    assert all(m >= gen.HELPER_MASS_KG for m in masses.values()), masses
    if "gripper" in name:
        assert masses["palm_link"] == pytest.approx(0.4222, abs=2e-3)  # base + 2 jaws + tcp token
        assert masses["tesollo_right_rl_dg_1_1"] == gen.HELPER_MASS_KG  # frozen fake hand frame
    elif "dg5f" in name:
        assert masses["palm_link"] > 0.5  # adapter + base + palm lumped onto the palm frame
        assert masses["tesollo_right_rl_dg_2_1"] > 0.01
        assert masses["palm_x"] == gen.HELPER_MASS_KG
    else:
        assert masses["r_hl_palm_2"] > 0 and masses["ps_r_x"] == gen.HELPER_MASS_KG
    if name == "openarm_dg5f-m_bi_right":
        hand = sum(m for n, m in masses.items() if n.startswith(("tesollo_right_rl_dg_", "rl_dg_")) or n == "palm_link")
        assert hand == pytest.approx(1.763, abs=2e-3)


def test_directory_equals_filename_convention(outputs):
    """hdgp's get_robot_urdf_path requires <dir>/<dir>.urdf."""
    for name, path in outputs.items():
        assert path.parent.name == name and path.name == f"{name}.urdf"
        assert (gen.HDGP_FABRIC_DIR / name).is_dir(), "hdgp consumer dir missing"
