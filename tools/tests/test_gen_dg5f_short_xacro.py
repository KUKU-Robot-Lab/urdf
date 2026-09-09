"""The generated DG5F-short fragments must stay a faithful, resolvable copy."""
from __future__ import annotations

import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import gen_dg5f_short_xacro as gen  # noqa: E402
import generate_rl_urdf as rl  # noqa: E402

SIDES = list(gen.SIDE_CONFIG)


@pytest.fixture(scope="module")
def outputs() -> dict[str, Path]:
    return gen.generate_all()


def vendor_root(side: str) -> ET.Element:
    return ET.parse(gen.SOURCE_DIR / f"dg5f_{side}_short.urdf").getroot()


@pytest.mark.parametrize("side", SIDES)
def test_mesh_uris_resolve_through_asset_roots(outputs, side):
    """Every rewritten URI must land on a file ASSET_ROOTS can find."""
    root = ET.parse(outputs[side]).getroot()
    uris = [m.get("filename") for m in root.iter("mesh")]
    assert uris, "no meshes in the generated fragment"
    for uri in uris:
        package, _, relative = uri.removeprefix("package://").partition("/")
        assert package in rl.ASSET_ROOTS, uri
        assert (rl.ASSET_ROOTS[package] / relative).is_file(), uri


@pytest.mark.parametrize("side", SIDES)
def test_kinematics_copied_verbatim(outputs, side):
    """Only the header is added: names, types, origins and limits are the vendor's."""
    generated = ET.parse(outputs[side]).getroot()
    vendor = vendor_root(side)

    def joints(root):
        return {j.get("name"): (
            j.get("type"),
            j.find("origin").get("xyz") if j.find("origin") is not None else None,
            (j.find("limit").get("lower"), j.find("limit").get("upper"))
            if j.find("limit") is not None else None,
        ) for j in root.iter("joint")}

    def masses(root):
        return {l.get("name"): l.find("inertial/mass").get("value")
                for l in root.iter("link") if l.find("inertial") is not None}

    added = {gen.SIDE_CONFIG[side]["adapter_joint"]}
    assert set(joints(generated)) - set(joints(vendor)) == added
    for name, value in joints(vendor).items():
        assert joints(generated)[name] == value, name
    assert masses(generated) == masses(vendor)


@pytest.mark.parametrize("side", SIDES)
def test_short_palm_offset(outputs, side):
    """The short variant's defining geometry: palm 47.8mm closer to the flange."""
    root = ET.parse(outputs[side]).getroot()
    prefix = "rj" if side == "right" else "lj"
    palm = next(j for j in root.iter("joint") if j.get("name") == f"{prefix}_dg_palm")
    assert palm.find("origin").get("xyz") == "0 0 0.022"


@pytest.mark.parametrize("side", SIDES)
def test_header_attaches_side_base_link(outputs, side):
    """eef/tesollo_*_wrapper.xacro and generate_rl_urdf.py address these names."""
    root = ET.parse(outputs[side]).getroot()
    link_prefix = gen.SIDE_CONFIG[side]["link_prefix"]
    assert any(l.get("name") == f"{side}_base_link" for l in root.iter("link"))
    joint = next(j for j in root.iter("joint")
                 if j.get("name") == gen.SIDE_CONFIG[side]["adapter_joint"])
    assert joint.find("parent").get("link") == f"{side}_base_link"
    assert joint.find("child").get("link") == f"{link_prefix}_dg_mount"


@pytest.mark.parametrize("side", SIDES)
def test_hand_mass_is_the_cad_release(outputs, side):
    """CAD release (1.570 kg), not the driver copies' rounded 1.685 kg."""
    root = ET.parse(outputs[side]).getroot()
    total = sum(float(l.find("inertial/mass").get("value"))
                for l in root.iter("link") if l.find("inertial") is not None)
    assert total == pytest.approx(1.570, abs=1e-3)
