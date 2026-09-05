"""Tests for the resting-pose self-collision audit.

Slow (~2 min for all assets): builds proximity indices for every collision
mesh. Deselect with `-k "not audit"` during quick iterations.
"""

from __future__ import annotations

import copy
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

TOOLS_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(TOOLS_DIR))

import audit_self_collision as audit  # noqa: E402
import generate_rl_urdf as gen  # noqa: E402

RL_NAMES = list(gen.SOURCES.keys())


@pytest.fixture(scope="module", autouse=True)
def generated_outputs(tmp_path_factory) -> None:
    # temp dir: never strip the audited filter pairs from generated/rl/ manifests
    gen.OUT_DIR = tmp_path_factory.mktemp("generated_rl_audit")
    assert gen.main(["--skip-audit"]) == 0


@pytest.mark.parametrize("name", RL_NAMES)
def test_audit_passes_for_generated_assets(name: str) -> None:
    """Zero pose AND every registered task-home pose must be raw-clean."""
    for pose_name, findings in audit.audit_asset(gen.OUT_DIR / f"{name}_rl.urdf").items():
        failures = [f for f in findings if f.verdict == "FAIL"]
        assert not failures, [
            (pose_name, f.link_a, f.link_b, f.hull_depth_m, f.raw_depth_m, f.reason)
            for f in failures
        ]


def test_left_wrist_home_pair_is_filtered() -> None:
    """Regression: l_al_5<->l_al_7 only approaches at the LEFT task home.

    Zero pose leaves it 19mm clear, so a zero-only audit misses it and the
    built USD lacked the filter - measured 5.4kN phantom contact at the left
    gripper home (j7 flex narrows the raw clearance to 3.2mm). The registered
    task-home poses in audit_poses.yaml must catch it, and the union the
    generator writes to the manifest must carry it on BOTH sides (mirror
    symmetrization). (On the retired sensor asset the hand-mounted right
    link7 also produced a zero-pose hull WARN; the stock-gripper link7 does
    not, so that mechanism is no longer asserted here.)
    """
    urdf_path = gen.OUT_DIR / "openarm_gripper_bi_rl.urdf"
    links = audit.urdf_link_names(urdf_path)

    poses = audit.load_audit_poses(urdf_path.stem)
    legacy = poses.get("left_gripper_legacy_home")
    assert legacy and legacy["l_aj_7"] == pytest.approx(1.3563)
    home_findings = audit.audit_urdf(urdf_path, legacy)
    warned = {(f.link_a, f.link_b) for f in home_findings if f.verdict == "WARN"}
    assert ("l_al_5", "l_al_7") in warned

    all_findings = [f for findings in audit.audit_asset(urdf_path).values() for f in findings]
    union = audit.filtered_pairs(all_findings, links)
    assert ("l_al_5", "l_al_7") in union and ("r_al_5", "r_al_7") in union


def test_filtered_pairs_mirror_requires_existing_links() -> None:
    """Mirroring must not invent pairs for links the asset does not have."""
    finding = audit.Finding("body_link", "l_hl_gripper_base", 0.001, None, "WARN", "test")
    links = {"body_link", "l_hl_gripper_base", "r_hl_base"}  # no r_hl_gripper_base
    assert audit.filtered_pairs([finding], links) == [("body_link", "l_hl_gripper_base")]


def test_audit_detects_reintroduced_penetration(tmp_path: Path) -> None:
    """Regression guard: un-fixing the mount stack must FAIL the audit.

    Reverts both self-collision fixes on the right arm of the bi_s asset:
    stock link7 collision (gripper motor + bolts) and the adapter plate
    collision. The motor section then penetrates the hand mount again.
    """
    tree = ET.parse(gen.OUT_DIR / "openarm_dg5f-s_bi_rl.urdf")
    root = tree.getroot()
    stock_collision = str(
        gen.ASSET_ROOTS["openarm_description"] / "meshes" / "arm" / "v10" / "collision" / "link7_symp.stl"
    )
    for link in root.findall("link"):
        if link.get("name") == "r_al_7":
            mesh = link.find("collision/geometry/mesh")
            assert mesh is not None
            mesh.set("filename", f"file://{stock_collision}")
        if link.get("name") == "r_hl_adapter":
            visual = link.find("visual")
            assert visual is not None and link.find("collision") is None
            collision = copy.deepcopy(visual)
            collision.tag = "collision"
            mesh = collision.find("geometry/mesh")
            assert mesh is not None
            mesh.set(
                "filename",
                mesh.get("filename", "").replace("/visual/", "/collision/").replace(
                    "rl_dg_mount.dae", "rl_dg_mount_c.STL"
                ),
            )
            link.append(collision)

    broken = tmp_path / "broken.urdf"
    tree.write(broken)
    failures = [f for f in audit.audit_urdf(broken) if f.verdict == "FAIL"]
    assert failures, "audit missed the reintroduced mount-stack penetration"
    failing_links = {link for f in failures for link in (f.link_a, f.link_b)}
    assert "r_al_7" in failing_links
