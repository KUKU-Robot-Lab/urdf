#!/usr/bin/env python3
"""Audit generated RL URDFs for resting-pose self-collision penetrations.

With articulation self-collision enabled, PhysX collides every NON-adjacent
link pair (pairs directly connected by a joint are always excluded). Any pair
that already interpenetrates at the rest pose produces phantom repulsion
forces: the robot moves, jitters, or gets pushed past joint limits with zero
action. This audit reproduces the PhysX pairing rule and measures penetration
for every non-adjacent pair, in two modes:

- raw:  signed distance against the actual collision meshes. A raw penetration
        is a real asset defect and FAILS the audit.
- hull: convex hull per collision element - the worst-case importer
        approximation (collider_type "convex_hull"), which fills concave
        pockets. Hull-only penetrations (raw clear) WARN: they are safe when
        the USD is imported with "convex_decomposition" (tools/build_usd.py
        enforces this). Notable ones are documented with reasons in
        tools/self_collision_allowlist.yaml.

Pairs whose raw meshes cannot be made watertight (even after merging duplicate
vertices and filling holes) cannot be disproven and FAIL conservatively.

Zero pose alone is a blind spot: a pair can be clear at zero but approach
within cooking-inflation range only at a task home (measured: the sensor
asset's left wrist home flex narrows l_al_5<->l_al_7 from 19mm to 3.2mm).
Task homes are registered in tools/audit_poses.yaml and audited in addition
to zero pose; WARN pairs from every pose are unioned into the manifest
filter list, symmetrized across the left/right arm mirror.

Run standalone (all generated RL assets, zero pose + registered poses):

    python3 tools/audit_self_collision.py [asset names...] [--pose j=rad ...]

generate_rl_urdf.py also runs this audit after generating each asset.
"""

from __future__ import annotations

import argparse
import fnmatch
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RL_DIR = ROOT / "generated" / "rl"
ALLOWLIST_PATH = Path(__file__).resolve().parent / "self_collision_allowlist.yaml"
POSES_PATH = Path(__file__).resolve().parent / "audit_poses.yaml"

# Penetrations below this are treated as contact-solver noise, not defects.
PENETRATION_LIMIT_M = 0.0005
# Cap on query points per mesh to keep the audit fast; deterministic stride.
MAX_QUERY_POINTS = 4000


@dataclass
class Finding:
    link_a: str
    link_b: str
    hull_depth_m: float
    raw_depth_m: float | None  # None = raw check unavailable (non-watertight)
    verdict: str  # "FAIL" | "WARN"
    reason: str


@dataclass
class LocalGeom:
    """One collision element's geometry in its local frame, cached per mesh."""

    raw_mesh: object
    raw_query: object | None  # ProximityQuery; None when not watertight
    hull_mesh: object
    hull_query: object
    bounds: np.ndarray  # (2, 3) local AABB


_GEOM_CACHE: dict[tuple[str, tuple[float, float, float]], LocalGeom] = {}


def rpy_matrix(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def axis_angle_matrix(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    kx, ky, kz = axis
    k_cross = np.array([[0.0, -kz, ky], [kz, 0.0, -kx], [-ky, kx, 0.0]])
    return np.eye(3) + np.sin(angle) * k_cross + (1 - np.cos(angle)) * (k_cross @ k_cross)


def origin_transform(el: ET.Element | None) -> np.ndarray:
    transform = np.eye(4)
    if el is None:
        return transform
    origin = el.find("origin")
    if origin is not None:
        transform[:3, 3] = [float(v) for v in (origin.get("xyz") or "0 0 0").split()]
        transform[:3, :3] = rpy_matrix(*[float(v) for v in (origin.get("rpy") or "0 0 0").split()])
    return transform


def _index_geometry(key, raw) -> LocalGeom:
    import trimesh

    if not raw.is_watertight:
        # Vendor STLs routinely carry duplicate vertices a few nanometres apart,
        # which splits the surface into sheets that fill_holes() cannot close -
        # the tessellator emitted the same corner twice. Collapsing at 1um
        # (three orders below the tightest clearance this audit reports) closes
        # them without moving geometry: measured 0.00000mm vertex deviation and
        # identical bounds on all 13 meshes it repairs. Without it the audit
        # cannot measure the mesh at all and FAILs the pair conservatively.
        raw.merge_vertices(digits_vertex=6)
        raw.update_faces(raw.nondegenerate_faces())
        raw.update_faces(raw.unique_faces())
        raw.remove_unreferenced_vertices()
        raw.fill_holes()
    raw_query = trimesh.proximity.ProximityQuery(raw) if raw.is_watertight else None
    hull = raw.convex_hull
    geom = LocalGeom(raw, raw_query, hull, trimesh.proximity.ProximityQuery(hull), raw.bounds.copy())
    _GEOM_CACHE[key] = geom
    return geom


def local_geometry(path: str, scale: tuple[float, float, float]) -> LocalGeom:
    """Load, scale, and index one collision mesh; cached across assets."""
    import trimesh

    key = (path, scale)
    if key in _GEOM_CACHE:
        return _GEOM_CACHE[key]
    source = trimesh.load(path, force="mesh")
    vertices = source.vertices * np.array(scale)
    faces = source.faces
    if np.prod(np.sign(scale)) < 0:
        faces = faces[:, ::-1]  # mirrored scale flips winding
    return _index_geometry(key, trimesh.Trimesh(vertices, faces, process=True))


def primitive_geometry(element: ET.Element) -> LocalGeom | None:
    """Build cached geometry for a URDF primitive (box/cylinder/sphere)."""
    import trimesh

    if element.tag == "box":
        size = tuple(float(v) for v in (element.get("size") or "").split())
        key: tuple = ("box", size)
        if key not in _GEOM_CACHE:
            _index_geometry(key, trimesh.creation.box(extents=size))
    elif element.tag == "cylinder":
        radius, length = float(element.get("radius") or 0), float(element.get("length") or 0)
        key = ("cylinder", radius, length)
        if key not in _GEOM_CACHE:
            _index_geometry(key, trimesh.creation.cylinder(radius=radius, height=length, sections=48))
    elif element.tag == "sphere":
        radius = float(element.get("radius") or 0)
        key = ("sphere", radius)
        if key not in _GEOM_CACHE:
            _index_geometry(key, trimesh.creation.icosphere(subdivisions=3, radius=radius))
    else:
        return None
    return _GEOM_CACHE[key]


def query_points(mesh) -> np.ndarray:
    vertices = np.asarray(mesh.vertices)
    stride = max(1, len(vertices) // MAX_QUERY_POINTS)
    return vertices[::stride]


@dataclass
class Part:
    geom: LocalGeom
    world: np.ndarray  # (4, 4) local -> world


def part_bounds(part: Part) -> np.ndarray:
    lo, hi = part.geom.bounds
    corners = np.array([[x, y, z] for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])])
    world = (part.world[:3, :3] @ corners.T).T + part.world[:3, 3]
    return np.array([world.min(axis=0), world.max(axis=0)])


class UrdfModel:
    def __init__(self, urdf_path: Path, pose: dict[str, float] | None = None):
        self.root = ET.parse(urdf_path).getroot()
        self.pose = pose or {}
        self.parent_of: dict[str, tuple[str, ET.Element]] = {}
        self.adjacent: set[frozenset[str]] = set()
        for joint in self.root.findall("joint"):
            parent_el, child_el = joint.find("parent"), joint.find("child")
            assert parent_el is not None and child_el is not None
            parent, child = parent_el.get("link"), child_el.get("link")
            assert parent and child
            self.parent_of[child] = (parent, joint)
            self.adjacent.add(frozenset((parent, child)))
        self._world_cache: dict[str, np.ndarray] = {}

    def joint_transform(self, joint: ET.Element) -> np.ndarray:
        transform = origin_transform(joint)
        if joint.get("type") in {"revolute", "continuous"}:
            angle = self.pose.get(joint.get("name") or "", 0.0)
            if angle:
                axis_el = joint.find("axis")
                axis_str = axis_el.get("xyz") if axis_el is not None else None
                axis = np.array([float(v) for v in (axis_str or "1 0 0").split()])
                rot = np.eye(4)
                rot[:3, :3] = axis_angle_matrix(axis, angle)
                transform = transform @ rot
        return transform

    def world_transform(self, link: str) -> np.ndarray:
        if link in self._world_cache:
            return self._world_cache[link]
        if link not in self.parent_of:
            transform = np.eye(4)
        else:
            parent, joint = self.parent_of[link]
            transform = self.world_transform(parent) @ self.joint_transform(joint)
        self._world_cache[link] = transform
        return transform

    def link_parts(self) -> dict[str, list[Part]]:
        parts: dict[str, list[Part]] = {}
        for link in self.root.findall("link"):
            name = link.get("name")
            assert name
            entries = []
            for collision in link.findall("collision"):
                geometry = collision.find("geometry")
                if geometry is None or len(geometry) == 0:
                    continue
                shape = geometry[0]
                if shape.tag == "mesh":
                    filename = shape.get("filename")
                    assert filename
                    scale = tuple(float(v) for v in (shape.get("scale") or "1 1 1").split())
                    geom = local_geometry(filename.removeprefix("file://"), scale)
                else:
                    geom = primitive_geometry(shape)
                    if geom is None:
                        continue
                world = self.world_transform(name) @ origin_transform(collision)
                entries.append(Part(geom, world))
            if entries:
                parts[name] = entries
        return parts


def penetration(parts_a: list[Part], parts_b: list[Part], mode: str) -> float | None:
    """Deepest penetration between two part sets (meters), by signed distance.

    mode "hull" always resolves; mode "raw" returns None when neither side has
    a watertight mesh to query against.
    """
    depths = []
    for target_parts, other_parts in ((parts_a, parts_b), (parts_b, parts_a)):
        for target in target_parts:
            if mode == "raw":
                query = target.geom.raw_query
                if query is None:
                    continue
            else:
                query = target.geom.hull_query
            inverse = np.linalg.inv(target.world)
            for other in other_parts:
                source_mesh = other.geom.hull_mesh if mode == "hull" else other.geom.raw_mesh
                points = query_points(source_mesh)
                to_target = inverse @ other.world
                local = (to_target[:3, :3] @ points.T).T + to_target[:3, 3]
                depths.append(query.signed_distance(local).max())
    return max(depths) if depths else None


def load_allowlist() -> list[dict]:
    import yaml

    data = yaml.safe_load(ALLOWLIST_PATH.read_text(encoding="utf-8")) or {}
    return data.get("pairs", [])


def allowlisted_entry(link_a: str, link_b: str, allowlist: list[dict]) -> dict | None:
    for entry in allowlist:
        pat_a, pat_b = entry["links"]
        if (fnmatch.fnmatch(link_a, pat_a) and fnmatch.fnmatch(link_b, pat_b)) or (
            fnmatch.fnmatch(link_a, pat_b) and fnmatch.fnmatch(link_b, pat_a)
        ):
            return entry
    return None


def audit_urdf(urdf_path: Path, pose: dict[str, float] | None = None) -> list[Finding]:
    """Return FAIL/WARN findings for one RL URDF (empty list = clean)."""
    model = UrdfModel(urdf_path, pose)
    link_parts = model.link_parts()
    allowlist = load_allowlist()
    names = sorted(link_parts)
    bounds = {name: np.array([
        np.min([part_bounds(p)[0] for p in parts], axis=0),
        np.max([part_bounds(p)[1] for p in parts], axis=0),
    ]) for name, parts in link_parts.items()}

    findings: list[Finding] = []
    for i, link_a in enumerate(names):
        for link_b in names[i + 1 :]:
            if frozenset((link_a, link_b)) in model.adjacent:
                continue  # PhysX always excludes joint-connected pairs
            if (bounds[link_a][1] < bounds[link_b][0] - PENETRATION_LIMIT_M).any() or (
                bounds[link_b][1] < bounds[link_a][0] - PENETRATION_LIMIT_M
            ).any():
                continue
            hull_depth = penetration(link_parts[link_a], link_parts[link_b], "hull")
            if hull_depth is None or hull_depth < PENETRATION_LIMIT_M:
                continue
            raw_depth = penetration(link_parts[link_a], link_parts[link_b], "raw")
            entry = allowlisted_entry(link_a, link_b, allowlist)
            if raw_depth is None:
                if entry is None:
                    findings.append(Finding(link_a, link_b, hull_depth, None, "FAIL",
                                            "hull penetration and no watertight mesh to disprove it"))
                else:
                    findings.append(Finding(link_a, link_b, hull_depth, None, "WARN",
                                            f"unverified (non-watertight) - {entry.get('reason', 'allowlisted')}"))
            elif raw_depth >= PENETRATION_LIMIT_M:
                if entry is not None and entry.get("accept_raw"):
                    findings.append(Finding(
                        link_a, link_b, hull_depth, raw_depth, "WARN",
                        f"accepted resting penetration - {entry.get('reason', 'allowlisted')}"))
                else:
                    findings.append(Finding(link_a, link_b, hull_depth, raw_depth, "FAIL",
                                            "raw meshes interpenetrate at rest"))
            else:
                reason = (entry.get("reason", "allowlisted") if entry is not None
                          else "hull-only; safe with convex_decomposition import")
                findings.append(Finding(link_a, link_b, hull_depth, raw_depth, "WARN", reason))
    return findings


def load_audit_poses(asset_stem: str) -> dict[str, dict[str, float]]:
    """Registered task-home poses for one asset (pose name -> {joint: rad})."""
    import yaml

    if not POSES_PATH.is_file():
        return {}
    data = yaml.safe_load(POSES_PATH.read_text(encoding="utf-8")) or {}
    poses = data.get(asset_stem) or {}
    return {name: {j: float(v) for j, v in joints.items()} for name, joints in poses.items()}


def audit_asset(urdf_path: Path) -> dict[str, list[Finding]]:
    """Audit one asset at zero pose plus every registered task-home pose."""
    results = {"zero": audit_urdf(urdf_path)}
    for pose_name, pose in load_audit_poses(urdf_path.stem).items():
        results[pose_name] = audit_urdf(urdf_path, pose)
    return results


def urdf_link_names(urdf_path: Path) -> set[str]:
    root = ET.parse(urdf_path).getroot()
    return {name for link in root.findall("link") if (name := link.get("name"))}


def mirror_link(name: str) -> str | None:
    if name.startswith("r_"):
        return "l_" + name[2:]
    if name.startswith("l_"):
        return "r_" + name[2:]
    return None


def filtered_pairs(
    findings: list[Finding], link_names: set[str] | None = None
) -> list[tuple[str, str]]:
    """WARN pairs that need USD collision filtering.

    Every WARN pair is a place where a convex approximation differs from the
    raw geometry (hull artifacts, cooking inflation, nested vendor shells).
    The built USD uses convexDecomposition colliders, whose cooking still
    inflates a few millimetres (measured: a 2.8mm raw clearance produced
    ~4kN phantom contacts), so ALL of them are collision-filtered, not only
    the near-contact ones.

    When link_names is given, pairs are symmetrized across the left/right
    arm mirror: the arms are mirrored geometry, so any near-contact pair on
    one side exists on the other at the mirrored pose even if the audited
    poses never reach it (regression: r_al_5<->r_al_7 WARNed at zero pose
    but the l_ mirror only approaches at the left task home).
    """
    def ordered(a: str, b: str) -> tuple[str, str]:
        return (a, b) if a <= b else (b, a)

    pairs = {ordered(f.link_a, f.link_b) for f in findings if f.verdict == "WARN"}
    if link_names is not None:
        for link_a, link_b in list(pairs):
            mirrored_a, mirrored_b = mirror_link(link_a), mirror_link(link_b)
            if mirrored_a in link_names and mirrored_b in link_names:
                pairs.add(ordered(mirrored_a, mirrored_b))
    return sorted(pairs)


def report(name: str, findings: list[Finding]) -> bool:
    """Print one asset's findings; returns True when the asset passes."""
    failures = [f for f in findings if f.verdict == "FAIL"]
    print(f"[{name}] {'PASS' if not failures else 'FAIL'} "
          f"({len(failures)} fail, {len(findings) - len(failures)} warn)")
    for f in findings:
        raw = "n/a" if f.raw_depth_m is None else f"{f.raw_depth_m * 1000:+.1f}mm"
        print(f"  {f.verdict}: {f.link_a} <-> {f.link_b} "
              f"hull {f.hull_depth_m * 1000:+.1f}mm raw {raw} - {f.reason}")
    return not failures


def parse_pose(entries: list[str]) -> dict[str, float]:
    pose = {}
    for entry in entries:
        name, _, value = entry.partition("=")
        if not value:
            raise SystemExit(f"invalid --pose entry (want name=rad): {entry}")
        pose[name] = float(value)
    return pose


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("names", nargs="*",
                        help="Asset names (default: every generated/rl/*_rl.urdf).")
    parser.add_argument("--pose", action="append", default=[], metavar="JOINT=RAD",
                        help="Audit only this custom pose (repeatable joint overrides). "
                             "Default: zero pose plus every pose in audit_poses.yaml.")
    args = parser.parse_args(argv)

    if args.names:
        paths = [RL_DIR / f"{name.removesuffix('_rl')}_rl.urdf" for name in args.names]
    else:
        paths = sorted(RL_DIR.glob("*_rl.urdf"))
    missing = [p for p in paths if not p.is_file()]
    if missing:
        parser.error(f"missing RL URDF(s): {', '.join(str(p) for p in missing)}")

    pose = parse_pose(args.pose)
    ok = True
    for path in paths:
        if pose:
            ok &= report(f"{path.stem}@custom", audit_urdf(path, pose))
        else:
            for pose_name, findings in audit_asset(path).items():
                ok &= report(f"{path.stem}@{pose_name}", findings)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
