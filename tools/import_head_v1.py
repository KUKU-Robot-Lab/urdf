#!/usr/bin/env python3
"""Import the Fusion `head_v1` pan/tilt head (hdgp/assets/simulation_setting/head_v1)
into `vendor/head_v1/` as a 3-link URDF + link-local meshes for generate_rl_urdf.py.

Why a tool and not hand-written files: head_v1.usda/obj are regenerated from the
Fusion document (see its README). Re-run this after every regeneration so the
URDF joints, meshes and inertials cannot drift from the CAD.

Source conventions (head_v1.usda, verified 2026-09-05):
  - meters, Z-up, every link Xform is identity -> all link geometry is expressed
    in the SAME frame (the Fusion origin) at the zero pose; joints carry the axis
    point as localPos0 == localPos1.
  - links: base_link, pan_link, tilt_link, camera_link (camera_link is bolted to
    tilt_link through a fixed joint at identity).
  - optical frames under camera_link (color/depth/left_ir/right_ir/ir_projector),
    all with the ROS optical orientation; +Z(optical) = head +X = viewing direction.

Output conventions (this URDF, kept identical to the previous vendor head so that
generate_rl_urdf.py, the manifests, sim2real/scripts/head_fk_chain.py and the
hand-eye chain keep their link/joint names):
  - base_link  (root)            : frame = Fusion origin (base plate bottom +20 mm)
  - joint_pan  base -> mid_link  : origin = pan axis point, axis (0,0,-1)  [*]
  - joint_tilt mid  -> camera_link: origin = tilt axis point (in mid frame), axis (0,1,0)
  - camera_link = tilt_link + camera_link MERGED (one rigid body; the D435i is
    bolted to the tilt bracket).  Merging keeps the RL body list unchanged.
  - meshes are written in each LINK-LOCAL frame, in meters (scale 1).
  - inertials come from the USD mass properties (Fusion, real part masses);
    the merged link uses the parallel-axis theorem.
  [*] The previous head used axis (0,0,-1) for pan and sim2real encodes the
      encoder sign against that convention (head_fk_chain.PAN_ENCODER_TO_URDF).
      Keeping it avoids silently flipping the deployed FK.

Usage:
    python3 tools/import_head_v1.py            # writes vendor/head_v1/
    python3 tools/import_head_v1.py --check    # parse + print, write nothing
"""
from __future__ import annotations

import argparse
import json
import re
import struct
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
SRC_DIR = ROOT.parent / "hdgp" / "assets" / "simulation_setting" / "head_v1"
SRC_USDA = SRC_DIR / "usd" / "head_v1.usda"
SRC_OBJ = SRC_DIR / "meshes" / "visual" / "head_v1.obj"
DST_DIR = ROOT / "vendor" / "head_v1"

USD_LINKS = ("base_link", "pan_link", "tilt_link", "camera_link")
#: URDF link name -> USD bodies merged into it.
URDF_LINKS = {
    "base_link": ("base_link",),
    "mid_link": ("pan_link",),
    "camera_link": ("tilt_link", "camera_link"),
}
OPTICAL_FRAMES = ("color_frame", "depth_frame", "left_ir_frame", "right_ir_frame", "ir_projector_frame")

# Joint placeholders carried over from the previous vendor head (Fusion has none).
PAN_AXIS = "0 0 -1"
TILT_AXIS = "0 1 0"
JOINT_LIMIT = 'lower="-1.570796" upper="1.570796" effort="1.0" velocity="6.0"'
JOINT_DYNAMICS = 'damping="0.02" friction="0.0"'


# ---------------------------------------------------------------------------
# USDA parsing (text; the file is small enough to regex)
# ---------------------------------------------------------------------------
_NUM = r"[-+0-9.eE]+"


def _vec(text: str, key: str, n: int) -> np.ndarray:
    m = re.search(rf"{re.escape(key)} = \(({_NUM}(?:,\s*{_NUM}){{{n - 1}}})\)", text)
    if m is None:
        raise ValueError(f"'{key}' not found")
    return np.array([float(v) for v in m.group(1).split(",")])


def _scalar(text: str, key: str) -> float:
    m = re.search(rf"{re.escape(key)} = ({_NUM})", text)
    if m is None:
        raise ValueError(f"'{key}' not found")
    return float(m.group(1))


def _points(block: str) -> np.ndarray:
    m = re.search(r"point3f\[\] points = \[(.*?)\]", block, re.S)
    if m is None:
        raise ValueError("points not found")
    vals = re.findall(rf"\(({_NUM}),\s*({_NUM}),\s*({_NUM})\)", m.group(1))
    return np.array([[float(a), float(b), float(c)] for a, b, c in vals])


def _faces(block: str) -> np.ndarray:
    counts = re.search(r"int\[\] faceVertexCounts = \[(.*?)\]", block, re.S)
    idx = re.search(r"int\[\] faceVertexIndices = \[(.*?)\]", block, re.S)
    if counts is None or idx is None:
        raise ValueError("faceVertexCounts/Indices not found")
    c = np.array([int(v) for v in counts.group(1).split(",") if v.strip()])
    if not np.all(c == 3):
        raise ValueError(f"non-triangle faces present (counts != 3: {np.unique(c)})")
    i = np.array([int(v) for v in idx.group(1).split(",") if v.strip()])
    return i.reshape(-1, 3)


def parse_usda(text: str) -> dict:
    links: dict[str, dict] = {}
    pattern = re.compile(
        r'\n    def Xform "(base_link|pan_link|tilt_link|camera_link)"(.*?)(?=\n    def Xform "|\n    def Physics)',
        re.S,
    )
    for m in pattern.finditer(text):
        name, body = m.group(1), m.group(2)
        col = re.search(r'def Mesh "Collision".*?(?=\n        \}\n)', body, re.S)
        if col is None:
            raise ValueError(f"{name}: Collision mesh not found")
        info = {
            "mass": _scalar(body, "float physics:mass"),
            "com": _vec(body, "point3f physics:centerOfMass", 3),
            "diag": _vec(body, "float3 physics:diagonalInertia", 3),
            "quat_wxyz": _vec(body, "quatf physics:principalAxes", 4),
            "collision_points": _points(col.group(0)),
            "collision_faces": _faces(col.group(0)),
            "frames": {},
        }
        for frame in OPTICAL_FRAMES:
            fm = re.search(rf'def Xform "{frame}"(.*?)\n        \}}', body, re.S)
            if fm is not None:
                info["frames"][frame] = {
                    "xyz": _vec(fm.group(1), "double3 xformOp:translate", 3),
                    "quat_wxyz": _vec(fm.group(1), "quatf xformOp:orient", 4),
                }
        links[name] = info
    missing = set(USD_LINKS) - set(links)
    if missing:
        raise ValueError(f"links missing in USDA: {sorted(missing)}")

    def joint(name: str) -> dict:
        m = re.search(rf'def PhysicsRevoluteJoint "{name}".*?\n    \}}', text, re.S)
        if m is None:
            raise ValueError(f"joint {name} not found")
        blk = m.group(0)
        p0, p1 = _vec(blk, "point3f physics:localPos0", 3), _vec(blk, "point3f physics:localPos1", 3)
        if not np.allclose(p0, p1):
            raise ValueError(f"{name}: localPos0 != localPos1 - link frames are not identity")
        am = re.search(r'uniform token physics:axis = "(\w)"', blk)
        if am is None:
            raise ValueError(f"{name}: physics:axis not found")
        return {"point": p0, "axis": am.group(1)}

    return {"links": links, "pan": joint("pan_joint"), "tilt": joint("tilt_joint")}


# ---------------------------------------------------------------------------
# OBJ parsing (visual): 'o <link>' groups, global vertex indices, 'f a b c'
# ---------------------------------------------------------------------------
def parse_obj(text: str) -> tuple[np.ndarray, dict[str, np.ndarray]]:
    verts: list[list[float]] = []
    faces: dict[str, list[list[int]]] = {}
    current = None
    for line in text.splitlines():
        if line.startswith("v "):
            verts.append([float(v) for v in line.split()[1:4]])
        elif line.startswith("o ") or line.startswith("g "):
            current = line[2:].strip()
            faces.setdefault(current, [])
        elif line.startswith("f "):
            if current is None:
                raise ValueError("face before any group")
            idx = [int(tok.split("/")[0]) - 1 for tok in line.split()[1:]]
            if len(idx) != 3:
                raise ValueError("non-triangle face in OBJ")
            faces[current].append(idx)
    missing = set(USD_LINKS) - set(faces)
    if missing:
        raise ValueError(f"OBJ groups missing: {sorted(missing)}")
    return np.array(verts), {k: np.array(v) for k, v in faces.items()}


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def quat_to_rot(q: np.ndarray) -> np.ndarray:
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def inertia_about_com(link: dict) -> np.ndarray:
    R = quat_to_rot(link["quat_wxyz"])
    return R @ np.diag(link["diag"]) @ R.T


def merge_inertials(parts: list[dict]) -> tuple[float, np.ndarray, np.ndarray]:
    mass = float(sum(p["mass"] for p in parts))
    com = np.sum([p["mass"] * p["com"] for p in parts], axis=0) / mass
    inertia = np.zeros((3, 3))
    for p in parts:
        d = p["com"] - com
        inertia += inertia_about_com(p) + p["mass"] * (np.dot(d, d) * np.eye(3) - np.outer(d, d))
    return mass, com, inertia


def write_obj(path: Path, verts: np.ndarray, faces: np.ndarray, name: str) -> None:
    used = np.unique(faces)
    remap = {old: new for new, old in enumerate(used)}
    lines = [f"# head_v1 {name} - link-local frame, meters (import_head_v1.py)", f"o {name}"]
    lines += [f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in verts[used]]
    lines += [f"f {remap[a] + 1} {remap[b] + 1} {remap[c] + 1}" for a, b, c in faces]
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def write_stl(path: Path, verts: np.ndarray, faces: np.ndarray, name: str) -> None:
    tri = verts[faces]
    n = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    norms = np.linalg.norm(n, axis=1, keepdims=True)
    n = np.divide(n, norms, out=np.zeros_like(n), where=norms > 0)
    header = f"head_v1 {name} link-local meters".encode("ascii").ljust(80, b"\0")
    with open(path, "wb") as f:
        f.write(header)
        f.write(struct.pack("<I", len(faces)))
        for normal, t in zip(n, tri):
            f.write(struct.pack("<3f", *normal))
            for v in t:
                f.write(struct.pack("<3f", *v))
            f.write(struct.pack("<H", 0))


# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
def fmt(v) -> str:
    return " ".join(f"{float(x):.6f}" for x in np.atleast_1d(v))


def build(check_only: bool) -> dict:
    usd = parse_usda(SRC_USDA.read_text(encoding="utf-8"))
    obj_verts, obj_faces = parse_obj(SRC_OBJ.read_text(encoding="utf-8"))
    pan_pt, tilt_pt = usd["pan"]["point"], usd["tilt"]["point"]
    if usd["pan"]["axis"] != "Z" or usd["tilt"]["axis"] != "Y":
        raise ValueError(f"unexpected joint axes: pan {usd['pan']['axis']} tilt {usd['tilt']['axis']}")

    #: link-local frame origin (in the Fusion frame) per URDF link
    origins = {"base_link": np.zeros(3), "mid_link": pan_pt, "camera_link": tilt_pt}
    summary: dict = {
        "source": {"usda": str(SRC_USDA), "obj": str(SRC_OBJ)},
        "pan_axis_point_m": pan_pt.tolist(),
        "tilt_axis_point_m": tilt_pt.tolist(),
        "joint_pan_origin_in_base": pan_pt.tolist(),
        "joint_tilt_origin_in_mid": (tilt_pt - pan_pt).tolist(),
        "links": {},
        "optical_frames_in_camera_link": {},
    }
    for frame, data in usd["links"]["camera_link"]["frames"].items():
        summary["optical_frames_in_camera_link"][frame] = {
            "xyz": (data["xyz"] - tilt_pt).tolist(),
            "quat_wxyz_optical": data["quat_wxyz"].tolist(),
        }

    inertial_xml: dict[str, str] = {}
    meshes: dict[str, dict] = {}
    for urdf_name, usd_names in URDF_LINKS.items():
        parts = [usd["links"][n] for n in usd_names]
        mass, com, inertia = merge_inertials(parts)
        origin = origins[urdf_name]
        com_local = com - origin
        summary["links"][urdf_name] = {
            "usd_bodies": list(usd_names),
            "mass_kg": mass,
            "com_local_m": com_local.tolist(),
            "inertia_about_com_kgm2": inertia.tolist(),
        }
        ixx, iyy, izz = inertia[0, 0], inertia[1, 1], inertia[2, 2]
        ixy, ixz, iyz = inertia[0, 1], inertia[0, 2], inertia[1, 2]
        inertial_xml[urdf_name] = (
            f'    <inertial>\n      <origin xyz="{fmt(com_local)}" rpy="0 0 0"/>\n'
            f'      <mass value="{mass:.6f}"/>\n'
            f'      <inertia ixx="{ixx:.6e}" ixy="{ixy:.6e}" ixz="{ixz:.6e}"\n'
            f'               iyy="{iyy:.6e}" iyz="{iyz:.6e}" izz="{izz:.6e}"/>\n    </inertial>'
        )
        # visual: OBJ groups merged, collision: USD collision meshes merged
        vis_faces = np.concatenate([obj_faces[n] for n in usd_names])
        col_v, col_f, offset = [], [], 0
        for n in usd_names:
            pts, fcs = usd["links"][n]["collision_points"], usd["links"][n]["collision_faces"]
            col_v.append(pts - origin)
            col_f.append(fcs + offset)
            offset += len(pts)
        meshes[urdf_name] = {
            "vis_verts": obj_verts - origin, "vis_faces": vis_faces,
            "col_verts": np.concatenate(col_v), "col_faces": np.concatenate(col_f),
        }
        vb = meshes[urdf_name]["vis_verts"][np.unique(vis_faces)]
        summary["links"][urdf_name]["visual_aabb_local_m"] = [vb.min(0).tolist(), vb.max(0).tolist()]
        summary["links"][urdf_name]["collision_tris"] = int(len(meshes[urdf_name]["col_faces"]))

    if check_only:
        return summary

    (DST_DIR / "meshes" / "visual").mkdir(parents=True, exist_ok=True)
    (DST_DIR / "meshes" / "collision").mkdir(parents=True, exist_ok=True)
    (DST_DIR / "urdf").mkdir(parents=True, exist_ok=True)
    stem = {"base_link": "head_base", "mid_link": "head_mid", "camera_link": "head_camera"}
    for urdf_name, m in meshes.items():
        write_obj(DST_DIR / "meshes" / "visual" / f"{stem[urdf_name]}.obj", m["vis_verts"], m["vis_faces"], stem[urdf_name])
        write_stl(DST_DIR / "meshes" / "collision" / f"{stem[urdf_name]}.stl", m["col_verts"], m["col_faces"], stem[urdf_name])

    urdf = f"""<?xml version="1.0"?>
<!--
  openarm D435i pan/tilt head — head_v1 (Fusion "rl_ws / simulation setting", 2026-09-05).
  GENERATED by tools/import_head_v1.py from hdgp/assets/simulation_setting/head_v1 — do not edit.

  Links (same names as the previous vendor head so downstream naming is unchanged):
    base_link   : base plate + brackets + pan servo body            (frame = Fusion origin,
                  = base plate bottom +20 mm)
    mid_link    : pan horn/platform + tilt servo body               (rotated by PAN, XC330 #1)
    camera_link : tilt horn + U bracket + camera plate + D435i      (rotated by TILT, XC330 #2)
                  = USD tilt_link + camera_link merged (fixed joint at identity)

  Kinematics (axis points read from head_v1.usda joints, meters):
    joint_pan  : base_link -> mid_link   , axis {PAN_AXIS} , at {fmt(pan_pt)} in base frame
    joint_tilt : mid_link  -> camera_link, axis {TILT_AXIS} , at {fmt(tilt_pt - pan_pt)} in mid frame
  Optical frames (in camera_link frame, +X viewing, +Z up; from head_v1.usda camera_link):
    color  {fmt(usd['links']['camera_link']['frames']['color_frame']['xyz'] - tilt_pt)}
    depth  {fmt(usd['links']['camera_link']['frames']['depth_frame']['xyz'] - tilt_pt)}

  Meshes are LINK-LOCAL, meters (scale 1). Inertials from Fusion mass properties
  (servo/RealSense masses, see the head_v1 README). Joint limits/effort/velocity are
  placeholders carried over from the previous head (XC330 range not modelled).
-->
<robot name="openarm_d435i_head_v1">

  <link name="base_link">
{inertial_xml['base_link']}
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/visual/head_base.obj" scale="1 1 1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/collision/head_base.stl" scale="1 1 1"/></geometry>
    </collision>
  </link>

  <joint name="joint_pan" type="revolute">
    <parent link="base_link"/>
    <child  link="mid_link"/>
    <origin xyz="{fmt(pan_pt)}" rpy="0 0 0"/>
    <axis xyz="{PAN_AXIS}"/>
    <limit {JOINT_LIMIT}/>
    <dynamics {JOINT_DYNAMICS}/>
  </joint>

  <link name="mid_link">
{inertial_xml['mid_link']}
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/visual/head_mid.obj" scale="1 1 1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/collision/head_mid.stl" scale="1 1 1"/></geometry>
    </collision>
  </link>

  <joint name="joint_tilt" type="revolute">
    <parent link="mid_link"/>
    <child  link="camera_link"/>
    <origin xyz="{fmt(tilt_pt - pan_pt)}" rpy="0 0 0"/>
    <axis xyz="{TILT_AXIS}"/>
    <limit {JOINT_LIMIT}/>
    <dynamics {JOINT_DYNAMICS}/>
  </joint>

  <link name="camera_link">
{inertial_xml['camera_link']}
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/visual/head_camera.obj" scale="1 1 1"/></geometry>
    </visual>
    <collision>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/collision/head_camera.stl" scale="1 1 1"/></geometry>
    </collision>
  </link>

</robot>
"""
    (DST_DIR / "urdf" / "head.urdf").write_text(urdf, encoding="utf-8")
    (DST_DIR / "head_data.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    (DST_DIR / "README.md").write_text(
        "# head_v1 (vendor copy for the RL URDF pipeline)\n\n"
        "Generated by `tools/import_head_v1.py` from `hdgp/assets/simulation_setting/head_v1`\n"
        "(Fusion `rl_ws / simulation setting`, 2026-09-05). Re-run the tool after the Fusion\n"
        "export changes; do not edit `urdf/head.urdf` or the meshes by hand.\n\n"
        "- 3 links (base/mid/camera) — USD `tilt_link` and `camera_link` are merged into `camera_link`.\n"
        "- meshes are link-local, meters, scale 1 (visual: OBJ from the Fusion export, collision: USD\n"
        "  convex-decomposition meshes).\n"
        "- `head_data.json` carries the axis points, merged inertials and the D435i optical frames\n"
        "  expressed in `camera_link` (`generate_rl_urdf.py` puts `color_frame` at `head_cam_view`).\n"
        "- mount height on the body column is NOT here: `generate_rl_urdf.HEAD_MOUNT_XYZ`.\n",
        encoding="utf-8",
    )
    return summary


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=(__doc__ or "").split("\n")[0])
    ap.add_argument("--check", action="store_true", help="parse and print, write nothing")
    args = ap.parse_args(argv)
    summary = build(check_only=args.check)
    print(json.dumps({k: v for k, v in summary.items() if k != "links"}, indent=1))
    for name, link in summary["links"].items():
        print(f"{name}: mass {link['mass_kg']:.4f} kg  com_local {np.round(link['com_local_m'], 4).tolist()}"
              f"  aabb {np.round(link['visual_aabb_local_m'], 4).tolist()}  col_tris {link['collision_tris']}")
    if not args.check:
        print(f"wrote {DST_DIR}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
