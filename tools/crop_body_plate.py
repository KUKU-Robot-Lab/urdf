#!/usr/bin/env python3
"""Crop the OpenArm body_link0 meshes for the RL assets.

Two cuts, both producing pre-shifted meshes in generated/rl/meshes/:

1. Mount plate (bottom). The vendor meshes put z=0 at the bottom of an 8mm
   mount plate. The real robot uses a different mount thickness, so the RL
   assets define the robot origin at the plate TOP instead: everything below
   z=8mm is sliced away and the result is translated down by 8mm.

2. Head housing (2026-09-05). The vendor body ends in a shoulder housing /
   head pocket (an open shell, z 0.605-0.765 in the shifted frame) that the
   old vendor head sat in. head_v1 (vendor/head_v1) carries its own base
   plate, so the WHOLE housing is removed and only the 60x60 pillar is kept,
   cut at the head base plate underside - otherwise body_link and the head
   links interpenetrate at rest. Visual: every mesh component that starts
   inside the housing band is dropped (the pillar starts at z=0 and
   survives), then the pillar is cut at the plate underside. Collision (one
   fused watertight mesh): everything from the housing bottom up is sliced
   off and replaced by a box over the pillar footprint up to the plate
   underside. The cut height is not a constant here: generate_rl_urdf.py
   derives it from the head mount (HEAD_MOUNT_XYZ) and the head_v1 base-plate
   geometry, and the output file name carries it in mm, so a mount change can
   never reuse a stale crop.

Outputs are consumed by generate_rl_urdf.py.
"""

from __future__ import annotations

from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BODY_MESH_DIR = ROOT / "vendor" / "openarm_description" / "meshes" / "body" / "v10"
OUT_MESH_DIR = ROOT / "generated" / "rl" / "meshes"

# Vendor plate: z in [0, 8] mm. New origin = plate top.
PLATE_TOP_MM = 8.0

CROP_JOBS = {
    "collision": (
        BODY_MESH_DIR / "collision" / "body_link0_symp.stl",
        OUT_MESH_DIR / "body_link0_symp_cut.stl",
    ),
    "visual": (
        BODY_MESH_DIR / "visual" / "body_link0.stl",
        OUT_MESH_DIR / "body_link0_visual_cut.stl",
    ),
}


def _slice(mesh, plane_origin_mm: list[float], plane_normal: list[float], label: str):
    """Keep the half-space on the normal side; cap when the geometry allows it."""
    try:
        sliced = mesh.slice_plane(plane_origin=plane_origin_mm, plane_normal=plane_normal, cap=True)
    except Exception as exc:  # noqa: BLE001 - trimesh capping fails on open shells
        print(f"  cap failed for {label} ({type(exc).__name__}: {exc}); leaving the cut open")
        sliced = mesh.slice_plane(plane_origin=plane_origin_mm, plane_normal=plane_normal, cap=False)
    if sliced is None or len(sliced.faces) == 0:
        raise ValueError(f"slicing produced an empty mesh for {label}")
    return sliced


def crop_mesh(source: Path, output: Path) -> None:
    """Slice off z < PLATE_TOP_MM and shift the mesh down by PLATE_TOP_MM."""
    import trimesh

    if not source.is_file():
        raise FileNotFoundError(f"missing vendor mesh: {source}")

    mesh = trimesh.load(source, force="mesh")
    sliced = _slice(mesh, [0.0, 0.0, PLATE_TOP_MM], [0.0, 0.0, 1.0], source.name)
    sliced.apply_translation([0.0, 0.0, -PLATE_TOP_MM])

    zmin, zmax = sliced.bounds[0][2], sliced.bounds[1][2]
    if zmin < -1e-3:
        raise ValueError(f"cropped mesh still extends below origin: zmin={zmin}")

    output.parent.mkdir(parents=True, exist_ok=True)
    sliced.export(output)
    print(f"cropped {source.relative_to(ROOT)} -> {output.relative_to(ROOT)} "
          f"(z range [{zmin:.3f}, {zmax:.3f}] mm)")


# Components of the visual mesh that begin above this height are the housing
# shell (measured on the vendor mesh: every housing part starts at z=0.605, the
# pillar at z=0); the pillar footprint is measured from the same mesh.
HOUSING_BAND_MIN_M = 0.5


def _housing_and_pillar(visual_mesh):
    """Split the plate-cropped visual mesh into (pillar, housing parts, housing zmin)."""
    parts = visual_mesh.split(only_watertight=False)
    housing = [q for q in parts if q.bounds[0][2] > HOUSING_BAND_MIN_M * 1000.0]
    keep = [q for q in parts if q.bounds[0][2] <= HOUSING_BAND_MIN_M * 1000.0]
    if not housing:
        raise ValueError("no housing components found above the band - vendor mesh changed?")
    pillar = max((q for q in keep if q.bounds[1][2] > HOUSING_BAND_MIN_M * 1000.0),
                 key=lambda q: q.bounds[1][2])
    housing_zmin_mm = min(q.bounds[0][2] for q in housing)
    return keep, pillar, housing_zmin_mm


def remove_head_housing(top_z_m: float) -> dict[str, Path]:
    """Write the housing-free, top-cut body meshes; returns paths by kind."""
    import trimesh

    top_mm = top_z_m * 1000.0
    visual_src, collision_src = CROP_JOBS["visual"][1], CROP_JOBS["collision"][1]
    outputs = {kind: top_cropped_name(CROP_JOBS[kind][1], top_z_m) for kind in CROP_JOBS}

    visual = trimesh.load(visual_src, force="mesh")
    keep, pillar, housing_zmin_mm = _housing_and_pillar(visual)
    body = trimesh.util.concatenate(keep)
    body = _slice(body, [0.0, 0.0, top_mm], [0.0, 0.0, -1.0], f"{visual_src.name} top")
    body.export(outputs["visual"])
    print(f"body visual: dropped {len(visual.split(only_watertight=False)) - len(keep)} housing parts "
          f"(z>={housing_zmin_mm:.1f} mm), cut at {top_mm:.0f} mm -> {outputs['visual'].name}")

    collision = trimesh.load(collision_src, force="mesh")
    below = _slice(collision, [0.0, 0.0, housing_zmin_mm], [0.0, 0.0, -1.0], f"{collision_src.name} housing")
    (px0, py0, _), (px1, py1, _) = pillar.bounds
    box = trimesh.creation.box(extents=[px1 - px0, py1 - py0, top_mm - housing_zmin_mm])
    box.apply_translation([(px0 + px1) / 2, (py0 + py1) / 2, (housing_zmin_mm + top_mm) / 2])
    merged = trimesh.util.concatenate([below, box])
    if merged.bounds[1][2] > top_mm + 1e-3:
        raise ValueError(f"collision crop left geometry above {top_mm} mm")
    merged.export(outputs["collision"])
    print(f"body collision: sliced at {housing_zmin_mm:.1f} mm + pillar box "
          f"{px1 - px0:.0f}x{py1 - py0:.0f} mm to {top_mm:.0f} mm -> {outputs['collision'].name} "
          f"(watertight={merged.is_watertight})")
    return outputs


def top_cropped_name(output: Path, top_z_m: float) -> Path:
    """<stem>_nohousing_top<mm>.stl - the crop height is part of the name (no stale reuse)."""
    return output.with_name(f"{output.stem}_nohousing_top{round(top_z_m * 1000):d}{output.suffix}")


def ensure_cropped_meshes(top_z_m: float | None = None) -> dict[str, Path]:
    """Create cropped meshes when missing or stale. Returns output paths by kind.

    With ``top_z_m`` the returned meshes additionally have the vendor head
    housing removed and the pillar cut at that height; without it only the
    mount plate is cropped.
    """
    outputs: dict[str, Path] = {}
    for kind, (source, output) in CROP_JOBS.items():
        if not output.is_file() or output.stat().st_mtime < source.stat().st_mtime:
            crop_mesh(source, output)
        outputs[kind] = output
    if top_z_m is None:
        return outputs
    wanted = {kind: top_cropped_name(output, top_z_m) for kind, (_, output) in CROP_JOBS.items()}
    stale = any(not w.is_file() or w.stat().st_mtime < outputs[k].stat().st_mtime for k, w in wanted.items())
    return remove_head_housing(top_z_m) if stale else wanted


def main() -> int:
    for source, output in CROP_JOBS.values():
        crop_mesh(source, output)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
