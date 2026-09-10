#!/usr/bin/env python3
"""빌드된 USD 에서 관절별 `physics:JointEquivalentInertia` 를 뽑아 JSON 으로 남긴다.

왜 이게 필요한가 (2026-09-10). Tesollo 가 자기 Isaac Sim 자산
(`repo/tesollo/tesollo_model/dg5f*/usd/*/configuration/*_physics.usd`)에서 손 드라이브
게인을 **관절 관성 비례**로 준다 — 20관절 전부에서 정확히

    stiffness = 625  x JointEquivalentInertia      (USD 각도 단위)
    damping   = 0.25 x JointEquivalentInertia

숫자를 베끼면 우리 자산(팔이 붙고 마운트 프레임이 추가됨)에서 틀린다. 규칙을 우리 자산의
관성에 적용해야 한다. IsaacLab 액추에이터는 관성을 못 읽으므로 빌드 산출물로 남긴다.

    ../IsaacLab/isaaclab.sh -p tools/dump_joint_inertia.py <asset-dir-or-usd> [-o out.json]

★09.10 `pxr` 는 Isaac 앱을 띄운 뒤에야 import 된다(5.1). 그래서 main 이 AppLauncher 를
  먼저 올린다 — 맨몸 python3 로 부르면 ModuleNotFoundError: pxr 로 죽는다.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

SUFFIX = "_joint_inertia.json"


def dump(usd_path: Path) -> dict[str, float]:
    from pxr import Usd

    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        raise SystemExit(f"USD 를 열 수 없다: {usd_path}")
    # ★★09.10 **구동 관절만** 담는다. fixed joint prim 도 이 속성을 그대로 들고 오는데,
    #   소비처(`grasp_fj/fj_core_cfg._vendor_isaac_hand_gains`)는 "표는 자산에서 나왔으니
    #   자산과 일치한다"는 전제로 **표의 키에서 관절 목록을 뽑는다**. 용접한 관절이 표에
    #   남아 있으면 IsaacLab 액추에이터가 없는 관절에 게인을 걸다 부팅에서 죽는다
    #   (`Not all regular expressions are matched`). 09.10 thumb_1 용접에서 실제로 났다.
    #   그 전까지 안 터진 이유는 표에 있던 fixed(mount/base/flange_adapter/palm_ee)가
    #   손 액추에이터 정규식 `_[1-4]` 에 우연히 안 걸렸기 때문이다 — 운이었다.
    _DRIVEN = {"PhysicsRevoluteJoint", "PhysicsPrismaticJoint"}
    out: dict[str, float] = {}
    for prim in stage.Traverse():
        if str(prim.GetTypeName()) not in _DRIVEN:
            continue
        attr = prim.GetAttribute("physics:JointEquivalentInertia")
        if not attr:
            continue
        value = attr.Get()
        if value is None:
            continue
        name = prim.GetName()
        if name in out and abs(out[name] - float(value)) > 1e-12:
            raise SystemExit(f"{usd_path}: 관절 {name} 의 관성이 두 값으로 나온다")
        out[name] = float(value)
    if not out:
        raise SystemExit(f"{usd_path}: JointEquivalentInertia 가 하나도 없다")
    return out


def resolve_usd(target: Path) -> Path:
    if target.is_dir():
        cands = sorted(p for p in target.glob("*.usd") if "_right" not in p.stem and "_left" not in p.stem)
        if not cands:
            raise SystemExit(f"{target}: 최상위 .usd 를 못 찾았다")
        return cands[0]
    return target


def main(argv: list[str] | None = None) -> int:
    from isaaclab.app import AppLauncher

    ap = argparse.ArgumentParser()
    ap.add_argument("target", type=Path, help="자산 디렉터리 또는 .usd 경로")
    ap.add_argument("-o", "--out", type=Path, default=None)
    AppLauncher.add_app_launcher_args(ap)
    ap.set_defaults(headless=True)
    args = ap.parse_args(argv)
    app = AppLauncher(args).app          # ★이 줄 뒤에야 pxr 이 import 된다

    usd = resolve_usd(args.target)
    table = dump(usd)
    out = args.out or usd.with_name(usd.stem + SUFFIX)
    out.write_text(json.dumps(table, indent=1, sort_keys=True) + "\n", encoding="utf-8")
    print(f"[dump_joint_inertia] {usd.name} -> {out} ({len(table)} joints)", flush=True)
    app.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
