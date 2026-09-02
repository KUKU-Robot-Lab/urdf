#!/usr/bin/env python3
"""이미 빌드된 USD 의 **유령 질량**만 외과적으로 0 으로 만든다 (재빌드 없이).

★09.02 실측 사고. URDF 에서 `<inertial>` 이 없는 링크는 "질량 없는 좌표 프레임"인데,
`merge_fixed_joints=False`(fingertip 센서·palm_body 이름 보존을 위해 의도적)로 임포트하면
그 프레임도 각각 rigid body 가 되고, 질량이 안 적혀 있으면 PhysX 가 **기본값 1.0 kg** 을
붙인다. 하필 손끝이라 중력 모멘트가 통째로 바뀌었다:

    l_hl_gripper_tcp 1.0 · r_hl_mount/palm_alias/palm_ee 각 1.0 = 총 **4 kg**
    → sim 좌팔 정적 처짐 j7 **11.07°** vs 실기 4.2° (게인은 70/60/10 로 동일). 2.6배.

    ★★질량 **0 은 소용없다**(09.02 실측): USD 에 0.0 이 적혀 있어도 PhysX 는 강체에
    질량 0 을 허용하지 않아 **기본값 1.0 kg** 으로 대체한다. 그래서 아주 작은 양수를
    넣는다. 1e-4 kg 이면 손끝에서 0.0001×9.81×0.12 ≈ 0.00012 N·m — 실기 j7 중력토크
    0.76 N·m 의 0.02% 라 무시할 수 있고, 솔버도 0 질량 강체 문제를 겪지 않는다.

`build_usd.py` 에는 같은 처리를 `zero_massless_frames()` 로 넣어 새 빌드에서는 재발하지
않는다. 이 스크립트는 **이미 만들어진 자산**(재빌드하면 임포터 드리프트로 질량 외의 것도
바뀔 수 있다)을 최소 변경으로 고치기 위한 것이다.

실행 (Isaac 환경 필요 — pxr 은 앱 기동 후에만 import 된다):

    /home/user/rl_ws/IsaacLab/isaaclab.sh -p tools/patch_ghost_masses.py \\
        --usd /home/user/rl_ws/hdgp/assets/robot/openarm_tesollo_sensor_rl_lgrip/openarm_tesollo_sensor_rl.usd \\
        --urdf /home/user/rl_ws/urdf/generated/rl/openarm_tesollo_sensor_rl.urdf \\
        [--dry-run]

`--dry-run` 이면 무엇을 바꿀지만 출력하고 저장하지 않는다.
"""
import argparse

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
parser.add_argument("--usd", required=True, help="패치할 USD (레이어 스택의 최상위)")
parser.add_argument("--urdf", required=True, help="그 USD 를 만든 URDF")
parser.add_argument("--dry-run", action="store_true")
AppLauncher.add_app_launcher_args(parser)
parser.set_defaults(headless=True)
args_cli = parser.parse_args()

app_launcher = AppLauncher(args_cli)
simulation_app = app_launcher.app

import xml.etree.ElementTree as ET  # noqa: E402
from pathlib import Path  # noqa: E402

from pxr import Gf, Usd, UsdPhysics  # noqa: E402


def main() -> int:
    usd_path, urdf_path = Path(args_cli.usd), Path(args_cli.urdf)
    massless = {lk.get("name") for lk in ET.parse(urdf_path).getroot().findall("link")
                if lk.find("inertial") is None and lk.get("name")}
    print(f"[patch] URDF 무질량 링크 {len(massless)}개: {sorted(massless)}", flush=True)

    stage = Usd.Stage.Open(str(usd_path))
    targets, skipped = [], []
    for prim in stage.Traverse():
        if prim.GetName() not in massless:
            continue
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            skipped.append(prim.GetName())
            continue
        cur = None
        if prim.HasAPI(UsdPhysics.MassAPI):
            attr = UsdPhysics.MassAPI(prim).GetMassAttr()
            cur = attr.Get() if attr else None
        targets.append((prim, prim.GetName(), cur))

    print(f"[patch] 강체인 무질량 프레임 {len(targets)}개"
          + (f" · 강체 아님(건너뜀) {sorted(set(skipped))}" if skipped else ""),
          flush=True)
    for _, name, cur in targets:
        print(f"    {name:<26} 현재 질량 "
              f"{('미기재(PhysX 기본 1.0)' if cur is None else f'{cur:.3f}')}", flush=True)
    if args_cli.dry_run:
        print("[patch] DRY-RUN — 저장하지 않음", flush=True)
        return 0

    for prim, name, _ in targets:
        api = UsdPhysics.MassAPI.Apply(prim)
        api.CreateMassAttr().Set(1e-4)
        api.CreateDiagonalInertiaAttr().Set(Gf.Vec3f(1e-4, 1e-4, 1e-4))
    for layer in stage.GetUsedLayers():
        if not layer.anonymous and layer.permissionToSave and layer.dirty:
            layer.Save()
    print(f"[patch] 저장 완료 — {len(targets)}개 질량 0", flush=True)

    # 재검증: 다시 열어 0 이 아닌 유령이 남았는지 본다
    check = Usd.Stage.Open(str(usd_path))
    left = []
    for prim in check.Traverse():
        if prim.GetName() in massless and prim.HasAPI(UsdPhysics.RigidBodyAPI):
            attr = (UsdPhysics.MassAPI(prim).GetMassAttr()
                    if prim.HasAPI(UsdPhysics.MassAPI) else None)
            val = attr.Get() if attr else None
            if val is None or val > 1e-3:
                left.append((prim.GetName(), val))
    if left:
        print(f"[patch] ★검증 실패 — 유령이 남았다: {left}", flush=True)
        return 1
    print("[patch] 검증 통과 — USD 질량 미세값. ★런타임(PhysX)에서 러너 `mass` 로 재확인할 것", flush=True)
    return 0


if __name__ == "__main__":
    code = main()
    simulation_app.close()
    raise SystemExit(code)
