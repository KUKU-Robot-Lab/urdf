#!/usr/bin/env python3
"""한 자산의 팔 시작 자세를 **다른 자산의 palm 포즈에 맞춰** 푼다(순수 기구학, GPU 불필요).

왜 필요한가 (2026-09-10). `RobotProfile.arm_reset_joint_pos` 는 **절대 관절값**이라
자산마다 따로 풀어야 한다. 손 마운트 길이가 다르면(dg5f-m 대비 short 는 palm 이 47.8mm
당겨졌다) 같은 palm 포즈를 내는 팔 관절값이 달라지기 때문이다. 구 설계는 이걸 홈 기준
**델타**로 들고 있어서 이식이 불가능했고, short 가 홈에 앉아 시작 거리 243mm 로 돌았다.

목표는 palm 링크의 **world 포즈**(위치 + 회전)다. palm 이후 손 체인은 두 자산이 동일하므로
palm 이 맞으면 손끝도 맞는다(프로필 주석: 손 전 프레임 0.023mm 이내 일치).

    python3 tools/solve_arm_reset_pose.py \
        --src generated/rl/openarm_dg5f-m_bi_rl.urdf \
        --dst generated/rl/openarm_dg5f-m-short_bi_rl.urdf \
        --src-q -0.1587,0.0283,0.3856,0.9464,-0.2090,0.3247,0.8023 \
        --dst-seed 0.2667,0.4487,0.4923,0.7184,-0.0460,0.6496,0.4762 \
        --link r_hl_palm --joints 'r_aj_([1-7])'
"""

from __future__ import annotations

import argparse
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares


def _rpy(r: float, p: float, y: float) -> np.ndarray:
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp,     cp * sr,                cp * cr]])


def _axis_rot(a: np.ndarray, t: float) -> np.ndarray:
    a = a / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


class Urdf:
    def __init__(self, path: Path):
        root = ET.parse(path).getroot()
        self.joints: dict[str, dict] = {}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = np.array([float(v) for v in (o.get("xyz") or "0 0 0").split()]) if o is not None else np.zeros(3)
            rpy = [float(v) for v in (o.get("rpy") or "0 0 0").split()] if o is not None else [0.0, 0.0, 0.0]
            a = j.find("axis")
            lim = j.find("limit")
            self.joints[j.get("name")] = dict(
                parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                xyz=xyz, R=_rpy(*rpy), type=j.get("type"),
                axis=np.array([float(v) for v in a.get("xyz").split()]) if a is not None else None,
                lo=float(lim.get("lower")) if lim is not None and lim.get("lower") else None,
                hi=float(lim.get("upper")) if lim is not None and lim.get("upper") else None)
        self.by_child = {v["child"]: (k, v) for k, v in self.joints.items()}

    def chain(self, link: str) -> list[tuple[str, dict]]:
        out, cur = [], link
        while cur in self.by_child:
            name, jj = self.by_child[cur]
            out.append((name, jj))
            cur = jj["parent"]
        return list(reversed(out))

    def pose(self, link: str, q: dict[str, float]) -> np.ndarray:
        T = np.eye(4)
        for name, jj in self.chain(link):
            R = jj["R"]
            if jj["type"] in ("revolute", "continuous"):
                R = R @ _axis_rot(jj["axis"], q.get(name, 0.0))
            M = np.eye(4)
            M[:3, :3], M[:3, 3] = R, jj["xyz"]
            T = T @ M
        return T


def _rot_err(Ra: np.ndarray, Rb: np.ndarray) -> np.ndarray:
    """회전 오차를 3벡터(축·각)로."""
    E = Ra.T @ Rb
    ang = math.acos(max(-1.0, min(1.0, (np.trace(E) - 1.0) / 2.0)))
    if ang < 1e-9:
        return np.zeros(3)
    v = np.array([E[2, 1] - E[1, 2], E[0, 2] - E[2, 0], E[1, 0] - E[0, 1]])
    return v / (2.0 * math.sin(ang)) * ang


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", type=Path, required=True, help="목표 포즈를 내는 자산 URDF")
    ap.add_argument("--dst", type=Path, required=True, help="풀 자산 URDF")
    ap.add_argument("--src-q", required=True, help="src 의 팔 관절값 (쉼표)")
    ap.add_argument("--dst-seed", required=True, help="dst 초기 추정 (쉼표) — 보통 그 자산의 홈")
    ap.add_argument("--link", default="r_hl_palm")
    ap.add_argument("--joints", default=r"r_aj_([1-7])")
    ap.add_argument("--rot-weight", type=float, default=0.3, help="회전 오차 가중(m/rad)")
    ap.add_argument("--target-offset", default=None,
                    help="src 포즈에서 이만큼 **평행이동**한 곳을 목표로 (dx,dy,dz m). "
                         "회전은 src 그대로 — 같은 접근 자세를 유지하며 물러날 때 쓴다.")
    args = ap.parse_args(argv)

    src, dst = Urdf(args.src), Urdf(args.dst)
    pat = re.compile(args.joints)
    names = sorted((n for n in src.joints if pat.fullmatch(n)), key=lambda n: pat.fullmatch(n).group(1))
    if [n for n in dst.joints if pat.fullmatch(n)] == []:
        raise SystemExit(f"{args.dst}: '{args.joints}' 에 걸리는 관절이 없다")

    q_src = [float(v) for v in args.src_q.split(",")]
    seed = np.array([float(v) for v in args.dst_seed.split(",")])
    if not (len(q_src) == len(seed) == len(names)):
        raise SystemExit(f"관절 수 불일치: names {len(names)} · src-q {len(q_src)} · seed {len(seed)}")

    T_tgt = src.pose(args.link, dict(zip(names, q_src)))
    if args.target_offset:
        T_tgt = T_tgt.copy()
        T_tgt[:3, 3] = T_tgt[:3, 3] + np.array([float(v) for v in args.target_offset.split(",")])
    lo = np.array([dst.joints[n]["lo"] for n in names])
    hi = np.array([dst.joints[n]["hi"] for n in names])

    def resid(q):
        T = dst.pose(args.link, dict(zip(names, q)))
        return np.concatenate([T[:3, 3] - T_tgt[:3, 3],
                               args.rot_weight * _rot_err(T_tgt[:3, :3], T[:3, :3])])

    best = None
    rng = np.random.default_rng(0)
    for k in range(12):
        x0 = seed if k == 0 else np.clip(seed + rng.normal(0.0, 0.25, size=seed.shape), lo, hi)
        r = least_squares(resid, np.clip(x0, lo, hi), bounds=(lo, hi), xtol=1e-14, ftol=1e-14, gtol=1e-14)
        if best is None or r.cost < best.cost:
            best = r

    T = dst.pose(args.link, dict(zip(names, best.x)))
    dp = float(np.linalg.norm(T[:3, 3] - T_tgt[:3, 3]))
    dr = float(np.linalg.norm(_rot_err(T_tgt[:3, :3], T[:3, :3])))
    margin = float(min(np.min(best.x - lo), np.min(hi - best.x)))
    print(f"[solve_arm_reset_pose] 목표 {args.link} = {np.round(T_tgt[:3, 3], 6).tolist()}")
    print(f"[solve_arm_reset_pose] 수렴 위치오차 {dp * 1e3:.4f} mm · 회전오차 {math.degrees(dr):.4f}° "
          f"· 관절한계 여유 {margin:.4f} rad ({math.degrees(margin):.2f}°)")
    print("arm_reset_joint_pos=(" + ", ".join(f"{v:.4f}" for v in best.x) + "),")
    return 0 if (dp < 1e-3 and dr < math.radians(0.5)) else 1


if __name__ == "__main__":
    sys.exit(main())
