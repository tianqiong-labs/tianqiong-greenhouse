"""gap_v2：以“连杆到棚膜的垂直最短距离”为约束，重规划 紧凑→预接近 段。

约束（全部进入 RRT 状态判据，不是事后筛选）：
  - planner 原有：关节限位、伸缩行程、地面、墙面、卷筒壳体
  - 新增：链上每个点到棚膜折线的垂直距离 - R_JOINT >= target
目标 20 与 30 各跑一组；结果写入 out/experiments/gap_v2，不触碰 baseline/ 与正式轨迹。
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
EXP = OUT / 'experiments' / 'gap_v2'
EXP.mkdir(parents=True, exist_ok=True)
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')
pb = importlib.import_module('publish')

FILM = pl.FILM
R_J = pl.R_JOINT if hasattr(pl, 'R_JOINT') else 73.0
R = 73.0


def film_perp(P):
    """链上点到棚膜折线【线段】的最短距离（修正：此前误用顶点距离，会高估间隙）。"""
    A = FILM[:-1]; B = FILM[1:]
    AB = B - A; L2 = (AB ** 2).sum(1)
    P2 = P[:, [0, 2]]
    t = np.clip(((P2[:, None, :] - A[None, :, :]) * AB[None, :, :]).sum(2) / L2[None, :], 0, 1)
    proj = A[None, :, :] + t[:, :, None] * AB[None, :, :]
    return float(np.linalg.norm(P2[:, None, :] - proj, axis=2).min()) - R


_ORIG = pl.violation


def make_validator(target):
    def v(q, L4, yaw=0.0, dy=0.0, want_clear=False):
        ok = _ORIG(q, L4, yaw, dy)
        if not ok:
            return (False, -1e9, -1e9, -1e9) if want_clear else False
        P = pl.chain3d(q, L4, yaw, dy, n_per=10)
        gap = film_perp(P)
        if gap < target:
            return (False, gap, -1e9, -1e9) if want_clear else False
        if want_clear:
            return True, gap, 1e9, 1e9
        return True
    return v


def seg_metrics(nodes, target):
    segs, cum = t2.build(np.asarray(nodes, float))
    ts24, st24 = t2.sample(segs, cum, 24)
    ts240, st240 = t2.sample(segs, cum, 240)
    gaps = []
    for q in st240:
        gaps.append(film_perp(pl.chain3d(q[:4], q[4], 0.0, 0.0, n_per=10)))
    v, a = t2.kin(st240, 240)
    tip = pl.chain3d(st24[-1][:4], st24[-1][4], 0.0, 0.0, n_per=2)[-1]
    return dict(T=round(cum[-1], 2), film_perp_min=round(min(gaps), 2),
                n24=len(ts24), n240=len(ts240), v=v, a=a,
                tip=[round(float(x), 1) for x in tip],
                nodes=len(nodes))


if __name__ == '__main__':
    Q = np.array(json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))['path'], float)
    q_pack = Q[-1]
    loop = json.loads((OUT / 'modeA_loop.json').read_text(encoding='utf8'))
    q_pre = np.r_[loop['ik']['pre']['q'], loop['ik']['pre']['L4']]

    base = seg_metrics([q_pack, q_pre], 0)
    print('v1 基线（紧凑→预接近）:', base)
    rows = [dict(version='v1-baseline', target=0, **base)]

    for target in (20.0, 30.0):
        pl.violation = make_validator(target)
        try:
            path = None
            for sd in (0, 1, 2, 3):
                p, it, dt = pl.rrt_connect(q_pack, q_pre, 5, budget=28.0, seed=sd)
                if p is not None:
                    path = pl.shortcut(p, 5, seed=sd); break
            if path is None:
                rows.append(dict(version=f'v2-target{int(target)}', target=target, found=False))
                print(f'  目标 {target:.0f} mm: 未找到')
                continue
            m = seg_metrics(path, target)
            # 用原始判据再确认没有突破其它约束
            pl.violation = _ORIG
            ok_all = all(_ORIG(q[:4], q[4]) for q in path)
            rows.append(dict(version=f'v2-target{int(target)}', target=target, found=True,
                             ok_other_constraints=bool(ok_all), **m,
                             path=[[round(float(x), 2) for x in r] for r in np.asarray(path, float)]))
            print(f'  目标 {target:.0f} mm: FOUND  T={m["T"]}s  垂直膜间隙 {m["film_perp_min"]}  v={m["v"]}  a={m["a"]}')
        finally:
            pl.violation = _ORIG

    (EXP / 'compare.json').write_text(json.dumps(dict(
        metric='连杆到棚膜折线的垂直最短距离（已扣半径 73 mm）',
        rows=rows), ensure_ascii=False, indent=1), encoding='utf8')
    print('\n写入', EXP / 'compare.json')
    for r in rows:
        print('  ', {k: r[k] for k in ('version', 'found', 'film_perp_min', 'T') if k in r})
