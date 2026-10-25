"""模式 A 第二阶段：yaw 回转段 + 作业目标位可达性（按用户给的坐标假设）。

作业目标（设计假设，非作物工艺）：
    Z=0 地面；冠层 800；示意采摘目标 Z=750；预接近位 Z=950；X=2600；
    Y 以各作业臂根座为基准，允许搜索偏移。
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')

X_WORK, Z_WORK, Z_PRE = 2600.0, 750.0, 950.0
YAW_TARGET = 35.0


def tip_of(q, L4, yaw=0.0, dy=0.0):
    P = pl.chain3d(q, L4, yaw, dy, n_per=2)
    return P[-1]


def ok_full(q, L4, yaw=0.0, dy=0.0):
    return pl.violation(q, L4, yaw, dy, want_clear=True)


def ik_search(y_root, target, yaw, dy, n=120000, seed=3, keep=40):
    """随机采样 + 局部细化，找末端命中目标且全约束合法的构型。"""
    rng = np.random.default_rng(seed)
    A = rng.uniform(-150, 150, size=(n, 4))
    L4 = rng.uniform(1000, 1600, size=(n, 1))
    cand = np.hstack([A, L4])
    errs = []
    for c in cand:
        p = tip_of(c[:4], c[4], yaw, dy)
        # x/z 命中 + Y 偏移限制在 0..800 mm（以根座为基准的可达邻域）
        ypen = 0.0 if 0.0 <= p[1] <= 800.0 else min(abs(p[1]), abs(p[1] - 800.0)) * 3
        errs.append(abs(p[2] - target[2]) + abs(p[0] - target[0]) + ypen)
    errs = np.array(errs)
    idx = np.argsort(errs)[:keep]
    best = None
    for i in idx:
        q = cand[i].copy()
        e = errs[i]
        for _ in range(120):
            step = np.zeros(5)
            step[:4] = rng.normal(0, 1.5, 4)
            step[4] = rng.normal(0, 8.0)
            for s in (1.0, -1.0):
                t = q + s * step
                t[:4] = np.clip(t[:4], -150, 150); t[4] = np.clip(t[4], 1000, 1600)
                p = tip_of(t[:4], t[4], yaw, dy)
                ypen = 0.0 if 0.0 <= p[1] <= 800.0 else min(abs(p[1]), abs(p[1] - 800.0)) * 3
                ne = abs(p[2] - target[2]) + abs(p[0] - target[0]) + ypen
                if ne < e:
                    q, e = t, ne
                    break
        ok, f, nb, hz = ok_full(q[:4], q[4], yaw, dy)
        p0 = tip_of(q[:4], q[4], yaw, dy)
        if ok and 0.0 <= p0[1] <= 800.0 and (best is None or e < best['err']):
            p = tip_of(q[:4], q[4], yaw, dy)
            best = dict(err=float(e), q=[round(float(x), 1) for x in q[:4]], L4=round(float(q[4]), 1),
                        tip=[round(float(x), 1) for x in p], film=round(float(f), 1),
                        housing=round(float(hz), 1),
                        neighbour=(round(float(nb), 1) if nb > -1e8 else None))
    return best


def yaw_sweep(y_root, q_pack, L4_pack, steps=71):
    """紧凑态下 0→35° 逐帧扫掠，返回最小间隙与最紧时刻。"""
    film, nb, hz, worst = 1e9, 1e9, 1e9, None
    for k in range(steps):
        yaw = YAW_TARGET * k / (steps - 1)
        ok, f, n, h = ok_full(q_pack, L4_pack, yaw, 0.0)
        af = pl.abs_film(q_pack, L4_pack, yaw, 0.0)
        if af < film:
            film, worst = af, yaw
        nb = min(nb, n if n > -1e8 else 1e9)
        hz = min(hz, h if h > -1e8 else 1e9)
        if not ok:
            return dict(ok=False, at_yaw=round(yaw, 2), film=round(af, 2))
    return dict(ok=True, steps=steps, film=round(float(film), 2), housing=round(float(hz), 2),
                neighbour=round(float(nb), 2) if nb < 1e8 else None,
                tightest_yaw=round(float(worst), 1))


if __name__ == '__main__':
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    q_pack = tr['path'][-1][:4]
    L4_pack = tr['path'][-1][4]
    print('紧凑姿态:', [round(x, 1) for x in q_pack], 'L4', L4_pack)

    sw = yaw_sweep(pl.Y0[pl.HERO], q_pack, L4_pack)
    print('yaw 0→35° 扫掠:', sw)

    out = dict(yaw_sweep=sw, targets={})
    for idx in (1, 4):
        y_root = float(pl.Y0[idx])
        for name, z in (('pre', Z_PRE), ('work', Z_WORK)):
            r = ik_search(y_root, (X_WORK, y_root, z), YAW_TARGET, 0.0)
            key = f'A{idx+1:02d}_{name}'
            out['targets'][key] = dict(z=z, y_root=y_root, result=r)
            print(f'  {key}: ' + (f"命中 误差{r['err']:.1f} mm  末端{r['tip']}  膜{r['film']} 壳体{r['housing']} 邻臂{r['neighbour']}"
                                 if r else '不可达（无合法构型）'))
    (OUT / 'modeA_stage2.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/modeA_stage2.json')
