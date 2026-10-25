"""v1 vs v2-30 的完整六臂场景对比验收（同一代理、同一定义、同一白名单、补位后 Y）。

v2 只替换"紧凑→预接近"一段，其余段与 v1 相同；两版都在同一时间轴体系下重算。
"""
import importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')
sc = importlib.import_module('scene_check')

R_LINK = sc.R_LINK


def timeline_with(mid_nodes):
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    loop = json.loads((OUT / 'modeA_loop.json').read_text(encoding='utf8'))
    fold = np.array(tr['path'], float)
    q_dep, q_pack = fold[0], fold[-1]
    q_pre = np.r_[loop['ik']['pre']['q'], loop['ik']['pre']['L4']]
    q_work = np.r_[loop['ik']['work']['q'], loop['ik']['work']['L4']]
    legs = [('部署→紧凑', fold), ('紧凑→预接近', np.asarray(mid_nodes, float)), ('预接近→采摘', np.vstack([q_pre, q_work]))]
    seq = []
    for name, nodes in legs:
        ts, st = t2.sample(*t2.build(np.asarray(nodes, float)), 24)
        seq.append((name, ts, st))
    for name, nodes in reversed(legs):
        ts, st = t2.sample(*t2.build(np.asarray(nodes, float)[::-1]), 24)
        seq.append((f'返回·{name}', ts, st))
    tl, off = [], 0.0
    for name, ts, st in seq:
        tl += [(off + float(t), name, q) for t, q in zip(ts, st)]
        off += float(ts[-1])
    return tl, off, fold, q_pack, q_pre


def evaluate(tl):
    y_h = float(pl.Y_WORK[1]); y5 = float(pl.Y_WORK[4])
    statics = [(i, y, sc.chain(pl.A0, 1600.0, y)) for i, y in enumerate(pl.Y_WORK) if i not in (1, 4)]
    rail_lo = np.array([4620 - 60, -1e9, -60.0]); rail_hi = np.array([4620 + 60, 1e9, 60.0])
    wall_lo = np.array([-600.0, -1e9, 0.0]); wall_hi = np.array([0.0, 1e9, 3150.0])
    hz_lo = np.array([pl.HZ_LO[0], -1e9, pl.HZ_LO[1]]); hz_hi = np.array([pl.HZ_HI[0], 1e9, pl.HZ_HI[1]])
    w = dict(film=1e9, static=1e9, housing=1e9, rail=1e9, wall=1e9)
    at = {k: None for k in w}
    bad = 0
    tight = []
    for t, leg, q in tl:
        Pm = sc.chain(q[:4], q[4], y_h)
        vals = dict(film=sc.film_gap(Pm),
                    static=min(sc.scene_gap(Pm, S, y_h, y) for _, y, S in statics),
                    housing=sc.box_gap(Pm, hz_lo, hz_hi, 0.0),
                    rail=sc.box_gap(Pm, rail_lo, rail_hi, 0.0),
                    wall=sc.box_gap(Pm, wall_lo, wall_hi, 0.0))
        for k, v in vals.items():
            if v < w[k]:
                w[k], at[k] = v, (round(t, 2), leg)
            if v < 0:
                bad += 1
        if vals['film'] < 100:
            tight.append((t, leg))
    return w, at, bad, tight


def dense_window(tl, window, fps=240):
    """在给定时间窗内按 240 fps 重采（此处对整段做加密采样检查）。"""
    lo, hi = window
    sel = [q for t, leg, q in tl if lo <= t <= hi]
    if not sel:
        return None
    gaps = [sc.film_gap(sc.chain(q[:4], q[4], float(pl.Y_WORK[1]))) for q in sel]
    return round(min(gaps), 2)


if __name__ == '__main__':
    cmp = json.loads((OUT / 'experiments' / 'gap_v2' / 'compare.json').read_text(encoding='utf8'))
    v2 = next(r for r in cmp['rows'] if r['version'] == 'v2-target30')['path']
    loop = json.loads((OUT / 'modeA_loop.json').read_text(encoding='utf8'))
    v1_mid = np.array(loop['legs'][0]['traj']['q'], float)   # v1 真实轨迹采样（24fps）

    res = {}
    for name, mid in (('v1', v1_mid), ('v2-30', np.array(v2, float))):
        tl, T, fold, q_pack, q_pre = timeline_with(mid)
        w, at, bad, tight = evaluate(tl)
        dense = dense_window(tl, (min(t for t, _ in tight), max(t for t, _ in tight))) if tight else None
        res[name] = dict(total_s=round(T, 2), frames=len(tl), worst={k: round(v, 1) for k, v in w.items()},
                         at={k: at[k] for k in at}, violations=bad,
                         dense_film=dense, tight_frames=len(tight))
        print(f'{name}: T={T:.2f}s 帧{len(tl)} 膜{w["film"]:.1f} 邻{w["static"]:.1f} 壳{w["housing"]:.1f} '
              f'穿透{bad} 紧区{dense}mm')

    (OUT / 'experiments' / 'gap_v2' / 'scene_compare.json').write_text(
        json.dumps(dict(model=dict(r_link=R_LINK, r_joint=sc.R_JOINT, y_state='compensated',
                                   y_work=pl.Y_WORK.tolist()),
                        results=res), ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 experiments/gap_v2/scene_compare.json')
