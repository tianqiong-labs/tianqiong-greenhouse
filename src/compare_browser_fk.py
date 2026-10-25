"""浏览器渲染节点 TRS ↔ FK 独立比对（不分解 matrixWorld，直接读 position/quaternion）。

已核实：对象父级为单位变换（Group/Scene 均 identity），因此渲染节点的世界变换 == 自身 TRS。
比较量：位置差的模、四元数夹角。
"""
import importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
bind = importlib.import_module('bind_heads')
anim = bind.anim
m = anim.m
HERO = anim.HERO
Y = [float(x) for x in pl.Y_WORK]


def quat_angle(a, b):
    a = np.asarray(a, float); b = np.asarray(b, float)
    d = abs(float(np.dot(a, b)) / (np.linalg.norm(a) * np.linalg.norm(b)))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


def q_at(tl, frames, t):
    if abs(t - tl['total_s']) < 1e-6:
        return tl['exact_endpoint']['q']
    k = int(round(t * tl['fps']))
    return frames[max(0, min(len(frames) - 1, k))]['q']


def expected(q, BINDS, R0S):
    out = {}
    for idx in (1, 4):
        y_abs = HERO * anim.PITCH + Y[idx]
        recs, _ = bind.packlib.hero_arm(f'M{HERO+1}_M1_', idx, y_abs,
                                        np.asarray(q[:4], float), float(q[4]), m)
        for r in recs:
            pos, quat, _sc = m.transform(r)
            out[r['name']] = (np.asarray(pos, float), np.asarray(quat, float))
        w = bind.arm_world(BINDS[idx], q, idx, y_abs, R0s=R0S)
        for nm, (pos, quat) in w.items():
            out[nm] = (np.asarray(pos, float), np.asarray(quat, float))
    return out


if __name__ == '__main__':
    tl = json.loads((OUT / 'modeA_v2_timeline.json').read_text(encoding='utf8'))
    frames = tl['frame_records']
    data = json.loads((OUT / 'browser_trs.json').read_text(encoding='utf8'))

    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0, yaw=0.0,
              lower=0.0, compensate=0.0, pack=0.0)
    base = anim.frame_records(st)
    q0 = frames[0]['q']
    BINDS, R0S = {}, {}
    for idx in (1, 4):
        y_abs = HERO * anim.PITCH + Y[idx]
        recs_i = [r for r in base if r['name'].startswith(f'M{HERO+1}_')
                  and f'_A{idx+1:02d}_' in r['name'] and r['group'] in ('head', 'gate', 'magnet')]
        bl = bind.build_bindings(recs_i, q0, y_abs, idx)
        pts0 = bind.arm_points(q0, idx, y_abs)
        for b in bl:
            R0, _ = bind.frame_from(pts0, b['J'] - 1)
            R0S[b['name']] = R0
            b['arm'] = idx
        BINDS[idx] = bl

    rows = []
    for rec in data:
        t = rec['t']
        exp = expected(q_at(tl, frames, t), BINDS, R0S)
        for nm, v in rec['o'].items():
            if nm not in exp:
                continue
            bp = np.array(v[0:3], float); bq = np.array(v[3:7], float)
            ep, eq = exp[nm]
            rows.append(dict(t=t, name=nm, pos=float(np.linalg.norm(bp - ep)),
                             rot=quat_angle(bq, eq)))
    pos = np.array([r['pos'] for r in rows]); rot = np.array([r['rot'] for r in rows])
    worst = sorted(rows, key=lambda r: -r['pos'])[:3]
    print(f'样本 {len(rows)}（{len(data)} 时刻 × {len(data[0]["o"])} 对象）')
    print(f'最大位置误差 {pos.max():.6f} mm  平均 {pos.mean():.6f}')
    print(f'最大旋转误差 {rot.max():.6f}°   平均 {rot.mean():.6f}')
    print('位置差最大 3 项:', [(r['name'], round(r['pos'], 6), r['t']) for r in worst])
    ok = pos.max() < 0.5 and rot.max() < 0.05
    res = dict(samples=len(rows), objects=sorted({r['name'] for r in rows}),
               times=[r['t'] for r in data],
               max_pos_err_mm=round(float(pos.max()), 6), mean_pos_err_mm=round(float(pos.mean()), 6),
               max_rot_err_deg=round(float(rot.max()), 6), mean_rot_err_deg=round(float(rot.mean()), 6),
               worst=worst, verdict='PASS' if ok else 'FAIL',
               note='读取浏览器节点的 position/quaternion 原始值，父级已核实为 identity')
    (OUT / 'browser_fk_check.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    print('判定:', res['verdict'])
