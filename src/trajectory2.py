"""模式 A 轨迹：分段余弦时间律（节点处速度为 0，加速度全域有界）+ 24/48 fps 复检 + 闸门发布。

为什么不用"整体梯形 + 时间缩放"：路径在中间节点有方向突变（关节速度方向改变），
那是**不连续加速度**，缩放时间只能把它按 1/k² 压小，压到合规需要 47 倍时间（791 s），不可用。
分段余弦在每个节点把速度收到 0，加速度幅值有界，代价是节点处略作停顿——对数字样机可接受，
而且这个停顿是真实的动作规划选择，不是渲染层的障眼法。
"""
import importlib.util, json, math, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.util.module_from_spec(importlib.util.spec_from_file_location('pl', HERE / 'planner.py'))
pl = importlib.import_module('planner')
pb = importlib.import_module('publish')

V_J, A_J = 30.0, 120.0          # deg/s, deg/s^2  设计假设
V_T, A_T = 250.0, 1000.0        # mm/s, mm/s^2    设计假设

Q = np.array(json.loads((OUT / 'planA.json').read_text(encoding='utf8'))['best']['path'], float)


def build(pts):
    segs, cum = [], [0.0]
    for a, b in zip(pts[:-1], pts[1:]):
        d = b - a
        L = float(np.linalg.norm(d))
        if L < 1e-9:
            continue
        r = np.abs(d) / L
        vs = [V_J / r[i] for i in range(4) if r[i] > 1e-9] + ([V_T / r[4]] if r[4] > 1e-9 else [])
        as_ = [A_J / r[i] for i in range(4) if r[i] > 1e-9] + ([A_T / r[4]] if r[4] > 1e-9 else [])
        vlim, alim = min(vs), min(as_)
        T = max(math.pi * L / (2 * vlim), math.pi * math.sqrt(L / (2 * alim)))   # 余弦峰值 = Lπ/(2T)
        segs.append(dict(a=a, d=d, L=L, T=T, vlim=vlim, alim=alim))
        cum.append(cum[-1] + T)
    return segs, cum


def state(segs, cum, t):
    t = min(max(t, 0.0), cum[-1])
    k = int(np.clip(np.searchsorted(cum, t) - 1, 0, len(segs) - 1))
    tau = 0.0 if segs[k]['T'] < 1e-12 else (t - cum[k]) / segs[k]['T']
    s = segs[k]['L'] * (1 - math.cos(math.pi * tau)) / 2
    return segs[k]['a'] + segs[k]['d'] * (s / max(1e-12, segs[k]['L']))


def sample(segs, cum, fps):
    n = int(math.ceil(cum[-1] * fps)) + 1
    ts = np.arange(n) / fps
    return ts, np.array([state(segs, cum, t) for t in ts])


def verify(states, ts):
    film = hz = 1e9
    tw = None
    for q, t in zip(states, ts):
        ok, f, n, h = pl.violation(q[:4], q[4], want_clear=True)
        af = pl.abs_film(q[:4], q[4])
        if af < film:
            film, tw = af, float(t)
        hz = min(hz, h if h > -1e8 else 1e9)
        if not ok:
            return dict(ok=False, at=round(float(t), 3), film_mm=round(af, 2))
    return dict(ok=True, film_mm=round(float(film), 2), housing_mm=round(float(hz), 2),
                tightest_t=round(tw, 2))


def kin(states, fps):
    v = np.abs(np.diff(states, axis=0)) * fps
    a = np.abs(np.diff(states, n=2, axis=0)) * fps * fps
    return (dict(joint_degs=round(float(v[:, :4].max()), 2), tel_mms=round(float(v[:, 4].max()), 2)),
            dict(joint_degs2=round(float(a[:, :4].max()), 1), tel_mms2=round(float(a[:, 4].max()), 1)))


if __name__ == '__main__':
    segs, cum = build(Q)
    T = cum[-1]
    print(f'段数 {len(segs)}  总时长 {T:.2f}s')
    for i, s in enumerate(segs):
        print(f'  seg{i}: L={s["L"]:.1f} T={s["T"]:.2f}s vlim={s["vlim"]:.1f}/s alim={s["alim"]:.1f}/s²')

    ts240, s240 = sample(segs, cum, 240)
    v, a = kin(s240, 240)
    ok240 = verify(s240, ts240)
    ts24, s24 = sample(segs, cum, 24)
    ts48, s48 = sample(segs, cum, 48)
    c24, c48 = verify(s24, ts24), verify(s48, ts48)
    print('240fps 实测:', v, a)
    print('24fps :', c24)
    print('48fps :', c48)
    compliant = (v['joint_degs'] <= V_J + 0.5 and a['joint_degs2'] <= A_J * 1.02
                 and v['tel_mms'] <= V_T + 1 and a['tel_mms2'] <= A_T * 1.02
                 and c24['ok'] and c48['ok'] and ok240['ok'])
    print('合规:', compliant)

    rid, d = pb.new_run('trajectory', dict(
        fp_source='planner.py@5D+cosine', fp_geometry='roof_path+wall+housing',
        fp_params=dict(R=pl.R, LIM=pl.LIM, V_J=V_J, A_J=A_J, V_T=V_T, A_T=A_T),
        endpoints=dict(start=Q[0].tolist(), goal=Q[-1].tolist())))
    payload = dict(meta=dict(run_id=rid, total_s=round(T, 3), segments=len(segs),
                             kinematically_compliant=bool(compliant)),
                   path=[list(map(float, r)) for r in Q],
                   profile=[dict(T=round(s['T'], 3), L=round(s['L'], 1),
                                 vlim=round(s['vlim'], 2), alim=round(s['alim'], 2)) for s in segs],
                   trajectory=dict(t=[round(float(x), 4) for x in ts24],
                                   q=[[round(float(v_), 3) for v_ in row] for row in s24]),
                   checks=dict(f240=ok240, f24=c24, f48=c48, velocity=v, acceleration=a),
                   assumptions=dict(v_joint_degs=V_J, a_joint_degs2=A_J, v_tel_mms=V_T, a_tel_mms2=A_T,
                                    note='设计假设，非执行器认证；间隙为当前候选模型值，非安全净空；节点处速度归零'))
    pb.finish(d, payload, status='ok' if compliant else 'invalid',
              note='运动学' + ('合规' if compliant else '不合规'))
    ok, why = pb.publish(d, OUT / 'planA_trajectory.json',
                         expect=dict(fp_source='planner.py@5D+cosine', fp_geometry='roof_path+wall+housing'))
    print('发布:', ok, why, '| run', rid)
