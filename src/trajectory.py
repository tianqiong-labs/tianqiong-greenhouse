"""5 维路径的时间参数化 + 24/48 fps 逐帧复检 + 经闸门发布。

时间参数化：沿已验几何路径做**时间缩放**（不改路径形状），梯形速度曲线。
限制值为设计假设，不是执行器认证：
    关节速度 30 °/s，加速度 120 °/s²
    第四段伸缩 250 mm/s，加速度 1000 mm/s²
即"最慢关节项"决定整条路径的可行速度，属保守做法。
"""
import importlib.util, json, math, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
spec = importlib.util.spec_from_file_location('pl', HERE / 'planner.py')
pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)
pub = importlib.util.module_from_spec(importlib.util.spec_from_file_location('pb', HERE / 'publish.py'))
import importlib
pb = importlib.import_module('publish')

V_JOINT, A_JOINT = 30.0, 120.0        # deg/s, deg/s^2  (设计假设)
V_TEL, A_TEL = 250.0, 1000.0          # mm/s, mm/s^2   (设计假设)

PATH = json.loads((OUT / 'planA.json').read_text(encoding='utf8'))['best']['path']
Q = np.array(PATH, float)             # (N,5)


def dense(qs, step_deg=0.5, step_mm=5.0):
    out = [qs[0]]
    for a, b in zip(qs[:-1], qs[1:]):
        n = int(max(1, math.ceil(max(np.abs(b[:4] - a[:4]).max() / step_deg,
                                    abs(b[4] - a[4]) / step_mm)))) * 2
        for t in np.linspace(0, 1, n + 1)[1:]:
            out.append(a + (b - a) * t)
    return np.array(out)


def limits_along(path):
    """把各关节的速度/加速度限制映射成路径弧长上的允许值。"""
    seg = np.diff(path, axis=0)
    L = np.linalg.norm(seg, axis=1)
    keep = L > 1e-9
    seg, L = seg[keep], L[keep]
    vmax = np.inf; amax = np.inf
    for d, s in zip(seg, L):
        r = np.abs(d) / s                       # 单位弧长上各关节变化率
        for i in range(4):
            if r[i] > 1e-12:
                vmax = min(vmax, V_JOINT / r[i])
        if r[4] > 1e-12:
            vmax = min(vmax, V_TEL / r[4])
    # 加速度按同样比例保守估计（曲率忽略，路径近似直线段拼成）
    for d, s in zip(seg, L):
        r = np.abs(d) / s
        for i in range(4):
            if r[i] > 1e-12:
                amax = min(amax, A_JOINT / r[i])
        if r[4] > 1e-12:
            amax = min(amax, A_TEL / r[4])
    return float(vmax), float(amax)


def time_profile(path):
    seg = np.diff(path, axis=0)
    L = float(np.linalg.norm(seg, axis=1).sum())
    vmax, amax = limits_along(path)
    if L >= vmax * vmax / amax:
        ta = vmax / amax
        T = ta + (L - 0.5 * amax * ta * ta) / vmax
    else:
        vp = math.sqrt(amax * L)
        ta = vp / amax
        T = 2 * ta
    return L, vmax, amax, T, ta


def state_at(s_path, cum, path, s):
    i = int(np.clip(np.searchsorted(cum, s) - 1, 0, len(cum) - 2))
    seg_len = cum[i + 1] - cum[i]
    f = 0.0 if seg_len < 1e-12 else (s - cum[i]) / seg_len
    return path[i] + (path[i + 1] - path[i]) * float(np.clip(f, 0, 1))


def sample(T, ta, L, amax, vmax, cum, path, fps):
    n = int(math.ceil(T * fps)) + 1
    ts = np.arange(n) / fps
    rows = []
    for t in ts:
        if t < ta:
            s = 0.5 * amax * t * t
        elif t < T - ta:
            s = 0.5 * amax * ta * ta + vmax * (t - ta)
        else:
            td = T - t
            s = L - 0.5 * amax * td * td
        rows.append(state_at(min(max(s, 0), L), cum, path, s))
    return ts, np.array(rows)


def verify(states, ts, tag):
    film = hz = nb = 1e9
    worst_t = None
    for q, t in zip(states, ts):
        ok, f, n, h = pl.violation(q[:4], q[4], want_clear=True)
        af = pl.abs_film(q[:4], q[4])
        if af < film: film, worst_t = af, t
        hz = min(hz, h if h > -1e8 else 1e9)
        nb = min(nb, n if n > -1e8 else 1e9)
        if not ok:
            return dict(tag=tag, ok=False, at=round(float(t), 3), film=round(af, 1))
    return dict(tag=tag, ok=True, film_mm=round(float(film), 2), housing_mm=round(float(hz), 2),
                neighbour_mm=round(float(nb), 2) if nb < 1e8 else None,
                tightest_t=round(float(worst_t), 3) if worst_t is not None else None)


def kinematics(states, dt):
    v = np.abs(np.diff(states, axis=0)) / dt
    a = np.abs(np.diff(states, n=2, axis=0)) / (dt * dt)
    return (dict(joint_max_degs=round(float(v[:, :4].max()), 2),
                 tel_max_mms=round(float(v[:, 4].max()), 1)),
            dict(joint_max_degs2=round(float(a[:, :4].max()), 1),
                 tel_max_mms2=round(float(a[:, 4].max()), 1)))


def fit_time(P, cum, L, vmax, amax, ta, tol=1e-3, iters=12):
    """反复放大时间，直到实测关节速度/加速度同时落入限值。"""
    k = 1.0
    hist = []
    for it in range(iters):
        Ts = (
            L / max(1e-9, vmax / k) + (vmax / k) / max(1e-9, amax / (k * k)))
        # 用缩放后的参数重新采样（保持几何形状，只拉长时间）
        v_k, a_k, ta_k = vmax / k, amax / (k * k), (vmax / k) / (amax / (k * k))
        ta_k = min(ta_k, Ts / 2)
        ts, st = sample(Ts, ta_k, L, a_k, v_k, cum, P, 240)
        v = np.abs(np.diff(st, axis=0)) * 240
        acc = np.abs(np.diff(st, n=2, axis=0)) * 240 * 240
        vr = max(v[:, :4].max() / V_JOINT, v[:, 4].max() / V_TEL)
        ar = max(acc[:, :4].max() / A_JOINT, acc[:, 4].max() / A_TEL)
        hist.append((round(float(Ts), 2), round(float(vr), 3), round(float(ar), 3)))
        if max(vr, ar) <= 1.0 + tol:
            return Ts, ta_k, v_k, a_k, k, hist
        k *= max(vr, ar) ** 0.75 * 1.02
    return Ts, ta_k, v_k, a_k, k, hist


if __name__ == '__main__':
    P = dense(Q)
    seg = np.diff(P, axis=0)
    cum = np.r_[0, np.cumsum(np.linalg.norm(seg, axis=1))]
    L, vmax, amax, T, ta = time_profile(P)
    print(f'路径弧长 {L:.1f}（角度+mm 混合单位） 允许速度 {vmax:.2f}/s 允许加速度 {amax:.2f}/s² 总时长 {T:.2f}s')

    T, ta, vmax, amax, kscale, hist = fit_time(P, cum, L, vmax, amax, ta)
    print(f'时间缩放 k={kscale:.2f}，收敛过程 {hist[-3:]}')
    print(f'最终总时长 {T:.2f}s（原 16.57s 基准）')
    ts24, s24 = sample(T, ta, L, amax, vmax, cum, P, 24)
    ts48, s48 = sample(T, ta, L, amax, vmax, cum, P, 48)
    r24 = verify(s24, ts24, '24fps')
    r48 = verify(s48, ts48, '48fps-加密')
    print('24fps:', r24)
    print('48fps:', r48)
    v24, a24 = kinematics(s24, 1 / 24)
    print('最大关节速度/加速度:', v24, a24)

    rid, d = pb.new_run('trajectory', dict(fp_source='planner.py@5D', fp_geometry='roof_path+wall+housing',
                                           fp_params=dict(R=pl.R, LIM=pl.LIM, FLOOR=pl.FLOOR,
                                                          V_JOINT=V_JOINT, A_JOINT=A_JOINT,
                                                          V_TEL=V_TEL, A_TEL=A_TEL),
                                           endpoints=dict(start=Q[0].tolist(), goal=Q[-1].tolist())))
    payload = dict(meta=dict(run_id=rid, total_s=round(T, 3), fps=24, samples24=len(ts24), samples48=len(ts48)),
                   path=[list(map(float, r)) for r in Q],
                   trajectory=dict(t=[round(float(x), 4) for x in ts24],
                                   q=[[round(float(v), 3) for v in row] for row in s24]),
                   checks=dict(f24=r24, f48=r48, velocity=v24, acceleration=a24),
                   assumptions=dict(v_joint_degs=V_JOINT, a_joint_degs2=A_JOINT,
                                    v_telescope_mms=V_TEL, a_telescope_mms2=A_TEL,
                                    note='设计假设，非执行器认证；间隙为当前候选模型值，非安全净空'))
    kin_ok = (v24['joint_max_degs'] <= V_JOINT + 1e-6 and a24['joint_max_degs2'] <= A_JOINT + 1e-6
              and v24['tel_max_mms'] <= V_TEL + 1e-6 and a24['tel_max_mms2'] <= A_TEL + 1e-6)
    payload['meta']['kinematically_compliant'] = bool(kin_ok)
    pb.finish(d, payload, status='ok' if kin_ok else 'invalid',
              note='运动学限值校核' + ('通过' if kin_ok else '未通过：实测速度/加速度超出假设限值'))
    ok, why = pb.publish(d, OUT / 'planA_trajectory.json',
                         expect=dict(fp_source='planner.py@5D', fp_geometry='roof_path+wall+housing'))
    print('发布:', ok, why, '| run', rid)
    print('通过:', '是' if (r24['ok'] and r48['ok'] and ok) else '否')
