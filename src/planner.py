"""模式 A 关节空间路径规划器（RRT-Connect + 边的连续检查）。

q 的定义：连杆【绝对角】(deg)，与 model.fk 一致（已验证：A0 直接给出 −1.6/−20.1/−38.6/−57.2°）。
阶段 1：4 维 [pitch1..pitch4]
阶段 2：5 维 + L4
阶段 3：7 维 + yaw + 根部 Y 滑移

每个构型必须满足：关节限位、膜面(含 73 mm 包络)、地面、墙面、卷筒壳体、相邻四臂。
边检查：相邻节点之间再采样 k 个构型，逐点验算，避免"两端无碰撞、中间穿透"。
"""
import json, math, sys, time
from pathlib import Path
import importlib.util
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
spec = importlib.util.spec_from_file_location('gh', HERE / 'greenhouse.py')
gh = importlib.util.module_from_spec(spec); spec.loader.exec_module(gh)
m = gh.m

FILM = gh.roof_path()
R = 73.0            # 关节外壳半径（实际值）
MARGIN = 20.0       # 膜面余量
FLOOR = 0.0
LIM = 150.0         # 关节限位
L4_MIN, L4_MAX = 1000.0, 1600.0
YAW_LIM = 90.0
DY_MAX = 186.7      # 根部滑座行程（设计值）
HZ_LO = np.array([-285.0, 2765.0]); HZ_HI = np.array([-135.0, 2915.0])

Y0 = np.array(m.Y0, float)                       # 补位前初始位（装配基准）
# 双轨补位后的工作位：A01/A06 不动，A02/A03 前移，A04/A05 后移
Y_WORK = np.array([350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0])
Y_STATE = 'compensated'                          # 模式 A 的六臂场景使用工作位
HERO = 1                       # A02
SEGS = np.array([1150.0, 1380.0, 1520.0])
A0 = np.degrees(m.A0)
Q_START4 = A0.copy()
Q_GOAL4 = np.array([-60.0, 116.9, -71.5, 81.2])
L4_START, L4_GOAL = 1600.0, 1098.0


def film_z(x):
    return np.interp(np.asarray(x, float), FILM[:, 0], FILM[:, 1])


def chain3d(q, L4, yaw=0.0, dy=0.0, n_per=7):
    """返回 (N,3) 世界坐标采样点。q 为绝对段角(deg)。"""
    ang = np.radians(np.asarray(q, float))
    seg = np.r_[SEGS, L4]
    x, z = 0.0, 2800.0          # 根部在墙顶 z=2800，不是 0
    out = [np.array([0.0, 0.0, 2800.0])]
    for a, L in zip(ang, seg):
        n = max(3, int(n_per * L / 1200))
        t = np.linspace(0, 1, n + 1)[1:]
        xs = x + L * t * math.cos(a)
        zs = z + L * t * math.sin(a)
        out.append(np.c_[xs, np.zeros(len(xs)), zs])
        x, z = xs[-1], zs[-1]
    P = np.vstack(out)
    th = math.radians(yaw)
    X = P[:, 0] * math.cos(th) - P[:, 1] * math.sin(th)
    Y = P[:, 0] * math.sin(th) + P[:, 1] * math.cos(th)
    P[:, 0], P[:, 1] = X, Y + dy
    return P


def seg_aabb2d(p0, p1, lo, hi):
    d = p1 - p0
    t0, t1 = 0.0, 1.0
    for i in range(2):
        if abs(d[i]) < 1e-9:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                q = p0
                return float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0)))
        else:
            a = (lo[i] - p0[i]) / d[i]; b = (hi[i] - p0[i]) / d[i]
            if a > b: a, b = b, a
            t0, t1 = max(t0, a), min(t1, b)
    q = p0 + d * np.clip(0.5 * (t0 + t1), 0, 1)
    return float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0)))


def make_neighbours():
    out = []
    for idx, y in enumerate(Y_WORK):
        if idx == HERO:
            continue
        P = chain3d(A0, 1600.0, 0.0, y - Y_WORK[HERO], n_per=8)
        out.append(P)
    return out


NB = make_neighbours()


def violation(q, L4, yaw=0.0, dy=0.0, want_clear=False):
    """返回 (ok, 最小膜余量, 最小邻臂距离, 最小壳体距离)。"""
    if np.any(np.abs(q) > LIM) or not (L4_MIN - 1e-6 <= L4 <= L4_MAX + 1e-6):
        return (False, -1e9, -1e9, -1e9) if want_clear else False
    if abs(yaw) > YAW_LIM or not (0.0 - 1e-9 <= dy <= DY_MAX + 1e-9):
        return (False, -1e9, -1e9, -1e9) if want_clear else False
    P = chain3d(q, L4, yaw, dy)
    film = float((film_z(P[:, 0]) - MARGIN - P[:, 2]).min()) - R
    if P[:, 2].min() < FLOOR - 1.0:   # 臂端在锚轨上 z=0，属设计接触
        return (False, film, -1e9, -1e9) if want_clear else False
    rad = np.hypot(P[:, 0], P[:, 1] - dy)
    far = rad > 400.0
    if np.any(P[far, 0] < 20.0 - 1e-6):
        return (False, film, -1e9, -1e9) if want_clear else False
    if film < 0:
        return (False, film, -1e9, -1e9) if want_clear else False
    hz = min(seg_aabb2d(P[k][[0, 2]], P[k + 1][[0, 2]], HZ_LO, HZ_HI) for k in range(len(P) - 1)) - R
    if hz < 0:
        return (False, film, -1e9, hz) if want_clear else False
    nb = 1e9
    if abs(yaw) > 1e-9 or abs(dy) > 1e-9:
        for N in NB:
            nb = min(nb, float(np.linalg.norm(P[:, None, :] - N[None, :, :], axis=2).min()) - 2 * R)
        if nb < 0:
            return (False, film, nb, hz) if want_clear else False
    if want_clear:
        return True, film, nb, hz
    return True


def edge_ok(a, b, dims, k=9):
    for i in range(1, k + 1):
        q = a + (b - a) * (i / k)
        if not violation(*unpack(q, dims)):
            return False
    return True


def unpack(q, dims):
    q = np.asarray(q, float)
    L4 = q[4] if dims >= 5 else L4_START
    yaw = q[5] if dims >= 6 else 0.0
    dy = q[6] if dims >= 7 else 0.0
    return q[:4], L4, yaw, dy


def ok_state(q, dims):
    return violation(*unpack(q, dims))


def rrt_connect(q0, q1, dims, step=12.0, budget=45.0, seed=0, eps=4.0):
    rng = np.random.default_rng(seed)
    lo = np.array([-LIM] * 4 + ([L4_MIN] if dims >= 5 else []) +
                  ([-YAW_LIM] if dims >= 6 else []) + ([0.0] if dims >= 7 else []))
    hi = np.array([LIM] * 4 + ([L4_MAX] if dims >= 5 else []) +
                  ([YAW_LIM] if dims >= 6 else []) + ([DY_MAX] if dims >= 7 else []))
    trees = [dict(pts=[q0.copy()], par=[-1]), dict(pts=[q1.copy()], par=[-1])]
    t0 = time.time()
    it = 0
    while time.time() - t0 < budget:
        it += 1
        ti = it % 2
        other = 1 - ti
        if rng.random() < 0.12:
            qr = trees[other]['pts'][-1].copy()
        else:
            qr = rng.uniform(lo, hi)
        # extend
        P = np.array(trees[ti]['pts'])
        d = np.linalg.norm(P - qr, axis=1)
        i = int(np.argmin(d))
        qn = P[i] + (qr - P[i]) * min(1.0, step / max(1e-9, d[i]))
        if not ok_state(qn, dims):
            continue
        if not edge_ok(P[i], qn, dims):
            continue
        trees[ti]['pts'].append(qn)
        trees[ti]['par'].append(i)
        # connect
        P2 = np.array(trees[other]['pts'])
        d2 = np.linalg.norm(P2 - qn, axis=1)
        j = int(np.argmin(d2))
        q2 = P2[j]
        if np.linalg.norm(q2 - qn) <= step * 2:
            if edge_ok(qn, q2, dims):
                return build_path(trees, ti, other, len(trees[ti]['pts']) - 1, j), it, time.time() - t0
    return None, it, time.time() - t0


def build_path(trees, ti, other, ia, ib):
    pa = []
    i = ia
    while i >= 0:
        pa.append(trees[ti]['pts'][i]); i = trees[ti]['par'][i]
    pb = []
    i = ib
    while i >= 0:
        pb.append(trees[other]['pts'][i]); i = trees[other]['par'][i]
    return np.array(pa[::-1] + pb)


def shortcut(path, dims, n=150, seed=1):
    rng = np.random.default_rng(seed)
    path = list(path)
    for _ in range(n):
        if len(path) < 3:
            break
        i, j = sorted(rng.choice(len(path), 2, replace=False))
        if j - i < 2:
            continue
        if edge_ok(path[i], path[j], dims, k=12):
            path = path[:i + 1] + path[j:]
    return np.array(path)


def abs_film(q, L4, yaw=0.0, dy=0.0):
    """绝对膜间隙：膜面 z - 连杆 z - 半径，不含 MARGIN。"""
    P = chain3d(q, L4, yaw, dy)
    return float((film_z(P[:, 0]) - P[:, 2]).min()) - R


def report(path, dims):
    qs = np.array(path)
    film, nb, hz = 1e9, 1e9, 1e9
    worst = None
    for k in range(len(qs) - 1):
        for t in np.linspace(0, 1, 21):
            q = qs[k] + (qs[k + 1] - qs[k]) * t
            qq, l4, yy, dd = unpack(q, dims)
            f = abs_film(qq, l4, yy, dd)
            ok, _f, n, h = violation(qq, l4, yy, dd, want_clear=True)
            if f < film: film, worst = f, q
            nb = min(nb, n); hz = min(hz, h)
    return dict(nodes=len(qs), film_mm=round(float(film), 1),
                neighbour_mm=round(float(nb), 1), housing_mm=round(float(hz), 1),
                q_min=[round(float(x), 1) for x in qs[:, :4].min(0)],
                q_max=[round(float(x), 1) for x in qs[:, :4].max(0)],
                L4=(round(float(qs[:, 4].min()), 1), round(float(qs[:, 4].max()), 1)) if dims >= 5 else None,
                yaw=(round(float(qs[:, 5].min()), 1), round(float(qs[:, 5].max()), 1)) if dims >= 6 else None,
                dy=(round(float(qs[:, 6].min()), 1), round(float(qs[:, 6].max()), 1)) if dims >= 7 else None)


if __name__ == '__main__':
    seeds = [0, 1]
    out = dict(start=dict(q=Q_START4.tolist(), L4=L4_START), goal=dict(q=Q_GOAL4.tolist(), L4=L4_GOAL),
               params=dict(R=R, MARGIN=MARGIN, LIM=LIM, FLOOR=FLOOR), stages=[])
    s_ok = violation(Q_START4, L4_START, want_clear=True)
    g_ok = violation(Q_GOAL4, L4_GOAL, want_clear=True)
    print('起点可行:', s_ok)
    print('终点可行:', g_ok)
    if not (s_ok[0] and g_ok[0]):
        (OUT / 'planA_INVALID.json').write_text(json.dumps(dict(
            aborted=True, why='起点或终点违反约束，拒绝出结果',
            start=list(s_ok), goal=list(g_ok)), ensure_ascii=False, indent=1), encoding='utf8')
        print('!! 端点非法，已拒绝写 planA.json（见 planA_INVALID.json）')
        raise SystemExit(2)

    stages = [
        ('5D +L4', np.r_[Q_START4, L4_START], np.r_[Q_GOAL4, L4_GOAL], 5, 50.0),
        ('7D +yaw+dy', np.r_[Q_START4, L4_START, 0.0, 0.0], np.r_[Q_GOAL4, L4_GOAL, 0.0, 0.0], 7, 60.0),
    ]
    for name, q0, q1, dims, bud in stages:
        best = None
        for sd in seeds:
            path, it, dt = rrt_connect(q0, q1, dims, budget=bud, seed=sd)
            print(f'  [{name}] seed={sd} iters={it} t={dt:.1f}s', 'FOUND' if path is not None else 'fail')
            if path is not None:
                sm = shortcut(path, dims, seed=sd)
                rep = report(sm, dims)
                print('     ', rep)
                if best is None or rep['film_mm'] > best['report']['film_mm']:
                    best = dict(seed=sd, iters=it, seconds=round(dt, 1), nodes=len(path),
                                smoothed=len(sm), report=rep, path=sm.tolist())
                break
        out['stages'].append(dict(stage=name, dims=dims, found=best is not None, best=best))
        if best:
            print(f'  -> {name} 找到路径，停止加维')
            break
    import datetime as _dt
    out['meta'] = dict(run_at=_dt.datetime.now().isoformat(timespec='seconds'),
                       start_ok=list(s_ok), goal_ok=list(g_ok),
                       R=R, MARGIN=MARGIN, LIM=LIM, seeds=seeds,
                       note='报告的 film_mm 为绝对膜间隙（不含 MARGIN）')
    (OUT / 'planA.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('\n写入 out/planA.json')
