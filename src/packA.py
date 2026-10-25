"""模式 A 专用紧凑姿态搜索。

约束（全部按"膜面内侧、且留余量"处理，不用白名单绕）：
  1. 四个俯仰关节 |θ| ≤ 140°，第四段伸缩 1000–1600
  2. 整链（含杆件半径 R_LINK）位于膜面以下 z_film(x) - MARGIN
  3. 不离地：z ≥ 250 mm
  4. 与相邻四条部署臂 3D 距离 ≥ CLEAR（含杆件半径）
  5. yaw 0→35° 扫掠全程与相邻臂 3D 距离 ≥ CLEAR
目标：在满足上述约束下尽量紧凑（根部半径小），并留出最大间隙。
"""
import numpy as np, math, json
from pathlib import Path
import importlib.util

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('gh', HERE / 'greenhouse.py')
gh = importlib.util.module_from_spec(spec); spec.loader.exec_module(gh)
m = gh.m
OUT = HERE / 'out'

P = gh.roof_path()
R_LINK = 80.0
MARGIN = 30.0
FLOOR = 250.0
CLEAR = 120.0
YAW = 35.0

Y0 = np.array(m.Y0, float)
SEGS = np.array([1150.0, 1380.0, 1520.0], float)
HERO_IDX = [1, 4]      # A02, A05


def film_z(x):
    return np.interp(x, P[:, 0], P[:, 1])


def chain_pts(angles_deg, L4, n_per=8):
    """返回 (N,2) 的链上采样点（含节点）。"""
    ang = np.radians(np.r_[angles_deg, angles_deg[-1]])
    seg = np.r_[SEGS, L4]
    pts = [np.array([0.0, 2800.0])]
    cur = np.array([0.0, 2800.0])
    out = [cur.copy()]
    for a, L in zip(ang, seg):
        n = max(3, int(n_per * L / 1400))
        t = np.linspace(0, 1, n + 1)[1:]
        step = np.c_[L * t * math.cos(a), L * t * math.sin(a)]
        out.append(cur + step)
        cur = cur + step[-1]
    return np.vstack(out)


def deployed_chain():
    a = np.degrees(m.A0)
    return chain_pts(a, 1600.0, n_per=10)


def seg_dist_pt(pts, A, B):
    """点集到线段 AB 的距离（向量化）。"""
    ab = B - A
    t = np.clip(((pts - A) @ ab) / max(1e-9, ab @ ab), 0, 1)
    proj = A + np.outer(t, ab)
    return np.linalg.norm(pts - proj, axis=1)


def sweep_clearance(pts2d, neighbours_y, yaw_deg=YAW, steps=6):
    """把 2D 链绕根部轴旋转，检查与相邻臂（各自 y 平面）的 3D 最小距离。"""
    best = 1e9
    for k in range(steps + 1):
        th = math.radians(yaw_deg * k / steps)
        R = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        p = pts2d @ R.T
        for dy, npts2d in neighbours_y:
            a = np.c_[p, np.full(len(p), dy)]
            b = np.c_[npts2d, np.zeros(len(npts2d))]
            d = np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2).min()
            best = min(best, d)
    return best


HZ_LO = np.array([-285.0, 2765.0]); HZ_HI = np.array([-135.0, 2915.0])   # 母墙顶卷筒壳体 x-z 投影


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


def evaluate(cands, deployed, neigh):
    """cands: (N,5) = 4 角 + L4。返回 (ok_mask, clearance, radius)。"""
    N = len(cands)
    ok = np.ones(N, bool)
    clear = np.full(N, 1e9)
    rad = np.zeros(N)
    for i, c in enumerate(cands):
        pts = chain_pts(c[:4], c[4], n_per=6)
        zf = film_z(pts[:, 0])
        if np.any(pts[:, 1] + R_LINK > zf - MARGIN) or np.any(pts[:, 1] < FLOOR):
            ok[i] = False
            continue
        # 不得退到母墙面之后（墙 x -600..0），根节点除外
        far = np.hypot(pts[:, 0], pts[:, 1] - 2800.0) > 400.0
        if np.any(pts[far, 0] < 20.0):
            ok[i] = False
            continue
        # 卷筒壳体必须让开（含杆件半径 60 mm 余量）
        bad = False
        for k in range(len(pts) - 1):
            d = seg_aabb2d(pts[k], pts[k+1], HZ_LO, HZ_HI)
            if d < R_LINK + 60.0:
                bad = True; break
        if bad:
            ok[i] = False
            continue
        r = np.hypot(pts[:, 0], pts[:, 1] - 2800.0).max()
        rad[i] = r
        dmin = 1e9
        for dy, npts in neigh:
            a = np.c_[pts, np.full(len(pts), dy)]
            b = np.c_[npts, np.zeros(len(npts))]
            dmin = min(dmin, np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2).min())
        sc = sweep_clearance(pts, neigh)
        clear[i] = min(dmin, sc) - 2 * R_LINK
    ok &= clear >= CLEAR
    return ok, clear, rad


def main():
    dep = deployed_chain()
    # 相邻臂：A01/A03 各偏移 560，A04 偏移 1120（A05 亦为作业臂，单独处理）
    neigh = [(560.0, dep), (-560.0, dep), (1120.0, dep), (-1120.0, dep)]
    rng = np.random.default_rng(7)
    found = []
    for batch in range(40):
        # 角度朝"向下折"的先验采样：±(20..140)
        A = rng.uniform(-140, 140, size=(1500, 4))
        L4 = rng.uniform(1000, 1600, size=(1500, 1))
        cands = np.hstack([A, L4])
        ok, clear, rad = evaluate(cands, dep, neigh)
        for c, cl, r in zip(cands[ok], clear[ok], rad[ok]):
            found.append(dict(angles=[round(float(x), 1) for x in c[:4]], L4=round(float(c[4]), 1),
                              clear=round(float(cl), 1), radius=round(float(r), 1)))
        print(f'batch {batch}: keep={ok.sum()} total={len(found)}', flush=True)
        if len(found) > 400:
            break
    if not found:
        print('NO FEASIBLE POSE under these constraints')
        return
    # 先按间隙达标，再按紧凑度排序
    found.sort(key=lambda d: (d['radius'], -d['clear']))
    best = found[0]
    print('\n最佳姿态:', json.dumps(best, ensure_ascii=False))
    (OUT / 'packA_search.json').write_text(json.dumps(dict(
        params=dict(R_LINK=R_LINK, MARGIN=MARGIN, FLOOR=FLOOR, CLEAR=CLEAR, YAW=YAW),
        best=best, top=found[:20]), ensure_ascii=False, indent=1), encoding='utf8')


if __name__ == '__main__':
    main()
