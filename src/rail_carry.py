"""前沿锚轨搬运：刚体约束 + 逆运动学 + 碰撞检查。

模型
  机械臂：根部固定在高 2800 mm 的双导轨滑座上；平面内 4 段俯仰 + 第 4 段伸缩。
          目标点与本臂 y 相同，因此搬运过程中 yaw 保持 0（不引入额外自由度）。
  锚轨：沿 Y 的刚体梁，6 个抓取点与本模块 6 个滑座同 y；轨道位姿由各臂末端的
          可达位姿共同决定，不使用独立的轨道平移副来"演"运动。
  障碍：母墙箱体 x∈[-600,0]、z∈[0,2800]；地面 z=0；轨道自身。

检查：可达性、关节限位、伸缩行程、六臂一致性、最小间隙（离散采样 40 mm）。
"""
from pathlib import Path
import argparse, json, math
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'

L = np.array([1150.0, 1380.0, 1520.0])
TEL_MIN, TEL_MAX = 1000.0, 1600.0
ROOT_Z = 2800.0
A_LIM = math.radians(160.0)
ARM_R, JOINT_R = 49.0, 73.0
RAIL_R = 60.0
WALL = dict(x0=-600.0, x1=0.0, z0=0.0, z1=2800.0)
SAMPLE_MM = 40.0

SUPPORT_DEG = [-1.6, -20.1, -38.634, -57.182]


def fk2(angles, tel):
    """平面 FK：返回 5 个点 (x,z)，从根到末端。"""
    ls = np.r_[L, tel]
    a = np.asarray(angles, float)
    p = np.r_[0.0, np.cumsum(ls * np.cos(a))]
    z = ROOT_Z + np.r_[0.0, np.cumsum(ls * np.sin(a))]
    return np.c_[p, z]


def ik2(target, seeds, iters=160, tol=0.6):
    """平面 IK：变量 a1..a4、tel；返回最优解（最小位置误差，限位内）。"""
    tx, tz = target
    best = None
    cand = []
    for s in seeds:
        q = np.r_[np.deg2rad(s[:4]), s[4]]
        lam = 40.0
        for _ in range(iters):
            pts = fk2(q[:4], q[4])
            e = np.array([pts[-1, 0] - tx, pts[-1, 1] - tz])
            if np.linalg.norm(e) < tol:
                break
            # 数值雅可比 2x5
            J = np.zeros((2, 5))
            for k in range(5):
                dq = q.copy()
                dq[k] += 1e-6 if k < 4 else 1.0
                pts2 = fk2(dq[:4], dq[4])
                J[:, k] = (pts2[-1] - pts[-1]) / (1e-6 if k < 4 else 1.0)
            # 阻尼最小二乘 + 回归到种子的小正则
            JJt = J @ J.T + lam * lam * np.eye(2)
            dq = J.T @ np.linalg.solve(JJt, -e)
            q += 0.6 * dq
            q[:4] = np.clip(q[:4], -A_LIM, A_LIM)
            q[4] = np.clip(q[4], TEL_MIN, TEL_MAX)
        pts = fk2(q[:4], q[4])
        err = float(np.linalg.norm(pts[-1] - np.array([tx, tz])))
        cand.append((err, q.copy(), pts))
    return cand


def seg_points(p, q, step=SAMPLE_MM):
    n = max(2, int(np.linalg.norm(q - p) / step) + 1)
    return np.linspace(p, q, n)


def dist_point_box(pt, box):
    """点到箱体（x-z 平面，y 方向视为覆盖）的距离；内部为负。"""
    dx = max(box['x0'] - pt[0], 0.0, pt[0] - box['x1'])
    dz = max(box['z0'] - pt[1], 0.0, pt[1] - box['z1'])
    if dx == 0 and dz == 0:
        # 内部：取最小穿透深度（负值）
        return -min(pt[0] - box['x0'], box['x1'] - pt[0], pt[1] - box['z0'], box['z1'] - pt[1])
    return math.hypot(dx, dz)


def clearance_2d(pts, rail_pt, include_rail=True, tip_exclude=450.0):
    """一条臂的姿态（5 点折线）对墙、地面、轨道的二维最小间隙（已扣半径）。

    被本臂抓取的轨道不作为障碍：距末端 tip_exclude 以内的采样点跳过轨道项。
    """
    worst = 1e9
    tip = pts[-1]
    root = pts[0]
    for i in range(len(pts) - 1):
        for p in seg_points(pts[i], pts[i + 1]):
            # 根部装在母墙面上，250 mm 内的接触属于装配关系，不计入干涉
            d_wall = 1e9 if math.hypot(p[0] - root[0], p[1] - root[1]) < 250.0 else dist_point_box(p, WALL) - ARM_R
            d_ground = p[1] - ARM_R
            worst = min(worst, d_wall, d_ground)
            if include_rail and math.hypot(p[0] - tip[0], p[1] - tip[1]) > tip_exclude:
                d_rail = math.hypot(p[0] - rail_pt[0], p[1] - rail_pt[1]) - ARM_R - RAIL_R
                worst = min(worst, d_rail)
    # 关节点球
    for p in pts:
        if math.hypot(p[0] - root[0], p[1] - root[1]) >= 250.0:
            worst = min(worst, dist_point_box(p, WALL) - JOINT_R)
        worst = min(worst, p[1] - JOINT_R)
    # 非相邻连杆自碰（i 与 i+2）
    for i in range(len(pts) - 3):
        a = seg_points(pts[i], pts[i + 1], 30.0)
        b = seg_points(pts[i + 2], pts[i + 3], 30.0)
        d = np.min(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2))
        worst = min(worst, float(d) - 2 * ARM_R)
    return worst


def build_path(x0, z0, x1, z1, z_lift, n_lift=28, n_move=36, n_lower=30):
    """轨道参考点轨迹：抬升 -> 横移 -> 下放。"""
    segs = []
    for k in range(n_lift):
        t = k / (n_lift - 1)
        segs.append(('lift', x0, z0 + (z_lift - z0) * t))
    for k in range(1, n_move):
        t = k / (n_move - 1)
        segs.append(('move', x0 + (x1 - x0) * t, z_lift))
    for k in range(1, n_lower):
        t = k / (n_lower - 1)
        segs.append(('lower', x1, z_lift + (z1 - z_lift) * t))
    return segs


def seeds_support():
    return [SUPPORT_DEG + [TEL_MAX]]


def seeds_multi():
    base = SUPPORT_DEG + [TEL_MAX]
    out = [base]
    # 越过墙顶的姿态：先上扬再回落
    out.append([70, 95, 70, 40, TEL_MIN])
    out.append([120, 40, 100, 60, TEL_MIN])
    out.append([95, 150, 60, 20, TEL_MIN])
    out.append([40, 140, 30, 120, TEL_MIN])
    out.append([-30, 60, -40, -80, 1200])
    return out


def run(strategy='six', x1=-900.0, z1=300.0, z_lift=2900.0, arms=(0, 1, 2, 3, 4, 5)):
    path = build_path(4620.0, 0.0, x1, z1, z_lift)
    rows = []
    for phase, rx, rz in path:
        per_arm = []
        for ai in arms:
            cand = ik2((rx, rz), seeds_multi())
            scored = []
            for err, q, pts in cand:
                clr = clearance_2d(pts, (rx, rz), include_rail=True)
                scored.append((err, clr, q, pts))
            ok = [c for c in scored if c[0] < 2.0 and c[1] > 0]
            pick = max(ok, key=lambda c: c[1]) if ok else min(scored, key=lambda c: c[0])
            err, clr, q, pts = pick
            per_arm.append(dict(arm=ai + 1, err_mm=round(err, 2),
                                angles_deg=np.round(np.rad2deg(q[:4]), 1).tolist(),
                                tel_mm=round(float(q[4]), 1),
                                min_clear_mm=round(float(clr), 1),
                                collision_free=bool(err < 2.0 and clr > 0)))
        rows.append(dict(phase=phase, rail=[round(rx, 1), round(rz, 1)], arms=per_arm))
    return rows


def summarize(rows, strategy):
    errs = [a['err_mm'] for r in rows for a in r['arms']]
    clrs = [a['min_clear_mm'] for r in rows for a in r['arms']]
    tel = [a['tel_mm'] for r in rows for a in r['arms']]
    ang = [abs(v) for r in rows for a in r['arms'] for v in a['angles_deg']]
    reach_fail = sum(1 for e in errs if e > 2.0)
    collide = sum(1 for c in clrs if c < 0)
    return dict(strategy=strategy, samples=len(rows), arm_samples=len(errs),
                max_err_mm=round(max(errs), 2), reach_failures=reach_fail,
                min_clearance_mm=round(min(clrs), 1), collision_points=collide,
                tel_min=round(min(tel), 0), tel_max=round(max(tel), 0),
                max_abs_angle_deg=round(max(ang), 1),
                feasible=bool(reach_fail == 0 and collide == 0))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--strategy', default='six')
    ap.add_argument('--seat-z', type=float, default=300.0)
    ap.add_argument('--limit', type=float, default=160.0, help='关节角限位（度）')
    ap.add_argument('--lift', type=float, default=2900.0)
    ap.add_argument('--json-out', default='rail_carry.json')
    args = ap.parse_args()
    global A_LIM
    A_LIM = math.radians(args.limit)

    arms = (0, 1, 2, 3, 4, 5) if args.strategy == 'six' else (1, 2, 3)
    rows = run(args.strategy, z1=args.seat_z, z_lift=args.lift, arms=arms)
    summ = summarize(rows, args.strategy)
    summ['joint_limit_deg'] = args.limit

    # 失败点定位：按阶段统计
    by_phase = {}
    for r in rows:
        bad = [a for a in r['arms'] if a['min_clear_mm'] < 0 or a['err_mm'] > 2.0]
        if bad:
            by_phase.setdefault(r['phase'], []).append(dict(rail=r['rail'],
                                                           worst=min(a['min_clear_mm'] for a in bad),
                                                           err=max(a['err_mm'] for a in bad)))
    summ['failures_by_phase'] = {k: len(v) for k, v in by_phase.items()}
    summ['worst_examples'] = {k: sorted(v, key=lambda z: z['worst'])[:2] for k, v in by_phase.items()}

    (OUT / args.json_out).write_text(json.dumps(dict(summary=summ, samples=rows), ensure_ascii=False, indent=1),
                                     encoding='utf8')
    print(json.dumps(summ, ensure_ascii=False, indent=1)[:1200])
    print('written', OUT / args.json_out)


if __name__ == '__main__':
    main()
