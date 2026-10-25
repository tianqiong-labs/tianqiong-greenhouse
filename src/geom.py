"""统一几何距离（唯一定义）+ 基础单元测试 + 不变量 + v2 严格复认证。

所有模块（规划器/边检查/场景/报告）都应调用这里的函数；禁止再有第二套实现。
距离定义：点到【折线段】的距离（min over segments），已扣代理半径。
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
FILM = pl.FILM                      # (x,z) 折线
R = 73.0
EPS = 1e-9


def point_seg_dist(p, a, b):
    """点到线段的解析距离（2D）。"""
    p, a, b = np.asarray(p, float), np.asarray(a, float), np.asarray(b, float)
    ab = b - a
    L2 = float(ab @ ab)
    if L2 < EPS:
        return float(np.linalg.norm(p - a))
    t = float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
    return float(np.linalg.norm(p - (a + t * ab)))


def polyline_dist(p, poly):
    return min(point_seg_dist(p, poly[i], poly[i + 1]) for i in range(len(poly) - 1))


def film_gap(q, L4, n_per=12, radius=R):
    """返回 (垂直最近距离, 竖直距离)，均已扣半径。竖直仅在膜面 x 覆盖范围内取。"""
    P = pl.chain3d(q, L4, 0.0, 0.0, n_per=n_per)
    P2 = P[:, [0, 2]]
    perp = min(polyline_dist(p, FILM) for p in P2) - radius
    x0, x1 = FILM[:, 0].min(), FILM[:, 0].max()
    ins = (P2[:, 0] >= x0) & (P2[:, 0] <= x1)
    vert = (float((pl.film_z(P2[ins, 0]) - P2[ins, 1]).min()) - radius) if ins.any() else float('inf')
    return perp, vert


def invariants(q, L4, radius=R, tol=1e-6):
    perp, vert = film_gap(q, L4, radius=radius)
    msgs = []
    if not (math.isfinite(perp) and math.isfinite(vert)):
        msgs.append('非有限数值')
    if perp > vert + tol:
        msgs.append(f'违反 perp<=vert: {perp:.3f} > {vert:.3f}')
    return (len(msgs) == 0), msgs, perp, vert


def _selftest():
    cases = [
        ('点到线段中部', (0, 10), ((0, 0), (100, 0)), 10.0),
        ('投影落在 A 之外', (-10, 0), ((0, 0), (100, 0)), 10.0),
        ('投影落在 B 之外', (110, 0), ((0, 0), (100, 0)), 10.0),
        ('点在段上', (50, 0), ((0, 0), (100, 0)), 0.0),
        ('退化线段', (3, 4), ((0, 0), (0, 0)), 5.0),
    ]
    ok = True
    for name, p, (a, b), want in cases:
        got = point_seg_dist(p, a, b)
        good = abs(got - want) < 1e-9
        ok &= good
        print(f'  [{"PASS" if good else "FAIL"}] {name}: {got:.6f} (期望 {want})')
    poly = np.array([[0, 0], [100, 0], [100, 100]], float)
    got = polyline_dist((50, 50), poly)
    good = abs(got - 50.0) < 1e-9
    ok &= good
    print(f'  [{"PASS" if good else "FAIL"}] 折角处取近段: {got:.6f} (期望 50)')
    got2 = polyline_dist((105, 105), poly)
    good2 = abs(got2 - 7.0710678) < 1e-6
    ok &= good2
    print(f'  [{"PASS" if good2 else "FAIL"}] 折角外侧: {got2:.6f} (期望 7.071068)')
    return ok


def strict_edge(a, b, required, depth=0, max_depth=6, k=4):
    """自适应边检查：运动上界 Σ|Δθ|·力臂 + |ΔL4|。"""
    LEVER = np.array([5650, 4500, 3120, 1600], float)
    pts = [a + (b - a) * (i / k) for i in range(k + 1)]
    gaps = [film_gap(q[:4], q[4])[0] for q in pts]
    worst = min(gaps)
    dth = np.abs(np.radians(pts[0][:4] - pts[1][:4]))
    disp_max = float((dth * LEVER).sum() + abs(pts[0][4] - pts[1][4]))
    if worst - disp_max / 2 >= required:
        return True, worst
    if depth >= max_depth:
        return False, worst
    best = 1e9
    for i in range(k):
        ok, g = strict_edge(pts[i], pts[i + 1], required, depth + 1, max_depth, k)
        if not ok:
            return False, g
        best = min(best, g)
    return True, best


def certify(path, required):
    worst = 1e9
    for a, b in zip(path[:-1], path[1:]):
        ok, g = strict_edge(np.asarray(a, float), np.asarray(b, float), required)
        if not ok:
            return False, round(g, 2)
        worst = min(worst, g)
    return True, round(worst, 2)


if __name__ == '__main__':
    print('=== 单元测试 ===')
    ok = _selftest()
    print('单元测试总判定:', 'PASS' if ok else 'FAIL')

    print('\n=== 不变量抽查（v2 轨迹 200 个构型）===')
    cmp = json.loads((OUT / 'experiments' / 'gap_v2' / 'compare.json').read_text(encoding='utf8'))
    v2 = np.array(next(r for r in cmp['rows'] if r['version'] == 'v2-target30')['path'], float)
    v20 = np.array(next(r for r in cmp['rows'] if r['version'] == 'v2-target20')['path'], float)
    bad = 0
    for a, b in zip(v2[:-1], v2[1:]):
        for t in np.linspace(0, 1, 40):
            q = a + (b - a) * t
            good, msgs, perp, vert = invariants(q[:4], q[4])
            if not good:
                bad += 1
                print('   ', msgs, f'perp={perp:.2f} vert={vert:.2f}')
    print('不变量违反数:', bad)

    print('\n=== 严格复认证（阈值 20 mm）===')
    ok2, g2 = certify(v2, 20.0)
    print(f'v2-30: {"通过" if ok2 else "未通过"}  最小 {g2} mm')
    ok20, g20 = certify(v20, 20.0)
    print(f'v2-20（回归用例）: {"通过(异常)" if ok20 else "按预期拒绝"}  最小 {g20} mm')

    res = dict(unit_tests=bool(ok), invariants_violations=bad,
               v2_30=dict(passed=bool(ok2), min_mm=g2),
               v2_20=dict(passed=bool(ok20), min_mm=g20, rejected_as_expected=not ok20),
               metric='point-to-segment, radius 73 mm')
    (OUT / 'experiments' / 'gap_v2' / 'recert_20mm.json').write_text(
        json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 experiments/gap_v2/recert_20mm.json')
