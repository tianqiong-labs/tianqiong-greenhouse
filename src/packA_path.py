"""验证模式 A 紧凑姿态的连续可达性（部署态 → 紧凑态 → yaw → 展开）。

不假设直线插值可行：先试关节空间直线，再试按关节顺序分步折；
每种策略都逐点检查膜面余量、相邻臂 3D 距离、yaw 扫掠，取最优。
"""
import numpy as np, math, json
from pathlib import Path
import importlib.util

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('pk', HERE / 'packA.py')
pk = importlib.util.module_from_spec(spec); spec.loader.exec_module(pk)

A0 = np.degrees(pk.m.A0)
BEST = json.loads((HERE / 'out' / 'packA_search.json').read_text(encoding='utf8'))['best']
TARGET = np.array(BEST['angles'] + [BEST['L4']])
DEP_L4 = 1600.0


def state_pts(q):
    return pk.chain_pts(q[:4], q[4], n_per=8)


def min_neighbour(ptsq):
    best = 1e9
    for dy, npts in NEIGH:
        a = np.c_[ptsq, np.full(len(ptsq), dy)]
        b = np.c_[npts, np.zeros(len(npts))]
        best = min(best, float(np.linalg.norm(a[:, None, :] - b[None, :, :], axis=2).min()))
    return best


def score(q):
    pts = state_pts(q)
    film = float((pk.film_z(pts[:, 0]) - pk.MARGIN - pts[:, 1]).min()) - pk.R_LINK
    floor = float(pts[:, 1].min())
    nb = min_neighbour(pts) - 2 * pk.R_LINK
    return film, nb, floor


dep = pk.deployed_chain()
NEIGH = [(560.0, dep), (-560.0, dep), (1120.0, dep), (-1120.0, dep)]


def sample_path(path_pts, label):
    worst_f, worst_n, worst_t = 1e9, 1e9, 0
    for i, q in enumerate(path_pts):
        f, n, fl = score(q)
        if f < worst_f: worst_f, worst_t = f, i
        worst_n = min(worst_n, n)
    ok = worst_f >= 0 and worst_n >= 0
    print(f'  {label:36s} 膜余量 {worst_f:8.1f}  邻臂间隙 {worst_n:8.1f}  '
          f'{"OK" if ok else "FAIL @%.2f" % (worst_t / max(1, len(path_pts)-1))}')
    return ok, worst_f, worst_n


def make_paths(n=60):
    q0 = np.r_[A0, DEP_L4]
    q1 = TARGET
    out = {}
    out['直线插值'] = [q0 + (q1 - q0) * (i / n) for i in range(n + 1)]
    import itertools
    # 分步折：依次把第 k 个关节移到目标（其余暂不动），最后统一移伸缩
    for order in itertools.permutations(range(4)):
        cur = q0.copy()
        pts = [cur.copy()]
        for k in order:
            for i in range(1, n // 4 + 1):
                nxt = cur.copy()
                nxt[k] = q0[k] + (q1[k] - q0[k]) * i / (n // 4)
                pts.append(nxt.copy())
            cur[k] = q1[k]
        for i in range(1, n // 4 + 1):
            nxt = cur.copy()
            nxt[4] = q0[4] + (q1[4] - q0[4]) * i / (n // 4)
            pts.append(nxt.copy())
        out['分步:' + ''.join(str(k + 1) for k in order)] = pts
    return out


if __name__ == '__main__':
    print(f'部署 A0 = {np.round(A0,1).tolist()}  L4={DEP_L4}')
    print(f'目标     = {TARGET.tolist()}')
    f0, n0, fl0 = score(np.r_[A0, DEP_L4])
    print(f'部署态自检: 膜余量 {f0:.1f}  邻臂 {n0:.1f}  最低点 {fl0:.1f}\n')
    best = None
    for label, pts in make_paths().items():
        ok, f, n = sample_path(pts, label)
        if ok and (best is None or min(f, n) > best[1]):
            best = (label, min(f, n), pts)
    if best:
        print(f'\n可用路径: {best[0]} (最差余量 {best[1]:.1f} mm, {len(best[2])} 点)')
        (HERE / 'out' / 'packA_path.json').write_text(json.dumps(dict(
            strategy=best[0], target=TARGET.tolist(), a0=A0.tolist(),
            states=[list(map(float, q)) for q in best[2]]), ensure_ascii=False, indent=1), encoding='utf8')
    else:
        print('\n无可用路径：需要改机构（连杆长度/关节限位）或改动作顺序')
