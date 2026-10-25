"""膜面实体检查：机械臂轴线/关节 vs 膜【三角面】的真实最近距离。

为什么必须用三角面：膜在角点之间是直段、中间无顶点，用"顶点最近点"会把 8 mm 的间隙
算成 960 mm（采样伪影）。本脚本用 catalog.faces 三角化后做点-三角距离。

坐标陷阱（本脚本已修正）：planner.Y_WORK 是【模块局部 Y】，模块 2 的绝对 Y 需加 2×4250。
"""
import json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
STUDIO = HERE.parent / 'studio'
PITCH = 4250.0
R_JOINT, R_LINK = 73.0, 49.0


def load():
    s = (STUDIO / 'house_data_v2.js').read_text(encoding='utf8')
    return json.loads(s.split('=', 1)[1].rstrip(';'))


def tri_of(cat, mesh):
    V = np.array(cat[mesh]['vertices'], float)
    F = np.array(cat[mesh]['faces'], int)
    if F.shape[1] == 3:
        return V[F]
    tris = []
    for f in F:
        for i in range(1, len(f) - 1):
            tris.append(V[[f[0], f[i], f[i + 1]]])
    return np.array(tris)


def pt_tri(P, T):
    A, B, C = T[:, 0], T[:, 1], T[:, 2]
    AB, AC = B - A, C - A
    AP = P[:, None, :] - A[None]
    d1 = (AB * AP).sum(2); d2 = (AC * AP).sum(2)
    den = (AB * AB).sum(1) * (AC * AC).sum(1) - (AB * AC).sum(1) ** 2
    den = np.where(np.abs(den) < 1e-12, 1e-12, den)
    v = ((AC * AC).sum(1)[None] * d1 - (AB * AC).sum(1)[None] * d2) / den[None]
    w = ((AB * AB).sum(1)[None] * d2 - (AB * AC).sum(1)[None] * d1) / den[None]
    proj = A[None] + v[..., None] * AB[None] + w[..., None] * AC[None]
    inside = (v >= 0) & (w >= 0) & (v + w <= 1)
    dp = np.linalg.norm(P[:, None, :] - proj, axis=2)

    def segd(Q1, Q2):
        D = Q2 - Q1
        t = np.clip(((P[:, None, :] - Q1[None]) * D[None]).sum(2) / ((D * D).sum(1)[None] + 1e-12), 0, 1)
        return np.linalg.norm(P[:, None, :] - (Q1[None] + t[..., None] * D[None]), axis=2)
    de = np.minimum(np.minimum(segd(A, B), segd(B, C)), segd(C, A))
    return np.where(inside, dp, de).min(axis=1)


if __name__ == '__main__':
    import importlib
    pl = importlib.import_module('planner')
    d = load()
    T = tri_of(d['catalog'], next(b['mesh'] for b in d['base'] if b['name'] == 'M2_FILM'))
    fr = json.loads((OUT / 'modeA_v2_timeline.json').read_text(encoding='utf8'))['frame_records']
    Y_ABS = [1 * PITCH + float(pl.Y_WORK[1]), 1 * PITCH + float(pl.Y_WORK[4])]   # 英雄模块索引 1
    best = (1e9, None, None); below = 0; n = 0
    for fi in range(0, len(fr), 2):
        q = np.asarray(fr[fi]['q'], float)
        base = pl.chain3d(q[:4], q[4], 0.0, 0.0, n_per=2)
        gj = gl = 1e9
        for y in Y_ABS:
            P = base.copy(); P[:, 1] = y
            gj = min(gj, float(pt_tri(P, T).min()) - R_JOINT)
            # 沿每段按 40 mm 密集取样（不只中点）
            samp = []
            for a, b in zip(P[:-1], P[1:]):
                L = float(np.linalg.norm(b - a)); nseg = max(2, int(L / 40.0))
                for t in np.linspace(0, 1, nseg + 1)[1:-1]:
                    samp.append(a + (b - a) * t)
            if samp:
                gl = min(gl, float(pt_tri(np.array(samp), T).min()) - R_LINK)
        g = min(gj, gl); n += 1
        if g < 30: below += 1
        if g < best[0]: best = (g, fi, 'joint' if gj < gl else 'link')
    print(f'帧 {n}（每 2 帧）  最小间隙 {best[0]:.1f} mm @帧{best[1]} ({best[2]}) '
          f't={fr[best[1]]["video_time"]:.2f}s')
    print(f'<30 mm 的帧: {below} ({below/n*100:.1f}%)')
    res = dict(method='点-三角距离（膜 faces 三角化）', frames=n,
               min_gap_mm=round(best[0], 2), at_frame=best[1], at_t=fr[best[1]]['video_time'],
               part=best[2], below_30=below,
               y_abs=Y_ABS, radii=dict(joint=R_JOINT, link=R_LINK))
    (OUT / 'film_mesh_check.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    sys.exit(0 if best[0] > 0 else 1)
