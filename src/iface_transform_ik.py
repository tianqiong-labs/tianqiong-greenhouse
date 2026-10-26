"""固定安装变换 + 姿态约束 IK（不引入补偿座）。

坐标系：
  W 世界：+X 棚内，+Y 沿墙，+Z 上
  E 末段系：原点在末端球心，x 轴沿末段方向，y 沿 Y
  I 接口系：原点同球心，+X 为跑道槽开口方向，+Z 为腕柱目标轴向

固定安装：I 相对 E 绕 Y 轴旋转 beta = -a4_0（部署态），使部署态 I 与世界对齐。
         W_T_I(q) = Trans(tip(q)) * Rot_Y(beta)
复现：q=q0 时 W_T_I 应为单位旋转、球心 (4500,0,0)。
"""
import math, json
from pathlib import Path
import importlib.util, numpy as np

HERE = Path(__file__).resolve().parent
sp = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp); sp.loader.exec_module(pl)

Q0 = [-1.6, -20.1, -38.6, -57.2]
L0 = 1600.0
BETA = -Q0[3]                      # +57.2°，绕 Y
TOL_POS = 0.5                      # mm
TOL_TILT = 2.0                     # deg
REL_LIMIT = 140.0                  # 相对关节限位（deg）


def tip(q, L4):
    p = pl.chain3d(q, L4)[-1]
    return float(p[0]), float(p[2])


def tilt_of(a4):
    """固定安装下接口相对世界 +Z 的倾角（= |a4 - a4_0|）。"""
    return abs(a4 - Q0[3])


def rot_report(a4):
    return dict(a4=a4, beta=BETA, tilt_deg=round(tilt_of(a4), 3),
                aligned=tilt_of(a4) <= TOL_TILT)


def search(tx, a4_center=None, a4_tol=TOL_TILT, step=1.5):
    """a4 固定安装约束下的位置搜索：a4 必须落在 a4_0 ± a4_tol 内。"""
    a4c = Q0[3] if a4_center is None else a4_center
    best = None
    hits = []
    for a1 in np.arange(-30, 20.5, 2.5):
        for a2 in np.arange(-55, 20.5, 2.5):
            for a3 in np.arange(-70, 10.5, 2.5):
                for a4 in np.arange(a4c - a4_tol, a4c + a4_tol + 0.01, step):
                    for L4 in (1000, 1100, 1200, 1300, 1400, 1500, 1600):
                        x, z = tip([a1, a2, a3, a4], float(L4))
                        d = math.hypot(x - tx, z)
                        if d <= TOL_POS:
                            rel = [a1, a2 - a1, a3 - a2, a4 - a3]
                            ok = all(abs(v) <= REL_LIMIT for v in rel)
                            hits.append(dict(q=[float(a1), float(a2), float(a3), float(a4)],
                                             L4=float(L4), res=round(d, 3),
                                             rel=[round(v, 1) for v in rel],
                                             rel_ok=ok, tilt=round(tilt_of(a4), 2)))
                            if ok and (best is None or d < best['res']):
                                best = dict(q=[float(a1), float(a2), float(a3), float(a4)],
                                            L4=float(L4), res=round(d, 3),
                                            rel=[round(v, 1) for v in rel],
                                            rel_ok=ok, tilt=round(tilt_of(a4), 2))
    return best, hits


if __name__ == '__main__':
    # 1) 部署态复现
    x0, z0 = tip(Q0, L0)
    rep = dict(tip=(round(x0, 6), round(z0, 6)),
               expect=(4500.0, 0.0),
               err=round(math.hypot(x0 - 4500.0, z0), 6),
               rot_identity=abs(tilt_of(Q0[3])) < 1e-9)
    print(f"部署态复现: tip=({x0:.3f},{z0:.3f}) 误差 {rep['err']:.6f} mm, 接口旋转单位={rep['rot_identity']}")

    # 2) 扰动：a4 变化 -> 接口随之倾斜，安装变换不变
    pert = [rot_report(a) for a in (-70, -57.2, -45, -24)]
    for p in pert:
        print(f"  a4={p['a4']:6.1f}° -> 接口倾角 {p['tilt_deg']:5.2f}° aligned={p['aligned']}")

    # 3) 姿态约束 IK
    out = dict(reproduce=rep, perturbation=pert, candidates={})
    for tx in (4250.0, 4280.0, 4230.0):
        best, hits = search(tx)
        out['candidates'][str(int(tx))] = dict(best=best, n_hits=len(hits),
                                               n_rel_ok=sum(1 for h in hits if h['rel_ok']))
        if best:
            print(f"  x={tx:.0f}: 解 {len(hits)}，相对限位合格 {out['candidates'][str(int(tx))]['n_rel_ok']}，"
                  f"最佳残差 {best['res']} mm, a4={best['q'][3]:.1f}°, L4={best['L4']:.0f}, tilt={best['tilt']}°")
        else:
            print(f"  x={tx:.0f}: 本轮覆盖范围内未找到（a4 ∈ {Q0[3]}±{TOL_TILT}°）")

    (HERE / 'out' / 'iface_transform_ik.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/iface_transform_ik.json')
