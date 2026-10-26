"""G5 世界位姿溯源 + G2 解析圆柱轴线。

G5 证据链：
  iface_gamma2_final.py 末尾 asm.save('gamma2_final_LOCKED.step')，零件来自 scene()，
  scene() 不含任何 TIP_X 平移 -> 该 STEP 为局部坐标（球心拟合 (0,0,0) 已验证）。
  gamma2_instantiate.py 的 inst(y) 施加 translate((TIP_X, y_world, 0)) -> 世界位姿。
  故 STEP->W = Translate(4500, y_world, 0)，旋转为单位阵（若零件本身无旋转）。
本脚本用"实例化 STEP 的球心"独立复核该结论。
"""
import cadquery as cq, numpy as np, math, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
TIP_X, Y_WORK = 4500.0, [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]


def sphere_fit(shape):
    best = None
    for s in shape.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'SPHERE':
                continue
            vs, _ = f.tessellate(0.3)
            P = np.array([[v.x, v.y, v.z] for v in vs])
            if len(P) < 50:
                continue
            A = np.c_[2 * P, np.ones(len(P))]
            b = (P ** 2).sum(1)
            x, *_ = np.linalg.lstsq(A, b, rcond=None)
            c = x[:3]; r = math.sqrt(x[3] + c @ c)
            res = np.abs(np.linalg.norm(P - c, axis=1) - r)
            cand = (len(P), c, r, float(np.sqrt((res ** 2).mean())))
            if best is None or cand[0] > best[0]:
                best = cand
    return best


def cyl_axes(shape):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    rows = []
    for s in shape.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'CYLINDER':
                continue
            try:
                ad = BRepAdaptor_Surface(f.wrapped)
                if ad.GetType() != GeomAbs_Cylinder:
                    continue
                cyl = ad.Cylinder()
                ax = cyl.Axis(); loc = ax.Location(); d = ax.Direction()
                rows.append(dict(vol=round(s.Volume(), 0), r=round(cyl.Radius(), 4),
                                 loc=[round(loc.X(), 4), round(loc.Y(), 4), round(loc.Z(), 4)],
                                 dir=[round(d.X(), 4), round(d.Y(), 4), round(d.Z(), 4)]))
            except Exception as e:
                rows.append(dict(err=str(e)[:60]))
    return rows


if __name__ == '__main__':
    res = {}
    # 1) 局部 STEP 的球心（复核）
    loc = cq.importers.importStep(str(OUT / 'gamma2_final_LOCKED.step'))
    n, c, r, rms = sphere_fit(loc)
    print(f"局部 STEP 球心 {np.round(c, 4)} r={r:.4f} RMS={rms:.2e} 点={n}")
    res['local'] = dict(center=[round(float(v), 4) for v in c], r=round(float(r), 4), rms=rms, n=n)

    # 2) 实例化 STEP 的球心（独立验证 STEP->W 平移）
    cands = sorted(OUT.glob('gamma2_inst_*.step'))
    print("实例化 STEP:", [p.name for p in cands])
    ev = []
    for p in cands:
        if p.name == 'gamma2_inst_all18.step':
            continue
        shp = cq.importers.importStep(str(p))
        n2, c2, r2, rms2 = sphere_fit(shp)
        ev.append(dict(file=p.name, center=[round(float(v), 4) for v in c2],
                       r=round(float(r2), 4), rms=float(f"{rms2:.2e}")))
        print(f"  {p.name}: 球心 {np.round(c2,3)} r={r2:.4f}")

    # 判定：是否存在 (4500, Y_WORK[i], 0)
    hit = []
    for e in ev:
        cx, cy, cz = e['center']
        for i, y in enumerate(Y_WORK, 1):
            if abs(cx - TIP_X) < 0.5 and abs(cy - y) < 0.5 and abs(cz) < 0.5:
                hit.append(dict(file=e['file'], arm=f'A{i:02d}', y=round(cy, 1)))
    print("\n与 TIP_X/Y_WORK 匹配:", hit if hit else "无")
    res['instantiated'] = ev
    res['match'] = hit

    # 3) 解析圆柱轴线
    rows = cyl_axes(loc)
    uniq = [r_ for r_ in rows if 'r' in r_]
    seen = {}
    for r_ in uniq:
        seen.setdefault((r_['r'], tuple(r_['dir'])), r_)
    print("\n解析圆柱面（去重，半径/轴向/位置）:")
    for (rr, dd), r_ in sorted(seen.items())[:10]:
        print(f"  r={rr:7.3f} dir={dd} loc={r_['loc']} vol={r_['vol']}")
    res['cylinders'] = [dict(r=k[0], dir=list(k[1]), loc=v['loc'], vol=v['vol'])
                        for k, v in sorted(seen.items())]

    verdict = 'POSITION_RECOVERED' if hit else 'POSITION_UNCONFIRMED'
    res['g5_verdict'] = verdict
    print(f"\nG5 判定: {verdict}（旋转仍需 G2/G3 轴线证据）")
    (OUT.parent / 'iface_g5_g2.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/iface_g5_g2.json')
