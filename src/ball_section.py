"""跑道槽球头截面复核（Assembly V2 实际几何）。

球头：上半球 r67（z 0…67）+ 凸台 r24（z 58…76），孔 Ø32（z 14…76），
跑道槽 x −16…104、y ±16、z 12…78（贯通 +X）。
输出：逐层截面面积、最窄截面位置、截面连通性、关键剖面 SVG。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)


def ball():
    hemi = cq.Workplane("XY").sphere(67).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=58).circle(24).extrude(18)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=14).circle(16).extrude(62))
    b = b.cut(cq.Workplane("XY").box(120, 32, 66).translate((44, 0, 45)))
    return b


def section_at(shape, z, t=0.6):
    slab = cq.Workplane("XY").box(300, 300, t).translate((0, 0, z))
    s = shape.val().intersect(slab.val())
    v = s.Volume()
    solids = s.Solids()
    bb = s.BoundingBox() if v > 1e-6 else None
    return dict(z=z, area=round(v / t, 1), n_solids=len(solids),
                x=[round(bb.xmin, 1), round(bb.xmax, 1)] if bb else None,
                y=[round(bb.ymin, 1), round(bb.ymax, 1)] if bb else None)


if __name__ == '__main__':
    b = ball()
    print(f'球头体积 {b.val().Volume():.0f} mm3, 有效 {b.val().isValid()}')
    rows = []
    for z in range(0, 82, 2):
        r = section_at(b, float(z))
        rows.append(r)
        print(f"  z={z:3d}  面积 {r['area']:9.1f} mm2  连通块 {r['n_solids']}  "
              f"x {r['x']}  y {r['y']}")
    # 最窄截面（排除 z<2 的贴地薄层）
    cand = [r for r in rows if r['z'] >= 2 and r['area'] > 0]
    worst = min(cand, key=lambda r: r['area'])
    print(f"\n最小截面：z={worst['z']} 面积 {worst['area']:.1f} mm2")
    multi = [r for r in rows if r['n_solids'] > 1]
    print(f"截面被分成多块（承载路径断开）的层：{[r['z'] for r in multi] or '无'}")
    # 凸台壁厚参考
    print("凸台名义壁厚 = 24 − 16 = 8 mm；跑道槽切掉 +X 侧后仅剩 −X 侧与两侧")
    for nm, z in (('sec_z30', 30.0), ('sec_z76', 76.0)):
        sec = cq.Workplane("XY").workplane(offset=z).add(b).section()
        try:
            cq.exporters.export(sec, str(OUT / f'ball_{nm}.svg'))
            cq.exporters.export(sec, str(OUT / f'ball_{nm}.dxf'))
            print('导出', nm)
        except Exception as e:
            print('导出失败', nm, type(e).__name__)
    (OUT / 'ball_section_report.json').write_text(json.dumps(
        dict(sections=rows, min_section=worst, split_layers=[r['z'] for r in multi]),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 ball_section_report.json')
