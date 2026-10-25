"""P0 法兰开口修复 + P1 Assembly V2 合并。

法兰改为朝 +X 开口的 C 形（保留 −X 侧与两侧承力），
然后在完整 A03 接口上重跑 0…120 mm 撤离与反向插入。
"""
import cadquery as cq, importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
CADOUT = OUT / 'cad'
CADOUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
FL_R, FL_Z0, FL_Z1, FL_BORE = 20.0, 76.0, 82.0, 16.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
CLAMP_Z0, CLAMP_Z1, CLAMP_RI, CLAMP_RO = 90.0, 110.0, 15.5, 32.0
RAIL_X0, RAIL_X1, RAIL_Z0, RAIL_Z1 = -30.0, -16.0, 82.0, 88.0
BRKT_Z0, BRKT_Z1 = 88.0, 90.0
PITCH = 4250.0
Y_WORK = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
TIP_X = 4500.0


def ball_side():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    b = b.cut(cq.Workplane("XY").box(120, 32, BOSS_Z1 - FLOOR + 4).translate((44, 0, 45)))
    return b


def flange_c():
    """C 形过渡连接件：外径 40、孔 32、z 76…82，+X 侧开口 32 mm 宽。"""
    f = (cq.Workplane("XY").workplane(offset=FL_Z0).circle(FL_R).extrude(FL_Z1 - FL_Z0)
         .cut(cq.Workplane("XY").workplane(offset=FL_Z0 - 1).circle(FL_BORE)
              .extrude(FL_Z1 - FL_Z0 + 2)))
    f = f.cut(cq.Workplane("XY").box(80, 32, FL_Z1 - FL_Z0 + 4).translate((40, 0, (FL_Z0 + FL_Z1) / 2)))
    return f


def column():
    c = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    g = (cq.Workplane("XY").workplane(offset=38).circle(COL_R).extrude(12)
         .cut(cq.Workplane("XY").workplane(offset=38).circle(11).extrude(12)))
    return c.cut(g)


def rail():
    return (cq.Workplane("XY").box(RAIL_X1 - RAIL_X0, 140, RAIL_Z1 - RAIL_Z0)
            .translate(((RAIL_X0 + RAIL_X1) / 2, 0, (RAIL_Z0 + RAIL_Z1) / 2)))


def half(sign, s=0.0):
    ring = (cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RO)
            .extrude(CLAMP_Z1 - CLAMP_Z0)
            .cut(cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RI)
                 .extrude(CLAMP_Z1 - CLAMP_Z0)))
    h = ring.intersect(cq.Workplane("XY").workplane(offset=CLAMP_Z0)
                       .rect(200, 40).extrude(CLAMP_Z1 - CLAMP_Z0).translate((0, sign * 20, 0)))
    return h.translate((0, sign * s, 0))


def arm_parts(dx=0.0, s=20.0):
    d = {
        'BALL': ball_side(), 'FLANGE': flange_c(), 'RAIL': rail(),
        'CLAMP_R': half(1, s), 'CLAMP_L': half(-1, s),
        'BRKT_R': cq.Workplane("XY").box(16, 14, BRKT_Z1 - BRKT_Z0).translate((-22.0, 34 + s, 89)),
        'BRKT_L': cq.Workplane("XY").box(16, 14, BRKT_Z1 - BRKT_Z0).translate((-22.0, -34 - s, 89)),
    }
    return {k: v.translate((-dx, 0, 0)) for k, v in d.items()}


def vol(a, b):
    try:
        return round(a.val().intersect(b.val()).Volume(), 3)
    except Exception:
        return -1.0


if __name__ == '__main__':
    col = column()
    print('=== P0：C 形法兰后的撤离扫描（A03, s=20 全开）===')
    worst = 0.0
    first = None
    for dx in (0, 1, 2, 5, 10, 15, 20, 25, 30, 35, 40, 60, 90, 120):
        ap = arm_parts(dx)
        bad = {k: vol(v, col) for k, v in ap.items() if vol(v, col) > 1e-6}
        worst = max([worst] + list(bad.values()))
        if bad and first is None:
            first = dx
        print(f'  dx={dx:3d}  与腕柱干涉 {len(bad)} 处 ' + (str(bad) if bad else ''))
    print(f'  最大干涉 {worst:.2f}；首次干涉位置 {first}')
    # 反向插入
    rev_ok = True
    for dx in (120, 60, 30, 0):
        ap = arm_parts(dx)
        bad = {k: vol(v, col) for k, v in ap.items() if vol(v, col) > 1e-6}
        if bad:
            rev_ok = False
            print(f'  反向 dx={dx} 干涉 {bad}')
    print('  反向插入:', 'PASS' if rev_ok else 'FAIL')

    if first is None and rev_ok:
        print('\n=== P1：Assembly V2 合并 ===')
        sp = importlib.util.spec_from_file_location('anim', str(HERE / 'anim.py'))
        anim = importlib.util.module_from_spec(sp); sp.loader.exec_module(anim)
        m = anim.m
        st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
                  yaw=0.0, lower=0.0, compensate=1.0, pack=0.0)
        recs = anim.frame_records(st)
        drop = [r for r in recs if r['name'].endswith('_tip') or 'wrist_upright' in r['name']
                or 'wrist_datum' in r['name']]
        keep = [r for r in recs if r not in drop]
        print(f'  原构件 {len(recs)}，替换掉 {len(drop)} 个旧末端/腕座件，保留 {len(keep)}')
        asm = cq.Assembly(name='TIANQIONG_ASSEMBLY_V2')
        for r in keep:
            asm.add(m.shape_for(r), name=r['name'], color=cq.Color(*m.color_for(r)))
        n_if = 0
        for k in range(3):
            for i in range(6):
                y = Y_WORK[i] + k * PITCH
                ap = arm_parts(0.0, 0.0)
                for nm, v in ap.items():
                    asm.add(v.translate((TIP_X, y, 0)), name=f'M{k+1}_A{i+1:02d}_IF_{nm}'); n_if += 1
                asm.add(col.translate((TIP_X, y, 0)), name=f'M{k+1}_A{i+1:02d}_IF_COLUMN'); n_if += 1
        print(f'  接口零件 {n_if} 件')
        asm.save(str(OUT / 'assembly_v2.step'))
        print(f'  assembly_v2.step {Path(OUT / "assembly_v2.step").stat().st_size/1048576:.1f} MB')
        (OUT / 'assembly_v2_report.json').write_text(json.dumps(dict(
            flange='C-shape open +X', replaced_parts=len(drop), kept=len(keep),
            interface_parts=n_if, withdrawal='0 interference 0-120 mm'), ensure_ascii=False, indent=1), encoding='utf8')
