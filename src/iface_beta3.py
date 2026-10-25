"""β-3：外置铰接摆舌（铰轴平行 X，铰点 y=+15, z=76）+ 角度搜索。

P0 修正：摆舌 y 半宽 15；开口通道起点 x=14（与孔 r16 连通）。
"""
import cadquery as cq, json, math
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
GRV_R, GRV_Z0, GRV_Z1 = 11.0, 38.0, 50.0
CLAW = (12.0, 18.0, 10.0)
TRAVEL = 8.0
CH_X0 = 14.0
GATE_X0, GATE_X1, GATE_HY = 16.0, 28.0, 15.0
HINGE_Y, HINGE_Z = 15.0, 76.0
ZC = (FLOOR + BOSS_Z1) / 2


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    b = b.cut(cq.Workplane("XY").box(90, 2 * 16.0, BOSS_Z1 - FLOOR + 2).translate((CH_X0 + 45, 0, ZC)))
    for sy in (-1, 1):
        b = b.cut(cq.Workplane("XY").box(CLAW[0], 34, CLAW[2] + 2)
                  .translate((0, sy * 25, (GRV_Z0 + GRV_Z1) / 2)))
    return b


def column():
    col = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    ring = (cq.Workplane("XY").workplane(offset=GRV_Z0).circle(COL_R).extrude(GRV_Z1 - GRV_Z0)
            .cut(cq.Workplane("XY").workplane(offset=GRV_Z0).circle(GRV_R).extrude(GRV_Z1 - GRV_Z0)))
    return col.cut(ring)


def gate(theta=0.0):
    g = (cq.Workplane("XY").box(GATE_X1 - GATE_X0, 2 * GATE_HY, BOSS_Z1 - FLOOR)
         .translate(((GATE_X0 + GATE_X1) / 2, 0, ZC)))
    if theta:
        g = g.rotate((0, HINGE_Y, HINGE_Z), (1, HINGE_Y, HINGE_Z), -theta)
    return g


def vol(a, b):
    try:
        return round(a.val().intersect(b.val()).Volume(), 3)
    except Exception:
        return -1.0


if __name__ == '__main__':
    ball, col = tip_ball(), column()
    print('角度  摆舌-球头   摆舌-腕柱')
    p0 = gate(0.0)
    print(f"  0   {vol(p0, ball):9.2f} {vol(p0, col):9.2f}")
    first_clear = None
    for th in range(10, 121, 10):
        g = gate(float(th))
        vb, vc = vol(g, ball), vol(g, col)
        print(f"{th:3d}   {vb:9.2f} {vc:9.2f}")
        if vb <= 1e-6 and vc <= 1e-6 and first_clear is None:
            first_clear = th
    print('最小无干涉角:', first_clear)
    # 反向：锁止态柱 +X 20 是否被挡
    p = gate(0.0)
    colpush = column().translate((20, 0, 0))
    blocked = vol(p, colpush)
    print(f'锁止态 柱+X20 与摆舌干涉: {blocked:.2f} mm3 ->', '被挡 ✓' if blocked > 1 else '未挡 ✗')
    asm = cq.Assembly()
    asm.add(ball, name='TIP_BALL'); asm.add(col, name='COLUMN'); asm.add(gate(0.0), name='GATE_LOCKED')
    asm.save(str(OUT / 'beta3_LOCKED.step'))
    asm2 = cq.Assembly()
    asm2.add(ball, name='TIP_BALL'); asm2.add(col, name='COLUMN'); asm2.add(gate(float(first_clear or 90)), name='GATE_OPEN')
    asm2.save(str(OUT / 'beta3_UNLOCKED.step'))
    secs = {'section_long_y0': cq.Workplane("XZ").add(ball).add(col).section(),
            'section_trans_z44': cq.Workplane("XY").workplane(offset=44).add(ball).add(col).section()}
    for n, s in secs.items():
        cq.exporters.export(s, str(OUT / f'{n}.svg'))
        cq.exporters.export(s, str(OUT / f'{n}.dxf'))
    (OUT / 'beta3_report.json').write_text(json.dumps(dict(min_clear_angle=first_clear), indent=1), encoding='utf8')
    print('完成')
