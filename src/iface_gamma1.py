"""γ-1：双侧对开外置卡箍（在球头之上的外部空间抱合腕柱）。

空间判断：球头 z 0…67、凸台 58…76；卡箍置于 z 90…110，
其上方为锚轨上压条 z 133.5…146.5，下方为凸台顶 76 → 净空 14 / 23 mm。
卡箍内径 31（柱 30，单边 0.5）、外径 64；两半绕 Z 轴在 y=±32 处铰接。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
CLAMP_Z0, CLAMP_Z1 = 90.0, 110.0
CLAMP_RI, CLAMP_RO = 15.5, 32.0
HINGE_Y = 32.0
ZC = (CLAMP_Z0 + CLAMP_Z1) / 2


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    b = b.cut(cq.Workplane("XY").box(90, 32, BOSS_Z1 - FLOOR + 2).translate((59, 0, ZC_A := 45)))
    return b


def column():
    return cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)


def half(sign, theta=0.0):
    ring = (cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RO)
            .extrude(CLAMP_Z1 - CLAMP_Z0)
            .cut(cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RI)
                 .extrude(CLAMP_Z1 - CLAMP_Z0)))
    h = ring.intersect(cq.Workplane("XY").workplane(offset=CLAMP_Z0)
                       .rect(200, 40).extrude(CLAMP_Z1 - CLAMP_Z0)
                       .translate((0, sign * 20, 0)))
    if theta:
        h = h.rotate((0, sign * HINGE_Y, ZC), (0, sign * HINGE_Y, ZC + 1), -sign * theta)
    return h


def vol(a, b):
    try:
        return round(a.val().intersect(b.val()).Volume(), 3)
    except Exception:
        return -1.0


if __name__ == '__main__':
    ball, col = tip_ball(), column()
    print('θ    半环-球头  半环-柱  环-环')
    worst = 0.0
    for th in (0, 10, 20, 30, 45, 60, 75, 90):
        a, b2 = half(1, th), half(-1, th)
        wa, wb, wc = vol(a, ball), vol(a, col), vol(a, b2)
        worst = max(worst, wa, wb, wc)
        print(f'{th:3d}  {wa:9.2f} {wb:8.2f} {wc:8.2f}')
    print('最大干涉:', worst)
    # 锁止阻挡
    blocked = vol(half(1, 0), col.translate((20, 0, 0)))
    print('锁止态 柱+X20 vs 半环:', blocked, '->', '被挡' if blocked > 1 else '未挡')
    # 打开后撤离
    for dx in (0, 30, 60, 120):
        b = ball.translate((-dx, 0, 0))
        o = vol(half(1, 90).translate((-dx, 0, 0)), col)
        print(f'  撤离 {dx:3d}: 球-柱 {vol(b, col):8.2f}  半环-柱 {o:8.2f}')
    asm = cq.Assembly()
    for n, s in (('TIP_BALL', ball), ('COLUMN', col),
                 ('CLAMP_R', half(1, 0)), ('CLAMP_L', half(-1, 0))):
        asm.add(s, name=n)
    asm.save(str(OUT / 'gamma1_LOCKED.step'))
    asm2 = cq.Assembly()
    for n, s in (('TIP_BALL', ball), ('COLUMN', col),
                 ('CLAMP_R', half(1, 90)), ('CLAMP_L', half(-1, 90))):
        asm2.add(s, name=n)
    asm2.save(str(OUT / 'gamma1_OPEN.step'))
    sec = cq.Workplane("XY").workplane(offset=ZC).add(ball).add(col).add(half(1, 0)).add(half(-1, 0)).section()
    cq.exporters.export(sec, str(OUT / 'gamma1_section_z100.svg'))
    cq.exporters.export(sec, str(OUT / 'gamma1_section_z100.dxf'))
    (OUT / 'gamma1_report.json').write_text(json.dumps(dict(max_interference=worst,
                                                            blocked=blocked), indent=1), encoding='utf8')
    print('完成')
