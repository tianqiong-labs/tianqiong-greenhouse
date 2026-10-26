"""γ-2 收口：真实导轨 + 滑块 + 支架 + 机械限位 + 双向序列。

空间布局依据：
  腕柱 Ø30（r15）在 x=0,y=0；+X 通道 x≥14、y ±16 必须保持畅通；
  凸台顶 z=76；卡箍 z 90…110；上压条 z 133.5 起。
  唯一可用走线：−X 内侧（x −30…−16），距柱轴 ≥16，且不在通道内。
"""
import sys as _sys
_sys.path.insert(0, str(__import__('pathlib').Path(__file__).resolve().parent))
from verify_guards import safe_intersect, gate, IntersectError
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
GRV_R, GRV_Z0, GRV_Z1 = 11.0, 38.0, 50.0
CLAW = (12.0, 18.0, 10.0)
CLAMP_Z0, CLAMP_Z1 = 90.0, 110.0
CLAMP_RI, CLAMP_RO = 15.5, 32.0
RAIL_X0, RAIL_X1 = -30.0, -16.0
RAIL_Z0, RAIL_Z1 = 82.0, 88.0
CARR_Z0, CARR_Z1 = 84.0, 88.0
BRKT_Z0, BRKT_Z1 = 88.0, 90.0   # 顶面与卡箍底面同面，不重叠
STROKE = 20.0
ZMID = (FLOOR + BOSS_Z1) / 2


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    b = b.cut(cq.Workplane("XY").box(90, 32, BOSS_Z1 - FLOOR + 2).translate((59, 0, ZMID)))
    for sy in (-1, 1):
        b = b.cut(cq.Workplane("XY").box(CLAW[0], 34, CLAW[2] + 2).translate((0, sy * 25, 44)))
    return b


def column(dx=0.0, dz=0.0):
    c = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    g = (cq.Workplane("XY").workplane(offset=GRV_Z0).circle(COL_R).extrude(GRV_Z1 - GRV_Z0)
         .cut(cq.Workplane("XY").workplane(offset=GRV_Z0).circle(GRV_R).extrude(GRV_Z1 - GRV_Z0)))
    return c.cut(g).translate((dx, 0, dz))


def claw(retracted, sy, dx=0.0, dz=0.0):
    cy = sy * (GRV_R + CLAW[1] / 2 + (8.0 if retracted else 0.0))
    return cq.Workplane("XY").box(*CLAW).translate((dx, cy, 44 + dz))


def half(sign, s=0.0, dx=0.0, dz=0.0):
    ring = (cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RO)
            .extrude(CLAMP_Z1 - CLAMP_Z0)
            .cut(cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RI)
                 .extrude(CLAMP_Z1 - CLAMP_Z0)))
    h = ring.intersect(cq.Workplane("XY").workplane(offset=CLAMP_Z0)
                       .rect(200, 40).extrude(CLAMP_Z1 - CLAMP_Z0).translate((0, sign * 20, 0)))
    return h.translate((dx, sign * s, dz))   # s 沿 Y 开合，dx 沿 X 撤离


def rail():
    return (cq.Workplane("XY").box(RAIL_X1 - RAIL_X0, 140, RAIL_Z1 - RAIL_Z0)
            .translate(((RAIL_X0 + RAIL_X1) / 2, 0, (RAIL_Z0 + RAIL_Z1) / 2)))


def carriage(sign, s=0.0, dz=0.0):
    y = sign * (24 + s)
    return (cq.Workplane("XY").box(RAIL_X1 - RAIL_X0, 18, CARR_Z1 - CARR_Z0)
            .translate(((RAIL_X0 + RAIL_X1) / 2, y, (CARR_Z0 + CARR_Z1) / 2 + dz)))


def bracket(sign, s=0.0, dz=0.0):
    y = sign * (24 + s)
    return (cq.Workplane("XY").box(16, 14, BRKT_Z1 - BRKT_Z0)
            .translate((-22.0, y, (BRKT_Z0 + BRKT_Z1) / 2 + dz)))


def vol(a, b):
    return round(safe_intersect(a, b), 3)


def scene(s, ball_dx=0.0, ball_dz=0.0, claws_ret=True):
    b = tip_ball().translate((-ball_dx, 0, -ball_dz))
    c = column()
    parts = {'BALL': b, 'COLUMN': c, 'RAIL': rail().translate((-ball_dx, 0, -ball_dz)),
             'CLAMP_R': half(1, s, -ball_dx, -ball_dz), 'CLAMP_L': half(-1, s, -ball_dx, -ball_dz),
             'CARR_R': carriage(1, s, -ball_dz).translate((-ball_dx, 0, 0)),
             'CARR_L': carriage(-1, s, -ball_dz).translate((-ball_dx, 0, 0)),
             'BRKT_R': bracket(1, s, -ball_dz).translate((-ball_dx, 0, 0)),
             'BRKT_L': bracket(-1, s, -ball_dz).translate((-ball_dx, 0, 0)),
             'CLAW_R': claw(claws_ret, 1, -ball_dx, -ball_dz),
             'CLAW_L': claw(claws_ret, -1, -ball_dx, -ball_dz)}
    return parts


def check(parts, label):
    names = list(parts)   # 腕柱与导轨必须参与检查，不得整件排除
    bad = {}
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            v = vol(parts[names[i]], parts[names[j]])
            if v > 1e-6:
                bad[f'{names[i]}×{names[j]}'] = v
    print(f'{label:28s} 干涉 {len(bad)} 处 ' + (str(bad) if bad else ''))
    return bad


if __name__ == '__main__':
    print('=== 开合扫描（s=0…20）===')
    worst = 0
    for s in (0, 2.5, 5, 7.5, 10, 12.5, 15, 17.5, 20):
        bad = check(scene(s), f'OPEN s={s:4.1f}')
        worst = max([worst] + list(bad.values()))
    print('开合最大干涉:', worst)

    print('\n=== 锁止阻挡（未解锁时柱 +X）===')
    p = scene(0.0, claws_ret=False)
    blk = vol(p['CLAMP_R'], p['COLUMN'].translate((20, 0, 0)))
    print(f'  柱+X20 vs 半环: {blk:.2f} mm3 -> {"挡止成立" if blk > 1 else "未挡"}')

    print('\n=== 正向：LOCKED→OPEN→WITHDRAW ===')
    check(scene(20, claws_ret=True), 'FULLY_OPEN')
    for dx in (0, 30, 60, 90, 120):
        bad = check(scene(20, ball_dx=dx), f'WITHDRAW dx={dx}')
    print('\n=== 反向：APPROACH→INSERTED→CLOSING→LOCKED ===')
    for dx in (120, 60, 30, 0):
        check(scene(20, ball_dx=dx), f'APPROACH dx={dx}')
    check(scene(0, ball_dx=0), 'CLOSING s=0')
    check(scene(0, ball_dx=0, claws_ret=False), 'LOCKED')

    asm = cq.Assembly()
    for n, sh in scene(0.0, claws_ret=False).items():
        asm.add(sh, name=n)
    asm.save(str(OUT / 'gamma2_final_LOCKED.step'))
    asm2 = cq.Assembly()
    for n, sh in scene(20.0, ball_dx=120).items():
        asm2.add(sh, name=n)
    asm2.save(str(OUT / 'gamma2_final_SEPARATED.step'))
    sec = cq.Workplane("XY").workplane(offset=100).add(
        cq.Workplane("XY").workplane(offset=0).toPending() if False else scene(0.0)['CLAMP_R']).section()
    cq.exporters.export(sec, str(OUT / 'gamma2_final_section_z100.svg'))
    print('\nSTEP/SVG 输出完成')
