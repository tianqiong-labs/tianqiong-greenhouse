"""γ-2 接口实例化：先 A03 单套，再同模块 6 套、整棚 18 套。

父刚体划分（关键）：
  机械臂侧 ARM_SIDE：球头+凸台、过渡法兰、卡箍导轨、支架、滑块、卡箍半环、轴向锁爪
  锚轨侧 RAIL_SIDE：腕柱、腕柱底座
锁止时两侧接触，解锁后各自随父刚体运动；不得整体作为一个刚体。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
FLANGE_R, FLANGE_Z0, FLANGE_Z1, FLANGE_BORE = 20.0, 76.0, 82.0, 16.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
CLAMP_Z0, CLAMP_Z1 = 90.0, 110.0
CLAMP_RI, CLAMP_RO = 15.5, 32.0
GATE_X0, GATE_X1 = 16.0, 28.0
RAIL_X0, RAIL_X1, RAIL_Z0, RAIL_Z1 = -30.0, -16.0, 82.0, 88.0
BRKT_Z0, BRKT_Z1 = 88.0, 90.0
STROKE = 20.0
PITCH = 4250.0
Y_NOM = [350.0, 910.0, 1470.0, 2030.0, 2590.0, 3150.0]
Y_WORK = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
TIP_X = 4500.0


def ball_side():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    b = b.cut(cq.Workplane("XY").box(90, 32, BOSS_Z1 - FLOOR + 2).translate((59, 0, 45)))
    return b


def flange():
    f = (cq.Workplane("XY").workplane(offset=FLANGE_Z0).circle(FLANGE_R)
         .extrude(FLANGE_Z1 - FLANGE_Z0))
    return f.cut(cq.Workplane("XY").workplane(offset=FLANGE_Z0 - 1).circle(FLANGE_BORE)
                 .extrude(FLANGE_Z1 - FLANGE_Z0 + 2))


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


def inst(y_world, s=0.0):
    """返回该臂位的一套接口：ARM_SIDE / RAIL_SIDE 两个字典。"""
    arm = {
        'BALL': ball_side(), 'FLANGE': flange(), 'RAIL': rail(),
        'CLAMP_R': half(1, s), 'CLAMP_L': half(-1, s),
        'BRKT_R': (cq.Workplane("XY").box(16, 14, BRKT_Z1 - BRKT_Z0)
                   .translate((-22.0, 34 + s, (BRKT_Z0 + BRKT_Z1) / 2))),
        'BRKT_L': (cq.Workplane("XY").box(16, 14, BRKT_Z1 - BRKT_Z0)
                   .translate((-22.0, -34 - s, (BRKT_Z0 + BRKT_Z1) / 2))),
    }
    rail_side = {'COLUMN': column()}
    tr = lambda sh: sh.translate((TIP_X, y_world, 0))
    return {k: tr(v) for k, v in arm.items()}, {k: tr(v) for k, v in rail_side.items()}


def vol(a, b):
    try:
        return round(a.val().intersect(b.val()).Volume(), 3)
    except Exception:
        return -1.0


def check_single(y, tag):
    arm, rail = inst(y)
    bad = {}
    # 臂侧内部
    na = list(arm)
    for i in range(len(na)):
        for j in range(i + 1, len(na)):
            v = vol(arm[na[i]], arm[na[j]])
            if v > 1e-6:
                bad[f'{na[i]}×{na[j]}'] = v
    # 两侧（锁止态应接触但不穿透）
    for a in na:
        v = vol(arm[a], rail['COLUMN'])
        if v > 1e-6:
            bad[f'{a}×COLUMN'] = v
    print(f'{tag:24s} 干涉 {len(bad)} 处 ' + (str(bad) if bad else ''))
    return bad


if __name__ == '__main__':
    print('=== 1. A03 单套（模块 1，补位后 Y=1283.3）===')
    y = Y_WORK[2]
    bad = check_single(y, 'A03 LOCKED')
    a3, r3 = inst(y)
    print(f'  凸台顶 76 → 法兰 76…82 → 导轨 82：间隙 '
          f'{vol(a3["FLANGE"], a3["RAIL"]):.2f}（0=面接触）')
    print(f'  法兰 vs 腕柱：{vol(a3["FLANGE"], r3["COLUMN"]):.2f}（应 0，孔 Ø32 让开 Ø30）')

    asm = cq.Assembly()
    for k, v in a3.items():
        asm.add(v, name=f'A03_{k}')
    for k, v in r3.items():
        asm.add(v, name=f'A03_{k}_RAILSIDE')
    asm.save(str(OUT / 'gamma2_inst_A03.step'))

    print('\n=== 2. 同模块 6 套（模块 1，补位后 Y）===')
    total_bad = 0
    for i in range(6):
        total_bad += len(check_single(Y_WORK[i], f'A{i+1:02d}'))
    asm = cq.Assembly()
    for i in range(6):
        a_i, r_i = inst(Y_WORK[i])
        for k, v in a_i.items():
            asm.add(v, name=f'M1_A{i+1:02d}_{k}')
        for k, v in r_i.items():
            asm.add(v, name=f'M1_A{i+1:02d}_{k}_RAILSIDE')
    asm.save(str(OUT / 'gamma2_inst_module1.step'))
    print(f'  模块 1 合计干涉 {total_bad} 处')

    print('\n=== 3. 整棚 18 套 ===')
    asm = cq.Assembly()
    n = 0
    for k in range(3):
        for i in range(6):
            a_k, r_k = inst(Y_WORK[i] + k * PITCH)
            for nm, v in a_k.items():
                asm.add(v, name=f'M{k+1}_A{i+1:02d}_{nm}'); n += 1
            for nm, v in r_k.items():
                asm.add(v, name=f'M{k+1}_A{i+1:02d}_{nm}_RAILSIDE'); n += 1
    asm.save(str(OUT / 'gamma2_inst_all18.step'))
    print(f'  实体件数 {n}，导出 gamma2_inst_all18.step')
