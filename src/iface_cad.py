"""接口组 I · 真实实体 CAD（CadQuery 2.8）

修正后的参数：
  球头      r=67 上半球 + 一体凸台 Ø48（z 58…76）
  盲孔      Ø32（r16），孔底 z=14，孔底壁厚 14
  腕柱      Ø30（r15），下端 z=14（坐于孔底，不再穿入）
  环槽      Ø22（r11），z 38…50（宽 12）
  锁爪 ×2   12×18×10，沿 ±Y 滑动，啮合 4 mm，行程 8 mm
  颈环      Ø60/Ø30，z 54…64
  滑块连接座 100×100×30，z 164…194，内孔 Ø34
"""
import cadquery as cq
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R = 67.0
BORE_R = 16.0
FLOOR_Z = 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
COL_R = 15.0
COL_Z0, COL_Z1 = 14.0, 194.0
GRV_R, GRV_Z0, GRV_Z1 = 11.0, 38.0, 50.0
CLAW = (12.0, 18.0, 10.0)
CLAW_TIP_Y = GRV_R           # 啮合时爪尖到 r=11
CLAW_TRAVEL = 8.0
RING_R, RING_Z0, RING_Z1 = 30.0, 76.0, 86.0
BASE = (100.0, 100.0)
BASE_Z0, BASE_Z1 = 164.0, 194.0
BASE_BORE_R = 17.0


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    body = hemi.union(boss)
    body = body.cut(cq.Workplane("XY").workplane(offset=FLOOR_Z).circle(BORE_R)
                    .extrude(BOSS_Z1 - FLOOR_Z))
    for sy in (-1, 1):
        win = (cq.Workplane("XY").box(CLAW[0], 34, CLAW[2] + 2)
               .translate((0, sy * 25, (GRV_Z0 + GRV_Z1) / 2)))
        body = body.cut(win)
    return body


def wrist_column():
    col = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    groove = (cq.Workplane("XY").workplane(offset=GRV_Z0).circle(COL_R).extrude(GRV_Z1 - GRV_Z0)
              .cut(cq.Workplane("XY").workplane(offset=GRV_Z0).circle(GRV_R).extrude(GRV_Z1 - GRV_Z0)))
    col = col.cut(groove)
    return col


def claw(retracted=False):
    cy = CLAW_TIP_Y + CLAW[1] / 2 + (CLAW_TRAVEL if retracted else 0.0)
    return cq.Workplane("XY").box(CLAW[0], CLAW[1], CLAW[2]).translate(
        (0, cy, (GRV_Z0 + GRV_Z1) / 2))


def neck_ring():
    ring = cq.Workplane("XY").workplane(offset=RING_Z0).circle(RING_R).extrude(RING_Z1 - RING_Z0)
    return ring.cut(cq.Workplane("XY").workplane(offset=RING_Z0).circle(COL_R)
                    .extrude(RING_Z1 - RING_Z0))


def slider_base():
    b = cq.Workplane("XY").box(BASE[0], BASE[1], BASE_Z1 - BASE_Z0).translate(
        (0, 0, (BASE_Z0 + BASE_Z1) / 2))
    return b.cut(cq.Workplane("XY").workplane(offset=BASE_Z0).circle(BASE_BORE_R)
                 .extrude(BASE_Z1 - BASE_Z0))


def shift(shape, dx=0.0, dy=0.0, dz=0.0):
    return shape.translate((dx, dy, dz))


def build_state(state):
    ball = tip_ball()
    col = wrist_column()
    ring = neck_ring()
    base = slider_base()
    if state == 'locked':
        cl = claw(False)
        parts = {'TIP_BALL': ball, 'COLUMN': col, 'RING': ring, 'BASE': base,
                 'CLAW_R': cl, 'CLAW_L': cl.mirror("XZ")}
    elif state == 'unlocked':
        cl = claw(True)
        parts = {'TIP_BALL': ball, 'COLUMN': col, 'RING': ring, 'BASE': base,
                 'CLAW_R': cl, 'CLAW_L': cl.mirror("XZ")}
    else:  # separated: 球头沿 -X 撤离 120
        parts = {'TIP_BALL': shift(ball, dx=-120.0), 'COLUMN': col, 'RING': ring,
                 'BASE': base}
    return parts


if __name__ == '__main__':
    report = {}
    for state in ('locked', 'unlocked', 'separated'):
        parts = build_state(state)
        names = list(parts)
        info = {}
        for n in names:
            s = parts[n]
            info[n] = dict(valid=bool(s.val().isValid()), volume=round(s.val().Volume(), 1))
        # 干涉
        inter = {}
        for i in range(len(names)):
            for j in range(i + 1, len(names)):
                a, b = parts[names[i]].val(), parts[names[j]].val()
                v = a.intersect(b).Volume()
                if v > 1e-6:
                    inter[f'{names[i]}×{names[j]}'] = round(v, 3)
        report[state] = dict(parts=info, interference=inter)
        asm = cq.Assembly()
        for n in names:
            asm.add(parts[n], name=n)
        asm.save(str(OUT / f'iface_group1_{state}.step'))
        for n in names:
            cq.exporters.export(parts[n], str(OUT / f'{n}.step'))
        print(f'[{state}] 零件 {len(names)}，干涉对数 {len(inter)}')
        for n, d in info.items():
            print(f"   {n:9s} valid={d['valid']} volume={d['volume']:.1f} mm3")
        for k, v in inter.items():
            print(f'   干涉 {k}: {v:.3f} mm3')

    # 关键尺寸复核
    probe = dict(
        bore_radius=BORE_R, column_radius=COL_R,
        radial_clearance=round(BORE_R - COL_R, 2),
        floor_z=FLOOR_Z, column_bottom_z=COL_Z0,
        axial_overlap=round(COL_Z0 - FLOOR_Z, 2),
        bore_mouth_z=BOSS_Z1, insert_depth=round(BOSS_Z1 - FLOOR_Z, 1),
        groove_depth=round(COL_R - GRV_R, 1), groove_width=round(GRV_Z1 - GRV_Z0, 1),
        claw_bearing_area=round(2 * CLAW[0] * (COL_R - GRV_R), 1),
        column_end_area=round(3.14159265 * COL_R ** 2, 1),
    )
    print('\n关键尺寸:', probe)
    import json
    (OUT / 'iface_group1_report.json').write_text(
        json.dumps(dict(report=report, probe=probe), ensure_ascii=False, indent=1), encoding='utf8')
    print('STEP 输出目录:', OUT)
