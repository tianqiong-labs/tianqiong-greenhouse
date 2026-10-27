"""单模块母墙 v3（台阶式，单根圆导轨，封闭壳体）。

改动：
  1. 双导轨 → **单根圆导轨 Ø80**（可被滚轮/卡爪夹持），坐在台阶立面上；
  2. 壳体改为**逐块平铺、无缝隙**：后板／后顶板／台阶立面／前顶板／前板／四块端板边界严格相接；
  3. 滚轮托架移到**靠近内侧墙面**（x −160…−30），托住机械臂近根部。

外形：x −800…0（导轨凸出至 +20），y 0…3500，z 0…3000（前段 2600）。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX = 3500.0, 800.0
WZ_B, WZ_F = 3000.0, 2600.0
STEP_X = -300.0
COL_Y, COL_X = 100.0, 100.0
BEAM_H = 60.0
GIRT = 60.0
SKIN = 30.0
FLOOR_T = 100.0
RAIL_R = 40.0
RAIL_X, RAIL_Z = -260.0, 2820.0
N_BAY = 6
YS = [COL_Y / 2 + (WY - COL_Y) * i / N_BAY for i in range(N_BAY + 1)]
X_BACK = -WX + COL_X / 2      # −750
X_FRONT = STEP_X + COL_X / 2  # −250

# 壳体分块边界（严格平铺，不留缝）
B_BACK0, B_BACK1 = -WX, -WX + SKIN          # −800…−770 后板
R_BACK0, R_BACK1 = B_BACK1, STEP_X          # −770…−300 后段内部
RISE0, RISE1 = STEP_X, STEP_X + SKIN        # −300…−270 台阶立面
F_TOP0, F_TOP1 = RISE1, -SKIN               # −270…−30 前段顶/端
F_SKIN0, F_SKIN1 = -SKIN, 0.0               # −30…0 前板


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


def slab(x0, x1, y0, y1, z0, z1):
    return box(x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


P = []
# 底板
P.append(('底板', slab(-WX, 0, 0, WY, 0, FLOOR_T)))
# 后板（通高）
P.append(('后板', slab(B_BACK0, B_BACK1, 0, WY, FLOOR_T, WZ_B)))
# 后顶板
P.append(('后顶板', slab(R_BACK0, R_BACK1, 0, WY, WZ_B - SKIN, WZ_B)))
# 后段两端板（顶板之下）
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    P.append((f'后端板_{i}', slab(R_BACK0, R_BACK1, y0, y1, FLOOR_T, WZ_B - SKIN)))
# 台阶立面（2600…3000）
P.append(('台阶立面', slab(RISE0, RISE1, 0, WY, WZ_F, WZ_B)))
# 前顶板
P.append(('前顶板', slab(F_TOP0, F_TOP1, 0, WY, WZ_F - SKIN, WZ_F)))
# 前段两端板
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    P.append((f'前端板_{i}', slab(F_TOP0, F_TOP1, y0, y1, FLOOR_T, WZ_F - SKIN)))
# 前板
P.append(('前板', slab(F_SKIN0, F_SKIN1, 0, WY, FLOOR_T, WZ_F - SKIN)))
# 骨架：后立柱 / 前立柱
for y in YS:
    h = WZ_B - FLOOR_T
    P.append((f'后立柱_{y:.0f}', box(COL_X, COL_Y, h, X_BACK, y, FLOOR_T + h / 2)))
for y in YS:
    h = WZ_F - SKIN - FLOOR_T
    P.append((f'前立柱_{y:.0f}', box(COL_X, COL_Y, h, X_FRONT, y, FLOOR_T + h / 2)))
# 横梁 / 横撑
P.append(('后上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, WZ_B - SKIN - BEAM_H / 2)))
P.append(('前上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_FRONT, WY / 2, WZ_F - SKIN - BEAM_H / 2)))
P.append(('下横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, FLOOR_T + BEAM_H / 2)))
for gz in (900.0, 1600.0, 2300.0):
    P.append((f'后横撑_{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, gz)))
for gz in (900.0, 1700.0):
    P.append((f'前横撑_{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_FRONT, WY / 2, gz)))
# 单根圆导轨 + 鞍座
P.append(('圆导轨', cq.Workplane("XZ").circle(RAIL_R).extrude(WY)
          .translate((RAIL_X, WY, RAIL_Z))))
for y in YS:
    P.append((f'导轨鞍座_{y:.0f}', slab(STEP_X, RAIL_X, y - 30, y + 30, RAIL_Z - 30, RAIL_Z + 30)))
# 滚轮托架（靠近内侧墙面）
for y in YS:
    P.append((f'托架座_{y:.0f}', slab(-160, -30, y - 15, y + 15, WZ_F, WZ_F + 70)))
    P.append((f'滚轮_{y:.0f}', cq.Workplane("XZ").circle(40).extrude(30)
              .translate((-95, y + 15, WZ_F + 70))))

asm = cq.Assembly()
for n, s in P:
    asm.add(s, name=n)
step = OUT / '单模块母墙_v3.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()
info = dict(version='v3（台阶式·单圆导轨·封闭壳体）', parts=len(P),
            bbox=dict(x=[round(bb.xmin, 1), round(bb.xmax, 1)],
                      y=[round(bb.ymin, 1), round(bb.ymax, 1)],
                      z=[round(bb.zmin, 1), round(bb.zmax, 1)]),
            geometry=dict(后段=dict(x=[-800, STEP_X], h=WZ_B),
                          前段=dict(x=[STEP_X, 0], h=WZ_F), 高差=WZ_B - WZ_F,
                          导轨=dict(形式='圆导轨', 直径=2 * RAIL_R, 轴线_z=RAIL_Z, 轴线_x=RAIL_X,
                                    可夹持='圆柱面，滚轮/卡爪可抱夹'),
                          滚轮托架=dict(位置='靠近内侧墙面 x −160…−30', 中心_x=-95, 顶面_z=WZ_F + 110),
                          壳体='后板/后顶板/台阶立面/前顶板/前板/四端板 逐块平铺'),
            参数=dict(柱=f'{COL_Y}×{COL_X}', 横梁高=BEAM_H, 横撑=GIRT, 板厚=SKIN,
                      地板厚=FLOOR_T, 跨数=N_BAY),
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / '单模块母墙_v3.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps(info, ensure_ascii=False))
