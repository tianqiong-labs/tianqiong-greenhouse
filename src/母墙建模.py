"""单模块母墙 v2（台阶式，总高 3000）。

台阶：后段 x −800…−300 高 3000；前段 x −300…0 高 2600；立面高差 400。
双导轨全部坐在台阶立面上（z 2670 / 2840），伸出 50。
低平面 z=2600 上按每根立柱配一组滚轮托架，托住机械臂。
零件统一中文命名。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX = 3500.0, 800.0
WZ_B, WZ_F = 3000.0, 2600.0      # 后段 / 前段
STEP_X = -300.0
COL_Y, COL_X = 100.0, 100.0
BEAM_H = 60.0
GIRT = 60.0
SKIN = 30.0
FLOOR_T = 100.0
RAIL = 50.0
RAIL_Z = (2670.0, 2840.0)
N_BAY = 6
YS = [COL_Y / 2 + (WY - COL_Y) * i / N_BAY for i in range(N_BAY + 1)]
X_BACK = -WX + COL_X / 2         # −750
X_FRONT = STEP_X + COL_X / 2     # −250


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


P = []
# ── 地板
P.append(('底板', box(WX, WY, FLOOR_T, -WX / 2, WY / 2, FLOOR_T / 2)))
# ── 后段立柱 / 前段立柱
for y in YS:
    h = WZ_B - FLOOR_T
    P.append((f'后立柱_{y:.0f}', box(COL_X, COL_Y, h, X_BACK, y, FLOOR_T + h / 2)))
for y in YS:
    h = WZ_F - FLOOR_T
    P.append((f'前立柱_{y:.0f}', box(COL_X, COL_Y, h, X_FRONT, y, FLOOR_T + h / 2)))
# ── 横梁
P.append(('后上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, WZ_B - SKIN - BEAM_H / 2)))
P.append(('前上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_FRONT, WY / 2, WZ_F - SKIN - BEAM_H / 2)))
P.append(('下横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, FLOOR_T + BEAM_H / 2)))
# ── 横撑
P.append(('后横撑_900', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, 900)))
P.append(('后横撑_1600', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, 1600)))
P.append(('后横撑_2300', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, 2300)))
P.append(('前横撑_900', box(COL_X, WY - 2 * SKIN, GIRT, X_FRONT, WY / 2, 900)))
P.append(('前横撑_1700', box(COL_X, WY - 2 * SKIN, GIRT, X_FRONT, WY / 2, 1700)))
# ── 外壳
P.append(('后板', box(SKIN, WY, WZ_B - FLOOR_T, -WX + SKIN / 2, WY / 2, FLOOR_T + (WZ_B - FLOOR_T) / 2)))
P.append(('前板', box(SKIN, WY, WZ_F - FLOOR_T, -SKIN / 2, WY / 2, FLOOR_T + (WZ_F - FLOOR_T) / 2)))
P.append(('台阶立面', box(SKIN, WY, 400.0, STEP_X - SKIN / 2, WY / 2, (WZ_F + WZ_B) / 2)))
P.append(('后顶板', box(WX + STEP_X, WY, SKIN, (STEP_X - WX) / 2, WY / 2, WZ_B - SKIN / 2)))
P.append(('前顶板', box(-STEP_X, WY, SKIN, STEP_X / 2, WY / 2, WZ_F - SKIN / 2)))
for i, y in enumerate((SKIN / 2, WY - SKIN / 2)):
    P.append((f'后端板_{i}', box(-STEP_X, SKIN, WZ_B - FLOOR_T, (STEP_X - WX) / 2, y, FLOOR_T + (WZ_B - FLOOR_T) / 2)))
    P.append((f'前端板_{i}', box(-STEP_X, SKIN, WZ_F - FLOOR_T, STEP_X / 2, y, FLOOR_T + (WZ_F - FLOOR_T) / 2)))
# ── 双导轨（坐在台阶立面上）
for rz in RAIL_Z:
    P.append((f'导轨_{int(rz)}', box(RAIL, WY, RAIL, STEP_X + RAIL / 2, WY / 2, rz)))
# ── 滚轮托架（低平面，每根立柱一组）
for y in YS:
    P.append((f'托架座_{y:.0f}', box(140, 30, 80, STEP_X + 70, y, WZ_F + 40)))
    P.append((f'滚轮_{y:.0f}', cq.Workplane("XZ").circle(40).extrude(36).translate((STEP_X + 110, y + 18, WZ_F + 80))))

asm = cq.Assembly()
for n, s in P:
    asm.add(s, name=n)
step = OUT / '单模块母墙_v2.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()
info = dict(version='v2（台阶式）', parts=len(P),
            bbox=dict(x=[round(bb.xmin, 1), round(bb.xmax, 1)],
                      y=[round(bb.ymin, 1), round(bb.ymax, 1)],
                      z=[round(bb.zmin, 1), round(bb.zmax, 1)]),
            geometry=dict(后段=dict(x=[-800, STEP_X], h=WZ_B),
                          前段=dict(x=[STEP_X, 0], h=WZ_F),
                          高差=WZ_B - WZ_F, 导轨_z=list(RAIL_Z), 导轨座位='台阶立面',
                          滚轮托架='每立柱一组，低平面 z=2600'),
            参数=dict(柱=f'{COL_Y}×{COL_X}', 横梁高=BEAM_H, 横撑=GIRT, 板厚=SKIN,
                      地板厚=FLOOR_T, 导轨=RAIL, 跨数=N_BAY),
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / '单模块母墙_v2.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps(info, ensure_ascii=False))
