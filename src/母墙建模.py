"""单模块母墙 v1.2（墙体外壳 + 内骨架）。

相对 v1.1 的改动：
  1. 地板加厚到 100 mm 夹芯板（原 4 mm 太薄）；
  2. 删除 4 处角部斜撑（Knee）——功能不明确，不需要；
  3. 补齐墙体外壳：前板、背板、顶板、两端板，形成封闭壳体；
  4. 双导轨改为外挂在前板外侧（x 0…50），进棚内 50 mm。

坐标：x ∈ [−800, 50]（含外挂导轨），y ∈ [0, 3500]，z ∈ [0, 2800]；+X 朝棚内。
"""
import cadquery as cq, json, math
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX, WZ = 3500.0, 800.0, 2800.0
COL_Y, COL_X = 100.0, 100.0
BEAM_H = 60.0
GIRT = 60.0
GIRT_Z = (900.0, 1600.0, 2300.0)
SKIN = 30.0
FLOOR_T = 100.0
RAIL = 50.0
RAIL_Z = (2370.0, 2670.0)
N_BAY = 6
YS = [COL_Y / 2 + (WY - COL_Y) * i / N_BAY for i in range(N_BAY + 1)]
X_BACK = -WX + COL_X / 2        # −750
Z_FLOOR = FLOOR_T               # 100
Z_TOP = WZ - SKIN               # 2770


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


parts = []
# 1 地板 100 厚
parts.append(('FLOOR', box(WX, WY, FLOOR_T, -WX / 2, WY / 2, FLOOR_T / 2)))
# 2 立柱（落在地板上）
H = Z_TOP - Z_FLOOR
for y in YS:
    parts.append((f'COL_{y:.0f}', box(COL_X, COL_Y, H, X_BACK, y, Z_FLOOR + H / 2)))
# 3 上下横梁
parts.append(('BEAM_B', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, Z_FLOOR + BEAM_H / 2)))
parts.append(('BEAM_T', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, Z_TOP - BEAM_H / 2)))
# 4 横撑 3 道（无斜撑）
for gz in GIRT_Z:
    parts.append((f'GIRT_{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, gz)))
# 5 外壳：前板 / 背板 / 顶板 / 两端板
parts.append(('SKIN_FRONT', box(SKIN, WY, Z_TOP - Z_FLOOR, -SKIN / 2, WY / 2, Z_FLOOR + (Z_TOP - Z_FLOOR) / 2)))
parts.append(('SKIN_BACK', box(SKIN, WY, Z_TOP - Z_FLOOR, -WX + SKIN / 2, WY / 2, Z_FLOOR + (Z_TOP - Z_FLOOR) / 2)))
parts.append(('SKIN_TOP', box(WX, WY, SKIN, -WX / 2, WY / 2, WZ - SKIN / 2)))
for i, y in enumerate((SKIN / 2, WY - SKIN / 2)):
    parts.append((f'SKIN_END_{i}', box(WX, SKIN, Z_TOP - Z_FLOOR, -WX / 2, y, Z_FLOOR + (Z_TOP - Z_FLOOR) / 2)))
# 6 双导轨（外挂在前板外侧）
for rz in RAIL_Z:
    parts.append((f'RAIL_{int(rz)}', box(RAIL, WY, RAIL, RAIL / 2, WY / 2, rz)))
# 7 导轨端部限位
for rz in RAIL_Z:
    for y in (60.0, WY - 60.0):
        parts.append((f'STOP_{int(rz)}_{int(y)}', box(RAIL, 60, 120, RAIL / 2, y, rz)))

asm = cq.Assembly()
for n, s in parts:
    asm.add(s, name=n)
step = OUT / '单模块母墙_v1.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()
info = dict(version='v1.2', parts=len(parts),
            bbox=dict(x=[round(bb.xmin, 1), round(bb.xmax, 1)],
                      y=[round(bb.ymin, 1), round(bb.ymax, 1)],
                      z=[round(bb.zmin, 1), round(bb.zmax, 1)]),
            sections=dict(col=[COL_Y, COL_X], beam_h=BEAM_H, girt=GIRT, girt_z=list(GIRT_Z),
                          skin=SKIN, floor_t=FLOOR_T, rail=RAIL, rail_z=list(RAIL_Z),
                          bays=N_BAY, knee='已删除'),
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / '单模块母墙_v1.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps(info, ensure_ascii=False))
