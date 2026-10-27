"""单模块母墙 v4：T 形槽导轨、封闭端墙、可调 V 形滚轮托座、ASCII 零件名 + 中文清单。

修正 v3 的四个问题：
  1. 圆导轨 → **T 形槽导轨**（外口 40、内腔 70，可抱夹滑块）；
  2. 前顶板只到 x=−30，前端上沿 30×30 缺口 → 本次铺到 x=0，前板让出顶部；
  3. 滚轮托座重做：悬臂梁 + 立板 + 两只错高滚轮，形成 V 形承托，真正对上臂的下表面；
  4. STEP 里中文名会丢（显示空白）→ 改用 ASCII 零件名，中文对照写入《零件清单.csv》。
"""
import cadquery as cq, csv, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX = 3500.0, 800.0
WZ_B, WZ_F = 3000.0, 2600.0
STEP_X = -300.0
COL_Y, COL_X = 100.0, 100.0
BEAM_H = GIRT = 60.0
SKIN = 30.0
FLOOR_T = 100.0
N_BAY = 6
YS = [COL_Y / 2 + (WY - COL_Y) * i / N_BAY for i in range(N_BAY + 1)]
X_BACK, X_FRONT = -WX + COL_X / 2, STEP_X + COL_X / 2

B0, B1 = -WX, -WX + SKIN          # 后板
RB0, RB1 = B1, STEP_X             # 后段内部
R0, R1 = STEP_X, STEP_X + SKIN    # 台阶立面
F0, F1 = R1, -SKIN                # 前段内部
S0, S1 = -SKIN, 0.0               # 前板


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


def slab(x0, x1, y0, y1, z0, z1):
    return box(x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


BOM = []          # (ascii, 中文, 说明)
P = []


def add(ascii_name, cn, shape, note=''):
    P.append((ascii_name, shape))
    BOM.append(dict(name=ascii_name, 中文=cn, 说明=note))


# ── 壳体
add('FLOOR', '底板', slab(-WX, 0, 0, WY, 0, FLOOR_T), '100 厚夹芯板')
add('SKIN_BACK', '后板', slab(B0, B1, 0, WY, FLOOR_T, WZ_B), '30 厚')
add('TOP_BACK', '后顶板', slab(RB0, RB1, 0, WY, WZ_B - SKIN, WZ_B), '30 厚')
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    add(f'END_BACK_{i+1}', f'后端板{i+1}', slab(RB0, RB1, y0, y1, FLOOR_T, WZ_B - SKIN), '30 厚')
add('RISER', '台阶立面', slab(R0, R1, 0, WY, WZ_F, WZ_B), '导轨安装面')
add('TOP_FRONT', '前顶板', slab(F0, 0, 0, WY, WZ_F - SKIN, WZ_F), '铺到 x=0，消除上沿缺口')
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    add(f'END_FRONT_{i+1}', f'前端板{i+1}', slab(F0, F1, y0, y1, FLOOR_T, WZ_F - SKIN), '30 厚')
add('SKIN_FRONT', '前板', slab(S0, S1, 0, WY, FLOOR_T, WZ_F - SKIN), '30 厚，顶部让给前顶板')

# ── 骨架
for y in YS:
    h = WZ_B - FLOOR_T
    add(f'COL_B_{y:.0f}', f'后立柱@{y:.0f}', box(COL_X, COL_Y, h, X_BACK, y, FLOOR_T + h / 2), '100×100')
for y in YS:
    h = WZ_F - SKIN - FLOOR_T
    add(f'COL_F_{y:.0f}', f'前立柱@{y:.0f}', box(COL_X, COL_Y, h, X_FRONT, y, FLOOR_T + h / 2), '100×100')
add('BEAM_TOP_B', '后上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, WZ_B - SKIN - BEAM_H / 2), '100×60')
add('BEAM_TOP_F', '前上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_FRONT, WY / 2, WZ_F - SKIN - BEAM_H / 2), '100×60')
add('BEAM_BOT', '下横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, FLOOR_T + BEAM_H / 2), '100×60')
for gz in (900.0, 1600.0, 2300.0):
    add(f'GIRT_B_{int(gz)}', f'后横撑@{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, gz), '60×60')
for gz in (900.0, 1700.0):
    add(f'GIRT_F_{int(gz)}', f'前横撑@{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_FRONT, WY / 2, gz), '60×60')

# ── T 形槽导轨（外口 40 / 内腔 70 / 槽深 30，槽口朝 +X）
RAIL_X0, RAIL_X1 = STEP_X, STEP_X + 100.0
RAIL_Z0, RAIL_Z1 = 2740.0, 2800.0
RAIL_ZC = (RAIL_Z0 + RAIL_Z1) / 2
rail = box(RAIL_X1 - RAIL_X0, WY, RAIL_Z1 - RAIL_Z0, (RAIL_X0 + RAIL_X1) / 2, WY / 2, RAIL_ZC)
rail = rail.cut(box(12, 40, 30, RAIL_X1 - 6, WY / 2, RAIL_ZC))      # 外口
rail = rail.cut(box(23, 70, 30, RAIL_X1 - 12 - 11.5, WY / 2, RAIL_ZC))  # 内腔
add('RAIL_T', 'T形槽导轨', rail, '外口 40 / 内腔 70 / 深 30，可抱夹')
for y in YS:
    add(f'SADDLE_{y:.0f}', f'导轨鞍座@{y:.0f}', slab(STEP_X, RAIL_X0 + 20, y - 30, y + 30, RAIL_Z0 - 20, RAIL_Z0), '与立面连接')

# ── 滚轮托座：悬臂梁 + 立板 + 两只错高滚轮（V 形承托）
for y in YS:
    add(f'ARM_SUP_{y:.0f}', f'托座悬臂@{y:.0f}', slab(-30, 210, y - 40, y + 40, WZ_F, WZ_F + 30), '80 宽，悬挑 210')
    add(f'POST_{y:.0f}', f'托座立板@{y:.0f}', slab(180, 210, y - 40, y + 40, WZ_F + 30, WZ_F + 160), '30 厚')
    add(f'ROLLER_{y:.0f}_A', f'托座滚轮A@{y:.0f}',
        cq.Workplane("XZ").circle(30).extrude(60).translate((110, y + 30, WZ_F + 165)), 'Ø60，低')
    add(f'ROLLER_{y:.0f}_B', f'托座滚轮B@{y:.0f}',
        cq.Workplane("XZ").circle(30).extrude(60).translate((185, y + 30, WZ_F + 195)), 'Ø60，高')

asm = cq.Assembly()
for n, s in P:
    asm.add(s, name=n)
step = OUT / '单模块母墙_v4.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()

with open(OUT / '零件清单.csv', 'w', newline='', encoding='utf-8-sig') as f:
    w = csv.DictWriter(f, fieldnames=['name', '中文', '说明'])
    w.writeheader(); w.writerows(BOM)

info = dict(version='v4', parts=len(P),
            bbox={k: [round(getattr(bb, k + 'min'), 1), round(getattr(bb, k + 'max'), 1)] for k in 'xyz'},
            rail=dict(type='T 形槽', x=[RAIL_X0, RAIL_X1], z=[RAIL_Z0, RAIL_Z1],
                      slot=dict(外口=40, 内腔=70, 深=30, 朝向='+X')),
            support=dict(悬臂='x −30…210', 立板='x 180…210', 滚轮='Ø60 两只，错高 30'),
            fix='前顶板铺到 x=0；零件名改 ASCII，中文名见 零件清单.csv',
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / '单模块母墙_v4.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps(info, ensure_ascii=False))
