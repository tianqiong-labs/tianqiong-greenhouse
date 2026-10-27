"""单模块母墙 v5：台阶式 + 顶部通长闭式矩形槽（滑块 400×120×80）。

方案依据（筛选级计算，V=1.3 kN、M=1.4 kN·m）：
  合力偏心 e=M/V=1077 mm，远超开式支承的 Lb/6=67 mm → 必须闭式槽带上唇；
  块长 400 时力偶 ±3.5 kN、端部承压 0.97 MPa、上拔 2.85 kN → 普通 Q235 即可，无需淬火。
零件名一律中文；STEP 内名称可能显示为空，另有《零件清单.csv》。
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

B0, B1 = -WX, -WX + SKIN
RB0, RB1 = B1, STEP_X
R0, R1 = STEP_X, STEP_X + SKIN
F0, F1 = R1, -SKIN
S0, S1 = -SKIN, 0.0

# ── 顶部闭式槽（通长）
CH_X0, CH_X1 = -760.0, -350.0     # 410 宽
CH_WALL = 105.0                    # 两侧壁宽 → 槽口 200
CH_Z0 = WZ_B                       # 3000 槽底
CH_FLOOR = 30.0                    # 底板厚
CH_CAV_Z0 = CH_Z0 + CH_FLOOR       # 3030
CH_CAV_Z1 = CH_CAV_Z0 + 90.0       # 3120 内腔顶
CH_LIP = 10.0                      # 上唇厚
CH_Z1 = CH_CAV_Z1 + CH_LIP         # 3130 槽顶

SL_X0, SL_X1 = CH_X0 + 5, CH_X1 - 5   # 400
SL_Z0, SL_Z1 = CH_CAV_Z0 + 5, CH_CAV_Z1 - 5  # 80
SL_Y = 120.0
SL_Y_POS = 1750.0

# ── 下部地轨（示意，位置待定）
GR_X0, GR_X1 = 4000.0, 4120.0
GR_Z0, GR_Z1 = 0.0, 40.0
GR_RAIL_Z = 40.0

BOM, P = [], []


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


def slab(x0, x1, y0, y1, z0, z1):
    return box(x1 - x0, y1 - y0, z1 - z0, (x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2)


def add(name, cn, shape, note=''):
    P.append((name, shape)); BOM.append(dict(零件=name, 中文名=cn, 说明=note))


# 壳体
add('底板', '底板', slab(-WX, 0, 0, WY, 0, FLOOR_T), '100 厚夹芯')
add('后板', '后板', slab(B0, B1, 0, WY, FLOOR_T, WZ_B), '30 厚')
add('后顶板', '后顶板', slab(RB0, RB1, 0, WY, WZ_B - SKIN, WZ_B), '30 厚')
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    add(f'后端板{i+1}', f'后端板{i+1}', slab(RB0, RB1, y0, y1, FLOOR_T, WZ_B - SKIN), '30 厚')
add('台阶立面', '台阶立面', slab(R0, R1, 0, WY, WZ_F, WZ_B), '导轨/附件安装面')
add('前顶板', '前顶板', slab(F0, 0, 0, WY, WZ_F - SKIN, WZ_F), '30 厚')
for i, (y0, y1) in enumerate(((0, SKIN), (WY - SKIN, WY))):
    add(f'前端板{i+1}', f'前端板{i+1}', slab(F0, F1, y0, y1, FLOOR_T, WZ_F - SKIN), '30 厚')
add('前板', '前板', slab(S0, S1, 0, WY, FLOOR_T, WZ_F - SKIN), '30 厚')

# 骨架
for y in YS:
    h = WZ_B - FLOOR_T
    add(f'后立柱@{y:.0f}', f'后立柱@{y:.0f}', box(COL_X, COL_Y, h, X_BACK, y, FLOOR_T + h / 2), '100×100')
for y in YS:
    h = WZ_F - SKIN - FLOOR_T
    add(f'前立柱@{y:.0f}', f'前立柱@{y:.0f}', box(COL_X, COL_Y, h, X_FRONT, y, FLOOR_T + h / 2), '100×100')
add('后上横梁', '后上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, WZ_B - SKIN - BEAM_H / 2))
add('前上横梁', '前上横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_FRONT, WY / 2, WZ_F - SKIN - BEAM_H / 2))
add('下横梁', '下横梁', box(COL_X, WY - 2 * SKIN, BEAM_H, X_BACK, WY / 2, FLOOR_T + BEAM_H / 2))
for gz in (900.0, 1600.0, 2300.0):
    add(f'后横撑@{int(gz)}', f'后横撑@{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_BACK, WY / 2, gz))
for gz in (900.0, 1700.0):
    add(f'前横撑@{int(gz)}', f'前横撑@{int(gz)}', box(COL_X, WY - 2 * SKIN, GIRT, X_FRONT, WY / 2, gz))

# 顶部闭式矩形槽（通长 3500）
add('槽底板', '槽底板', slab(CH_X0, CH_X1, 0, WY, CH_Z0, CH_CAV_Z0), '30 厚')
add('槽左壁', '槽左壁', slab(CH_X0, CH_X0 + CH_WALL, 0, WY, CH_CAV_Z0, CH_CAV_Z1), '105 宽')
add('槽右壁', '槽右壁', slab(CH_X1 - CH_WALL, CH_X1, 0, WY, CH_CAV_Z0, CH_CAV_Z1), '105 宽')
add('槽左唇', '槽左唇', slab(CH_X0, CH_X0 + CH_WALL, 0, WY, CH_CAV_Z1, CH_Z1), '10 厚，承上拔')
add('槽右唇', '槽右唇', slab(CH_X1 - CH_WALL, CH_X1, 0, WY, CH_CAV_Z1, CH_Z1), '10 厚，承上拔')
for i, (y0, y1) in enumerate(((0, 30.0), (WY - 30.0, WY))):
    add(f'槽端封板{i+1}', f'槽端封板{i+1}', slab(CH_X0, CH_X1, y0, y1, CH_CAV_Z0, CH_CAV_Z1), '行程限位')

# 滑块（示例 1 件）
add('滑块', '滑块', slab(SL_X0, SL_X1, SL_Y_POS - SL_Y / 2, SL_Y_POS + SL_Y / 2, SL_Z0, SL_Z1),
    '400×120×80，单边间隙 5')

# 下部地轨（示意）
add('地轨底板', '地轨底板', slab(GR_X0, GR_X1, 0, WY, GR_Z0, GR_Z1), '示意，位置待定')
add('地轨', '地轨', slab(GR_X0 + 10, GR_X1 - 10, 0, WY, GR_Z1, GR_Z1 + 80), '示意，位置待定')

asm = cq.Assembly()
for n, s in P:
    asm.add(s, name=n)
step = OUT / '单模块母墙_v5.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()

with open(OUT / '零件清单.csv', 'w', newline='', encoding='utf-8-sig') as f:
    w = csv.DictWriter(f, fieldnames=['零件', '中文名', '说明']); w.writeheader(); w.writerows(BOM)

info = dict(版本='v5', 件数=len(P),
            bbox={k: [round(getattr(bb, k + 'min'), 1), round(getattr(bb, k + 'max'), 1)] for k in 'xyz'},
            顶部槽=dict(内腔=[CH_X1 - CH_X0, CH_CAV_Z1 - CH_CAV_Z0], 槽口=CH_X1 - CH_X0 - 2 * CH_WALL,
                    唇厚=CH_LIP, 通长=WY),
            滑块=dict(尺寸=[SL_X1 - SL_X0, SL_Y, SL_Z1 - SL_Z0], 位置Y=SL_Y_POS, 单边间隙=5),
            计算依据=dict(V_kN=1.30, M_kNm=1.40, 力偶_kN=3.50, 端部承压_MPa=0.97),
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / '单模块母墙_v5.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps(info, ensure_ascii=False))
