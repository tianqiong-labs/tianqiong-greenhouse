"""单模块母墙 v1.1（结构级，修正斜撑/吊挂/面板平面）。

坐标：x ∈ [−800, 0] 母墙，y ∈ [0, 3500]，z ∈ [0, 2800]；+X 朝棚内。
修正点：
  1. 斜撑按 (y0,z0)→(y1,z1) 用 atan2 求角，长度=斜边，端点收在横梁内缘；
  2. 斜撑置于立柱前方独立平面 x −710…−670，不与立柱实体重叠；
  3. 面板移到 x −660…−610，避开斜撑；
  4. 吊挂板从面板前缘 −660 伸到导轨后缘 −50，长度 610。
"""
import cadquery as cq, json, math
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX, WZ = 3500.0, 800.0, 2800.0
COL_Y, COL_X = 100.0, 100.0
BEAM_H = 60.0
BRACE = 60.0
BRA_X0, BRA_X1 = -710.0, -670.0
PAN_X0, PAN_X1 = -600.0, -550.0
PANEL_T = PAN_X1 - PAN_X0
RAIL = 50.0
RAIL_Z = (2370.0, 2670.0)
N_BAY = 6
YS = [COL_Y / 2 + (WY - COL_Y) * i / N_BAY for i in range(N_BAY + 1)]
X_BACK = -WX + COL_X / 2          # −750 立柱中心
Z0 = BEAM_H                        # 净跨下缘
Z1 = WZ - BEAM_H                   # 净跨上缘


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


parts = []
# 立柱
for y in YS:
    parts.append((f'COL_{y:.0f}', box(COL_X, COL_Y, WZ, X_BACK, y, WZ / 2)))
# 上下横梁
for z, nm in ((BEAM_H / 2, 'BEAM_B'), (WZ - BEAM_H / 2, 'BEAM_T')):
    parts.append((nm, box(COL_X, WY, BEAM_H, X_BACK, WY / 2, z)))
# 横撑（3 道）+ 角部斜撑（端跨 45°）
GIRT_Z = (700.0, 1400.0, 2100.0)
braces = []
for gz in GIRT_Z:
    parts.append((f'GIRT_{int(gz)}', box(BRA_X1 - BRA_X0, WY - 2 * COL_Y, BRACE,
                                         (BRA_X0 + BRA_X1) / 2, WY / 2, gz)))
KNEE = 450.0
knees = [(YS[0] + COL_Y / 2, Z0, 1, 1), (YS[0] + COL_Y / 2, Z1, 1, -1),
         (YS[-1] - COL_Y / 2, Z0, -1, 1), (YS[-1] - COL_Y / 2, Z1, -1, -1)]
for k, (y0, z0, sy, sz) in enumerate(knees):
    p1 = (y0 + sy * KNEE * math.cos(math.radians(45)), z0 + sz * KNEE * math.sin(math.radians(45)))
    dy, dz = p1[0] - y0, p1[1] - z0
    L = math.hypot(dy, dz); th = math.degrees(math.atan2(dz, dy))
    b = (cq.Workplane("XY").box(BRA_X1 - BRA_X0, L, BRACE)
         .rotate((0, 0, 0), (1, 0, 0), th)
         .translate(((BRA_X0 + BRA_X1) / 2, (y0 + p1[0]) / 2, (z0 + p1[1]) / 2)))
    parts.append((f'KNEE_{k}', b))
    braces.append(dict(k=k, y0=round(y0, 1), z0=z0, deg=round(th, 2), L=round(L, 1)))
# 面板
for i in range(N_BAY):
    y0, y1 = YS[i] + COL_Y / 2, YS[i + 1] - COL_Y / 2
    parts.append((f'PANEL_{i}',
                  box(PANEL_T, y1 - y0, Z1 - Z0, (PAN_X0 + PAN_X1) / 2, (y0 + y1) / 2, (Z0 + Z1) / 2)))
# 底板
parts.append(('FLOOR', box(WX, WY, 4, -WX / 2, WY / 2, 2)))
# 双导轨
for rz in RAIL_Z:
    parts.append((f'RAIL_{int(rz)}', box(RAIL, WY, RAIL, -RAIL / 2, WY / 2, rz)))
# 吊挂板：面板前缘 → 导轨后缘
for y in YS:
    for rz in RAIL_Z:
        L = (-RAIL) - PAN_X1
        parts.append((f'HANG_{y:.0f}_{int(rz)}',
                      box(L, 8, 120, (PAN_X1 + (-RAIL)) / 2, y, rz)))
# 端部限位
for rz in RAIL_Z:
    for y in (60.0, WY - 60.0):
        parts.append((f'STOP_{int(rz)}_{int(y)}', box(RAIL, 60, 120, -RAIL / 2, y, rz)))

asm = cq.Assembly()
for n, s in parts:
    asm.add(s, name=n)
step = OUT / 'module_wall_v1.step'
asm.save(str(step), 'STEP')
bb = asm.toCompound().BoundingBox()
info = dict(version='v1.1', parts=len(parts),
            bbox=dict(x=[bb.xmin, bb.xmax], y=[bb.ymin, bb.ymax], z=[bb.zmin, bb.zmax]),
            braces=braces,
            sections=dict(col=[COL_Y, COL_X], beam_h=BEAM_H, brace=BRACE,
                          brace_x=[BRA_X0, BRA_X1], panel_x=[PAN_X0, PAN_X1],
                          rail=RAIL, rail_z=list(RAIL_Z), bays=N_BAY),
            step_kb=round(step.stat().st_size / 1024, 1))
(OUT / 'module_wall_v1.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
print(json.dumps({k: info[k] for k in ('version', 'parts', 'bbox', 'step_kb')}, ensure_ascii=False))
print('braces:', braces)
