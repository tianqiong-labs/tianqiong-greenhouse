"""单模块母墙 v1（结构级）：骨架 + 面板 + 双导轨 + 吊挂。

坐标：x ∈ [−800, 0] 母墙，y ∈ [0, 3500]，z ∈ [0, 2800]；+X 朝棚内。
构件全部按真实截面建模（方管以实心近似，壁厚在参数里记录）。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'module'
OUT.mkdir(parents=True, exist_ok=True)

WY, WX, WZ = 3500.0, 800.0, 2800.0
COL = (100.0, 100.0)          # 立柱截面 y×x
BEAM_H = 60.0                 # 横梁高
BRACE = 60.0
PANEL_T = 50.0
RAIL = 50.0
RAIL_Z = (2370.0, 2670.0)
N_BAY = 6
YS = [WY * i / N_BAY for i in range(N_BAY + 1)]
X_BACK = -WX + COL[1] / 2     # 立柱中心（后立面内侧）
X_FRONT = 0.0


def box(lx, ly, lz, cx, cy, cz):
    return cq.Workplane("XY").box(lx, ly, lz).translate((cx, cy, cz))


parts = []
# 1 立柱
for y in YS:
    parts.append((f'COL_{y:.0f}', box(COL[1], COL[0], WZ, X_BACK, y, WZ / 2)))
# 2 上下横梁（贴后立面，通长）
for z, nm in ((BEAM_H / 2, 'BEAM_B'), (WZ - BEAM_H / 2, 'BEAM_T')):
    parts.append((nm, box(COL[1], WY, BEAM_H, X_BACK, WY / 2, z)))
# 3 斜撑（每跨一撇，交替）
for i in range(N_BAY):
    y0, y1 = YS[i], YS[i + 1]
    L = ((y1 - y0) ** 2 + (WZ - 2 * BEAM_H) ** 2) ** 0.5
    ang = 57.0 if i % 2 == 0 else -57.0
    b = (cq.Workplane("XY").box(BRACE, L, BRACE)
         .rotate((0, 0, 0), (1, 0, 0), 90 - abs(ang))  # 竖直平面内倾斜
         .translate((X_BACK, (y0 + y1) / 2, WZ / 2)))
    parts.append((f'BRACE_{i}', b))
# 4 面板（后侧，50 厚，分格随立柱）
for i in range(N_BAY):
    y0, y1 = YS[i], YS[i + 1]
    parts.append((f'PANEL_{i}',
                  box(PANEL_T, (y1 - y0) - COL[0], WZ - 2 * BEAM_H,
                      X_BACK + COL[1] / 2 + PANEL_T / 2, (y0 + y1) / 2, WZ / 2)))
# 5 底板
parts.append(('FLOOR', box(WX, WY, 4, -WX / 2, WY / 2, 2)))
# 6 双导轨（前立面，+X 侧）
for rz in RAIL_Z:
    parts.append((f'RAIL_{int(rz)}', box(RAIL, WY, RAIL, -RAIL / 2, WY / 2, rz)))
# 7 吊挂板：每根立柱一处，从后骨架伸到导轨
for y in YS:
    for rz in RAIL_Z:
        parts.append((f'HANG_{y:.0f}_{int(rz)}',
                      box(abs(X_BACK) + RAIL / 2, 8, 120, (X_BACK - RAIL / 2) / 2 + 0 * 0,
                          y, rz)))
# 8 导轨端部限位
for rz in RAIL_Z:
    for y in (60.0, WY - 60.0):
        parts.append((f'STOP_{int(rz)}_{int(y)}', box(RAIL, 60, 120, -RAIL / 2, y, rz)))

asm = cq.Assembly()
for n, s in parts:
    asm.add(s, name=n)
step = OUT / 'module_wall_v1.step'
asm.save(str(step), 'STEP')

bb = asm.toCompound().BoundingBox()
info = dict(parts=len(parts), bbox=dict(x=[bb.xmin, bb.xmax], y=[bb.ymin, bb.ymax],
                                        z=[bb.zmin, bb.zmax]),
            size_mm=dict(x=WX, y=WY, z=WZ), step=str(step),
            step_kb=round(step.stat().st_size / 1024, 1),
            sections=dict(col=list(COL), beam_h=BEAM_H, brace=BRACE, panel_t=PANEL_T,
                          rail=RAIL, rail_z=list(RAIL_Z), bays=N_BAY))
print(json.dumps(info, ensure_ascii=False, indent=1))

# 正视图：真实 HLR 投影，沿 −X 看
try:
    from cadquery.occ_impl.exporters.svg import getSVG
    svg = getSVG(asm.toCompound(), dict(projectionDir=(1, 0, 0), width=1400, height=1100,
                                        showAxes=False, showHidden=False,
                                        strokeWidth=0.6, marginLeft=20, marginTop=20))
    (OUT / 'module_wall_v1_front.svg').write_text(svg, encoding='utf8')
    print('SVG ok', len(svg))
except Exception as e:
    print('SVG fail', e)

(OUT / 'module_wall_v1.json').write_text(json.dumps(info, ensure_ascii=False, indent=1), encoding='utf8')
