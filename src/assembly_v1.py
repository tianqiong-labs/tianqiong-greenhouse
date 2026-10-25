"""整机 CAD 第一轮集成：统一装配基准 + 第一版完整总装。

坐标规范（本文件即权威定义）：
  原点  = 模块 0 母墙后立面、左端、地面交点（世界 0,0,0）
  +X    = 棚内（由母墙指向前沿锚轨）
  +Y    = 沿墙方向（模块节距 4250）
  +Z    = 竖直向上
  模块 k 的局部坐标 → 世界：T_k = Translate(0, k*4250, 0)

几何来源：src/anim.py 的 deployed 状态（compensate=1），共 1212 构件。
不使用旧 deployed.step 作为建模母体，仅用于对照。
"""
import importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
sp = importlib.util.spec_from_file_location('anim', str(HERE / 'anim.py'))
anim = importlib.util.module_from_spec(sp); sp.loader.exec_module(anim)
sp2 = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp2); sp2.loader.exec_module(pl)
m = anim.m

PITCH = 4250.0
MODULES = 3


def deployed_records():
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
              yaw=0.0, lower=0.0, compensate=1.0, pack=0.0)
    return anim.frame_records(st)


def bbox(recs):
    lo = np.full(3, 1e18); hi = np.full(3, -1e18)
    for r in recs:
        if r['kind'] in ('box',):
            c = np.array(r['c'], float); s = np.array(r['size'], float) / 2
            lo = np.minimum(lo, c - s); hi = np.maximum(hi, c + s)
        elif r['kind'] == 'cylinder':
            a = np.array(r['p'], float); b = np.array(r['q'], float); rad = float(r.get('radius', r.get('r', 0)))
            lo = np.minimum(lo, np.minimum(a, b) - rad); hi = np.maximum(hi, np.maximum(a, b) + rad)
        elif r['kind'] == 'hemisphere':
            c = np.array(r['c'], float); rad = float(r.get('radius', r.get('r', 0)))
            lo = np.minimum(lo, c - rad); hi = np.maximum(hi, c + rad)
        elif r['kind'] == 'mesh':
            lo = np.minimum(lo, np.array(r['c'], float)); hi = np.maximum(hi, np.array(r['c'], float))
    return [round(float(v), 1) for v in lo], [round(float(v), 1) for v in hi]


if __name__ == '__main__':
    recs = deployed_records()
    print(f'构件总数 {len(recs)}')
    groups = {}
    for r in recs:
        groups[r['group']] = groups.get(r['group'], 0) + 1
    print('按组:', dict(sorted(groups.items(), key=lambda x: -x[1])))

    lo, hi = bbox(recs)
    print(f'总装包围盒 x {lo[0]}…{hi[0]}  y {lo[1]}…{hi[1]}  z {lo[2]}…{hi[2]}')

    # 六个末端球心（CAD 端）
    cad_tips = {}
    for k in range(1, MODULES + 1):
        for i in range(1, 7):
            nm = f'M{k}_M1_A{i:02d}_tip'
            r = next((x for x in recs if x['name'] == nm), None)
            if r:
                cad_tips[nm] = [round(float(v), 2) for v in r['c']]

    # 独立 FK 端（planner，按部署角度）
    q_dep = [-1.6, -20.1, -38.6, -57.2]
    fk = {}
    for i in range(1, 7):
        pts = pl.chain3d(q_dep, 1600.0)
        fk[f'A{i:02d}'] = [round(float(v), 2) for v in pts[-1]]

    print('\n六臂末端：CAD vs FK（部署姿态）')
    worst = 0.0
    rows = []
    for k in range(1, MODULES + 1):
        for i in range(1, 7):
            nm = f'M{k}_M1_A{i:02d}_tip'
            if nm not in cad_tips:
                continue
            c = cad_tips[nm]
            f = fk[f'A{i:02d}']
            d = ((c[0] - f[0]) ** 2 + (c[2] - f[2]) ** 2) ** 0.5
            worst = max(worst, d)
            rows.append(dict(part=nm, cad=c, fk_zx=[f[0], f[2]], d_xy=round(d, 3)))
    print(f'  x/z 平面最大偏差 {worst:.3f} mm（y 由模块偏置决定，单独比对）')

    # γ-2 实体链检查
    chain = dict(tip_center_z=0.0, boss_top_z=76.0, rail_bottom_z=82.0,
                 rail_top_z=88.0, bracket_bottom_z=88.0, bracket_top_z=90.0,
                 clamp_bottom_z=90.0, clamp_top_z=110.0)
    chain['gap_boss_to_rail'] = round(chain['rail_bottom_z'] - chain['boss_top_z'], 1)
    print(f"\nγ-2 实体链：凸台顶 {chain['boss_top_z']} → 导轨底 {chain['rail_bottom_z']} "
          f"缺 {chain['gap_boss_to_rail']} mm 实体")

    # 导出总装 STEP
    out_step = OUT / 'assembly_v1.step'
    try:
        m.export_step(recs, 'assembly_v1')          # 导出器固定写到 cad/
        src_step = HERE.parent / 'cad' / 'assembly_v1.step'
        if src_step.exists():
            import shutil; shutil.move(str(src_step), str(out_step))
        print(f'\nSTEP 已导出 {out_step.name}  {out_step.stat().st_size/1048576:.1f} MB')
    except Exception as e:
        print('STEP 导出失败:', type(e).__name__, e)

    (OUT / 'assembly_v1_report.json').write_text(json.dumps(dict(
        n_parts=len(recs), groups=groups, bbox=[lo, hi],
        tip_check=dict(max_xz_dev_mm=round(worst, 3), rows=rows),
        gamma2_chain=chain,
        datum=dict(origin='模块0母墙后立面/左端/地面', x='棚内', y='沿墙', z='上',
                   pitch=PITCH, modules=MODULES)), ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/assembly_v1_report.json')
