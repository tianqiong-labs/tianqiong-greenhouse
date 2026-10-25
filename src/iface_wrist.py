"""接口组 I · 第一阶段：球头—腕柱配合实体化（有间隙的孔柱配合 + 锁止）

原状态：wrist_upright (25×25×194, x 4487.5..4512.5, z 0..194) 整段埋进球头，
        球头 r=67 把柱端完全包容（−67 mm），零间隙，不可装配。

本模块定义的候选配合：
    腕柱       Ø28，z 0…194（保留全长，不截短）
    球头内孔   Ø30，深 67（z 0…67），1.0 mm 单边径向间隙
    孔口倒角   3×45°，导向
    轴向限位   柱端面 z=0 坐于孔底 z=0（面接触）
    锁止槽     柱上 Ø24 环槽，z 38…44
    锁止销     Ø8，径向，行程 6 mm，插入 3 mm 进入环槽
    解锁       销回退 6 mm → 球头下移 67 mm 脱开
"""
import json, math
from pathlib import Path

BALL_R = 67.0
COLUMN_D = 28.0
BORE_D = 30.0
BORE_DEPTH = 67.0
CHAMFER = 3.0
GROOVE_D = 24.0
GROOVE_Z = (38.0, 44.0)
PIN_D = 8.0
PIN_TRAVEL = 6.0
PIN_ENGAGE = 3.0

CLAMP = dict(x=(4473.0, 4497.0), z=(13.5, 34.5))
CLAMP_LEN = 3500.0
MOD_Y0 = 4250.0
Y0 = [350.0, 910.0, 1470.0, 2030.0, 2590.0, 3150.0]
YW = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
SLOT_MARGIN = 4.0          # 槽边到扫掠包络的余量（单侧合计）


def radial_clearance():
    return (BORE_D - COLUMN_D) / 2.0


def half_width_at(z, part='ball'):
    """该件在高度 z 处的 Y 半宽（球/柱同轴于 x=4500）。"""
    if part == 'ball':
        if abs(z) > BALL_R:
            return 0.0
        return math.sqrt(max(BALL_R ** 2 - z ** 2, 0.0))
    if part == 'column':
        return COLUMN_D / 2.0
    return 0.0


def band_half_width(z0, z1):
    """扫掠件在压条 z 带内的最大 Y 半宽（包络，按最宽处取）。"""
    best = 0.0
    for z in (z0, (z0 + z1) / 2, z1):
        best = max(best, half_width_at(z, 'ball'), half_width_at(z, 'column'))
    return best


def slot_table():
    z0, z1 = CLAMP['z']
    hw = band_half_width(z0, z1)
    rows = []
    for i, (a, b) in enumerate(zip(Y0, YW), 1):
        role = '作业（可收臂）' if i in (2, 5) else '支撑（不可收臂）'
        # 支撑臂：球头随滑块扫 |ΔY|；作业臂收臂后 x≤601，只有静止占用
        sweep = abs(b - a) if role.startswith('支撑') else 0.0
        width = 2 * hw * (1 if sweep == 0 else 1) + sweep
        # 作业臂静止占用仍需球头宽度；扫掠量为 0
        width += SLOT_MARGIN
        rows.append(dict(arm=f'A{i:02d}', role=role, dy=round(b - a, 1),
                         sweep_mm=round(sweep, 1), slot_mm=round(width, 1)))
    return rows, hw


def initial_state():
    """改造后的配合间隙：孔柱为设计配合，压条为非法穿透（由槽解决）。"""
    return {
        'ball_bore_vs_column': round(radial_clearance(), 2),      # >0 表示有间隙
        'bore_depth_vs_column_insert': BORE_DEPTH,
        'ball_vs_clamp_mm': -53.17,      # 来自 rail_p0p1.py 实测
        'column_vs_clamp_mm': -9.5,      # x 向重叠
        'note': '孔柱为设计配合（正=间隙）；压条两处仍为穿透，需开槽',
    }


def after_slot_verdict(rows):
    """开槽后：六个臂位在压条 x-z 带内不再有实体，两处穿透归零。"""
    total = sum(r['slot_mm'] for r in rows)
    return dict(slot_total_mm=round(total, 1), ratio=round(total / CLAMP_LEN, 4),
                remaining_web_mm=round(CLAMP_LEN - total, 1))


if __name__ == '__main__':
    rows, hw = slot_table()
    print(f'球头/腕柱在压条 z 带内的最大 Y 半宽 {hw:.1f} mm（球 @z=13.5）')
    print(f'孔-柱单边径向间隙 {radial_clearance():.1f} mm')
    print('\n扫掠包络 → 槽宽')
    for r in rows:
        print(f"  {r['arm']} {r['role']:14s} ΔY={r['dy']:+7.1f} 扫掠 {r['sweep_mm']:6.1f} 槽宽 {r['slot_mm']:6.1f}")
    v = after_slot_verdict(rows)
    print(f"\n合计 {v['slot_total_mm']:.1f} mm = {v['ratio']*100:.1f}%，剩余压条 {v['remaining_web_mm']:.1f} mm")
    # 槽间剩余材料
    edges = []
    for r in rows:
        c = YW[int(r['arm'][1:]) - 1]        # 模块局部坐标（0..3500）
        w = r['slot_mm']
        edges.append((c - w / 2, c + w / 2))
    edges.sort()
    webs, prev = [], 0.0
    for lo, hi in edges:
        webs.append(round(lo - prev, 1)); prev = hi
    webs.append(round(CLAMP_LEN - prev, 1))
    print('剩余材料段:', webs, ' 最长无支撑', max(webs), 'mm')
    out = dict(params=dict(ball_r=BALL_R, column_d=COLUMN_D, bore_d=BORE_D,
                           bore_depth=BORE_DEPTH, chamfer=CHAMFER, groove_d=GROOVE_D,
                           groove_z=GROOVE_Z, pin_d=PIN_D, pin_travel=PIN_TRAVEL,
                           pin_engage=PIN_ENGAGE, margin=SLOT_MARGIN),
               initial=initial_state(), slots=rows, summary=v, webs=webs)
    Path(__file__).resolve().parent.joinpath('out', 'iface_wrist_p1.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/iface_wrist_p1.json')
