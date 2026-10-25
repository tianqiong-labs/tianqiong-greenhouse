"""P0/P1：锚轨-滑块-末端装配代理 + 补位扫掠。

P0  初始支撑态的接触分类：设计配合（允许接触） vs 非法穿透
P1  六臂补位扫掠：A02/A05 可收臂，A03/A04 必须保持承托姿态（不可收臂）
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'

sp = importlib.util.spec_from_file_location('anim', str(HERE / 'anim.py'))
anim = importlib.util.module_from_spec(sp); sp.loader.exec_module(anim)
sp2 = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp2); sp2.loader.exec_module(pl)
m = anim.m

Y0 = [350.0, 910.0, 1470.0, 2030.0, 2590.0, 3150.0]
YW = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
TIP_R = 67.0

RAIL_NAMES = ['base_web', 'upright_1', 'upright_2', 'upper_retainer_1', 'upper_retainer_2',
              'front_membrane_clamp', 'end_socket_L', 'end_socket_R']
SLIDER_NAMES = ['body', 'roller1', 'roller2', 'positive_latch', 'wrist_upright',
                'upper_offset_beam', 'wrist_datum']


def bbox_of(name, recs):
    r = next(x for x in recs if x['name'] == name)
    md, key = m.unit_mesh(r)
    V = np.array(md['vertices'], float)
    pos, quat, sc = m.transform(r)
    x, y, z, w = quat
    R = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                  [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                  [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    W = (V * np.array(sc)) @ R.T + np.array(pos)
    return [round(float(v), 1) for v in W.min(0)], [round(float(v), 1) for v in W.max(0)]


def sphere_box_gap(c, r, lo, hi):
    p = [min(max(c[i], lo[i]), hi[i]) for i in range(3)]
    return round(math.dist(c, p) - r, 2)


def box_box_gap(a_lo, a_hi, b_lo, b_hi):
    d = [max(b_lo[i] - a_hi[i], a_lo[i] - b_hi[i]) for i in range(3)]
    if all(v <= 0 for v in d):
        return -round(min(abs(v) for v in d), 2)
    return round(sum(max(0.0, v) ** 2 for v in d) ** 0.5, 2)


if __name__ == '__main__':
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
              yaw=0.0, lower=0.0, compensate=1.0, pack=0.0)
    recs = anim.frame_records(st)
    mod = 'M2_M1_'
    boxes = {}
    for n in RAIL_NAMES:
        nm = mod + 'P03_' + n
        lo, hi = bbox_of(nm, recs)
        boxes['rail:' + n] = (lo, hi)
    for n in SLIDER_NAMES:
        nm = mod + 'P03_slider02_' + n
        lo, hi = bbox_of(nm, recs)
        boxes['slider:' + n] = (lo, hi)
    tip = next(x for x in recs if x['name'] == mod + 'A02_tip')
    tip_c = [round(float(v), 1) for v in tip['c']]

    print('=== P0 初始支撑态（A02, 球心 z=0）===')
    print(f'球心 {tip_c}  r={TIP_R}')
    rows = []
    for k, (lo, hi) in boxes.items():
        g = sphere_box_gap(tip_c, TIP_R, lo, hi)
        if k.startswith('slider'):
            wlo, whi = boxes['slider:wrist_datum']
            gg = box_box_gap(lo, hi, wlo, whi) if k != 'slider:wrist_datum' else None
        else:
            gg = None
        rows.append((k, g, gg))
        print(f'  {k:32s} 球头间隙 {g:8.2f}   z[{lo[2]:7.1f},{hi[2]:7.1f}] x[{lo[0]:7.1f},{hi[0]:7.1f}]')

    # 分类
    allowed, illegal = [], []
    for k, g, _ in rows:
        if g < 0:
            (allowed if k == 'slider:wrist_datum' else illegal).append((k, g))
    print('\n允许接触（设计配合，需在碰撞中豁免）:')
    for k, g in allowed:
        print(f'  {k:32s} 重叠 {-g:.2f} mm')
    print('非法穿透:')
    for k, g in illegal:
        print(f'  {k:32s} 穿透 {-g:.2f} mm')
    if not illegal:
        print('  （无）')

    # ---- P1 补位扫掠 ----
    print('\n=== P1 补位扫掠 ===')
    sweep = []
    for i, (a, b) in enumerate(zip(Y0, YW), 1):
        role = '作业（可收臂）' if i in (2, 5) else '支撑（不可收臂）'
        slot_tip = 2 * TIP_R + abs(b - a)
        # 支撑臂：球头在 z0..67 内扫过；作业臂：收臂后 x<=600，不进入压条
        in_zone = role.startswith('支撑') or abs(b - a) == 0
        slot = slot_tip if in_zone else 2 * TIP_R
        sweep.append(dict(arm=f'A{i:02d}', y0=a, yw=b, dy=b - a, role=role,
                          slot_mm=round(slot, 1), in_clamp_zone=in_zone))
        print(f'  A{i:02d} {role:14s} ΔY={b-a:+7.1f}  槽宽 {slot:6.1f} mm  {"扫过压条" if in_zone else "不进入压条"}')
    total = sum(s['slot_mm'] for s in sweep)
    print(f'  合计 {total:.0f} mm / 3500 = {total/3500*100:.1f}%')

    data = dict(tip_center=tip_c, tip_r=TIP_R,
                rail_boxes={k: v for k, v in boxes.items()},
                allowed_contacts=allowed, illegal_penetration=illegal,
                sweep=sweep, slot_total_mm=round(total, 1),
                slot_ratio=round(total / 3500, 3))
    (OUT / 'rail_p0p1.json').write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/rail_p0p1.json')
