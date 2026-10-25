"""接口组 I · P1 几何闭合检查（修正三项）

1. r=67 上半球 + Ø32 竖直盲孔：逐高度剩余壁厚、孔口是否穿顶、孔底厚度、轴向接触面积
2. 锁止：Ø8 销 vs 槽宽的干涉；给出槽宽/槽深/啮合与剪切面积
3. 槽表：A02/A05 的两个部署位（补位前/补位后）都进入固定压条 → 228.6 mm
"""
import json, math
from pathlib import Path

R = 67.0
BORE_D = 32.0
BORE_R = BORE_D / 2
COLUMN_D = 30.0
GROOVE_D = 22.0
GROOVE_W = 12.0
CLAW_W = 10.0
CLAW_ENGAGE = 4.0
CLAW_TRAVEL = 8.0
CROWN_BOSS_D = 48.0
CROWN_BOSS_H = 18.0
BORE_FLOOR = 14.0

CLAMP_LEN = 3500.0
CLAMP_Z = (13.5, 34.5)
HW = 65.6            # 球头在压条 z 带内的 Y 半宽（实测 z=13.5 处）
MARGIN = 4.0
Y0 = [350.0, 910.0, 1470.0, 2030.0, 2590.0, 3150.0]
YW = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]


def wall_profile():
    """上半球 z 0..67，r_s(z)=sqrt(R^2-z^2)；孔半径 BORE_R。"""
    rows = []
    for z in (0, 10, 20, 30, 40, 50, 55, 60, 62, 65, 66, 67):
        rs = math.sqrt(max(R * R - z * z, 0))
        rows.append(dict(z=z, sphere_r=round(rs, 1), wall=round(rs - BORE_R, 1),
                         ok=rs > BORE_R))
    z_break = math.sqrt(max(R * R - BORE_R * BORE_R, 0))
    return rows, round(z_break, 2)


def latch_check():
    """原方案 Ø8 销 / 6 mm 槽 / 3 mm 啮合 的干涉；给出修正值。"""
    old = dict(pin_d=8.0, groove_w=6.0, groove_depth=(28 - 24) / 2, engage=3.0)
    old['pin_exceeds_groove_by'] = old['pin_d'] - old['groove_w']
    old['engage_exceeds_depth_by'] = old['engage'] - old['groove_depth']
    new = dict(column_d=COLUMN_D, groove_d=GROOVE_D,
               groove_depth=(COLUMN_D - GROOVE_D) / 2, groove_w=GROOVE_W,
               claw_w=CLAW_W, claw_engage=CLAW_ENGAGE, travel=CLAW_TRAVEL,
               shear_area_per_claw=CLAW_W * CLAW_ENGAGE,
               shear_area_two=2 * CLAW_W * CLAW_ENGAGE,
               axial_contact_area=round(math.pi / 4 * (BORE_D ** 2 - GROOVE_D ** 2), 0))
    return old, new


def slot_table():
    rows = []
    for i, (a, b) in enumerate(zip(Y0, YW), 1):
        support = i not in (2, 5)
        # 两个部署位都必须进入固定压条（补位前、补位后）；横移途中若收臂则不增加
        span = abs(b - a) if support else abs(b - a)
        width = 2 * HW + span + MARGIN
        rows.append(dict(arm=f'A{i:02d}', role='支撑' if support else '作业',
                         y_before=a, y_after=b,
                         occupied=[round(a - HW, 1), round(a + HW, 1)],
                         occupied_after=[round(b - HW, 1), round(b + HW, 1)],
                         slot_mm=round(width, 1),
                         note='两部署位占用包络' if not support else '两部署位 + 横移扫掠'))
    return rows


def layout(rows):
    segs = []
    for r in rows:
        lo = min(r['y_before'], r['y_after']) - HW
        hi = max(r['y_before'], r['y_after']) + HW
        segs.append((round(lo, 1), round(hi, 1)))
    segs.sort()
    webs, prev = [], 0.0
    for lo, hi in segs:
        webs.append((round(prev, 1), round(lo, 1), round(lo - prev, 1)))
        prev = hi
    webs.append((round(prev, 1), CLAMP_LEN, round(CLAMP_LEN - prev, 1)))
    return segs, webs


if __name__ == '__main__':
    print('=== 1. 半球开孔壁厚 ===')
    rows, zb = wall_profile()
    for r in rows:
        print(f"  z={r['z']:3.0f}  球半径 {r['sphere_r']:5.1f}  剩余壁厚 {r['wall']:6.1f}  {'OK' if r['ok'] else '穿顶'}")
    print(f"  孔口在 z={zb} 处穿出球面 → 顶部 67-{zb} = {67-zb:.2f} mm 无材料")
    print(f"  孔深与球高相同(67) → 孔底剩余厚度 0 mm")
    print(f"  修正：顶部一体凸台 Ø{CROWN_BOSS_D}×{CROWN_BOSS_H}（z 58…76），孔口上移至 z=76")
    print(f"        孔底加厚 {BORE_FLOOR} mm（孔底 z={BORE_FLOOR}），有效插入深度 {76-BORE_FLOOR:.0f} mm")

    print('\n=== 2. 锁止几何 ===')
    old, new = latch_check()
    print(f"  原方案：销 Ø{old['pin_d']} / 槽宽 {old['groove_w']} → 销比槽宽 {old['pin_exceeds_groove_by']:.0f} mm（干涉）")
    print(f"           槽深仅 {old['groove_depth']:.1f} mm，啮合 {old['engage']} → 超出 {old['engage_exceeds_depth_by']:.1f} mm")
    print(f"  修正：柱 Ø{new['column_d']}，槽 Ø{new['groove_d']}（深 {new['groove_depth']:.0f}），槽宽 {new['groove_w']}，"
          f"爪宽 {new['claw_w']}，啮合 {new['claw_engage']}，行程 {new['travel']}")
    print(f"        两爪剪切面积 {new['shear_area_two']:.0f} mm²；轴向接触环面积 {new['axial_contact_area']:.0f} mm²")

    print('\n=== 3. 槽表（A02/A05 计两个部署位）===')
    rows3 = slot_table()
    for r in rows3:
        print(f"  {r['arm']} {r['role']} 补位前 {r['y_before']:7.1f} 补位后 {r['y_after']:7.1f} "
              f"槽宽 {r['slot_mm']:6.1f}  {r['note']}")
    total = sum(r['slot_mm'] for r in rows3)
    print(f"  合计 {total:.1f} mm = {total/CLAMP_LEN*100:.1f}%")

    segs, webs = layout(rows3)
    print('\n=== 4. Y 向开槽区间（模块局部 0…3500）===')
    print('  开槽:', segs)
    print('  保留材料段 (起, 止, 长度):')
    for w in webs:
        print(f'    {w[0]:7.1f} … {w[1]:7.1f}   长 {w[2]:6.1f}')
    print(f"  最长保留段 {max(w[2] for w in webs):.1f} mm；单个最大开槽 {max(s[1]-s[0] for s in segs):.1f} mm")

    out = dict(wall=rows, bore_breakout_z=zb, crown_boss=dict(d=CROWN_BOSS_D, h=CROWN_BOSS_H),
               bore_floor=BORE_FLOOR, latch_old=old, latch_new=new,
               slots=rows3, slot_total_mm=round(total, 1), ratio=round(total / CLAMP_LEN, 4),
               slot_spans=segs, webs=webs)
    Path(__file__).resolve().parent.joinpath('out', 'iface_geom_check.json').write_text(
        json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/iface_geom_check.json')
