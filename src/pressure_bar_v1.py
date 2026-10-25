"""前膜压条避让 + 连续膜边夹持 候选（P0/P1/P2 初版）。

P0 槽包络：压条 z 带 13.5…34.5 内只有球头（r67）进入；法兰 76…82、卡箍 90…110、
          导轨 82…88、支架 88…90 均在带之上 -> 不参与槽宽。
P1 分段压条：在 3500 长压条上开 6 槽，保留 7 段材料。
P2 连续夹持：内侧连续背压轨 x 4390…4400（球头最远 4437，让开 37 mm）+ 柔性唇边接到压条。
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

MOD_LEN = 3500.0
BAR_X = (4473.0, 4497.0)
BAR_Z = (13.5, 34.5)
BALL_R = 67.0
MARGIN = 4.0
Y_NOM = [350.0, 910.0, 1470.0, 2030.0, 2590.0, 3150.0]
Y_WORK = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
BACK_X = (4390.0, 4400.0)


def slot_table():
    rows = []
    for i in range(6):
        support = i not in (1, 4)
        lo = min(Y_NOM[i], Y_WORK[i]) - BALL_R
        hi = max(Y_NOM[i], Y_WORK[i]) + BALL_R
        rows.append(dict(arm=f'A{i+1:02d}', role='支撑' if support else '作业',
                         y_lo=round(lo, 1), y_hi=round(hi, 1),
                         w_env=round(hi - lo, 1), w_cut=round(hi - lo + MARGIN, 1),
                         y_nom=Y_NOM[i], y_work=Y_WORK[i]))
    return rows


def bar_with_slots(rows):
    bar = cq.Workplane("XY").box(BAR_X[1] - BAR_X[0], MOD_LEN,
                                 BAR_Z[1] - BAR_Z[0]).translate(
        ((BAR_X[0] + BAR_X[1]) / 2, MOD_LEN / 2, (BAR_Z[0] + BAR_Z[1]) / 2))
    for r in rows:
        c = (r['y_lo'] + r['y_hi']) / 2
        w = r['w_cut']
        cut = cq.Workplane("XY").box(BAR_X[1] - BAR_X[0] + 4, w,
                                     BAR_Z[1] - BAR_Z[0] + 2).translate(
            ((BAR_X[0] + BAR_X[1]) / 2, c, (BAR_Z[0] + BAR_Z[1]) / 2))
        bar = bar.cut(cut)
    return bar


def backing_rail():
    return cq.Workplane("XY").box(BACK_X[1] - BACK_X[0], MOD_LEN, 8).translate(
        ((BACK_X[0] + BACK_X[1]) / 2, MOD_LEN / 2, 26))


def lip():
    """柔性唇边：从背压轨伸到压条，覆盖槽口，非承载件。"""
    return cq.Workplane("XY").box(BAR_X[0] - BACK_X[1], MOD_LEN, 1.2).translate(
        ((BACK_X[1] + BAR_X[0]) / 2, MOD_LEN / 2, 30))


if __name__ == '__main__':
    rows = slot_table()
    print('=== P0 六槽包络（模块局部坐标）===')
    for r in rows:
        print(f"  {r['arm']} {r['role']}  {r['y_lo']:7.1f}…{r['y_hi']:7.1f}  "
              f"包络 {r['w_env']:6.1f}  开槽 {r['w_cut']:6.1f}")
    total = sum(r['w_cut'] for r in rows)
    print(f'  合计 {total:.1f} mm = {total/MOD_LEN*100:.1f}%（旧基准 1371.2 / 39.2%）')

    bar = bar_with_slots(rows)
    print(f'  分段压条体积 {bar.val().Volume():.0f} mm3，有效 {bar.val().isValid()}')
    segs = sorted(rows, key=lambda r: r['y_lo'])
    prev = 0.0
    webs = []
    for r in segs:
        lo = (r['y_lo'] + r['y_hi']) / 2 - r['w_cut'] / 2
        webs.append(round(lo - prev, 1)); prev = (r['y_lo'] + r['y_hi']) / 2 + r['w_cut'] / 2
    webs.append(round(MOD_LEN - prev, 1))
    print(f'  保留段 {webs}；最长 {max(webs)} mm')

    br, lp = backing_rail(), lip()
    print(f'  背压轨 vs 球头（球头最远 x=4567）: 间隙 {BAR_X[0]-BALL_R-0:.0f} 不成立，改用 {BACK_X} -> 与球头间隙 {4437-BACK_X[1]:.0f} mm')
    print(f'  唇边体积 {lp.val().Volume():.0f} mm3（薄板 1.2 mm，非承载）')

    asm = cq.Assembly(name='PRESSURE_BAR_V1')
    asm.add(bar, name='BAR_SEGMENTED', color=cq.Color(0.6, 0.6, 0.65))
    asm.add(br, name='BACKING_RAIL', color=cq.Color(0.4, 0.6, 0.4))
    asm.add(lp, name='FLEX_LIP', color=cq.Color(0.9, 0.6, 0.3))
    asm.save(str(OUT / 'pressure_bar_v1.step'))
    (OUT / 'pressure_bar_v1.json').write_text(json.dumps(dict(
        slots=rows, total_cut=round(total, 1), ratio=round(total / MOD_LEN, 4),
        webs=webs, backing_x=BACK_X, lip_thickness=1.2,
        note='压条 z 带内仅球头参与；法兰/卡箍/导轨/支架在带之上'),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('导出 pressure_bar_v1.step / pressure_bar_v1.json')
