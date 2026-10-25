"""装配不变量检查（独立于 bind_heads 的绑定逻辑）。

断言：
  A1 六臂根座 Y == Y_WORK（补位后）
  A2 每个附属件与所属臂平面的 Y 偏差 <= 阈值（平面臂：yaw=0，所有安装件应在同一 y 平面内）
  A3 前端滑块的 Y 与对应根座一致
不依赖 bind_heads 的 local offset，因此可以抓住"绑定自洽但装配错位"。
"""
import importlib.util, json, sys
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
anim = importlib.import_module('anim')
pl = importlib.import_module('planner')
HERO = anim.HERO
Y_WORK_LOCAL = [float(x) for x in pl.Y_WORK]
MOD_OFF = HERO * anim.PITCH


def build(comp=1.0):
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
              yaw=0.0, lower=0.0, compensate=comp, pack=0.0)
    return anim.frame_records(st)


def yval(r):
    if r['kind'] == 'cylinder':
        return 0.5 * (float(r['p'][1]) + float(r['q'][1]))
    return float(r['c'][1])


if __name__ == '__main__':
    recs = build(1.0)
    errs = []
    # A1 根座
    for i in range(6):
        nm = f'M{HERO+1}_M1_A{i+1:02d}_link1'
        r = next((x for x in recs if x['name'] == nm), None)
        if r is None:
            errs.append(f'A1 缺失 {nm}'); continue
        want = MOD_OFF + Y_WORK_LOCAL[i]
        got = yval(r)
        if abs(got - want) > 0.5:
            errs.append(f'A1 {nm} y={got:.1f} 期望 {want:.1f}')
    # A2 附属件与本臂平面
    worst = 0.0
    for i in (1, 4):
        plane = MOD_OFF + Y_WORK_LOCAL[i]
        for r in recs:
            if not r['name'].startswith(f'M{HERO+1}_'):
                continue
            if f'_A{i+1:02d}_' not in r['name']:
                continue
            if r['group'] not in ('head', 'gate', 'magnet'):
                continue
            d = abs(yval(r) - plane)
            worst = max(worst, d)
            if d > 60.0:
                errs.append(f'A2 {r["name"]} |Δy|={d:.1f} mm')
    # A3 滑块
    for i in range(6):
        nm = f'M{HERO+1}_P03_slider{i+1:02d}'
        r = next((x for x in recs if x['name'] == nm), None)
        if r is None:
            continue
        want = MOD_OFF + Y_WORK_LOCAL[i]
        if abs(yval(r) - want) > 1.0:
            errs.append(f'A3 {nm} y={yval(r):.1f} 期望 {want:.1f}')
    print(f'附属件与臂平面最大 |Δy| = {worst:.1f} mm')
    print(f'违规 {len(errs)} 条')
    for e in errs[:8]:
        print('  ', e)
    ok = not errs
    (OUT / 'assembly_invariants.json').write_text(json.dumps(dict(
        ok=ok, max_att_dy_mm=round(worst, 2), violations=errs[:20],
        note='A1 根座 / A2 附属件平面 / A3 滑块；不依赖 bind_heads 的局部偏移'),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('判定:', 'PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)
