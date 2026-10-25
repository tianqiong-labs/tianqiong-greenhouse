"""余量扫描：在 5 维（4 俯仰 + L4）下，看"必须留多大膜余量"时仍能找到路径。

输出 out/planA_sweep.json：每个余量要求的成功与否、搜索预算、迭代数、实际最小余量。
比"找到一条 0.9 mm 的路径"有意义得多：它给出可行性的边界。
"""
import importlib.util, json, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('pl', HERE / 'planner.py')
pl = importlib.util.module_from_spec(spec); spec.loader.exec_module(pl)

q0 = np.r_[pl.Q_START4, pl.L4_START]
q1 = np.r_[pl.Q_GOAL4, pl.L4_GOAL]
rows = []
for need in (10, 20, 40, 60, 80, 100):
    pl.MARGIN = need
    ok0 = pl.violation(*pl.unpack(q0, 5), want_clear=True)[0]
    ok1 = pl.violation(*pl.unpack(q1, 5), want_clear=True)[0]
    best = None
    t0 = time.time()
    if ok0 and ok1:
        for sd in (0, 1, 2):
            path, it, dt = pl.rrt_connect(q0, q1, 5, budget=25.0, seed=sd)
            if path is not None:
                sm = pl.shortcut(path, 5, seed=sd)
                rep = pl.report(sm, 5)
                best = dict(seed=sd, iters=it, seconds=round(dt, 1), report=rep,
                            path=[[round(float(x), 2) for x in row] for row in sm])
                break
    rows.append(dict(margin_req=need, start_ok=bool(ok0), goal_ok=bool(ok1),
                     found=best is not None,
                     film_mm=best['report']['film_mm'] if best else None,
                     iters=best['iters'] if best else None,
                     seconds=best['seconds'] if best else round(time.time() - t0, 1),
                     nodes=best['report']['nodes'] if best else None,
                     path=best['path'] if best else None))
    print(f"  要求余量 {need:3d} mm -> 起点{'OK' if ok0 else 'NG'} 终点{'OK' if ok1 else 'NG'} "
          f"{'FOUND' if best else 'fail'}  实测 {rows[-1]['film_mm']} mm  迭代 {rows[-1]['iters']}")

(HERE / 'out' / 'planA_sweep.json').write_text(json.dumps(dict(stage='5D', rows=rows), ensure_ascii=False, indent=1),
                                               encoding='utf8')
print('写入 out/planA_sweep.json')
