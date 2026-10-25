"""Mode A v2-30 调试片数据源：按已验分段轨迹求值，生成统一时间轴（不做二次规划）。

关键纪律（用户明确要求）：
  - 直接调用已验证的分段余弦时间律在指定时刻求值；
  - 绝不把 24 fps 采样帧当成路径节点再重新时间参数化（此前导致 134.92 s 假时长）；
  - 输出同时用于渲染与碰撞，两者共享同一份 q(t)。
"""
import importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')

FPS = 24


def load_segments():
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    cmp = json.loads((OUT / 'experiments' / 'gap_v2' / 'compare.json').read_text(encoding='utf8'))
    loop = json.loads((OUT / 'modeA_loop.json').read_text(encoding='utf8'))
    fold = np.array(tr['path'], float)
    v2 = np.array(next(r for r in cmp['rows'] if r['version'] == 'v2-target30')['path'], float)
    q_pre = np.r_[loop['ik']['pre']['q'], loop['ik']['pre']['L4']]
    q_work = np.r_[loop['ik']['work']['q'], loop['ik']['work']['L4']]
    out = [('部署→紧凑', fold), ('紧凑→预接近(v2)', v2), ('预接近→采摘', np.vstack([q_pre, q_work]))]
    back = [(f'返回·{n}', np.asarray(p, float)[::-1]) for n, p in reversed(out)]
    return out + back


def build_timeline():
    rows, meta, t0 = [], [], 0.0
    for name, nodes in load_segments():
        segs, cum = t2.build(np.asarray(nodes, float))
        n = int(round(cum[-1] * FPS))
        for k in range(n):
            t = k / FPS
            q = t2.state(segs, cum, t)
            rows.append(dict(t=round(t0 + t, 4), seg=name, q=[round(float(x), 3) for x in q]))
        meta.append(dict(seg=name, t_start=round(t0, 3), t_end=round(t0 + cum[-1], 3),
                         duration=round(cum[-1], 3), nodes=len(nodes)))
        t0 += cum[-1]
    return rows, meta, t0


if __name__ == '__main__':
    rows, meta, T = build_timeline()
    print(f'总时长 {T:.2f}s  帧数 {len(rows)}')
    for m in meta:
        print(f'  {m["seg"]:18s} {m["t_start"]:7.2f} → {m["t_end"]:7.2f}  ({m["duration"]:.2f}s, {m["nodes"]} 节点)')

    # 一致性：折叠段是否复现已发布的 24fps 验证数据
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    pub = np.array(tr['trajectory']['q'], float)
    mine = np.array([r['q'] for r in rows if r['seg'] == '部署→紧凑'])
    n = min(len(pub), len(mine))
    diff = np.abs(pub[:n] - mine[:n])
    print(f'\n折叠段 vs 已发布验证数据: {n} 帧, 最大关节差 {diff.max():.4f}°, 伸缩差 {diff[:,4].max():.3f} mm')

    # 关键里程碑
    marks = {}
    for key in ('紧凑→预接近(v2)', '预接近→采摘'):
        sel = [r for r in rows if r['seg'] == key]
        marks[key] = dict(start=sel[0]['q'], end=sel[-1]['q'], frames=len(sel),
                          t=(sel[0]['t'], sel[-1]['t']))
    film, at = 1e9, None
    for r in rows:
        q = np.array(r['q'])
        f = pl.abs_film(q[:4], q[4])
        if f < film:
            film, at = f, (r['t'], r['seg'])
    print(f'全程竖直膜间隙最小 {film:.2f} mm @ t={at[0]:.2f}s ({at[1]})')

    (OUT / 'modeA_v2_timeline.json').write_text(json.dumps(dict(
        status='candidate (v2-30)', fps=FPS, total_s=round(T, 3), frames=len(rows),
        y_state='compensated', y_work=pl.Y_WORK.tolist(),
        segments=meta, milestones=marks,
        min_film_vertical_mm=round(float(film), 2), at=at,
        note='时间轴由已验证分段余弦律求值；未把采样帧当作节点；渲染与碰撞共用本文件'), ensure_ascii=False, indent=1),
        encoding='utf8')
    print('\n写入 out/modeA_v2_timeline.json')
