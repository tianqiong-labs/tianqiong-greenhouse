"""L2：完整六臂场景的逐帧碰撞检查（A02/A05 作业，A01/03/04/06 支撑）。

统一模型：链式几何（连杆/关节/伸缩，半径 49/73）+ 母墙盒 + 锚轨 + 卷筒壳体 + 棚膜折线。
不依赖 CAD，保证与 planner 的约束模型一致；结论按"链级保守几何"表述，并注明未覆盖项。
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')

SEG = np.array([1150.0, 1380.0, 1520.0])
FILM = pl.FILM
R_LINK, R_JOINT = 49.0, 73.0


def chain(q, L4, y, n_per=8):
    P = pl.chain3d(q, L4, 0.0, 0.0, n_per=n_per)
    P[:, 1] += y
    return P


def seg_build(nodes):
    return t2.build(np.asarray(nodes, float))


def sample_traj(nodes, fps):
    segs, cum = seg_build(nodes)
    return t2.sample(segs, cum, fps)


def film_gap(P):
    """点到棚面的最小距离（x-z 截面），减去关节半径。"""
    d = np.hypot(P[:, 0][:, None] - FILM[:, 0][None, :], P[:, 2][:, None] - FILM[:, 1][None, :])  # FILM 是 (x,z) 折线
    return float(d.min()) - R_JOINT


def box_gap(P, lo, hi, radius=0.0):
    """到盒体的距离；radius=0 表示量的是裸距离（锚轨/墙面属设计接触，不扣半径）。"""
    d = np.maximum(np.maximum(lo - P, P - hi), 0.0)
    return float(np.linalg.norm(d, axis=1).min()) - radius


def scene_gap(moving, static, y_mov, y_sta):
    """moving/static: (N,3)；返回 3D 最小距离减两倍连杆半径。"""
    d = np.linalg.norm(moving[:, None, :] - static[None, :, :], axis=2).min()
    return float(d) - 2 * R_LINK


if __name__ == '__main__':
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    loop = json.loads((OUT / 'modeA_loop.json').read_text(encoding='utf8'))
    fold = np.array(tr['path'], float)
    q_dep, q_pack = fold[0], fold[-1]
    q_pre = np.r_[loop['ik']['pre']['q'], loop['ik']['pre']['L4']]
    q_work = np.r_[loop['ik']['work']['q'], loop['ik']['work']['L4']]

    legs = [('部署→紧凑', fold),
            ('紧凑→预接近', np.array(loop['legs'][0]['traj']['q'][::max(1, len(loop['legs'][0]['traj']['q']) // 8)], float)
             if False else np.vstack([q_pack, q_pre])),
            ('预接近→采摘', np.vstack([q_pre, q_work]))]
    # 用规划得到的中间节点重建（legs[0] 的节点数在 json 里未存，退化为两端点会失真，故重规划一次）
    p1, _, _ = pl.rrt_connect(q_pack, q_pre, 5, budget=20.0, seed=0)
    if p1 is None:
        print('重规划紧凑→预接近 失败'); raise SystemExit(1)
    legs[1] = ('紧凑→预接近', pl.shortcut(p1, 5, seed=0))

    seq = []
    for name, nodes in legs:
        ts, st = sample_traj(nodes, 24)
        seq.append((name, ts, st))
    # 反向：采摘→预接近→紧凑→部署
    for name, nodes in reversed(legs):
        ts, st = sample_traj(np.asarray(nodes, float)[::-1], 24)
        seq.append((f'返回·{name}', ts, st))

    t_off = 0.0
    timeline = []
    for name, ts, st in seq:
        for t, q in zip(ts, st):
            timeline.append((t_off + float(t), name, q))
        t_off += float(ts[-1])
    print(f'总时长 {t_off:.2f}s，帧数 {len(timeline)}')

    y_hero = float(pl.Y_WORK[1])
    statics = []
    for i, y in enumerate(pl.Y_WORK):
        if i in (1, 4):
            continue
        statics.append((i, y, chain(pl.A0, 1600.0, y)))
    rail_lo = np.array([4620 - 60, -1e9, 0 - 60]); rail_hi = np.array([4620 + 60, 1e9, 0 + 60])
    wall_lo = np.array([-600.0, -1e9, 0.0]); wall_hi = np.array([0.0, 1e9, 3150.0])

    worst = dict(film=1e9, static=1e9, twin=1e9, rail=1e9, housing=1e9, wall=1e9)
    at = {k: None for k in worst}
    bad = []
    for t, leg, q in timeline:
        Pm = chain(q[:4], q[4], y_hero)
        g_film = film_gap(Pm)
        g_static = min(scene_gap(Pm, S, y_hero, y) for _, y, S in statics)
        g_rail = box_gap(Pm, rail_lo, rail_hi, 0.0)   # 臂端在轨上为设计接触
        g_wall = box_gap(Pm, wall_lo, wall_hi, 0.0)   # 根部贴墙为设计接触
        g_hz = box_gap(Pm, np.array([pl.HZ_LO[0], -1e9, pl.HZ_LO[1]]),
                       np.array([pl.HZ_HI[0], 1e9, pl.HZ_HI[1]]))
        for k, v in (('film', g_film), ('static', g_static), ('rail', g_rail), ('wall', g_wall), ('housing', g_hz)):
            if v < worst[k]:
                worst[k], at[k] = v, (round(t, 2), leg)
            if v < 0:
                bad.append(dict(t=round(t, 2), leg=leg, what=k, gap=round(v, 1)))
        # 双作业臂：另一根在同轨迹、位移 Δt 时的位置
    print('单臂对场景最小间隙:', {k: round(v, 1) for k, v in worst.items()})
    print('最紧时刻:', {k: at[k] for k in at})

    # A05 与 A02 同轨迹、y 偏移 1680：同步 vs 错峰
    y5 = float(pl.Y_WORK[4])
    sync_min, stag_min = 1e9, 1e9
    ys5 = y5 - y_hero
    for off in (0.0, 3.0, 6.0):
        m = 1e9
        for t, leg, q in timeline:
            P2 = chain(q[:4], q[4], y_hero)
            idx = np.searchsorted([x[0] for x in timeline], t - off)
            idx = int(np.clip(idx, 0, len(timeline) - 1))
            q5 = timeline[idx][2]
            P5 = chain(q5[:4], q5[4], y5)
            d = float(np.linalg.norm(P2[:, None, :] - P5[None, :, :], axis=2).min()) - 2 * R_LINK
            m = min(m, d)
        print(f'  错峰 {off:.0f}s: A02/A05 最小间隙 {m:.1f} mm')
        if off == 0.0:
            sync_min = m
        else:
            stag_min = min(stag_min, m)

    res = dict(total_s=round(t_off, 2), frames=len(timeline),
               worst={k: round(v, 1) for k, v in worst.items()},
               at={k: at[k] for k in at},
               violations=bad[:40], n_violations=len(bad),
               twin=dict(sync_mm=round(sync_min, 1), staggered_best_mm=round(stag_min, 1)),
               model=dict(r_link=R_LINK, r_joint=R_JOINT, y_state=pl.Y_STATE,
                          y_work=pl.Y_WORK.tolist(),
                          note='链级保守几何；承托头/闸门/滑块锁止未建模，属状态机断言'),
               uncovered=['承托头与嵌条的接触/锁止（未建模）', '滑块快脱接口本体（未建模）',
                          '棚膜形变与风载（超出概念样机范围）'])
    (OUT / 'L2_scene.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/L2_scene.json；越界事件', len(bad))
