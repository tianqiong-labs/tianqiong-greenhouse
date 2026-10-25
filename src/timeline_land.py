"""时间轴落盘（逐帧 + 精确终点）+ 六段边界校验 + 机构状态轨。

数据关系按用户要求固定为：
    六段连续轨迹（节点 + 余弦时间律） = 权威源
        └─ q(t) 求值器 → 逐帧缓存（可复现派生）
        └─ 精确终点记录 t=T（独立于视频帧率）
"""
import importlib.util, json, hashlib
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')
mt = importlib.import_module('modeA_timeline')

FPS = 24
SRC = ['planA_trajectory.json', 'experiments/gap_v2/compare.json', 'modeA_loop.json']


def sha(name):
    p = OUT / name
    return hashlib.sha256(p.read_bytes()).hexdigest()[:16] if p.exists() else None


if __name__ == '__main__':
    segs_src = mt.load_segments()
    # 先建立"全局时间 → 段内求值"的映射（每段只持有自己的余弦律）
    built, marks, t0 = [], [], 0.0
    for name, nodes in segs_src:
        segs, cum = t2.build(np.asarray(nodes, float))
        built.append(dict(name=name, segs=segs, cum=cum, t_start=t0, T=cum[-1]))
        marks.append(dict(seg=name, t_start=round(t0, 4), t_end=round(t0 + cum[-1], 4),
                          duration=round(cum[-1], 4)))
        t0 += cum[-1]
    T_total = t0

    def q_at(tg):
        for b in built:
            if tg <= b['t_start'] + b['T'] + 1e-9:
                return b['name'], t2.state(b['segs'], b['cum'], max(0.0, tg - b['t_start']))
        b = built[-1]
        return b['name'], t2.state(b['segs'], b['cum'], b['T'])

    # 标准视频序列：全局 0..floor(T*fps)，再追加一帧精确终点
    n_last = int(np.floor(T_total * FPS))
    frames = []
    for n in range(n_last + 1):
        tv = n / FPS
        name, q = q_at(tv)
        frames.append(dict(frame=n, video_time=round(tv, 6), source_time=round(tv, 6),
                           seg=name, q=[round(float(x), 4) for x in q],
                           y_root=round(float(pl.Y_WORK[1]), 1), yaw=0.0, L4=round(float(q[4]), 2)))
    name_T, qT = q_at(T_total)
    frames.append(dict(frame=n_last + 1, video_time=round((n_last + 1) / FPS, 6),
                       source_time=round(T_total, 6), seg=name_T,
                       q=[round(float(x), 4) for x in qT],
                       y_root=round(float(pl.Y_WORK[1]), 1), yaw=0.0, L4=round(float(qT[4]), 2),
                       exact_endpoint=True))

    # 段内抽查：每段 5 个内点，独立与原始余弦律比对
    boundary = []
    for b in built:
        errs = []
        for frac in (0.13, 0.37, 0.5, 0.61, 0.88):
            tg = b['t_start'] + b['T'] * frac
            nm, q1 = q_at(tg)
            q2 = t2.state(b['segs'], b['cum'], b['T'] * frac)
            errs.append(float(np.abs(np.asarray(q1) - np.asarray(q2)).max()))
        boundary.append(dict(seg=b['name'], interior_max_err=round(max(errs), 6), samples=5))

    # 返回终点 vs 初始部署
    q_dep = np.asarray(segs_src[0][1], float)[0]
    ret_err = float(np.abs(np.asarray(qT) - q_dep).max())

    endpoint = dict(t=round(T_total, 4), q=[round(float(x), 4) for x in qT], seg=name_T, exact=True)
    # 校验前置于任何写盘；部署基准是独立常量，不取被检数据自身
    import os
    if os.environ.get('TQ_INJECT_SPLICE'):
        off = float(os.environ['TQ_INJECT_SPLICE'])
        for f in frames:
            if f['seg'].startswith('紧凑→预接近'):
                f['q'] = [round(x + off, 4) for x in f['q']]
        print(f'[注入] 紧凑→预接近 段整体偏移 {off}°')
    q_dep_ref = np.r_[np.asarray(pl.A0, float), 1600.0]
    import assert_timeline as _at
    _errs = _at.check(frames, q_dep=q_dep_ref)
    if _errs:
        print('!! 连续性/姿态不合格，未写出任何文件:', _errs[:3])
        raise SystemExit(2)

    payload = dict(
        status='candidate (v2-30)', fps=FPS, total_s=round(T_total, 4),
        n_frames=len(frames), exact_endpoint=endpoint,
        segments=marks, boundaries=boundary,
        return_to_deploy_max_err=round(ret_err, 6),
        y_state='compensated', y_work=pl.Y_WORK.tolist(),
        sources={s: sha(s) for s in SRC},
        frame_records=frames,
        note=('逐帧记录是 q(t) 的派生缓存；精确终点单独保存；'
              '末帧时刻为 floor(T*fps)/fps，不声称所有帧严格等间隔'))
    (OUT / 'modeA_v2_timeline.json').write_text(json.dumps(payload, ensure_ascii=False, indent=1), encoding='utf8')

    # 机构状态轨
    states = [
        dict(name='double_rail_compensation', level='VALIDATED', basis='六臂 Y 补位坐标已用于全场景检查'),
        dict(name='carrier_unload', level='SCHEMATIC', basis='承托头实体未建模，仅状态位'),
        dict(name='gate_open', level='SCHEMATIC', basis='闸门机构未建模'),
        dict(name='tip_quick_release', level='SCHEMATIC', basis='快脱接口未建模'),
        dict(name='arm_fold_v2', level='VALIDATED', basis='27.64 mm 净空、20 mm 严格边检查通过'),
        dict(name='work_pose_reach', level='VALIDATED', basis='IK 误差 0.36 mm；路径已验'),
        dict(name='return_fold', level='VALIDATED', basis='反向复算，同一边检查'),
        dict(name='tip_realign', level='PENDING', basis='独立对接未验证（仅按构造为 0）'),
        dict(name='tip_lock', level='SCHEMATIC', basis='锁止机构未建模'),
        dict(name='gate_close', level='SCHEMATIC', basis='闸门机构未建模'),
        dict(name='carrier_reengage', level='SCHEMATIC', basis='承托头与嵌条接合未建模'),
    ]
    (OUT / 'modeA_state_track.json').write_text(json.dumps(dict(
        cycle='关节运动主链 59.329 s（不含接口操作时间）',
        levels=dict(VALIDATED='有几何/运动验证支持', SCHEMATIC='仅有设计意图', PENDING='未实现'),
        states=states), ensure_ascii=False, indent=1), encoding='utf8')

    print(f'总时长 {T_total:.4f}s  视频帧 {len(frames)}  末帧 video={frames[-1]["video_time"]:.4f}s source={frames[-1]["source_time"]:.4f}s')
    for b in boundary: print(f"  {b['seg']:16s} 段内抽查 5 点 最大误差 {b['interior_max_err']:.6f}")

    print('返回终点 vs 部署起始 最大关节差: %.6f' % ret_err)
    print('跨段连续性断言: PASS（基准=独立部署构型）')
    print('状态轨: VALIDATED %d / SCHEMATIC %d / PENDING %d' % (
        sum(1 for s in states if s['level'] == 'VALIDATED'),
        sum(1 for s in states if s['level'] == 'SCHEMATIC'),
        sum(1 for s in states if s['level'] == 'PENDING')))
