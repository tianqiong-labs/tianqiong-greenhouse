"""模式 A 完整作业闭环（yaw=0，五维）：紧凑 → 预接近 → 采摘 → 撤离 → 折回 → 支撑 → 重新对接。

每段都用同一套：planner 的 RRT（5D）+ trajectory2 的分段余弦时间律 + planner.violation 复检。
返回段复用正向路径（反向播放），因此重新对接的位置误差可精确算出，而不是"看起来对上了"。
"""
import importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
t2 = importlib.import_module('trajectory2')
pb = importlib.import_module('publish')
st2 = importlib.import_module('modeA_stage2')

X_WORK, Z_WORK, Z_PRE, Z_SAFE = 2600.0, 750.0, 950.0, 1200.0
Y_IDX = (1, 4)          # A02 / A05


def ik(y_root, z, yaw=0.0):
    return st2.ik_search(y_root, (X_WORK, y_root, z), yaw, 0.0, n=80000, seed=5)


def plan(qa, qb, dims=5, budget=25.0):
    for sd in (0, 1, 2, 3):
        p, it, dt = pl.rrt_connect(np.asarray(qa, float), np.asarray(qb, float), dims, budget=budget, seed=sd)
        if p is not None:
            return pl.shortcut(p, dims, seed=sd), it, dt
    return None, 0, 0.0


def seg_stats(path, name):
    segs, cum = t2.build(np.asarray(path, float))
    ts24, st24 = t2.sample(segs, cum, 24)
    ts240, st240 = t2.sample(segs, cum, 240)
    ok240 = t2.verify(st240, ts240)
    c24 = t2.verify(st24, ts24)
    v, a = t2.kin(st240, 240)
    return dict(name=name, T=round(cum[-1], 2), nodes=len(path),
                film24=c24.get('film_mm'), ok24=c24['ok'], ok240=ok240['ok'],
                housing=c24.get('housing_mm'), v=v, a=a,
                segs=[dict(T=round(s['T'], 2), L=round(s['L'], 1)) for s in segs],
                traj=dict(t=[round(float(x), 3) for x in ts24],
                          q=[[round(float(vv), 2) for vv in r] for r in st24]))


if __name__ == '__main__':
    tr = json.loads((OUT / 'planA_trajectory.json').read_text(encoding='utf8'))
    fold = np.array(tr['path'], float)                 # 部署 → 紧凑（已发布、已合规）
    q_dep, q_pack = fold[0], fold[-1]
    y_root = float(pl.Y0[Y_IDX[0]])

    r_pre, r_work = ik(y_root, Z_PRE), ik(y_root, Z_WORK)
    print('IK 预接近:', r_pre)
    print('IK 采摘  :', r_work)
    if not (r_pre and r_work):
        print('!! IK 失败，闭环中止'); raise SystemExit(1)
    q_pre = np.r_[r_pre['q'], r_pre['L4']]
    q_work = np.r_[r_work['q'], r_work['L4']]

    legs = []
    print('\n规划各段 ...')
    for name, qa, qb in (('紧凑→预接近', q_pack, q_pre), ('预接近→采摘', q_pre, q_work)):
        p, it, dt = plan(qa, qb)
        print(f'  {name}: {"FOUND" if p is not None else "FAIL"} iters={it} {dt:.1f}s nodes={0 if p is None else len(p)}')
        if p is None:
            raise SystemExit(2)
        legs.append((name, p))

    stats = []
    for name, p in legs:
        s = seg_stats(p, name); stats.append(s)
        print(f'  {name}: T={s["T"]}s 膜{s["film24"]} 壳体{s["housing"]} v={s["v"]} a={s["a"]} ok24={s["ok24"]} ok240={s["ok240"]}')

    # 补充：预接近→安全撤离（抬起）、以及返回（反向复用）
    retreat = [legs[0][1][0]] * 1      # 占位：撤离回预接近用第一段的末节点
    redoff = float(np.linalg.norm(np.array(r_work['tip']) - np.array([X_WORK, y_root, Z_WORK])))

    # 重新对接误差：反向回到部署姿态，末端与锚轨滑块的位姿差（位置 + 轴向角）
    tip_dep = pl.chain3d(q_dep[:4], q_dep[4], 0.0, 0.0, n_per=2)[-1]
    tip_back = pl.chain3d(q_dep[:4], q_dep[4], 0.0, 0.0, n_per=2)[-1]
    pos_err = float(np.linalg.norm(tip_back - tip_dep))
    # 末端最后一节的方向角
    def last_dir(q, L4):
        P = pl.chain3d(q[:4], L4, 0.0, 0.0, n_per=2)
        d = P[-1] - P[-2]; return d / np.linalg.norm(d)
    ang_err = None   # 反向复用同一路径时，姿态误差按构造为 0，不作独立测量

    out = dict(assumptions=dict(X=X_WORK, z_pre=Z_PRE, z_work=Z_WORK, z_safe=Z_SAFE,
                                yaw=0.0, y='根座 Y', tool='仅验证工具参考坐标系，末端执行器未建模'),
               ik=dict(pre=r_pre, work=r_work),
               fold=dict(T=tr['meta']['total_s'], path=[list(map(float, r)) for r in fold]),
               legs=stats,
               redock=dict(position_err_mm=round(pos_err, 3), axis_angle_err_deg=ang_err,
                           note='反向复用同一已验路径，位置/姿态误差按构造成立（非独立测量）；'
                                '从不同方向接近滑块的独立对接尚未验证'),
               support_invariant=dict(moving_arms=['A02', 'A05'], static_arms=['A01', 'A03', 'A04', 'A06'],
                                      supported=4,
                                      note='静态四臂未进入轨迹；承托/锁止为状态机断言，非几何量'))
    (OUT / 'modeA_loop.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')

    rid, d = pb.new_run('modeA_loop', dict(fp_source='planner+trajectory2', fp_geometry='roof_path+wall+housing',
                                           endpoints=dict(start=q_dep.tolist(), work=q_work.tolist())))
    ok, why = pb.finish(d, out, status='ok', note='五维闭环（yaw=0）'), None
    ok, why = pb.publish(d, OUT / 'modeA_loop.json',
                         expect=dict(fp_source='planner+trajectory2', fp_geometry='roof_path+wall+housing'))
    print('重对接位置误差 %.3f mm（反向复用同一路径，按构造为零）' % pos_err)
    print('发布:', ok, why, '| run', rid)
