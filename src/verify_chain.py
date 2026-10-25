"""验证链：自适应边检查 + 回归用例 + MANIFEST 溯源 + 严格并发回归。

边检查思路（不是简单加采样点）：
  对一条边 a→b，先按 k 段细分，测每个子段的端点间隙；
  用"相邻采样构型之间的链点最大位移"作为运动上界：
      若 位移/2 < 最小间隙 - 目标阈值，则认为该子段内不可能穿到目标以内；
  否则继续细分该子段，直到上界成立或达到最大深度（深度耗尽即判不合格，不放松判据）。
"""
import importlib.util, json, hashlib, os, shutil, time
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
pb = importlib.import_module('publish')

R = 73.0
FILM = pl.FILM


def film_perp(q, L4, n=10):
    P = pl.chain3d(q, L4, 0.0, 0.0, n_per=n)
    d = np.hypot(P[:, 0][:, None] - FILM[:, 0][None, :], P[:, 2][:, None] - FILM[:, 1][None, :])
    return float(d.min()) - R


# 各关节到末端的最大力臂（保守上界，与具体构型无关）
LEVER = np.array([1150 + 1380 + 1520 + 1600, 1380 + 1520 + 1600, 1520 + 1600, 1600], float)


def chain_disp(qa, qb):
    """笛卡尔运动上界：Σ|Δθ_i|·力臂_i + |ΔL4|（保守，不依赖采样点数）。"""
    dth = np.abs(np.radians(np.asarray(qa[:4], float) - np.asarray(qb[:4], float)))
    return float((dth * LEVER).sum() + abs(float(qa[4] - qb[4])))


def strict_edge(a, b, required, depth=0, max_depth=6, k=4):
    """返回 (ok, min_gap, worst_t)。自适应细分，深度耗尽即不合格。"""
    gaps = []
    pts = [a + (b - a) * (i / k) for i in range(k + 1)]
    for q in pts:
        gaps.append(film_perp(q[:4], q[4]))
    worst = min(gaps)
    disp = max(chain_disp(pts[i], pts[i + 1]) for i in range(k))
    # 运动上界：子段内链点偏离采样点的最大量不超过 disp/2
    if worst - disp / 2 >= required:
        return True, worst, None
    if depth >= max_depth:
        return False, worst, depth
    for i in range(k):
        ok, g, d = strict_edge(pts[i], pts[i + 1], required, depth + 1, max_depth, k)
        if not ok:
            return False, g, d
        worst = min(worst, g)
    return True, worst, None


def strict_path(path, required):
    worst = 1e9
    for a, b in zip(path[:-1], path[1:]):
        ok, g, d = strict_edge(np.asarray(a, float), np.asarray(b, float), required)
        if not ok:
            return False, round(g, 2), d
        worst = min(worst, g)
    return True, round(worst, 2), None


if __name__ == '__main__':
    cmp = json.loads((OUT / 'experiments' / 'gap_v2' / 'compare.json').read_text(encoding='utf8'))
    v20 = np.array(next(r for r in cmp['rows'] if r['version'] == 'v2-target20')['path'], float)
    v30 = np.array(next(r for r in cmp['rows'] if r['version'] == 'v2-target30')['path'], float)

    print('=== 回归用例 ===')
    ok20, g20, d20 = strict_path(v20, 20.0)
    print(f'v2-20 要求 20 mm -> {"通过(异常!)" if ok20 else "拒绝"}  实测 {g20} mm  深度 {d20}')
    ok30, g30, d30 = strict_path(v30, 30.0)
    print(f'v2-30 要求 30 mm -> {"通过" if ok30 else "拒绝"}  实测 {g30} mm  深度 {d30}')

    # 严格并发回归：旧任务先启动、新任务先发布、旧任务最后发布
    print('\n=== 严格并发回归 ===')
    tgt = OUT / 'runs' / '_strict_conc.json'
    def mk(rid, task, at, val):
        d = OUT / 'runs' / rid; d.mkdir(parents=True, exist_ok=True)
        (d / 'meta.json').write_text(json.dumps(dict(run_id=rid, kind='probe', task_id=task, at=at, status='ok')), encoding='utf8')
        (d / 'payload.json').write_text(json.dumps(dict(v=val)), encoding='utf8')
        return d
    d_old = mk('20261026-000000-strict-old-aaa', 'T-OLD', '2026-10-26T00:00:00', 'old')
    d_new = mk('20261027-000000-strict-new-bbb', 'T-NEW', '2026-10-27T00:00:00', 'new')
    # 旧任务"先启动"：只创建目录，不发布
    r1 = pb.publish(d_new, tgt, expect=dict(task_id='T-NEW'))
    sha_new = hashlib.sha256(tgt.read_bytes()).hexdigest()[:16]
    r2 = pb.publish(d_old, tgt, expect=dict(task_id='T-OLD'))
    sha_end = hashlib.sha256(tgt.read_bytes()).hexdigest()[:16]
    print('新任务发布:', r1, '| 旧任务迟到:', r2)
    print('最终 SHA 属于新任务:', sha_new == sha_end, sha_end)
    tgt.unlink(missing_ok=True)

    # MANIFEST 溯源
    print('\n=== MANIFEST 溯源 ===')
    man = json.loads((OUT / 'baseline' / 'MANIFEST.json').read_text(encoding='utf8'))
    man['metrics'] = {
        'fold_segment': dict(segment_id='deployed->compact', duration_s=11.417,
                             film_vertical_mm=42.25, film_perp_mm=None,
                             validation_run_id=man['source_run'],
                             artifact='planA_trajectory.json'),
        'pre_approach_segment_v1': dict(segment_id='compact->pre', duration_s=12.41,
                                        film_perp_mm=11.60, validation_run_id='modeA_loop-20261025-142655-modeA_loop-533c6f',
                                        artifact='modeA_loop.json'),
        'pre_approach_segment_v2': dict(segment_id='compact->pre', duration_s=13.79,
                                        film_perp_mm=38.04, validation_run_id='gap_v2-experiment',
                                        artifact='experiments/gap_v2/compare.json'),
        'six_arm_scene_v1': dict(film_perp_mm=11.5, static_mm=368.6, housing_mm=116.4,
                                 violations=0, artifact='experiments/gap_v2/scene_compare.json'),
        'six_arm_scene_v2': dict(film_perp_mm=38.0, static_mm=368.6, housing_mm=116.4,
                                 violations=0, artifact='experiments/gap_v2/scene_compare.json'),
    }
    for k, v in man['metrics'].items():
        p = OUT / v['artifact'] if not v['artifact'].startswith('experiments') else OUT / v['artifact']
        v['artifact_sha256'] = hashlib.sha256(p.read_bytes()).hexdigest() if p.exists() else None
    man['note'] = ('42.25 为折叠段"竖直"口径；11.5/38.0 为全场景"垂直"口径，两者不可混为一谈；'
                   'v2 为推荐候选，v1 保留回退')
    (OUT / 'baseline' / 'MANIFEST.json').write_text(json.dumps(man, ensure_ascii=False, indent=1), encoding='utf8')
    print('MANIFEST 已补 segment_id / validation_run_id / artifact_sha256')

    (OUT / 'experiments' / 'gap_v2' / 'strict_verify.json').write_text(json.dumps(dict(
        v2_20=dict(passed=bool(ok20), min_gap_mm=g20, rejected_as_expected=not ok20),
        v2_30=dict(passed=bool(ok30), min_gap_mm=g30),
        concurrency=dict(new_publish=r1, stale_publish=r2, final_sha_matches_new=(sha_new == sha_end))),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 experiments/gap_v2/strict_verify.json')
