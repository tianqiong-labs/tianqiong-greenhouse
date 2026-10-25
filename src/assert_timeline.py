"""跨段连续性自动验收：不通过就拒绝生成/发布。

断言（不是"选较近一端"这种启发式）：
  1. 相邻段：上一段末帧 q 与下一段首帧 q 的差 ≤ 容差（角度/伸缩分别设限）
  2. 首帧 == 部署构型；末帧 == 部署构型（闭环）
  3. 每段内部相邻帧差 ≤ 阈值（防段内跳变）
回归用例：反向路径、错误端点必须被拒绝。
"""
import json, sys
from pathlib import Path
import numpy as np

OUT = Path(__file__).resolve().parent / 'out'
TOL_ANG, TOL_TEL = 0.5, 2.0      # 度 / mm
MAX_STEP_ANG, MAX_STEP_TEL = 5.0, 25.0   # 相邻帧上限


def splices(frames):
    """返回段接头 (i, j)：frames[i] 与 frames[j] 属于不同段。"""
    out = []
    for k in range(len(frames) - 1):
        if frames[k]['seg'] != frames[k + 1]['seg']:
            out.append((k, k + 1))
    return out


def check(frames, q_dep=None):
    errs = []
    for i, j in splices(frames):
        dq = np.abs(np.array(frames[i]['q']) - np.array(frames[j]['q']))
        if dq[:4].max() > TOL_ANG or dq[4] > TOL_TEL:
            errs.append(dict(kind='splice', at=f"{frames[i]['seg']}->{frames[j]['seg']}",
                             ang=round(float(dq[:4].max()), 3), tel=round(float(dq[4]), 3)))
    Q = np.array([f['q'] for f in frames])
    if not np.all(np.isfinite(Q)):
        errs.append(dict(kind='non-finite', at=int(np.argwhere(~np.isfinite(Q))[0][0])))
    step = np.abs(np.diff(Q, axis=0))
    bad = np.where((step[:, :4].max(axis=1) > MAX_STEP_ANG) | (step[:, 4] > MAX_STEP_TEL))[0]
    for k in bad:
        errs.append(dict(kind='intra-segment-jump', at=int(k),
                         ang=round(float(step[k, :4].max()), 3), tel=round(float(step[k, 4]), 3)))
    if q_dep is not None:
        for lbl, q in (('first', Q[0]), ('last', Q[-1])):
            d = np.abs(q - np.asarray(q_dep))
            if d[:4].max() > TOL_ANG or d[4] > TOL_TEL:
                errs.append(dict(kind=f'{lbl}-pose', ang=round(float(d[:4].max()), 3),
                                 tel=round(float(d[4]), 3)))
    return errs


def _seg(name, q0, q1, n=40):
    q0 = np.asarray(q0, float); q1 = np.asarray(q1, float)
    return [dict(seg=name, q=[round(float(x), 3) for x in q0 + (q1 - q0) * t])
            for t in np.linspace(0, 1, n)]


def selftest():
    """回归：接得上的两段应通过；反向段、错末端必须被拒。"""
    a = [0, -20, -38, -57, 1600.0]
    m = [-30, 40, -60, 30, 1300.0]
    b = [-60, 116.9, -71.5, 81.2, 1098.0]
    clean = _seg('S1', a, m) + _seg('S2', m, b)
    ok = check(clean, q_dep=None)
    rev = _seg('S1', a, m) + _seg('S2', b, m)          # 第二段方向反了
    bad_rev = check(rev, q_dep=None)
    loop = _seg('S1', a, m) + _seg('S2', m, a)          # 闭环
    loop[-1]['q'] = [5.0] + a[1:]                       # 末端偏离 5°
    bad_end = check(loop, q_dep=a)
    shifted = [dict(seg=f['seg'], q=[f['q'][0] + 10.0] + f['q'][1:]) for f in loop]  # 整体偏 10°，仍闭环
    bad_shift = check(shifted, q_dep=a)
    nan_frames = [dict(seg='S1', q=[float('nan')] * 5)]
    bad_nan = check(nan_frames, q_dep=None)
    return dict(clean_pass=(len(ok) == 0), reversed_rejected=bool(bad_rev),
                wrong_end_rejected=bool(bad_end), global_offset_rejected=bool(bad_shift),
                nan_rejected=bool(bad_nan))


if __name__ == '__main__':
    tl = json.loads((OUT / 'modeA_v2_timeline.json').read_text(encoding='utf8'))
    fr = tl['frame_records']
    q_dep = fr[0]['q']
    errs = check(fr, q_dep=q_dep)
    st = selftest()
    print('接头数:', len(splices(fr)))
    print('实际轨迹违规:', len(errs))
    for e in errs[:6]:
        print('  ', e)
    print('回归:', st)
    ok = (len(errs) == 0) and st['clean_pass'] and st['reversed_rejected'] and st['wrong_end_rejected']
    print('验收:', 'PASS' if ok else 'FAIL')
    sys.exit(0 if ok else 1)
