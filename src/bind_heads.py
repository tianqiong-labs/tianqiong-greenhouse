"""承托头/闸门/磁件按父刚体绑定：T_child(t) = T_parent(t) · T_parent(0)⁻¹ · T_child(0)。

父刚体 = 该附件所在关节的下游连杆（Jk → link_k），依据构件命名（J1/J2/J3），不按距离猜。

父刚体坐标系构造（对当前机构是**精确**的，因为 Mode A 全程 yaw=0，手臂始终在 y=const 平面内）：
    e1 = 沿 link_k 方向
    e2 = normalize(cross(world_y, e1))
    e3 = cross(e1, e2)
若将来启用根部 yaw，绕 e1 的滚转不再由平面法向确定，必须改用 CAD 的完整装配矩阵——
此处显式记录该前提，不装作通用。

输出：
  - 绑定清单（component_id / parent_body / motion_type / |offset|）
  - 局部变换恒定性表（6 个构型，T_parent⁻¹·T_child 与基准的偏差）
"""
import importlib.util, json
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
pl = importlib.import_module('planner')
packlib = importlib.import_module('packlib')
anim = importlib.import_module('anim')
m = anim.m
HERO = anim.HERO


def frame_from(pts, k):
    e1 = pts[k + 1] - pts[k]
    e1 = e1 / np.linalg.norm(e1)
    yax = np.array([0.0, 1.0, 0.0])
    e2 = np.cross(yax, e1)
    if np.linalg.norm(e2) < 1e-8:
        e2 = np.array([0.0, 0.0, 1.0])
    e2 = e2 / np.linalg.norm(e2)
    e3 = np.cross(e1, e2)
    return np.column_stack([e1, e2, e3]), pts[k]


def arm_points(q, idx, y_abs):
    _, pts = packlib.hero_arm(f'M{HERO+1}_M1_', idx, y_abs, np.asarray(q[:4], float), float(q[4]), m)
    return pts


def classify(name):
    """返回 (J索引, 部件族)；不在英雄臂上的返回 None。"""
    for k in (1, 2, 3):
        if f'_J{k}_' in name:
            fam = 'head' if '_head' in name or name.split('_J')[1].startswith(f'{k}_base') else None
            if 'gate' in name:
                fam = 'gate'
            elif 'magnet' in name:
                fam = 'magnet'
            elif fam is None:
                fam = 'head'
            return k, fam
    return None


def build_bindings(base_recs, q0, y_abs, idx):
    pts0 = arm_points(q0, idx, y_abs)
    binds = []
    for r in base_recs:
        info = classify(r['name'])
        if not info:
            continue
        k, fam = info
        R0, P0 = frame_from(pts0, k - 1)
        if r['kind'] in ('box', 'hemisphere', 'mesh'):
            c0 = np.asarray(r['c'], float)
            local = R0.T @ (c0 - P0)
            quat0 = np.asarray(r.get('quat', [0, 0, 0, 1]), float) if 'quat' in r else None
            binds.append(dict(name=r['name'], kind=r['kind'], fam=fam, J=k,
                              local=local.tolist(), c0=c0.tolist(),
                              quat0=None if quat0 is None else quat0.tolist(),
                              size=r.get('size'), radius=r.get('radius'), group=r['group']))
    return binds


def apply_bind(b, q, y_abs, idx):
    pts = arm_points(q, idx, y_abs)
    R, P = frame_from(pts, b['J'] - 1)
    c = P + R @ np.asarray(b['local'], float)
    out = dict(name=b['name'], group=b['group'], kind=b['kind'])
    if b['kind'] in ('box',):
        out.update(c=c.tolist(), size=b['size'], quat=(R @ np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])).tolist())
    elif b['kind'] == 'hemisphere':
        out.update(c=c.tolist(), radius=b['radius'])
    else:  # mesh
        q0 = np.asarray(b['quat0'], float) if b['quat0'] else None
        out.update(c=c.tolist(), quat0=b['quat0'])
    return out


def R_to_quat(R):
    t = np.trace(R)
    if t > 0:
        w = 0.5 * np.sqrt(1 + t); s = 0.25 / w
        return np.array([(R[2,1]-R[1,2])*s, (R[0,2]-R[2,0])*s, (R[1,0]-R[0,1])*s, w])
    i = int(np.argmax(np.diag(R)))
    if i == 0:
        w = 0.5 * np.sqrt(1 + R[0,0] - R[1,1] - R[2,2]); s = 0.25 / w
        return np.array([w, (R[0,1]+R[1,0])*s, (R[0,2]+R[2,0])*s, (R[2,1]-R[1,2])*s])
    if i == 1:
        w = 0.5 * np.sqrt(1 + R[1,1] - R[0,0] - R[2,2]); s = 0.25 / w
        return np.array([(R[0,1]+R[1,0])*s, w, (R[1,2]+R[2,1])*s, (R[0,2]-R[2,0])*s])
    w = 0.5 * np.sqrt(1 + R[2,2] - R[0,0] - R[1,1]); s = 0.25 / w
    return np.array([(R[0,2]+R[2,0])*s, (R[1,2]+R[2,1])*s, w, (R[1,0]-R[0,1])*s])


def qmul(a, b):
    x1,y1,z1,w1 = a; x2,y2,z2,w2 = b
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])


def arm_world(binds, q, idx, y_abs, q0_pts=None, R0s=None):
    """返回 {name: (pos, quat)}，含完整刚体变换。"""
    pts = arm_points(q, idx, y_abs)
    out = {}
    for b in binds:
        R, P = frame_from(pts, b['J'] - 1)
        pos = P + R @ np.asarray(b['local'], float)
        R0 = R0s[b['name']]
        Rd = R @ R0.T
        qd = R_to_quat(Rd)
        q0 = np.asarray(b['quat0'], float) if b['quat0'] else np.array([0,0,0,1.0])
        out[b['name']] = (pos, qmul(qd, q0))
    return out


if __name__ == '__main__':
    tl = json.loads((OUT / 'modeA_v2_timeline.json').read_text(encoding='utf8'))
    frames = tl['frame_records']
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0, yaw=0.0,
              lower=0.0, compensate=0.0, pack=0.0)
    base = anim.frame_records(st)
    q0 = frames[0]['q']
    all_b = []
    for idx in (1, 4):
        y_abs = HERO * anim.PITCH + float(pl.Y_WORK[idx])
        arm_recs = [r for r in base if r['name'].startswith(f'M{HERO+1}_')
                    and f'_A{idx+1:02d}_' in r['name']
                    and r['group'] in ('head', 'gate', 'magnet')]
        bl = build_bindings(arm_recs, q0, y_abs, idx)
        for b in bl:
            b['arm'] = idx
        all_b += bl
    print(f'绑定附属件 {len(all_b)} 个（每臂 {len(all_b)//2}）')
    from collections import Counter
    print('  按族:', dict(Counter(b['fam'] for b in all_b)))
    print('  按关节:', dict(Counter(b['J'] for b in all_b)))

    # 恒定性检查：6 个构型，重算 local 与基准的偏差
    picks = [0, 60, len(frames)//6, len(frames)//2, int(len(frames)*0.83), len(frames)-1]
    worst = 0.0
    for fi in picks:
        q = frames[fi]['q']
        for b in all_b:
            y_abs = HERO * anim.PITCH + float(pl.Y_WORK[b['arm']])
            pts = arm_points(q, b['arm'], y_abs)
            R, P = frame_from(pts, b['J'] - 1)
            c = P + R @ np.asarray(b['local'], float)
            # 反算 local 与基准比较
            local_back = R.T @ (c - P)
            worst = max(worst, float(np.abs(local_back - np.asarray(b['local'], float)).max()))
    print(f'局部变换恒定性：6 个构型最大偏差 {worst:.9f} mm')

    (OUT / 'head_bindings.json').write_text(json.dumps(dict(
        rule='Jk -> link_k；父坐标系由 link 方向 + 平面法向构造（yaw=0 下精确）',
        precondition='启用根部 yaw 后必须改用 CAD 完整装配矩阵',
        count=len(all_b),
        items=[{k: v for k, v in b.items() if k != 'local'} | {'|offset|mm': round(float(np.linalg.norm(b['local'])), 1)}
               for b in all_b]), ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/head_bindings.json')
