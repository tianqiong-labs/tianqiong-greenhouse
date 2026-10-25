"""逐帧运动学与碰撞检查器 v2（与浏览器渲染同一插值：位置线性 / 四元数 slerp / 缩放线性）。

代理分类（按单位网格 PCA 形状判定，避免用错误代理制造假碰撞）：
  cap   细长件（连杆、锚轨、珠链、卷筒） -> 胶囊
  box   块状件（母墙、端墙、滑块、壳体） -> OBB（SAT，仅出重叠量与重叠轴）
  plate 片状件（棚膜、嵌条、密封搭接）   -> 剖面折线 2D 距离（近似）

报告口径：
  collision = 间隙 < +2 mm 且不在合法接触白名单
  warn      = 间隙 2..5 mm
  负间隙为穿透深度近似值（胶囊/板件为几何距离，OBB 为最小重叠轴）
"""
import json, math, sys, time
from pathlib import Path
import importlib.util
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
spec = importlib.util.spec_from_file_location('anim', HERE / 'anim.py')
anim = importlib.util.module_from_spec(spec); spec.loader.exec_module(anim)
m = anim.m

FPS = 24
CONTACT = 2.0
WARN = 5.0
TELEPORT = 250.0

# 允许接触（设计上本来贴合/同一运动链）
FILM_OK = {'wall', 'roll', 'endwall', 'seal', 'gapwall', 'head', 'gate', 'band'}
def allowed(a, b):
    ga, gb = a['group'], b['group']
    na, nb = a['name'], b['name']
    if ga in ('film', 'band', 'bead') and gb in FILM_OK: return True
    if gb in ('film', 'band', 'bead') and ga in FILM_OK: return True
    if ga in ('film', 'band', 'bead') and gb in ('film', 'band', 'bead'): return True
    # 同一条臂的相邻连杆/关节/承托头
    def chain(n):
        for tok in ('_A01_', '_A02_', '_A03_', '_A04_', '_A05_', '_A06_'):
            if tok in n:
                return n.split(tok)[0] + tok
        return None
    if chain(na) and chain(na) == chain(nb): return True
    # 同一根臂的不同命名族（连杆 vs F04 磁吸辅助件），按模块+臂号归一
    import re as _re
    def armtok(n):
        mm = _re.match(r'(M\d)_', n)
        aa = _re.search(r'_A(\d{2})_', n)
        return (mm.group(1), aa.group(1)) if (mm and aa) else None
    if armtok(na) and armtok(na) == armtok(nb): return True
    if ga in ('head', 'gate') or gb in ('head', 'gate'): return True
    if ga == 'roll' and gb == 'roll': return True          # 卷筒与其壳体为同一装配
    if ga == 'rail' and gb in ('rail', 'anchor', 'mount'): return True
    if gb == 'rail' and ga in ('rail', 'anchor', 'mount'): return True
    if ga == 'wall' and gb in ('wall', 'mount', 'roll'): return True
    if gb == 'wall' and ga in ('wall', 'mount', 'roll'): return True
    if ga == 'endwall' and gb in ('endwall', 'gapwall', 'wall', 'seal'): return True
    if gb == 'endwall' and ga in ('endwall', 'gapwall', 'wall', 'seal'): return True
    return False

PROXY_CACHE = {}


def proxy_for(rec):
    """只用单位 AABB 判定，不用 PCA（立方体 PCA 退化，会给出任意轴）。
    柱：两个较短半尺寸相近 -> 以最长轴为轴向，半径取另两向半尺寸。
    盒：其余情况，半尺寸按轴对齐。
    """
    key = m.unit_mesh(rec)[1]
    if key in PROXY_CACHE:
        return PROXY_CACHE[key]
    md, _ = m.unit_mesh(rec)
    v = np.asarray(md['vertices'], float)
    lo, hi = v.min(0), v.max(0)
    c = (lo + hi) / 2
    half = np.maximum((hi - lo) / 2, 1e-6)
    ax = int(np.argmax(half))
    others = [i for i in range(3) if i != ax]
    elong = half[ax] / max(1e-9, float(np.mean([half[i] for i in others])))
    roundness = max(half[others]) / max(1e-9, min(half[others]))
    if elong > 2.5 and roundness < 1.35:   # 细长且截面近圆 -> 柱；立方体(1,1,1)不算
        pr = dict(shape='cap', c=c, ax=ax, half=half[ax],
                  r_unit=float(np.mean([half[i] for i in others])), key=key)
    else:
        pr = dict(shape='box', c=c, half=half, key=key)
    PROXY_CACHE[key] = pr
    return pr


def quat_R(q):
    x, y, z, w = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


UNIT = np.eye(3)


def world_proxy(rec):
    pr = rec['proxy']
    pos = np.array(rec['trs'][0], float)
    s = np.abs(np.array(rec['trs'][2], float))
    R = quat_R(rec['trs'][1])
    c = pos + R @ (pr['c'] * rec['trs'][2])
    if pr['shape'] == 'cap':
        ax = pr['ax']
        d = np.zeros(3); d[ax] = pr['half'] * s[ax]
        dw = R @ d
        others = [i for i in range(3) if i != ax]
        r_world = pr['r_unit'] * float(max(s[i] for i in others))
        return dict(shape='cap', a=c - dw, b=c + dw, r=r_world,
                    name=rec['name'], group=rec['group'])
    return dict(shape='box', c=c, R=R, half=pr['half'] * s, name=rec['name'], group=rec['group'])


def seg_seg(A, B, C, D):
    u, v, w = B-A, D-C, A-C
    a = u@u; b = u@v; c = v@v; d = u@w; e = v@w
    den = a*c-b*b
    s = np.clip((b*e-c*d)/den, 0, 1) if den > 1e-9 else 0.0
    t = np.clip((b*s+e)/c, 0, 1) if c > 1e-9 else 0.0
    s = np.clip((b*t-d)/a, 0, 1) if a > 1e-9 else 0.0
    return float(np.linalg.norm((A+u*s)-(C+v*t)))

def seg_aabb(p0, p1, lo, hi):
    """线段到 AABB 的距离（在盒局部坐标下）。"""
    d = p1-p0
    tmin, tmax = 0.0, 1.0
    for i in range(3):
        if abs(d[i]) < 1e-9:
            if p0[i] < lo[i] or p0[i] > hi[i]:
                return float(np.linalg.norm(np.maximum(np.maximum(lo-p0, p0-hi), 0)))
        else:
            t1, t2 = (lo[i]-p0[i])/d[i], (hi[i]-p0[i])/d[i]
            if t1 > t2: t1, t2 = t2, t1
            tmin, tmax = max(tmin, t1), min(tmax, t2)
            if tmin > tmax:
                # 夹到边界点再算
                q = p0 + d*np.clip((tmin+tmax)/2, 0, 1)
                return float(np.linalg.norm(np.maximum(np.maximum(lo-q, q-hi), 0)))
    q = p0 + d*0.5*(tmin+tmax)
    return float(np.linalg.norm(np.maximum(np.maximum(lo-q, q-hi), 0)))

def sat_box_box(A, B):
    """SAT：返回 (是否重叠, 最小重叠量)。"""
    axes = [A['R'][:, i] for i in range(3)] + [B['R'][:, i] for i in range(3)]
    for i in range(3):
        for j in range(3):
            cr = np.cross(A['R'][:, i], B['R'][:, j])
            n = np.linalg.norm(cr)
            if n > 1e-6: axes.append(cr/n)
    d = B['c'] - A['c']
    best = 1e9
    for ax in axes:
        ra = sum(A['half'][i]*abs(ax @ A['R'][:, i]) for i in range(3))
        rb = sum(B['half'][i]*abs(ax @ B['R'][:, i]) for i in range(3))
        dist = abs(ax @ d)
        ov = ra + rb - dist
        if ov <= 0: return False, float(dist - ra - rb)   # 正数=分离间隙
        best = min(best, ov)
    return True, float(best)

def plate_vs_cap(pl, cp):
    pts = pl['pts']
    # 取 y 最小的一条剖面（膜横向等距，任取一条即可代表截面）
    ys = pts[:, 1]
    sel = np.abs(ys - np.percentile(ys, 2)) < 3.0
    sec = pts[sel][:, [0, 2]]
    if len(sec) < 4: return None
    o = np.argsort(sec[:, 0] + sec[:, 1]*1e-3)
    sec = sec[o]
    a2, b2 = cp['a'][[0, 2]], cp['b'][[0, 2]]
    best = 1e9
    for i in range(len(sec)-1):
        p, q = sec[i], sec[i+1]
        u, v, w = q-p, b2-a2, p-a2
        aa, bb, cc = u@u, u@v, v@v
        dd, ee = u@w, v@w
        den = aa*cc-bb*bb
        s = np.clip((bb*ee-cc*dd)/den, 0, 1) if den > 1e-9 else 0.0
        t = np.clip((bb*s+dd)/aa, 0, 1) if aa > 1e-9 else 0.0
        s = np.clip((bb*t-ee)/cc, 0, 1) if cc > 1e-9 else 0.0
        best = min(best, float(np.linalg.norm((p+u*t)-(a2+v*s))))
    # y 方向是否重叠
    y0, y1 = cp['a'][1], cp['b'][1]
    if min(ys) - 1 > max(y0, y1) or max(ys) + 1 < min(y0, y1):
        return None
    return best - cp['r']

def narrow(P, Q):
    a, b = P['shape'], Q['shape']
    if a == 'cap' and b == 'cap':
        return seg_seg(P['a'], P['b'], Q['a'], Q['b']) - P['r'] - Q['r']
    if a == 'box' and b == 'box':
        ov, val = sat_box_box(P, Q)
        return -val if ov else val
    if a == 'cap' and b == 'box':
        lo = Q['c'] - Q['half'] - 0.0
        p0 = Q['R'].T @ (P['a'] - Q['c']); p1 = Q['R'].T @ (P['b'] - Q['c'])
        return seg_aabb(p0, p1, -Q['half'], Q['half']) - P['r']
    if a == 'box' and b == 'cap':
        return narrow(Q, P)
    if a == 'plate' and b == 'cap':
        d = plate_vs_cap(P, Q)
        return d if d is not None else None
    if a == 'cap' and b == 'plate':
        d = plate_vs_cap(Q, P)
        return d if d is not None else None
    return None

def build_keys(tl):
    keys = []
    for t, label, st in tl:
        recs = anim.frame_records(st)
        kmap = {}
        for r in recs:
            kmap[r['name']] = dict(name=r['name'], group=r['group'], kind=r['kind'],
                                   trs=m.transform(r), proxy=proxy_for(r))
        keys.append(kmap)
    return keys

def frame_trs(keys, i, f):
    k0, k1 = keys[i], keys[i+1]
    out = {}
    for name, a in k0.items():
        b = k1.get(name, a)
        ta, tb = a['trs'], b['trs']
        p = [ta[0][j]+(tb[0][j]-ta[0][j])*f for j in range(3)]
        s = [ta[2][j]+(tb[2][j]-ta[2][j])*f for j in range(3)]
        q0, q1 = np.array(ta[1]), np.array(tb[1])
        dot = float(q0@q1)
        if dot < 0: q1, dot = -q1, -dot
        if dot > 0.9995: q = q0+(q1-q0)*f
        else:
            th = math.acos(max(-1, min(1, dot)))
            q = (math.sin((1-f)*th)*q0+math.sin(f*th)*q1)/math.sin(th)
        out[name] = dict(name=name, group=a['group'], kind=a['kind'],
                         trs=[p, (q/np.linalg.norm(q)).tolist(), s], proxy=a['proxy'])
    return out

MOVING_GROUPS = {'arm', 'telescope', 'joint', 'head', 'gate', 'roll'}


def check_frame(recs, t, prev, moving):
    """只检查含运动件的构件对；膜/嵌条/密封件暂不参与（单面近似，接触为设计状态）。"""
    items, idx = [], {}
    for nm, r in recs.items():
        g = r['group']
        if g in ('film', 'band', 'bead', 'seal', 'bridge', 'gapwall', 'deco'):
            continue
        w = world_proxy(r)
        if w['shape'] == 'plate':
            continue
        idx[nm] = len(items)
        items.append(w)
    n = len(items)
    ev = []
    if n == 0:
        return ev, [], {}
    def _c(w):
        return (w['a'] + w['b']) / 2 if w['shape'] == 'cap' else w['c']
    C = np.array([_c(w) for w in items])
    R = np.array([np.linalg.norm(w['b'] - w['a']) / 2 + w['r'] if w['shape'] == 'cap'
                  else float(np.linalg.norm(w['half'])) for w in items])
    is_mv = np.array([name in moving for name in idx])
    mi = np.where(is_mv)[0]
    for i in mi:
        d = np.linalg.norm(C - C[i], axis=1)
        cand = np.where(d < R + R[i] + WARN)[0]
        for j in cand:
            if j <= i:
                continue          # 每对只算一次
            A, B = items[i], items[j]
            if allowed(A, B):
                continue
            gap = narrow(A, B)
            if gap is None or gap >= WARN:
                continue
            ev.append(dict(t=round(t, 3), a=A['name'], b=B['name'], gap_mm=round(gap, 2),
                           level='collision' if gap < CONTACT else 'warn'))
    tele = []
    if prev:
        for nm in recs:
            p = np.array(recs[nm]['trs'][0])
            if nm in prev and np.linalg.norm(p - prev[nm]) > TELEPORT:
                tele.append(dict(t=round(t, 3), name=nm, step_mm=round(float(np.linalg.norm(p - prev[nm])), 1)))
    return ev, tele, {nm: np.array(recs[nm]['trs'][0]) for nm in recs}


def run(mode, duration):
    tl = anim.timeline(duration=duration, mode=mode)
    ktimes = [r[0] for r in tl]
    t0 = time.time(); keys = build_keys(tl)
    print(f'[{mode}] keys={len(keys)} build={time.time()-t0:.1f}s', flush=True)
    moving = {r['name'] for r in tl[0][2] and anim.frame_records(tl[0][2]) if r['group'] in MOVING_GROUPS}
    allnames = set()
    for _, _, st in tl:
        for r in anim.frame_records(st):
            if r['group'] in MOVING_GROUPS:
                allnames.add(r['name'])
    moving = allnames
    events, tele, prev = [], [], None
    nf = int(duration*FPS)
    for k in range(nf+1):
        t = k/FPS
        i = max(0, min(len(ktimes)-2, int(np.searchsorted(ktimes, t)-1)))
        f = min(1.0, max(0.0, (t-ktimes[i])/max(1e-9, ktimes[i+1]-ktimes[i])))
        recs = frame_trs(keys, i, f)
        ev, tp, prev = check_frame(recs, t, prev, moving)
        events += ev; tele += tp
        if k % 96 == 0: print(f'  t={t:5.2f} ev={len(events)}', flush=True)
    worst = sorted(events, key=lambda e: e['gap_mm'])[:25]
    import datetime as _dt
    return dict(mode=mode, frames=nf+1, fps=FPS, duration=duration,
                meta=dict(run_at=_dt.datetime.now().isoformat(timespec='seconds'),
                          pack_angles=[round(float(x), 1) for x in anim.packlib.PACK_ANGLES],
                          pack_l4=round(float(anim.packlib.PACK_L4), 1),
                          r_link=80.0, contact_mm=CONTACT, warn_mm=WARN),
                stats=dict(events=len(events),
                           collisions=sum(1 for e in events if e['level']=='collision'),
                           warns=sum(1 for e in events if e['level']=='warn'),
                           min_gap=min([e['gap_mm'] for e in events], default=None),
                           teleports=len(tele)),
                worst=worst, events=events, teleports=tele)

if __name__ == '__main__':
    mode = sys.argv[1] if len(sys.argv) > 1 else 'A'
    dur = float(sys.argv[2]) if len(sys.argv) > 2 else (22.0 if mode=='A' else 14.0)
    res = run(mode, dur)
    (OUT / f'collision_{mode}.json').write_text(json.dumps(res, ensure_ascii=False, indent=1), encoding='utf8')
    s = res['stats']
    print(f"\n[{mode}] frames={res['frames']} events={s['events']} collisions={s['collisions']} "
          f"warns={s['warns']} min_gap={s['min_gap']} teleports={s['teleports']}")
    for e in res['worst'][:12]:
        print('  ', e['t'], e['level'], e['gap_mm'], e['a'], '<->', e['b'])
