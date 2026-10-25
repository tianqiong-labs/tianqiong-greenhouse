"""Assembly V2 整机干涉矩阵（只读分析，不改几何）。

方法：包围盒粗筛 → 真实 B-Rep 体积交集。
对象：18 套接口 × {前膜压条, 上压条1/2, 锚轨腹板, 同臂滑块}
另做：跑道槽后的横向挡止回归（LOCKED 时柱 +X 20）。
"""
import cadquery as cq, importlib.util, json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
sp = importlib.util.spec_from_file_location('anim', str(HERE / 'anim.py'))
anim = importlib.util.module_from_spec(sp); sp.loader.exec_module(anim)
m = anim.m

PITCH = 4250.0
Y_WORK = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
TIP_X = 4500.0
FL_R, FL_Z0, FL_Z1, FL_BORE = 20.0, 76.0, 82.0, 16.0
CLAMP_Z0, CLAMP_Z1, CLAMP_RI, CLAMP_RO = 90.0, 110.0, 15.5, 32.0


def ball():
    hemi = cq.Workplane("XY").sphere(67).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=58).circle(24).extrude(18)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=14).circle(16).extrude(62))
    b = b.cut(cq.Workplane("XY").box(120, 32, 66).translate((44, 0, 45)))
    return b


def flange_c():
    f = (cq.Workplane("XY").workplane(offset=FL_Z0).circle(FL_R).extrude(FL_Z1 - FL_Z0)
         .cut(cq.Workplane("XY").workplane(offset=FL_Z0 - 1).circle(FL_BORE).extrude(FL_Z1 - FL_Z0 + 2)))
    return f.cut(cq.Workplane("XY").box(80, 32, FL_Z1 - FL_Z0 + 4).translate((40, 0, 79)))


def rail_arm():
    return cq.Workplane("XY").box(14, 140, 6).translate((-23, 0, 85))


def half(sign, s=0.0):
    ring = (cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RO).extrude(CLAMP_Z1 - CLAMP_Z0)
            .cut(cq.Workplane("XY").workplane(offset=CLAMP_Z0).circle(CLAMP_RI).extrude(CLAMP_Z1 - CLAMP_Z0)))
    h = ring.intersect(cq.Workplane("XY").workplane(offset=CLAMP_Z0).rect(200, 40)
                       .extrude(CLAMP_Z1 - CLAMP_Z0).translate((0, sign * 20, 0)))
    return h.translate((0, sign * s, 0))


def column():
    c = cq.Workplane("XY").workplane(offset=14).circle(15).extrude(180)
    g = (cq.Workplane("XY").workplane(offset=38).circle(15).extrude(12)
         .cut(cq.Workplane("XY").workplane(offset=38).circle(11).extrude(12)))
    return c.cut(g)


def arm_parts(s=0.0):
    return {'BALL': ball(), 'FLANGE': flange_c(), 'ARM_RAIL': rail_arm(),
            'CLAMP_R': half(1, s), 'CLAMP_L': half(-1, s)}


def _sh(x):
    return x.val() if hasattr(x, 'val') else x


def bbox_of(shape):
    b = _sh(shape).BoundingBox()
    return [b.xmin, b.ymin, b.zmin], [b.xmax, b.ymax, b.zmax]


def overlap(a, b, pad=0.0):
    return all(a[0][i] - pad <= b[1][i] and b[0][i] - pad <= a[1][i] for i in range(3))


def vol(a, b):
    try:
        return float(_sh(a).intersect(_sh(b)).Volume())
    except Exception:
        return 0.0


if __name__ == '__main__':
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
              yaw=0.0, lower=0.0, compensate=1.0, pack=0.0)
    recs = anim.frame_records(st)
    targets = {}
    keys = ('front_membrane_clamp', 'upper_retainer_1', 'upper_retainer_2',
            'base_web', 'slider_body')
    for r in recs:
        if r['group'] != 'rail':
            continue
        if 'wrist_' in r['name'] or r['name'].endswith('_tip'):
            continue
        if any(k in r['name'] for k in keys):
            targets[r['name']] = r
    print(f'锚轨/压条候选零件 {len(targets)} 个')

    tshapes = {n: m.shape_for(r) for n, r in targets.items()}
    tboxes = {n: bbox_of(s) for n, s in tshapes.items()}

    rows = []
    for k in range(3):
        for i in range(6):
            y = Y_WORK[i] + k * PITCH
            parts = arm_parts(0.0)
            for pn, ps in parts.items():
                pb = bbox_of(ps.translate((TIP_X, y, 0)))
                for tn, ts in tshapes.items():
                    if not overlap(pb, tboxes[tn], pad=60):
                        continue
                    v = vol(ps.translate((TIP_X, y, 0)), ts)
                    if v > 1e-3:
                        rows.append(dict(module=k + 1, arm=f'A{i+1:02d}', part=pn,
                                         target=tn, vol=round(v, 2), y=round(y, 1)))
    print(f'\n静态矩阵命中 {len(rows)} 条')
    agg = {}
    for r in rows:
        agg.setdefault(r['part'], []).append(r)
    for p, rs in agg.items():
        tot = sum(x['vol'] for x in rs)
        tg = sorted({x['target'] for x in rs})
        print(f"  {p:10s} {len(rs):3d} 处  合计 {tot:10.1f} mm3  涉及 {tg}")

    # 跑道槽后横向挡止回归
    col = column()
    lock_hit_ball = vol(ball(), col.translate((20, 0, 0)))
    lock_hit_fl = vol(flange_c(), col.translate((20, 0, 0)))
    print(f'\n横向挡止回归：柱+X20 vs 球头 {lock_hit_ball:.1f} mm3（跑道槽已开口，预期 0）')
    print(f'              柱+X20 vs C 形法兰 {lock_hit_fl:.1f} mm3（应 >0 = 挡止来自法兰）')

    (OUT / 'v2_interference_matrix.json').write_text(json.dumps(dict(
        hits=rows, agg={p: dict(n=len(v), vol=round(sum(x['vol'] for x in v), 1),
                                targets=sorted({x['target'] for x in v})) for p, v in agg.items()},
        lock_regression=dict(ball=round(lock_hit_ball,1), flange=round(lock_hit_fl,1), clamp=round(lock_hit_cl,1))),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/v2_interference_matrix.json')
