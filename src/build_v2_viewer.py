"""生成 house_data_v2.js：静态场景一次建成，1425 帧只写 A02/A05 的 TRS 增量。

数据源：out/modeA_v2_timeline.json（全局 24fps 序列 + 精确终点帧）。
六臂：A02/A05 走轨迹（补位后 Y），其余四臂与棚膜、锚轨、母墙保持静态。
"""
import importlib.util, json, hashlib
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
STUDIO = HERE.parent / 'studio'
anim = importlib.import_module('anim')
packlib = importlib.import_module('packlib')
m = anim.m
pl = importlib.import_module('planner')

HERO = anim.HERO
Y_COMP = [float(x) for x in pl.Y_WORK]


def base_records():
    st = dict(wound=anim.SURPLUS, gate=0.0, drop=0.0, release=0, retract=0.0,
              yaw=0.0, lower=0.0, compensate=0.0, pack=0.0)
    return anim.frame_records(st)


def hero_records(idx, q):
    """按时间轴关节值生成该臂的全部记录（补位后 Y）。"""
    y_abs = HERO * anim.PITCH + Y_COMP[idx]
    recs, _pts = packlib.hero_arm(f'M{HERO+1}_M1_', idx, y_abs, np.asarray(q[:4], float), float(q[4]), m)
    return recs


def main():
    tl = json.loads((OUT / 'modeA_v2_timeline.json').read_text(encoding='utf8'))
    frames = tl['frame_records']
    base = base_records()
    trs0 = {}
    for r in base:
        r['trs'] = m.transform(r)
        md, key = m.unit_mesh(r)
        r['vm'] = key
        trs0[r['name']] = r['trs']
        r['color'] = m.color_for(r)

    # 英雄臂的静态基准记录也要进 base（否则浏览器里没有对象可动）
    hero_base = {}
    for idx in (1, 4):
        q0 = frames[0]['q']
        for r in hero_records(idx, q0):
            r['trs'] = m.transform(r); md, key = m.unit_mesh(r)
            r['vm'] = key; r['color'] = m.color_for(r)
            hero_base[r['name']] = r
    # 用英雄臂记录替换 base 中同名项
    names_hero = set(hero_base)
    base = [r for r in base if r['name'] not in names_hero] + list(hero_base.values())

    catalog, out_base = {}, []
    for r in base:
        if r['vm'] not in catalog:
            md, _ = m.unit_mesh(r)
            catalog[r['vm']] = dict(md, color=r['color'])
        out_base.append(dict(name=r['name'], group=r['group'], mesh=r['vm'], trs=r['trs']))

    out_frames = []
    for f in frames:
        q = f['q']
        moves = {}
        for idx in (1, 4):
            y_abs = HERO * anim.PITCH + Y_COMP[idx]
            new = hero_records(idx, q)
            # 承托头随臂端平移
            tag = f'_A{idx+1:02d}_'
            old_pts = m.arm_points(y_abs, 0.0, 1600.0)
            new_pts = np.asarray(packlib.hero_arm(f'M{HERO+1}_M1_', idx, y_abs,
                                                 np.asarray(q[:4], float), float(q[4]), m)[1])
            shift = new_pts[4] - old_pts[4]
            for r in new:
                tr = m.transform(r)
                nm = r['name']
                if nm in trs0:
                    a, b = trs0[nm], tr
                    if any(abs(x - y) > 1e-6 for x, y in zip(a[0] + a[1] + a[2], b[0] + b[1] + b[2])):
                        moves[nm] = tr
        for nm, tr in list(trs0.items()):
            if HERO == 1 and (f'_A02_' in nm or f'_A05_' in nm) and nm.startswith('M2_'):
                pass
        # 承托头：以基准 TRS 平移
        for nm, trs in trs0.items():
            if not nm.startswith('M2_') or not (f'_A02_' in nm or f'_A05_' in nm):
                continue
            if any(k.startswith(nm.split('_head')[0]) and '_head' in k for k in moves):
                continue
        out_frames.append(dict(t=f['video_time'], vt=f['video_time'], st=f['source_time'],
                               seg=f['seg'], label=f"{f['seg']} · t={f['video_time']:.2f}s",
                               moves=moves))

    payload = dict(meta=dict(source='modeA_v2_timeline.json', status=tl['status'],
                             modules=anim.N, hero=HERO,
                             total_length_mm=anim.TOTAL,
                             length_mm=anim.TOTAL,
                             width_mm=anim.N * 3500 + (anim.N - 1) * 750 + 100,
                             membrane_width_mm=anim.N * 3500 + (anim.N - 1) * 750 + 100,
                             interfaces_per_module=18,
                             fps=tl['fps'], total_s=tl['total_s'], n_frames=len(out_frames),
                             y_state='compensated', y_work=Y_COMP,
                             exact_endpoint=tl['exact_endpoint']['t']),
                   catalog=catalog, base=out_base, frames=out_frames)
    p = STUDIO / 'house_data_v2.js'
    p.write_text('window.TQ_HOUSE=' + json.dumps(payload, separators=(',', ':'), ensure_ascii=False) + ';',
                 encoding='utf8')
    print('frames', len(out_frames), 'base', len(out_base), 'catalog', len(catalog),
          'size MB', round(p.stat().st_size / 1e6, 2))
    print('首帧 moves', len(out_frames[0]['moves']), ' 末帧 moves', len(out_frames[-1]['moves']))
    (OUT / 'viewer_v2_manifest.json').write_text(json.dumps(dict(
        file=str(p), sha256=hashlib.sha256(p.read_bytes()).hexdigest(),
        n_frames=len(out_frames), fps=tl['fps'], total_s=tl['total_s'],
        source_sha=hashlib.sha256((OUT / 'modeA_v2_timeline.json').read_bytes()).hexdigest()),
        ensure_ascii=False, indent=1), encoding='utf8')


if __name__ == '__main__':
    main()
