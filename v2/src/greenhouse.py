"""TIANQIONG whole-greenhouse generator (competition build).

Derived from the reviewed F04 R02 generator (model.py). Reuses its recovered
W01 / M02 / P03 sources and single-sheet membrane logic; adds:
  * BR750 connector for every module gap (R02 only had one, for 2 modules)
  * small-wall (750 mm) infill panels
  * end walls following the single-slope roof polyline, with a door opening
  * per-module motion states, so one hero module can reconfigure while the
    rest of the house stays deployed
  * a frame-delta viewer dataset (base records + per-frame moves) so a
    3-module house stays light enough to render in a browser.

Geometry only: no film FEA, no wind/snow, no safety case. Same limits as R02.
"""
from pathlib import Path
import importlib.util, json, math, sys
import numpy as np
import cadquery as cq

_LOCAL = Path(__file__).resolve().parent.parent / "cad"
R02 = _LOCAL if (_LOCAL / "model.py").exists() else Path(
    r"C:\Users\12753\Documents\Codex\2026-10-23\referenced-chatgpt-conversation-this-is-an\work\github_publish\F04_R02")
OUT = Path(__file__).resolve().parent
OUT.mkdir(parents=True, exist_ok=True)

_spec = importlib.util.spec_from_file_location("tqmodel", R02 / "model.py")
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)

N_MODULES = 3
PITCH = m.P['module_length'] + m.P['small_wall_gap']   # 4250
HERO = 1                                              # 0-based hero module
TOTAL_LEN = N_MODULES * m.P['module_length'] + (N_MODULES - 1) * m.P['small_wall_gap']


# ---------------------------------------------------------------- geometry add-ons
def shift(rec, dy):
    r = dict(rec)
    if r['kind'] in ('mesh', 'box', 'hemisphere'):
        r['c'] = [r['c'][0], r['c'][1] + dy, r['c'][2]]
    elif r['kind'] == 'cylinder':
        r['p'] = [r['p'][0], r['p'][1] + dy, r['p'][2]]
        r['q'] = [r['q'][0], r['q'][1] + dy, r['q'][2]]
    elif r['kind'] == 'sheet':
        r['y0'] = r['y0'] + dy
    else:
        raise ValueError('unknown kind ' + r['kind'])
    return r


def bridge_records(gap_index):
    """Exact BR750 source geometry, one per module gap."""
    y_base = m.P['module_length'] + gap_index * PITCH
    b = cq.Assembly(name='BR750'); cx = 4620
    m.R['cbox'](b, 'bridge_web', (cx, 375, 49), (120, 750, 34), 'beam')
    for i, x in enumerate([cx - 48, cx + 48]):
        m.R['cbox'](b, f'bridge_side_{i}', (x, 375, 96), (18, 750, 72), 'guide')
        m.R['cbox'](b, f'bridge_retainer_{i}', (x + (11 if i == 0 else -11), 375, 140), (38, 750, 13), 'guide')
    m.R['cbox'](b, 'bridge_soil_seal', (4700, 375, 12), (30, 750, 18), 'seal')
    m.R['cbox'](b, 'bridge_front_membrane_clamp', (4485, 375, 24), (24, 750, 21), 'seal')
    for y in [34, 716]:
        m.R['cbox'](b, f'bridge_dock_{y}', (cx, y, 96), (135, 64, 76), 'lock')
    recs = []
    for name, n in b.traverse():
        if isinstance(n.obj, cq.Shape):
            key = m.register('BR750_' + name, n.obj, m.color_tuple(n.color))
            recs.append(m.mesh_record(f'BR{gap_index+1}_{name}', 'rail', key, [0, y_base, 0]))
    return recs


def gap_wall_records(gap_index):
    """750 mm small-wall infill between two large modules."""
    y0 = m.P['module_length'] + gap_index * PITCH
    return [m.box(f'SW{gap_index+1}_panel', 'wall', [-300, y0 + 375, 1400], [600, 750, 2800]),
            m.box(f'SW{gap_index+1}_cap', 'wall', [-300, y0 + 375, 2840], [660, 810, 80]),
            m.box(f'SW{gap_index+1}_sill', 'wall', [-300, y0 + 375, 60], [660, 810, 120])]


def end_wall_record(index, y_start, profile, door=False):
    """Transverse end wall: closed polygon under the single-slope roof polyline."""
    pts = [[-600.0, 0.0], [-600.0, 2800.0]]
    pts += [[float(x), float(z)] for x, z in np.asarray(profile)[:9]]
    pts += [[4800.0, 0.0]]
    shape = cq.Workplane('XZ', origin=(0, y_start, 0)).polyline(pts).close().extrude(-80).val()
    if door:
        cut = cq.Workplane('XY').box(1100, 140, 2100).translate((3700, y_start + 40, 1050)).val()
        shape = shape.cut(cut)
    key = m.register(f'ENDWALL_{index}_{int(door)}', shape, m.COLORS['wall'])
    return m.mesh_record(f'ENDWALL_{index}', 'wall', key)


# ---------------------------------------------------------------- state assembly
ROOF_STANDOFF = m.P['roof_vertical_standoff']
DRUM_CENTER = (-210.0, 2800.0)     # 母墙侧卷筒中心：墙顶内嵌，切点 (-150, 2800) 处膜垂直上引
DRUM_CORE_R = 60.0


def roof_path():
    """棚面剖面折线：墙顶 -> 屋脊 -> 四段关节上方 -> 前沿。"""
    js = m.arm_points(0)[:, [0, 2]].copy()
    js[:, 1] += ROOF_STANDOFF
    path = [[-150.0, 2800.0], [-150.0, float(js[0, 1])]] + js.tolist() +            [[m.P['front_guide_x'], 280.0], [m.P['front_guide_x'], m.P['front_guide_z']]]
    return np.asarray(path, float)


def wall_wound_profile():
    """膜从母墙侧卷筒展开、越过墙顶屋脊、沿棚面到前沿压条。

    余量收在母墙侧卷筒上（不是前沿卷轴）。返回 (剖面点列, 余量, 卷筒中心)。
    """
    path = roof_path()
    surplus = m.P['membrane_stock_length'] - m.path_length(path)
    center = np.asarray(DRUM_CENTER, float)
    spiral = m.spiral_exact(path[0], center, surplus, DRUM_CORE_R, m.P['coil_pitch'])
    profile = np.vstack([spiral[::-1], path[1:]])   # 首点与螺旋末端重合，去掉重复
    return profile, surplus, center


def module_membrane_records(module_index):
    """每个大型模块一张横截面连续的膜；不按四段连杆切分。"""
    dy = module_index * gh_pitch()
    width = m.P['module_length'] + 2 * m.P['side_margin']
    y0 = dy - m.P['side_margin']
    profile, surplus, center = wall_wound_profile()
    tag = f'M{module_index+1}'
    recs = [m.sheet_record(f'{tag}_FILM', 'film', profile, y0, width, m.P['membrane_thickness'])]

    path = roof_path()
    for k in (1, 2, 3):
        seg = m.band_segment(path, k, m.P['band_width'] / 2)
        seg = m.offset_polyline(seg, m.P['membrane_thickness'] / 2 + m.P['band_thickness'] / 2)
        recs.append(m.sheet_record(f'{tag}_BAND{k}', 'band', seg, y0, width, m.P['band_thickness']))
        x, z = path[k]
        bz = z - m.P['bead_drop']
        recs.append(m.cylinder(f'{tag}_BEAD{k}', 'band', [x, y0, bz], [x, y0 + width, bz], m.P['bead_radius']))
        recs.append(m.box(f'{tag}_WEB{k}', 'band', [x, y0 + width / 2, (bz + 6 + z - .5) / 2],
                          [m.P['web_thickness'], width, z - .5 - bz - 6]))

    # 前沿压条（可释放，膜的前端夹在这里）
    recs.append(m.box(f'{tag}_CLAMP', 'head', [4485.0, y0 + width / 2, 24.0], [24, width, 21]))
    # 母墙侧卷筒 + 壳体
    recs.append(m.cylinder(f'{tag}_DRUM', 'roll', [center[0], y0, center[1]], [center[0], y0 + width, center[1]], 52.0))
    recs.append(m.box(f'{tag}_DRUM_HOUSING', 'roll', [center[0], y0 + width / 2, center[1] + 40],
                      [150, width, 150]))
    return recs


def seal_records(gap_index):
    """模块之间可拆卸的柔性密封搭接：一块窄膜 + 三条跨缝嵌条 + 压条。"""
    dy = m.P['module_length'] + gap_index * gh_pitch()
    overlap = 100.0
    y0 = dy - overlap
    width = m.P['small_wall_gap'] + 2 * overlap
    path = roof_path()
    tag = f'SEAL{gap_index+1}'
    recs = [m.sheet_record(tag, 'film', path, y0, width, m.P['membrane_thickness'])]
    for k in (1, 2, 3):
        x, z = path[k]
        bz = z - m.P['bead_drop']
        recs.append(m.cylinder(f'{tag}_BEAD{k}', 'band', [x, y0, bz], [x, y0 + width, bz], m.P['bead_radius']))
    recs.append(m.box(f'{tag}_LAP', 'wall', [4485.0, y0 + width / 2, 30.0], [26, width, 12]))
    return recs


def gh_pitch():
    return m.P['module_length'] + m.P['small_wall_gap']


def house_records(states, compensate=0.0):
    """每模块独立覆盖单元 + 模块间密封搭接；结构件按模块状态生成。"""
    recs = []
    for k, st in enumerate(states):
        part, _ = m.build_state(modules=1, **st)
        for r in part:
            if r['group'] not in ('wall', 'rail', 'arm', 'telescope', 'joint', 'head', 'gate', 'magnet'):
                continue
            if r['name'] == 'F04_W01_root_flashing_clamp':
                continue
            r2 = shift(r, k * gh_pitch())
            r2['name'] = f'M{k+1}_' + r['name']
            recs.append(r2)

    for k in range(N_MODULES):
        recs += module_membrane_records(k)
    for g in range(N_MODULES - 1):
        recs += seal_records(g)
        recs += bridge_records(g)
        recs += gap_wall_records(g)

    profile = roof_path()
    recs.append(end_wall_record(1, -80.0, profile, door=True))
    recs.append(end_wall_record(2, float(TOTAL_LEN), profile, door=False))
    return recs


def module_states(compensate=0.0, gate=0.0, drop=0.0, retract=0.0, lower=0.0):
    out = []
    for k in range(N_MODULES):
        st = dict(compensate=compensate)
        if k == HERO:
            st.update(gate=gate, drop=drop, retract=retract, lower=lower)
        out.append(st)
    return out


def smooth(t):
    t = np.clip(t, 0, 1)
    return float(6 * t ** 5 - 15 * t ** 4 + 10 * t ** 3)


def timeline(duration=24.0, steps=97):
    """0-6 compensate, 6-10 hero gates, 10-13 hero heads drop, 13-18 retract, 18-24 lowering."""
    result = []
    for t in np.linspace(0, duration, steps):
        state = dict(compensate=smooth(t / 6),
                     gate=smooth((t - 6) / 4),
                     drop=smooth((t - 10) / 3),
                     retract=smooth((t - 13) / 5),
                     lower=smooth((t - 18) / 6))
        if t <= 6:
            label = '整棚同步补位'
        elif t <= 10:
            label = '英雄模块：机械防脱门开启'
        elif t <= 13:
            label = '承托头下降、嵌条释放'
        elif t <= 18:
            label = '第四段回缩'
        else:
            label = '根俯仰下降（脱离候选）'
        result.append((float(t), label, state))
    return result


def write_viewer_data(frames, base_records, path):
    catalog = {}
    for data in [base_records] + [f['records'] for f in frames]:
        for r in data:
            md, key = m.unit_mesh(r)
            if key not in catalog:
                catalog[key] = dict(md, color=m.color_for(r))
            r['viewerMesh'] = key
            r['trs'] = m.transform(r)
            r['color'] = m.color_for(r)

    base = [dict(name=r['name'], group=r['group'], mesh=r['viewerMesh'], trs=r['trs']) for r in base_records]
    base_trs = {r['name']: r['trs'] for r in base_records}
    out_frames = []
    for f in frames:
        moves = {}
        for r in f['records']:
            a, b = base_trs[r['name']], r['trs']
            if any(abs(x - y) > 1e-6 for x, y in zip(a[0] + a[1] + a[2], b[0] + b[1] + b[2])):
                moves[r['name']] = b
        out_frames.append(dict(t=f['t'], label=f['label'], moves=moves))

    payload = dict(meta=dict(modules=N_MODULES, hero=HERO, total_length_mm=TOTAL_LEN,
                             membrane_width_mm=N_MODULES * 3500 + (N_MODULES - 1) * 750 + 100,
                             interfaces_per_module=18),
                   catalog=catalog, base=base, frames=out_frames)
    path.write_text('window.TQ_HOUSE=' + json.dumps(payload, separators=(',', ':'), ensure_ascii=False) + ';',
                    encoding='utf8')
    return payload


def main():
    frames = []
    base = None
    for t, label, st in timeline():
        recs = house_records(module_states(**st), compensate=st['compensate'])
        if base is None:
            base = recs
        frames.append(dict(t=t, label=label, records=recs))
        print(f'frame {t:5.1f}s records={len(recs)}', flush=True)

    payload = write_viewer_data(frames, base, OUT / 'house_data.js')
    size = (OUT / 'house_data.js').stat().st_size / 1e6
    print(f'\nbase records     : {len(payload["base"])}')
    print(f'catalog meshes   : {len(payload["catalog"])}')
    print(f'frames           : {len(payload["frames"])}')
    print(f'house_data.js    : {size:.1f} MB')
    print(f'total length     : {TOTAL_LEN} mm, membrane width {payload["meta"]["membrane_width_mm"]} mm')


if __name__ == '__main__':
    main()
