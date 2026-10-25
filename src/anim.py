"""最小可验证的膜卷绕 + 修正后的作业切换时序（调试动画用）。

状态参数
  wound     0..7200 mm   已卷入母墙侧卷筒的膜长（膜面从前沿向母墙退）
  gate/drop 0..1         2/5 号臂承托闸门与承托头
  release   0/1           末端与前沿滑块的快脱状态（闭链约束解除）
  retract   0..1         第四段回缩
  yaw_deg   0..35        根部回转（作业臂）
  lower     0..1         根部俯仰下压
  compensate 0..1        双轨同步补位

时序（26 s）：
  0–3   承托头解锁：闸门开、承托头下降
  3–5   前沿压条释放（膜前端脱开，准备收膜）
  5–15  收膜：wound 0→7200（卷径随层数增长）
  15–18 双轨同步补位
  18–20 末端快脱：release 0→1，滑块留在锚轨内
  20–23 第四段回缩
  23–26 根部回转 + 俯仰下压，进入作业区
"""
from pathlib import Path
import importlib.util, json, math
import numpy as np
import packlib

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'

spec = importlib.util.spec_from_file_location('gh', HERE / 'greenhouse.py')
gh = importlib.util.module_from_spec(spec)
spec.loader.exec_module(gh)
m = gh.m

N = gh.N_MODULES
PITCH = gh.gh_pitch()
TOTAL = gh.TOTAL_LEN
CORE_R = gh.DRUM_CORE_R
EXIT = np.array([-150.0, 2800.0])          # 膜离开卷筒的切点
CENTER = np.array(gh.DRUM_CENTER)          # (-210, 2800)
THICK = m.P['membrane_thickness']


def roll_radius(L):
    """卷绕 L mm 后的外半径（按等面积层叠）。"""
    return math.sqrt(CORE_R ** 2 + L * THICK / math.pi)


def roll_turns(L):
    r = roll_radius(L)
    return L / (2 * math.pi * max(CORE_R, (CORE_R + r) / 2))


def roll_profile(L):
    """从切点开始、向内卷的螺旋；总弧长 ≈ L，外半径随 L 增长。"""
    if L <= 1.0:
        return np.array([EXIT])
    r_out = roll_radius(L)
    theta_max = (r_out - CORE_R) * 2 * math.pi / THICK
    n = min(220, max(48, int(theta_max / 0.30)))
    theta0 = math.atan2(EXIT[1] - CENTER[1], EXIT[0] - CENTER[0])
    pts = []
    for i in range(n + 1):
        th = theta_max * (1 - i / n)
        r = CORE_R + THICK * th / (2 * math.pi)
        pts.append([CENTER[0] + r * math.cos(theta0 + th),
                    CENTER[1] + r * math.sin(theta0 + th)])
    return np.array(pts)


def roof_prefix(length):
    """棚面折线上从母墙起、长度不超过 length 的一段。"""
    path = gh.roof_path()
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    cum = np.r_[0, np.cumsum(seg)]
    if length >= cum[-1]:
        return path
    k = int(np.searchsorted(cum, length) - 1)
    k = max(0, min(k, len(path) - 2))
    t = (length - cum[k]) / max(seg[k], 1e-9)
    last = path[k] + (path[k + 1] - path[k]) * t
    return np.vstack([path[:k + 1], last])


_MEM_CACHE = {}


def membrane_records(module_index, wound):
    key = (module_index, round(wound, 1))
    if key in _MEM_CACHE:
        return _MEM_CACHE[key]
    recs = _membrane_records(module_index, wound)
    _MEM_CACHE[key] = recs
    return recs


def _membrane_records(module_index, wound):
    """每模块一张膜：卷绕 L 后，棚面上只剩余下的一段。"""
    dy = module_index * PITCH
    width = m.P['module_length'] + 2 * m.P['side_margin']
    y0 = dy - m.P['side_margin']
    tag = f'M{module_index+1}'
    path = gh.roof_path()
    total = m.path_length(path)
    deployed = max(0.0, m.P['membrane_stock_length'] - wound)   # 料长 - 已卷 = 棚面剩余
    body = roof_prefix(deployed)
    L = min(wound, m.P['membrane_stock_length'])
    recs = []
    if deployed > 10.0:
        # 螺旋由外向内生成，拼接时反向，使剖面从卷芯走到切点再沿棚面
        prof = np.vstack([roll_profile(L)[::-1], body[1:]])
        recs.append(m.sheet_record(f'{tag}_FILM', 'film', prof, y0, width, THICK))

    # 三条嵌条：只有仍被膜覆盖的那一段存在；收进卷筒的用卷筒上的短段近似
    cum = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    for k in (1, 2, 3):
        if cum[k] > deployed + 1.0:
            continue
        seg = m.band_segment(body, k, m.P['band_width'] / 2)
        seg = m.offset_polyline(seg, THICK / 2 + m.P['band_thickness'] / 2)
        recs.append(m.sheet_record(f'{tag}_BAND{k}', 'band', seg, y0, width, m.P['band_thickness']))
        x, z = body[k]
        bz = z - m.P['bead_drop']
        recs.append(m.cylinder(f'{tag}_BEAD{k}', 'band', [x, y0, bz], [x, y0 + width, bz], m.P['bead_radius']))
        recs.append(m.box(f'{tag}_WEB{k}', 'band', [x, y0 + width / 2, (bz + 6 + z - .5) / 2],
                          [m.P['web_thickness'], width, z - .5 - bz - 6]))
    recs.append(m.box(f'{tag}_CLAMP', 'head', [4485.0, y0 + width / 2, 24.0], [24, width, 21]))
    recs.append(m.cylinder(f'{tag}_DRUM', 'roll', [CENTER[0], y0, CENTER[1]],
                           [CENTER[0], y0 + width, CENTER[1]], max(52.0, roll_radius(L))))
    recs.append(m.box(f'{tag}_DRUM_HOUSING', 'roll', [CENTER[0], y0 + width / 2, CENTER[1] + 40],
                      [150, width, 150]))
    return recs


def rotate_about_z(rec, center_y, deg):
    """把一条记录绕根部竖轴旋转（用于根部回转）。"""
    if abs(deg) < 1e-9:
        return rec
    r = dict(rec)
    a = math.radians(deg)
    ca, sa = math.cos(a), math.sin(a)
    cy = center_y

    def rot(p):
        x, y, z = p
        return [x * ca - (y - cy) * sa, cy + x * sa + (y - cy) * ca, z]

    if r['kind'] == 'cylinder':
        r['p'], r['q'] = rot(r['p']), rot(r['q'])
    elif r['kind'] in ('box', 'hemisphere'):
        r['c'] = rot(r['c'])
    elif r['kind'] == 'mesh':
        r['c'] = rot(r['c'])
    return r


def _apply_pack(recs, k, state):
    """把英雄模块的 2/5 号臂换成 pack 轨迹生成的连杆，承托头随臂端平移。"""
    u = float(state.get('pack', 0.0))
    if u <= 1e-6 or k != HERO:
        return recs
    out = []
    for idx in (1, 4):
        y_abs = k * PITCH + float(m.Y0[idx])
        a0 = np.degrees(m.A0)
        ang, L4 = packlib.blend(a0, u)
        new_recs, new_pts = packlib.hero_arm(f'M{k+1}_M1_', idx, y_abs, ang, L4, m)
        old_pts = m.arm_points(y_abs, 0.0, 1600.0 - 600.0 * float(state.get('retract', 0.0)))
        shift = new_pts[4] - old_pts[4]
        tag = f'_A{idx+1:02d}_'
        keep, moved = [], []
        for r in recs:
            if tag not in r['name']:
                keep.append(r); continue
            if r['group'] in ('arm', 'telescope', 'joint'):
                continue                      # 旧连杆/关节整段丢弃
            if r['group'] in ('head', 'gate', 'magnet', 'lock'):
                r2 = dict(r)
                for key in ('p', 'q', 'c'):
                    if key in r2:
                        r2[key] = [r2[key][0] + shift[0], r2[key][1] + shift[1], r2[key][2] + shift[2]]
                moved.append(r2); continue
            keep.append(r)
        recs = keep + moved + new_recs
    return recs


def frame_records(state):
    """一帧的完整构件集合。"""
    comp = state['compensate']
    recs = []
    for k in range(N):
        st = dict(compensate=comp,
                  gate=state['gate'] if k == HERO else 0.0,
                  drop=state['drop'] if k == HERO else 0.0,
                  retract=state['retract'] if k == HERO else 0.0,
                  lower=state['lower'] if k == HERO else 0.0)
        part, _ = m.build_state(modules=1, **st)
        for r in part:
            if r['group'] not in ('wall', 'rail', 'arm', 'telescope', 'joint', 'head', 'gate', 'magnet'):
                continue
            r2 = gh.shift(r, k * PITCH)
            r2['name'] = f'M{k+1}_' + r['name']
            if k == HERO and ('_A02_' in r2['name'] or '_A05_' in r2['name']):
                idx = 1 if '_A02_' in r2['name'] else 4
                root_y = k * PITCH + float(m.Y0[idx])
                r2 = rotate_about_z(r2, root_y, state['yaw'])
            recs.append(r2)
        recs += membrane_records(k, state['wound'])
    for g in range(N - 1):
        recs += gh.seal_records(g)
        recs += gh.bridge_records(g)
        recs += gh.gap_wall_records(g)
    profile = gh.roof_path()
    recs = _apply_pack(recs, HERO, state)
    recs.append(gh.end_wall_record(1, -80.0, profile, door=True))
    recs.append(gh.end_wall_record(2, float(TOTAL), profile, door=False))
    return recs


HERO = gh.HERO
SURPLUS = m.P['membrane_stock_length'] - m.path_length(gh.roof_path())   # 602 mm，部署态已卷在筒上


def smooth(u):
    u = float(np.clip(u, 0, 1))
    return u * u * (3 - 2 * u)


def timeline(duration=None, step=0.25, mode='A'):
    """mode A = 棚膜保持展开的农业作业切换；B = 整棚收纳；M = 旧混合时序（仅留档）。"""
    SUR = SURPLUS
    STOCK = m.P['membrane_stock_length']
    rows, t = [], 0.0
    if mode == 'M':
        duration = 26.0 if duration is None else duration
    elif mode == 'A':
        duration = 26.0 if duration is None else duration
    else:
        duration = 14.0 if duration is None else duration

    while t <= duration + 1e-9:
        st = dict(wound=SUR, gate=0.0, drop=0.0, release=0, retract=0.0,
                  yaw=0.0, lower=0.0, compensate=0.0, pack=0.0)
        if mode == 'A':
            if t < 3:
                st['gate'] = smooth(t / 3); st['drop'] = smooth(max(0.0, t - 1.5) / 1.5)
                label = 'A｜承托头解锁：闸门开启、承托头下降（膜面保持展开）'
            elif t < 5:
                st.update(gate=1.0, drop=1.0)
                label = 'A｜支撑确认：其余四臂接管棚面荷载'
            elif t < 8:
                st.update(gate=1.0, drop=1.0, compensate=smooth((t - 5) / 3))
                label = 'A｜双轨同步补位（膜面仍展开）'
            elif t < 10:
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1)
                label = 'A｜末端与前沿滑块快脱（滑块留在轨内）'
            elif t < 13:
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1,
                          retract=smooth((t - 10) / 3))
                label = 'A｜第四段回缩（脱轨后进行）'
            elif t < 16:
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1, retract=1.0,
                          pack=smooth((t - 13) / 3))
                label = 'A｜膜内侧紧凑折叠（专用姿态，非运输 Z 折）'
            elif t < 18:
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1, retract=1.0,
                          pack=1.0, yaw=35.0 * smooth((t - 16) / 2))
                label = 'A｜紧凑态根部回转（扫掠区内无邻臂）'
            elif t < 21:
                u = smooth((t - 18) / 3)
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1, retract=1.0,
                          pack=1.0 - 0.55 * u, yaw=35.0, lower=u)
                label = 'A｜展开进入作业区（占位姿态，待作业 IK）'
            elif t < 24:
                u = smooth((t - 21) / 3)
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=1, retract=1.0,
                          pack=1.0 - 0.55 * (1 - u), yaw=35.0 * (1 - u), lower=1 - u)
                label = 'A｜返回：先折回紧凑态再回转'
            else:
                u = smooth((t - 24) / 2)
                st.update(gate=1.0, drop=1.0, compensate=1.0, release=0,
                          retract=1.0, pack=1.0 * (1 - u), yaw=0.0, lower=0.0)
                label = 'A｜解除紧折叠、恢复承托与部署位'
        elif mode == 'B':
            if t < 12:
                st['wound'] = SUR + (STOCK - SUR) * smooth(t / 12)
                label = 'B｜母墙侧收膜：卷径随层数增长'
            else:
                st['wound'] = STOCK
                label = 'B｜膜已收妥（地锚回缩、搬轨、折叠、轮组待接入）'
        else:  # 旧混合时序，仅留档
            if t < 3:
                st['gate'] = smooth(t / 3); st['drop'] = smooth((t - 1.5) / 1.5)
                label = '承托头解锁：闸门开启、承托头下降'
            elif t < 5:
                label = '前沿压条释放（膜前端脱开，准备收膜）'
            elif t < 15:
                st['gate'] = 1.0; st['drop'] = 1.0
                st['wound'] = SUR + (STOCK - SUR) * smooth((t - 5) / 10)
                label = '母墙侧收膜：卷径随层数增长'
            elif t < 18:
                st.update(gate=1.0, drop=1.0, wound=STOCK, compensate=smooth((t - 15) / 3))
                label = '双轨同步补位（根座 + 前沿滑块）'
            elif t < 20:
                st.update(gate=1.0, drop=1.0, wound=STOCK, compensate=1.0, release=1)
                label = '末端与前沿滑块快脱（滑块留在锚轨内）'
            elif t < 23:
                st.update(gate=1.0, drop=1.0, wound=STOCK, compensate=1.0,
                          release=1, retract=smooth((t - 20) / 3))
                label = '第四段回缩（脱轨后进行）'
            else:
                u = smooth((t - 23) / 3)
                st.update(gate=1.0, drop=1.0, wound=STOCK, compensate=1.0,
                          release=1, retract=1.0, yaw=35.0 * u, lower=u)
                label = '根部回转 + 四关节俯仰协调进入作业区'
        rows.append((float(t), label, st))
        t += step
    return rows


def main(mode='M'):
    frames = []
    base = None
    for t, label, st in timeline(mode=mode):
        recs = frame_records(st)
        if base is None:
            base = recs
        frames.append(dict(t=t, label=label, records=recs, state=st))
    payload = gh.write_viewer_data(frames, base, OUT / 'house_data_anim.js')
    print('frames', len(frames), 'base', len(payload['base']), 'catalog', len(payload['catalog']),
          'size MB', round((OUT / 'house_data_anim.js').stat().st_size / 1e6, 2))
    # 轨迹数据（供审计与后续合并时间轴）
    traj = [dict(t=f['t'], label=f['label'], state=f['state'],
                 joints=dict(wound=round(f['state']['wound'], 1),
                             drum_deg=round(roll_turns(f['state']['wound']) * 360, 1),
                             drum_dia_mm=round(2 * roll_radius(f['state']['wound']), 2)))
            for f in frames]
    (OUT / 'anim_trajectory.json').write_text(json.dumps(traj, ensure_ascii=False, indent=1), encoding='utf8')
    print('trajectory written')


if __name__ == '__main__':
    main()
