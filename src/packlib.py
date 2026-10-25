"""模式 A 专用折臂：把 2/5 号臂从部署姿态折进"膜内侧紧凑区"，再回转。

轨迹来自 packA_search.json（数值搜索结果），不是手摆：
    角度 [-94.1, 41.8, -138.8, 46.0]，第四段 1366 mm
验证结论（packA_path.py）：部署→紧凑全程膜余量 ≥234.6 mm，邻臂 3D 间隙 ≥400 mm。
"""
import json, math
from pathlib import Path
import numpy as np

HERE = Path(__file__).resolve().parent


def load_pack(path=None):
    p = Path(path) if path else HERE / 'out' / 'packA_search.json'
    if p.exists():
        b = json.loads(p.read_text(encoding='utf8'))['best']
        return np.array(b['angles'], float), float(b['L4'])
    return np.array([-94.1, 41.8, -138.8, 46.0]), 1366.2


PACK_ANGLES, PACK_L4 = load_pack()


def blend(a0_deg, u):
    """u=0 部署，u=1 紧凑。"""
    a = a0_deg + (PACK_ANGLES - a0_deg) * u
    return a, 1600.0 + (PACK_L4 - 1600.0) * u


def hero_arm(prefix, idx, y_abs, angles_deg, L4, m):
    """按给定绝对段角生成该臂的连杆/关节/伸缩记录（与原始模型同一套半径）。"""
    pts = m.M['fk'](y_abs / 1000.0, np.radians(angles_deg), L4 / 1000.0) * 1000.0
    ident = f'{prefix}A{idx+1:02d}'
    recs = []
    for j in range(3):
        recs.append(m.cylinder(f'{ident}_link{j+1}', 'arm', pts[j], pts[j+1], 49))
    vec = (pts[4] - pts[3]) / L4
    recs.append(m.cylinder(f'{ident}_tel_outer', 'telescope', pts[3], pts[3] + 980 * vec, 62))
    recs.append(m.cylinder(f'{ident}_tel_inner', 'arm', pts[3] + 780 * vec, pts[4], 38))
    for j in range(4):
        recs.append(m.hemisphere(f'{ident}_joint{j+1}', pts[j], 73))
    recs.append(m.hemisphere(f'{ident}_tip', pts[4], 67))
    return recs, pts
