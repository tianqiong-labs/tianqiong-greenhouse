"""L2 接口组 I：末端快脱 + 前沿滑块锁止 —— 装配基准与简化实体（修订版）。

几何事实（模型实测）：
    末端球头   中心 (4500, y, 0)  r=67        x 4433..4567  z 0..67
    滑块本体   x 4574..4666  y ±120  z 157.5..184.5
    锚轨腹板   x 4560..4680  z 32..66
    前膜压条   x 4473..4497  z 13.5..34.5
    => 球头顶面 z=67 与滑块底面 z=157.5 之间缺 90.5 mm 实际连接

接口方案：
    滑块下挂一根**可升降对接杆**（接口自由度，不是机械臂自由度）
      - 收上位：杆底 z=97.5
      - 伸出位：杆底 z=37.5（行程 60），套住球头上方的颈环
    颈环 r=52，中部凹槽 r=42（z 24..42）
    两根摆爪在 y=±50 铰接，闭合后内表面 r≈42.5，卡进凹槽
    解锁：摆爪张开 25° → 对接杆上收 60 → 臂沿 -X 撤离 120 mm
"""
import json
from pathlib import Path

TIP_C = (4500.0, 0.0, 0.0)
TIP_R = 67.0
COLLAR_R, GROOVE_R, COLLAR_H = 40.0, 32.0, 18.0
COLLAR_Z0 = 54.0   # 球面在 z>=54 处半径<=40，颈环正好坐在球冠上
SLIDER_BOTTOM_Z = 157.5
POST_LEN = 60.0
POST_R, BORE_R = 50.0, 44.0
CONE_R_TOP, CONE_R_BOT, CONE_H = 60.0, 30.0, 20.0
LATCH_W, LATCH_T, LATCH_L = 12.0, 20.0, 36.0
LATCH_HINGE_Y = 42.0   # 内表面 r=32 正好贴凹槽
LATCH_CLOSE_DEG = 25.0
INSERT_TRAVEL = 52.5   # 杆底 97.5 -> 45，覆盖凹槽 57..69
RETRACT_X = 120.0


def parts(y, insert=INSERT_TRAVEL, latch_closed=True):
    """insert: 0=收上, 60=伸出到位。latch_closed: 摆爪是否闭合。"""
    post_bottom = SLIDER_BOTTOM_Z - POST_LEN - insert   # 收上=97.5，伸出 52.5 -> 45
    post_top = post_bottom + POST_LEN
    P = [
        dict(name='IF_post', kind='box', c=[TIP_C[0], y, (post_bottom + post_top) / 2],
             size=[2 * POST_R, 2 * POST_R, POST_LEN]),
        dict(name='IF_cone', kind='cylinder',
             p=[TIP_C[0], y, post_bottom - CONE_H], q=[TIP_C[0], y, post_bottom],
             radius=CONE_R_BOT),
        dict(name='IF_collar', kind='cylinder',
             p=[TIP_C[0], y, COLLAR_Z0], q=[TIP_C[0], y, COLLAR_Z0 + COLLAR_H],
             radius=COLLAR_R),
        dict(name='IF_collar_groove', kind='cylinder',
             p=[TIP_C[0], y, COLLAR_Z0 + 4], q=[TIP_C[0], y, COLLAR_Z0 + COLLAR_H - 4],
             radius=GROOVE_R),
    ]
    off = 0.0 if latch_closed else LATCH_T * 0.9
    for s in (-1, 1):
        P.append(dict(name=f'IF_latch_{"L" if s < 0 else "R"}', kind='box',
                      c=[TIP_C[0], y + s * (LATCH_HINGE_Y + off), post_bottom + LATCH_L / 2],
                      size=[LATCH_W, LATCH_T, LATCH_L]))
    return P


def _box(o):
    c, s = o['c'], o['size']
    return ([c[0] - s[0] / 2, c[1] - s[1] / 2, c[2] - s[2] / 2],
            [c[0] + s[0] / 2, c[1] + s[1] / 2, c[2] + s[2] / 2])


def box_gap(a, b):
    """两个轴对齐盒的最短距离（有重叠返回负值=穿透深度）。"""
    (alo, ahi), (blo, bhi) = _box(a), _box(b)
    d = [max(blo[i] - ahi[i], alo[i] - bhi[i]) for i in range(3)]
    pos = [max(0.0, v) for v in d]
    if all(v <= 0 for v in d):
        return -min(abs(v) for v in d)
    return (sum(v * v for v in pos)) ** 0.5


def checks(y):
    """关键配合的 3D 间隙。"""
    ext = parts(y, insert=INSERT_TRAVEL, latch_closed=True)
    ret = parts(y, insert=0.0, latch_closed=False)
    g = {p['name']: p for p in ext}

    clamp = dict(name='clamp', kind='box', c=[(4473 + 4497) / 2, y, (13.5 + 34.5) / 2],
                 size=[4497 - 4473, 3500, 34.5 - 13.5])
    rail = dict(name='rail', kind='box', c=[(4560 + 4680) / 2, y - 1750 + 1750, 49],
                size=[120, 3500, 34])
    slider = dict(name='slider', kind='box', c=[(4574 + 4666) / 2, y, 171],
                  size=[92, 240, 27])

    out = {}
    out['伸出_对接杆_vs_压条'] = round(box_gap(g['IF_post'], clamp), 1)
    out['伸出_对接杆_vs_锚轨腹板'] = round(box_gap(g['IF_post'], rail), 1)
    out['伸出_对接杆_vs_滑块'] = round(box_gap(g['IF_post'], slider), 1)
    out['伸出_对接杆底_z'] = round(SLIDER_BOTTOM_Z - POST_LEN - INSERT_TRAVEL, 1)
    out['伸出_凹槽被摆爪覆盖'] = round(
        min(COLLAR_Z0 + COLLAR_H - 4, SLIDER_BOTTOM_Z - POST_LEN - INSERT_TRAVEL + LATCH_L)
        - max(COLLAR_Z0 + 4, SLIDER_BOTTOM_Z - POST_LEN - INSERT_TRAVEL), 1)
    # 摆爪闭合：内表面到凹槽半径的夹紧量（正=过盈，负=间隙）
    inner = LATCH_HINGE_Y - LATCH_T / 2
    out['闭合_摆爪内面_r'] = round(inner, 1)
    out['闭合_对凹槽夹紧量(0=贴合)'] = round(GROOVE_R - inner, 1)
    out['闭合_对颈环让开量(负=干涉)'] = round(COLLAR_R - inner, 1)
    # 张开后
    r = {p['name']: p for p in ret}
    out['张开_摆爪内面_r'] = round(LATCH_HINGE_Y + LATCH_T * 0.9 - LATCH_T / 2, 1)
    out['收回_对接杆底_z'] = round(SLIDER_BOTTOM_Z - 0, 1)
    out['收回_杆底与球头顶间隙'] = round((SLIDER_BOTTOM_Z - POST_LEN) - TIP_R, 1)
    return out


def joints(y):
    return [
        dict(name='J_if_insert', type='prismatic', axis=[0, 0, -1], range=[0.0, INSERT_TRAVEL],
             engaged_at=INSERT_TRAVEL, note='滑块侧对接杆升降：0=收上 60=伸出套住颈环'),
        dict(name='J_if_latch_L', type='revolute', axis=[0, 0, 1], range=[0.0, LATCH_CLOSE_DEG],
             engaged_at=LATCH_CLOSE_DEG, note='左摆爪：0=张开 25=闭合'),
        dict(name='J_if_latch_R', type='revolute', axis=[0, 0, 1], range=[-LATCH_CLOSE_DEG, 0.0],
             engaged_at=-LATCH_CLOSE_DEG, note='右摆爪：0=张开 -25=闭合'),
        dict(name='J_if_retract', type='prismatic', axis=[-1, 0, 0], range=[0.0, RETRACT_X],
             engaged_at=0.0, note='臂侧撤离（机构动作，验证期用于脱开检查）'),
    ]


if __name__ == '__main__':
    y = 5066.7
    c = checks(y)
    print('接口组 I 关键配合')
    for k, v in c.items():
        print(f'  {k:28s} {v:8.1f} mm')
    spec = dict(
        datum=dict(origin=TIP_C, x='+X 朝锚轨', z='+Z 向上', insert_dir='-Z'),
        sizes=dict(tip_r=TIP_R, collar_r=COLLAR_R, groove_r=GROOVE_R, collar_z=[COLLAR_Z0, COLLAR_Z0 + COLLAR_H],
                   slider_bottom_z=SLIDER_BOTTOM_Z, post_len=POST_LEN, post_r=POST_R, bore_r=BORE_R,
                   latch=[LATCH_W, LATCH_T, LATCH_L], hinge_y=LATCH_HINGE_Y,
                   travel=INSERT_TRAVEL, retract=RETRACT_X),
        gap_found_mm=round(SLIDER_BOTTOM_Z - TIP_R, 1),
        clearances=c, joints=joints(y),
        states=['locked: insert=60, latch=±25',
                'unlocked: latch=0 (张开)',
                'withdrawn: insert=0 + 臂 -X 120',
                'redock: 臂 +X 120 → insert=60 → latch 闭合'])
    Path(__file__).resolve().parent.joinpath('out', 'iface_group1_spec.json').write_text(
        json.dumps(spec, ensure_ascii=False, indent=1), encoding='utf8')
    print('写入 out/iface_group1_spec.json')
