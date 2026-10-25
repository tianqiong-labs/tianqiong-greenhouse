"""接口组 I · 方案 β：开口 C 形座 + 双锁爪，连续分离验证。

拓扑改正：
  闭口盲孔（孔口 z=76、孔底 z=14）→ 朝 −X 开口的 C 形槽（包角 180°）
  锁爪只负责轴向止退，不再承担横向约束
  分离：解锁（爪退 8）→ 沿 −X 平移，柱从开口侧滑出（无需轴向行程）
"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)

R, BORE_R, FLOOR = 67.0, 16.0, 14.0
BOSS_R, BOSS_Z0, BOSS_Z1 = 24.0, 58.0, 76.0
COL_R, COL_Z0, COL_Z1 = 15.0, 14.0, 194.0
GRV_R, GRV_Z0, GRV_Z1 = 11.0, 38.0, 50.0
CLAW = (12.0, 18.0, 10.0)
OPEN_HALF = 16.0          # 开口在 Y 向半宽（> 柱半径 15）
TRAVEL = 8.0


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    body = hemi.union(boss)
    # 孔腔
    body = body.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    # 朝 +X 开口（朝锚轨）：切掉 x>0 半个型腔壁；臂向 -X 撤离时柱从开口滑出
    body = body.cut(cq.Workplane("XY").box(80, 2 * OPEN_HALF, BOSS_Z1 - FLOOR + 2)
                    .translate((40, 0, (FLOOR + BOSS_Z1) / 2)))
    # 爪窗
    for sy in (-1, 1):
        body = body.cut(cq.Workplane("XY").box(CLAW[0], 34, CLAW[2] + 2)
                        .translate((0, sy * 25, (GRV_Z0 + GRV_Z1) / 2)))
    return body


def column():
    col = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    ring = (cq.Workplane("XY").workplane(offset=GRV_Z0).circle(COL_R).extrude(GRV_Z1 - GRV_Z0)
            .cut(cq.Workplane("XY").workplane(offset=GRV_Z0).circle(GRV_R).extrude(GRV_Z1 - GRV_Z0)))
    return col.cut(ring)


def claw(retracted, sy=1):
    cy = sy * (GRV_R + CLAW[1] / 2 + (TRAVEL if retracted else 0.0))
    return cq.Workplane("XY").box(*CLAW).translate((0, cy, (GRV_Z0 + GRV_Z1) / 2))


def state(dx, claws_retracted):
    b = tip_ball().translate((-dx, 0, 0))
    c = column()
    # 锁爪装在球头窗口内，随球头一起撤离
    parts = {'BALL': b, 'COLUMN': c,
             'CLAW_R': claw(claws_retracted, 1).translate((-dx, 0, 0)),
             'CLAW_L': claw(claws_retracted, -1).translate((-dx, 0, 0))}
    return parts


def inter(a, b):
    return round(a.val().intersect(b.val()).Volume(), 3)


if __name__ == '__main__':
    seq = [('LOCKED', 0.0, False), ('UNLOCKING', 0.0, True),
           ('WITHDRAW_25%', 30.0, True), ('WITHDRAW_50%', 60.0, True),
           ('WITHDRAW_75%', 90.0, True), ('CLEAR', 120.0, True)]
    rows = []
    for name, dx, ret in seq:
        p = state(dx, ret)
        b, c = p['BALL'], p['COLUMN']
        r = dict(state=name, dx=dx, claws_retracted=ret,
                 ball_vs_column=inter(b, c),
                 ball_vs_claw_R=inter(b, p['CLAW_R']),
                 ball_vs_claw_L=inter(b, p['CLAW_L']),
                 claw_vs_column_R=inter(p['CLAW_R'], c),
                 claw_vs_column_L=inter(p['CLAW_L'], c))
        r['ok'] = all(v <= 1e-6 for k, v in r.items() if k not in ('state', 'dx', 'claws_retracted', 'ok'))
        rows.append(r)
        print(f"{name:14s} dx={dx:5.1f} 球-柱 {r['ball_vs_column']:8.3f} 球-爪 {r['ball_vs_claw_R']:8.3f} "
              f"爪-柱 {r['claw_vs_column_R']:8.3f}  {'OK' if r['ok'] else '干涉'}")
    # 导出各状态 STEP
    for name, dx, ret in seq:
        p = state(dx, ret)
        asm = cq.Assembly()
        for n, s in p.items():
            asm.add(s, name=n)
        asm.save(str(OUT / f'beta_{name}.step'))
    # 关键剖面：过 Y=0 的 XZ 面
    p = state(0.0, False)
    asm = cq.Assembly()
    for n, s in p.items():
        asm.add(s, name=n)
    asm.save(str(OUT / 'beta_LOCKED_section_y0.step'))
    (OUT / 'beta_sequence.json').write_text(json.dumps(rows, ensure_ascii=False, indent=1), encoding='utf8')
    print('导出', len(seq) + 1, '个 STEP 到', OUT)
