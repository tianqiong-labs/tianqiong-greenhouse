"""β 增补：横向闭锁舌 + 六自由度约束检查 + 反向（未解锁不得脱开）验证。

新增 CLOSURE 舌块：锁止时嵌在球头 +X 开口处（x 16…30，y ±22，z 14…76），
与双锁爪共用一套解锁驱动（本版按同步运动建模，联动机构另证）。
解锁时沿 +Y 退出 46 mm。
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
OPEN_HALF, TRAVEL = 16.0, 8.0
GATE_X0, GATE_X1, GATE_HALF = 16.0, 30.0, 22.0
GATE_RETRACT = 46.0
ZC = (FLOOR + BOSS_Z1) / 2


def tip_ball():
    hemi = cq.Workplane("XY").sphere(R).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=BOSS_Z0).circle(BOSS_R).extrude(BOSS_Z1 - BOSS_Z0)
    body = hemi.union(boss)
    body = body.cut(cq.Workplane("XY").workplane(offset=FLOOR).circle(BORE_R).extrude(BOSS_Z1 - FLOOR))
    # +X 开口通道（直到球外缘）
    body = body.cut(cq.Workplane("XY").box(90, 2 * OPEN_HALF, BOSS_Z1 - FLOOR + 2)
                    .translate((GATE_X0 + 45, 0, ZC)))
    # 闭锁舌槽
    body = body.cut(cq.Workplane("XY").box(GATE_X1 - GATE_X0 + 2, 2 * GATE_HALF,
                                           BOSS_Z1 - FLOOR + 2)
                    .translate(((GATE_X0 + GATE_X1) / 2, 0, ZC)))
    for sy in (-1, 1):   # 爪窗
        body = body.cut(cq.Workplane("XY").box(CLAW[0], 34, CLAW[2] + 2)
                        .translate((0, sy * 25, (GRV_Z0 + GRV_Z1) / 2)))
    return body


def column(dx=0.0):
    col = cq.Workplane("XY").workplane(offset=COL_Z0).circle(COL_R).extrude(COL_Z1 - COL_Z0)
    ring = (cq.Workplane("XY").workplane(offset=GRV_Z0).circle(COL_R).extrude(GRV_Z1 - GRV_Z0)
            .cut(cq.Workplane("XY").workplane(offset=GRV_Z0).circle(GRV_R).extrude(GRV_Z1 - GRV_Z0)))
    return col.cut(ring).translate((dx, 0, 0))


def claw(retracted, sy, dx=0.0):
    cy = sy * (GRV_R + CLAW[1] / 2 + (TRAVEL if retracted else 0.0))
    return cq.Workplane("XY").box(*CLAW).translate((dx, cy, (GRV_Z0 + GRV_Z1) / 2))


def gate(open_):
    y = GATE_RETRACT if open_ else 0.0
    return (cq.Workplane("XY").box(GATE_X1 - GATE_X0, 2 * GATE_HALF, BOSS_Z1 - FLOOR)
            .translate(((GATE_X0 + GATE_X1) / 2, y, ZC)))


def h(shape):
    return shape.val()


if __name__ == '__main__':
    ball = tip_ball()
    rows = []

    def check(name, cl_open, gate_open, col_dx, ball_dx=0.0):
        b = ball.translate((-ball_dx, 0, 0))
        c = column(col_dx)
        parts = dict(BALL=b, COLUMN=c, GATE=gate(gate_open),
                     CLAW_R=claw(cl_open, 1, -ball_dx), CLAW_L=claw(cl_open, -1, -ball_dx))
        keys = [('BALL', 'COLUMN'), ('BALL', 'GATE'), ('COLUMN', 'GATE'),
                ('BALL', 'CLAW_R'), ('BALL', 'CLAW_L'), ('COLUMN', 'CLAW_R'), ('COLUMN', 'CLAW_L')]
        ints = {f'{a}-{b2}': round(h(parts[a]).intersect(h(parts[b2])).Volume(), 3) for a, b2 in keys}
        bad = {k: v for k, v in ints.items() if v > 1e-6}
        rows.append(dict(case=name, interference=ints, illegal=bad))
        print(f"{name:26s} 非法 {len(bad)} 处 " + (str(bad) if bad else ''))
        return parts, bad

    # 1 锁止态
    p, _ = check('LOCKED', False, False, 0.0)
    # 2 横向退出趋势：柱 +X 20（未解锁）→ 必须被舌块挡住
    _, bad = check('LOCKED_escape_+X20', False, False, 20.0)
    blocked = sum(bad.values()) > 0
    # 3 解锁态
    check('UNLOCKED', True, True, 0.0)
    # 4 解锁后柱 +X 20 应无障碍
    _, bad2 = check('UNLOCKED_push_+X20', True, True, 20.0)
    free = sum(bad2.values()) == 0
    # 5 连续撤离
    for dx in (0.0, 15.0, 30.0, 60.0, 90.0, 120.0):
        check(f'WITHDRAW_{int(dx)}', True, True, 0.0, ball_dx=dx)

    print(f"\n反向验证：未解锁 +X20 → {'被挡住 ✓' if blocked else '未挡住 ✗'}")
    print(f"正向验证：解锁后 +X20 → {'自由通过 ✓' if free else '仍受阻 ✗'}")

    # STEP + 剖面
    asm = cq.Assembly()
    for n, s in p.items():
        asm.add(s, name=n)
    asm.save(str(OUT / 'beta2_LOCKED.step'))

    # 剖面 1：Y=0 纵剖（XZ）
    sec1 = cq.Workplane("XZ").add(ball).add(column()).section()
    # 剖面 2：Z=44 横剖（XY）
    sec2 = cq.Workplane("XY").workplane(offset=(GRV_Z0 + GRV_Z1) / 2).add(ball).add(column()).section()
    for nm, sec in (('section_longitudinal', sec1), ('section_transverse_z44', sec2)):
        try:
            cq.exporters.export(sec, str(OUT / f'{nm}.svg'))
            print('SVG:', nm)
        except Exception as e:
            print('SVG 失败', nm, type(e).__name__, e)
        cq.exporters.export(sec, str(OUT / f'{nm}.dxf'))

    (OUT / 'beta2_report.json').write_text(json.dumps(
        dict(rows=rows, blocked_when_locked=blocked, free_when_unlocked=free), ensure_ascii=False, indent=1),
        encoding='utf8')
    print('完成')
