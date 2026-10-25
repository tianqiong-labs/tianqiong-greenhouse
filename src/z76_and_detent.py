"""P0 z=76 截面参数化校核 + P1 双半环弹簧锁销止退。"""
import cadquery as cq, json
from pathlib import Path

OUT = Path(__file__).resolve().parent / 'out' / 'cad'
OUT.mkdir(parents=True, exist_ok=True)
T = 0.6


def ball():
    hemi = cq.Workplane("XY").sphere(67).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=58).circle(24).extrude(18)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=14).circle(16).extrude(62))
    return b.cut(cq.Workplane("XY").box(120, 32, 66).translate((44, 0, 45)))


def slab(z):
    return cq.Workplane("XY").box(300, 300, T).translate((0, 0, z))


def section_props(shape, z, x0=-26.0, x1=20.0, dx=1.0):
    """按 1 mm 竖条积分：A、形心 x、I_y、W_y。"""
    A = 0.0; Sx = 0.0; I_self = 0.0; xs = []
    x = x0
    while x < x1:
        strip = cq.Workplane("XY").box(dx, 300, T).translate((x + dx / 2, 0, z))
        s = shape.val().intersect(strip.val())
        v = s.Volume()
        if v > 1e-9:
            a = v / T
            h = a / dx
            A += a; Sx += a * (x + dx / 2); xs.append((x + dx / 2, h))
            I_self += dx * h ** 3 / 12.0
        x += dx
    if A <= 0:
        return dict(z=z, A=0.0)
    xc = Sx / A
    I_y = I_self + sum(dx * h * (xx - xc) ** 2 for xx, h in xs)
    ymax = max(abs(min(xs, key=lambda p: p[0])[0] - xc), abs(max(p[0] for p in xs) - xc))
    return dict(z=z, A=round(A, 1), xc=round(xc, 2), Iy=round(I_y, 0),
                Wy=round(I_y / ymax, 1) if ymax else None, width_x=round(x1 - x0, 1))


def half(sign, s=0.0):
    ring = (cq.Workplane("XY").workplane(offset=90).circle(32).extrude(20)
            .cut(cq.Workplane("XY").workplane(offset=90).circle(15.5).extrude(20)))
    h = ring.intersect(cq.Workplane("XY").workplane(offset=90).rect(200, 40)
                       .extrude(20).translate((0, sign * 20, 0)))
    return h.translate((0, sign * s, 0))


def bracket(sign, s=0.0):
    return cq.Workplane("XY").box(16, 14, 2).translate((-22.0, sign * (34 + s), 89))


def pin(sign, retracted=False):
    """Ø6 弹簧锁销：从支架穿入半环外侧止退孔。"""
    y = sign * (34 - (10.0 if retracted else 0.0))
    return cq.Workplane("YZ").workplane(offset=-23).circle(3).extrude(14).translate((0, y, 100))


def v(a, b):
    try:
        return round(a.val().intersect(b.val()).Volume(), 2)
    except Exception:
        return 0.0


if __name__ == '__main__':
    b = ball()
    print('=== P0：z=76 邻近截面 ===')
    rows = []
    for z in (70.0, 72.0, 74.0, 76.0, 77.0):
        r = section_props(b, z)
        rows.append(r)
        if r['A'] <= 0:
            print(f"  z={z:5.1f}  无材料")
            continue
        print(f"  z={z:5.1f}  A={r['A']:8.1f} mm2  xc={r['xc']:6.2f}  "
              f"Iy={r['Iy']:9.0f} mm4  Wy={r['Wy']:7.1f} mm3")
    print('  参考式：σ = M/Wy + F/A（材料与载荷未定，不作合格判定）')

    print('\n=== P1：锁销止退（半环 R 受力 +Y）===')
    base = half(1, 0.0)
    p_in = pin(1, retracted=False)
    p_out = pin(1, retracted=True)
    push = base.translate((0, 2.0, 0))
    hit_pin = v(push, p_in)
    clear_pin = v(push, p_out)
    print(f'  强制 +Y 2 mm（销在位）: 与销干涉 {hit_pin} mm3 -> {"止退成立" if hit_pin > 0.1 else "未挡"}')
    print(f'  强制 +Y 2 mm（销退回）: 与销干涉 {clear_pin} mm3 -> {"可通过" if clear_pin < 0.1 else "仍挡"}')

    asm = cq.Assembly(name='G2_DETENT')
    for n, sh in (('BALL', b), ('HALF_R', half(1)), ('HALF_L', half(-1)),
                  ('BRKT_R', bracket(1)), ('BRKT_L', bracket(-1)),
                  ('PIN_R', pin(1)), ('PIN_L', pin(1, retracted=True))):
        asm.add(sh, name=n)
    asm.save(str(OUT / 'gamma2_detent.step'))

    sec = cq.Workplane("XY").workplane(offset=76).add(b).section()
    cq.exporters.export(sec, str(OUT / 'ball_sec_z76_final.svg'))
    (OUT / 'z76_section.json').write_text(json.dumps(
        dict(sections=rows, detent=dict(hit=hit_pin, clear=clear_pin)),
        ensure_ascii=False, indent=1), encoding='utf8')
    print('\n导出 gamma2_detent.step / ball_sec_z76_final.svg / z76_section.json')
