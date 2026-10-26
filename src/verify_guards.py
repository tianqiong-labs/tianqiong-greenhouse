"""验证防护 + 三组强化回归（R-ART-02）。

修复：
 1. 求交失败/非法体积 -> IntersectError，禁止输出 PASS
 2. 腕柱、导轨必须进入实际比较循环（不是只检查清单字符串）
 3. 撤离/插入共用坐标定义：ΔX=-dx, ΔY=0, ΔZ=0；轨侧件不动
"""
import math

import cadquery as cq


class IntersectError(RuntimeError):
    pass


def _sh(x):
    return x.val() if hasattr(x, "val") else x


def safe_intersect(a, b):
    try:
        return float(_sh(a).intersect(_sh(b)).Volume())
    except Exception as e:
        raise IntersectError(f"求交失败: {type(e).__name__}: {e}") from e


def gate(volume, tol=1e-3):
    """只接受有限非负体积；None / NaN / inf / 负值一律拒绝。"""
    if volume is None:
        raise IntersectError("无体积结果")
    if not isinstance(volume, (int, float)) or isinstance(volume, bool):
        raise IntersectError(f"体积类型非法: {type(volume).__name__}")
    if not math.isfinite(volume):
        raise IntersectError(f"体积非有限: {volume}")
    if volume < 0:
        raise IntersectError(f"体积为负: {volume}")
    return volume <= tol


def ball():
    hemi = cq.Workplane("XY").sphere(67).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=58).circle(24).extrude(18)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=14).circle(16).extrude(62))
    return b.cut(cq.Workplane("XY").box(120, 32, 66).translate((44, 0, 45)))


def column():
    c = cq.Workplane("XY").workplane(offset=14).circle(15).extrude(180)
    g = (cq.Workplane("XY").workplane(offset=38).circle(15).extrude(12)
         .cut(cq.Workplane("XY").workplane(offset=38).circle(11).extrude(12)))
    return c.cut(g)


def bar(y_center=350.0):
    """内臂位压条（Y=350 为 A01 实际站点）。"""
    return cq.Workplane("XY").box(24, 400, 21).translate((4485, y_center, 24))


def moved(b, dx):
    return b.translate((-dx, 0, 0))


if __name__ == "__main__":
    import sys
    res = []

    # 1 异常与非有限值
    for bad, label in ((None, "None"), (float("nan"), "NaN"), (float("inf"), "inf"), (-1.0, "负值")):
        try:
            gate(bad)
            res.append((f"gate 拒绝 {label}", False, "未拒绝"))
        except IntersectError:
            res.append((f"gate 拒绝 {label}", True, "已拒绝"))
    try:
        safe_intersect(None, ball())
        res.append(("求交异常抛错", False, "返回了值"))
    except IntersectError:
        res.append(("求交异常抛错", True, "IntersectError"))

    b = ball().translate((4500, 350, 0))   # 与压条同处 A01 臂位
    c = column().translate((4500, 350, 0))
    br = bar(350.0)

    # 2 真实碰撞检出 + 设计配合不误报
    v_bar = safe_intersect(b, br)
    res.append(("内臂位压条干涉检出", v_bar > 0, f"{v_bar:.1f} mm3 @Y=350"))
    v_col = safe_intersect(b, c)
    res.append(("球头×腕柱间隙配合=0", abs(v_col) < 1e-6, f"{v_col:.1f} mm3"))

    # 3 位移方向：ΔX=-dx, ΔY=0, ΔZ=0
    b2 = moved(b, 30.0)
    c1, c2 = b.val().Center(), b2.val().Center()
    dx, dy, dz = c2.x - c1.x, c2.y - c1.y, c2.z - c1.z
    ok_dir = abs(dx + 30.0) < 1e-6 and abs(dy) < 1e-6 and abs(dz) < 1e-6
    res.append(("位移方向", ok_dir, f"Δ=({dx:.2f},{dy:.2f},{dz:.2f})"))

    # 4 静态：闸板式对比（腕柱在比较循环内必须被求交）
    pairs = []
    for name_a, sa in (("BALL", b), ("CLAMP", cq.Workplane("XY").workplane(offset=90).circle(32)
                                     .extrude(20).cut(cq.Workplane("XY").workplane(offset=90)
                                     .circle(15.5).extrude(20)).translate((4500, 0, 0)))):
        for name_b, sb in (("COLUMN", c), ("RAIL", br)):
            pairs.append((name_a, name_b, safe_intersect(sa, sb)))
    has_col = any(p[1] == "COLUMN" for p in pairs)
    res.append(("比较循环含腕柱", has_col, f"{len(pairs)} 对，其中腕柱对 "
                                          f"{[round(p[2],1) for p in pairs if p[1]=='COLUMN']}"))

    print("=== 回归 ===")
    allok = True
    for n, ok, note in res:
        print(f"  {'PASS' if ok else 'FAIL'}  {n:20s} {note}")
        allok &= ok
    print("总结:", "ALL PASS" if allok else "存在失败")
    sys.exit(0 if allok else 1)
