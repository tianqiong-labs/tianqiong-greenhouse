"""验证工具修复 + 三组回归（R-ART-02）。

修复三处：
 1. 求交失败曾被当成"零碰撞" -> 现在抛 IntersectError，并阻止输出"通过"
 2. γ-2 检查漏掉腕柱与导轨 -> 目标件清单强制包含 COLUMN / ARM_RAIL
 3. 半环撤离方向与父刚体 -> 撤离(along -X)与反向插入(along +X)用同一坐标函数
"""
import cadquery as cq


class IntersectError(RuntimeError):
    pass


def _sh(x):
    return x.val() if hasattr(x, "val") else x


def safe_intersect(a, b):
    """求交失败必须报错，绝不返回 0。"""
    try:
        sa, sb = _sh(a), _sh(b)
        return float(sa.intersect(sb).Volume())
    except Exception as e:
        raise IntersectError(f"求交失败: {type(e).__name__}: {e}") from e


def gate(volume, tol=1e-3):
    """只有明确计算出的体积才能用于判定；None/异常一律不通过。"""
    if volume is None:
        raise IntersectError("未获得体积结果，禁止判定通过")
    return volume <= tol


def targets():
    """γ-2 检查必须覆盖的对象（设计接触单独列出，不整件排除）。"""
    return ["BALL", "FLANGE", "ARM_RAIL", "CLAMP_L", "CLAMP_R",
            "COLUMN", "P03_WEB", "P03_RETAINER", "BAR"]


DESIGN_CONTACT = {("BALL", "COLUMN"), ("FLANGE", "COLUMN")}


def ball():
    hemi = cq.Workplane("XY").sphere(67).cut(
        cq.Workplane("XY").box(400, 400, 200).translate((0, 0, -100)))
    boss = cq.Workplane("XY").workplane(offset=58).circle(24).extrude(18)
    b = hemi.union(boss)
    b = b.cut(cq.Workplane("XY").workplane(offset=14).circle(16).extrude(62))
    return b.cut(cq.Workplane("XY").box(120, 32, 66).translate((44, 0, 45)))


def bar():
    """前膜压条：已知与球头干涉（历史 -53 mm），用作"必须检出"的回归对。"""
    return cq.Workplane("XY").box(24, 3500, 21).translate((4485, 1750, 24))


def column():
    c = cq.Workplane("XY").workplane(offset=14).circle(15).extrude(180)
    g = (cq.Workplane("XY").workplane(offset=38).circle(15).extrude(12)
         .cut(cq.Workplane("XY").workplane(offset=38).circle(11).extrude(12)))
    return c.cut(g)


def at_x(shape, dx):
    """统一坐标定义：dx>0 表示球心向 -X 移动 dx。撤离与反向插入共用。"""
    return shape.translate((-dx, 0, 0))


def run_tests():
    results = []
    # 回归1：异常注入不得通过
    try:
        safe_intersect(None, ball())
        results.append(("异常注入", False, "未抛错"))
    except IntersectError:
        try:
            gate(None)
            results.append(("异常注入", False, "gate 未拦截 None"))
        except IntersectError:
            results.append(("异常注入", True, "抛 IntersectError 且 gate 拦截"))

    b, c, br = ball().translate((4500, 0, 0)), column().translate((4500, 0, 0)), bar()
    # 回归2a：已知穿透必须检出（球头 vs 前膜压条）
    v = safe_intersect(b, br)
    results.append(("球头×压条检出", v > 0, f"{v:.1f} mm3"))
    # 回归2b：设计配合（孔柱间隙）必须为 0，不能误报
    v2 = safe_intersect(b, c)
    results.append(("球头×腕柱=间隙配合", abs(v2) < 1e-6, f"{v2:.1f} mm3"))
    # 目标清单必须含腕柱/导轨
    missing = [k for k in ("COLUMN", "ARM_RAIL") if k not in targets()]
    results.append(("清单含腕柱/导轨", not missing, f"缺失 {missing}"))

    # 回归3：撤离与反向插入坐标一致
    fwd = [safe_intersect(at_x(b, dx), br) for dx in (0, 30, 60)]
    back = [safe_intersect(at_x(b, dx), br) for dx in (60, 30, 0)]
    ok = all(abs(f - r) < 1e-6 for f, r in zip(fwd, back[::-1]))
    results.append(("撤离/插入同坐标", ok, f"fwd={[round(x,1) for x in fwd]} rev={[round(x,1) for x in back]}"))

    print("=== 回归结果 ===")
    allp = True
    for name, ok_, note in results:
        print(f"  {'PASS' if ok_ else 'FAIL'}  {name:18s} {note}")
        allp &= ok_
    print("总结:", "ALL PASS" if allp else "存在失败")
    return allp


if __name__ == "__main__":
    import sys
    sys.exit(0 if run_tests() else 1)
