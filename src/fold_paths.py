"""第二优先：18 根机械臂展开 / 收拢路径检查。

姿态由关节角线性（五段平滑）插值；每个采样点检查：
  * 母墙箱体、地面、非相邻连杆自碰
  * 相邻臂之间（yaw 保持 0 时各臂在平行平面内，间隙恒为 560-2r）
  * 关节角与伸缩行程是否超限
输出每根臂的最小间隙、发生位置、关节裕量。
"""
from pathlib import Path
import json, math
import numpy as np
import rail_carry as rc

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
SUPPORT = np.deg2rad(np.array(rc.SUPPORT_DEG))
FOLD = np.deg2rad(np.array([-75.0, 75.0, -75.0, 75.0]))
TEL_DEPLOY, TEL_FOLD = 1600.0, 1000.0
STEPS = 31


def smooth(u):
    return u * u * (3 - 2 * u)


def pose_at(u):
    a = SUPPORT + (FOLD - SUPPORT) * smooth(u)
    tel = TEL_DEPLOY + (TEL_FOLD - TEL_DEPLOY) * smooth(u)
    return a, tel


WITH_RAIL = False


def clearance(pts):
    """墙 + 地面 + 非相邻连杆自碰；WITH_RAIL 时把仍在位的轨道算作障碍。"""
    if WITH_RAIL:
        return rc.clearance_2d(pts, rail_pt=(4620.0, 0.0), include_rail=True, tip_exclude=0.0)
    return rc.clearance_2d(pts, rail_pt=(1e6, 1e6), include_rail=False)


def run():
    rows = []
    for arm in range(1, 19):
        worst = (1e9, None, None)
        samples = []
        max_ang = 0.0
        tel_min, tel_max = 1e9, -1e9
        for k in range(STEPS):
            u = k / (STEPS - 1)
            a, tel = pose_at(u)
            pts = rc.fk2(a, tel)
            c = clearance(pts)
            max_ang = max(max_ang, float(np.abs(a).max()))
            tel_min, tel_max = min(tel_min, tel), max(tel_max, tel)
            samples.append(round(float(c), 1))
            if k > 0 and c < worst[0]:
                worst = (c, u, a)
        rows.append(dict(arm=arm, min_clear_mm=round(worst[0], 1),
                         at_u=round(worst[1], 2) if worst[1] is not None else None,
                         angles_at_worst=np.round(np.rad2deg(worst[2]), 1).tolist() if worst[2] is not None else None,
                         max_abs_angle_deg=round(float(np.rad2deg(max_ang)), 1),
                         tel_range=[round(tel_min), round(tel_max)],
                         samples_mm=samples))
        print(f"arm{arm:02d}: min clearance {worst[0]:7.1f} mm @u={worst[1]:.2f}  |angle|max {max_ang:5.1f}°")

    clr = np.array([r['min_clear_mm'] for r in rows])
    # 去掉开头腕部代理触地（继承缺陷）后的统计
    late = np.array([min(r['samples_mm'][3:]) for r in rows])
    summary = dict(arms=len(rows), steps=STEPS,
                   min_clearance_mm=float(clr.min()),
                   arms_below_zero=int((clr < 0).sum()),
                   arms_below_10mm=int((clr < 10).sum()),
                   median_clearance_mm=float(np.median(clr)),
                   max_abs_angle_deg=float(max(r['max_abs_angle_deg'] for r in rows)),
                   arm_arm_clearance_mm=560.0 - 2 * rc.ARM_R,
                   joint_limit_deg=float(np.rad2deg(rc.A_LIM)),
                   min_clearance_after_u0_1_mm=float(late.min()),
                   arms_below_zero_after_u0_1=int((late < 0).sum()))
    (OUT / 'fold_paths.json').write_text(json.dumps(dict(summary=summary, rows=rows), ensure_ascii=False, indent=1),
                                         encoding='utf8')
    print('\n' + json.dumps(summary, ensure_ascii=False, indent=1))
    return summary


if __name__ == '__main__':
    import sys
    if '--with-rail' in sys.argv:
        WITH_RAIL = True
    if '--limit' in sys.argv:
        rc.A_LIM = math.radians(float(sys.argv[sys.argv.index('--limit') + 1]))
    run()
    if WITH_RAIL:
        print('（本次把仍在位的轨道算作障碍）')
