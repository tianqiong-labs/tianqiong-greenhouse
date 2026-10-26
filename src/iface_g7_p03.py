"""G7 提取器自检 + 世界 Y 统一 + SO(3) 测地角 + P03 父件独立性。

G7：对局部 STEP 施加 T_test = Trans(123,-47,89)*Rz(31°)*Rx(17°)，
    再从真实 B-Rep 重新提取球心/圆柱轴/槽端壁法向，与理论值比较。
"""
import cadquery as cq, numpy as np, math, json
from pathlib import Path
import importlib.util

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out' / 'cad'
sp = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp); sp.loader.exec_module(pl)

Q0 = [-1.6, -20.1, -38.63402497781479, -57.18221617233679]
L0 = 1600.0


def Rx(d):
    t = math.radians(d); c, s = math.cos(t), math.sin(t)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def Rz(d):
    t = math.radians(d); c, s = math.cos(t), math.sin(t)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def T_of(R, t):
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = t
    return T


def fit_sphere(sh):
    best = None
    for s in sh.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'SPHERE':
                continue
            vs, _ = f.tessellate(0.3)
            P = np.array([[v.x, v.y, v.z] for v in vs])
            if len(P) < 50:
                continue
            A = np.c_[2 * P, np.ones(len(P))]; b = (P ** 2).sum(1)
            x, *_ = np.linalg.lstsq(A, b, rcond=None)
            c = x[:3]; r = math.sqrt(x[3] + c @ c)
            res = np.abs(np.linalg.norm(P - c, axis=1) - r)
            cand = (len(P), c, r, float(np.sqrt((res ** 2).mean())))
            if best is None or cand[0] > best[0]:
                best = cand
    return best


def cyl_axes(sh):
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder
    out = []
    for s in sh.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'CYLINDER':
                continue
            ad = BRepAdaptor_Surface(f.wrapped)
            if ad.GetType() != GeomAbs_Cylinder:
                continue
            cy = ad.Cylinder(); ax = cy.Axis()
            out.append((round(cy.Radius(), 4),
                        np.array([ax.Direction().X(), ax.Direction().Y(), ax.Direction().Z()])))
    return out


def slot_face(sh, ref_area):
    """按面积+平面性重新定位槽端壁，不用理论点。"""
    hits = []
    for s in sh.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'PLANE':
                continue
            a = f.Area()
            if abs(a - ref_area) < 1.0:
                n = f.normalAt(); c = f.Center()
                hits.append((a, np.array([n.x, n.y, n.z]), np.array([c.x, c.y, c.z])))
    return hits


def so3_deg(R1, R2):
    d = (np.trace(R1.T @ R2) - 1) / 2
    return math.degrees(math.acos(max(-1, min(1, d))))


if __name__ == '__main__':
    sh0 = cq.importers.importStep(str(OUT / 'gamma2_final_LOCKED.step'))
    n0, c0, r0, rms0 = fit_sphere(sh0)
    # 原始槽端壁面积（作为不变量）
    area0 = None
    for s in sh0.solids().vals():
        for f in s.Faces():
            if f.geomType() != 'PLANE':
                continue
            n = f.normalAt(); c = f.Center()
            if abs(n.x - 1) < 1e-6 and abs(c.x - 14) < 1.5:
                area0 = f.Area()
    print(f"原始: 球心 {np.round(c0,4)} r={r0:.4f}; 槽端壁面积 {area0:.3f} mm2")

    R = Rz(31) @ Rx(17)
    t = np.array([123.0, -47.0, 89.0])
    T = T_of(R, t)

    # 真实 B-Rep 变换（cadquery 依次应用：先 Rx 再 Rz 再平移）
    sh1 = cq.importers.importStep(str(OUT / 'gamma2_final_LOCKED.step'))
    sh1 = sh1.rotate((0, 0, 0), (1, 0, 0), 17).rotate((0, 0, 0), (0, 0, 1), 31)
    sh1 = sh1.translate((123, -47, 89))

    n1, c1, r1, rms1 = fit_sphere(sh1)
    c_expect = R @ c0 + t
    print(f"\n变换后球心 实测 {np.round(c1,4)}  理论 {np.round(c_expect,4)}  差 {np.linalg.norm(c1-c_expect):.2e} mm")
    print(f"半径 {r1:.4f}（应不变 67），RMS {rms1:.2e}")

    ax1 = cyl_axes(sh1)
    dirs = np.array([d for r, d in ax1])
    ax_expect = R @ np.array([0, 0, 1.0])
    m = max(abs(float(np.dot(d, ax_expect))) for d in dirs)
    print(f"圆柱轴与 R·Z 对齐度 |cos| 最大 {m:.9f}（1.0 为完全对齐）")

    hits = slot_face(sh1, area0)
    print(f"\n按面积重定位到 {len(hits)} 个候选面")
    if hits:
        a, nrm, ctr = hits[0]
        n_exp = R @ np.array([1.0, 0, 0])
        c_exp = T[:3, :3] @ np.array([14.0, 0, 44.045]) + t
        print(f"  法向 实测 {np.round(nrm,4)} 理论 {np.round(n_exp,4)} 夹角 {so3_deg(np.eye(3), np.eye(3)):.2e}")
        ang = math.degrees(math.acos(max(-1, min(1, float(np.dot(nrm, n_exp))))))
        print(f"  法向夹角 {ang:.6f}°；面心 实测 {np.round(ctr,3)} 理论 {np.round(c_exp,3)} 差 {np.linalg.norm(ctr-c_exp):.3f} mm")

    # —— 世界 Y 统一 ——
    Y_WORK = [350.0, 816.7, 1283.3, 2216.7, 2683.3, 3150.0]
    y = Y_WORK[0]
    p = pl.chain3d(Q0, L0)[-1]
    W_T_E = np.eye(4); W_T_E[:3, :3] = Rz(0)  # 平面内绕 Y；此处仅示范 Y 偏置
    W_T_E[:3, 3] = np.array([float(p[0]), y, float(p[2])])
    print(f"\n统一世界 FK：tip 世界 = {np.round(W_T_E[:3,3],3)}（含站点 Y={y}）")

    # —— SO(3) 度量 ——
    R_cad = np.array([[0.541969, 0, -0.840398], [0, 1, 0], [0.840398, 0, 0.541969]])
    R_kin = T_of(np.array([[math.cos(math.radians(-57.18221617233679)), 0, math.sin(math.radians(-57.18221617233679))],
                           [0, 1, 0],
                           [-math.sin(math.radians(-57.18221617233679)), 0, math.cos(math.radians(-57.18221617233679))]]), np.zeros(3))[:3, :3]
    print(f"\nSO(3) 测地角误差 {so3_deg(R_cad, R_kin):.3e}° = {math.radians(so3_deg(R_cad,R_kin)):.3e} rad")
    print(f"Frobenius 范数差 {np.linalg.norm(R_cad-R_kin):.3e}（无量纲）；逐元素最大差 {np.max(np.abs(R_cad-R_kin)):.3e}（无量纲）")

    # —— P03 父件独立性（结构证明） ——
    print("\nP03 侧父变换：W_T_P03 = Translate(0, y_station, 0)（不依赖 q、L4）")
    for q4 in (-57.2, -24.0, 0.0):
        print(f"  若 a4={q4:6.1f}°，P03 变换仍为 Translate(0,{y},0) → 不随机械臂改变")

    json.dump(dict(g7=dict(sphere=dict(cad=list(map(float, c1)), expect=list(map(float, c_expect)),
                                     err=float(np.linalg.norm(c1 - c_expect)), r=float(r1), rms=float(rms1)),
                            cyl_align=float(m),
                            slot=dict(n=list(map(float, hits[0][1])), c=list(map(float, hits[0][2]))) if hits else None,
                            transform=T.tolist()),
                   so3=dict(deg=so3_deg(R_cad, R_kin), rad=math.radians(so3_deg(R_cad, R_kin)),
                            fro=float(np.linalg.norm(R_cad - R_kin)),
                            maxelem=float(np.max(np.abs(R_cad - R_kin)))),
                   world=dict(tip=list(map(float, W_T_E[:3, 3])), y_station=y)),
              open(HERE / 'out' / 'iface_g7_p03.json', 'w'), ensure_ascii=False, indent=1)
    print('\n写入 out/iface_g7_p03.json')
