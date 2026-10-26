"""γ-2 真实 SE(3) 固定安装变换（本轮关口）。

W_T_E(q)：原点=球心，+X 沿末段方向（绕 Y 旋转 R_Y(-a4)）
W_T_I(q0)：来自 CAD 安装基准——原点=球心，+X=开口(+X)，+Z=腕柱轴(+Z) → 单位旋转
E_T_I = inv(W_T_E(q0)) @ W_T_I(q0)，为常数
W_T_I(q) = W_T_E(q) @ E_T_I
"""
import math, json
from pathlib import Path
import importlib.util
import numpy as np

HERE = Path(__file__).resolve().parent
sp = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp); sp.loader.exec_module(pl)

Q0 = [-1.6, -20.1, -38.63402497781479, -57.18221617233679]
L0 = 1600.0
CAND = {
    4250: [0.0, -23.451559, -44.957417, -57.2],
    4280: [0.0, -26.313807, -41.716171, -57.2],
    4230: [0.0, -21.978773, -46.727991, -57.2],
}
L4_C = 1400.0


def tip(q, L4):
    p = pl.chain3d(q, L4)[-1]
    return np.array([float(p[0]), 0.0, float(p[2])])


def R_Y(deg):
    t = math.radians(deg)
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def W_T_E(q, L4):
    T = np.eye(4)
    T[:3, :3] = R_Y(-q[3])          # +X 指向末段方向
    T[:3, 3] = tip(q, L4)
    return T


def orthocheck(R):
    return dict(orth=float(np.max(np.abs(R.T @ R - np.eye(3)))),
                det=float(np.linalg.det(R)))


if __name__ == '__main__':
    # CAD 安装基准：接口系与世界系同向，原点在球心
    W_T_I0 = np.eye(4); W_T_I0[:3, 3] = tip(Q0, L0)
    E_T_E = W_T_E(Q0, L0)
    E_T_I = np.linalg.inv(E_T_E) @ W_T_I0

    print('=== E_T_I（固定安装，一次性求出）===')
    print(np.array2string(E_T_I, precision=6, suppress_small=True))

    # 复现
    W_T_I = W_T_E(Q0, L0) @ E_T_I
    rep_pos = float(np.linalg.norm(W_T_I[:3, 3] - W_T_I0[:3, 3]))
    rep_rot = float(np.degrees(np.arccos(np.clip((np.trace(W_T_I[:3, :3].T @ W_T_I0[:3, :3]) - 1) / 2, -1, 1))))
    oc = orthocheck(W_T_I[:3, :3])
    print(f"\n复现：位置误差 {rep_pos:.3e} mm，旋转误差 {rep_rot:.3e}°，"
          f"|R^T R - I| {oc['orth']:.2e}，det {oc['det']:.9f}")

    # 刚性扰动：改变关节与 L4，E_T_I 必须不变
    pert = []
    for dq, dL in (((7, -5, 4, 6), 0), ((0, 0, 0, 12), 0), ((0, 0, 0, 0), -120)):
        q = [Q0[i] + dq[i] for i in range(4)]
        E_T_E2 = W_T_E(q, L0 + dL)
        I2 = np.linalg.inv(E_T_E2) @ (E_T_E2 @ E_T_I)
        dev = float(np.max(np.abs(I2 - E_T_I)))
        # 接口是否随动：开口方向
        op = (W_T_E(q, L0 + dL) @ E_T_I)[:3, 0]
        pert.append(dict(dq=list(dq), dL=dL, dev=dev,
                         opening=[round(float(v), 4) for v in op]))
    print('\n扰动（E_T_I 偏差应为 0；开口方向随末段变化）：')
    for p in pert:
        print(f"  dq={p['dq']} dL={p['dL']:+d} -> 偏差 {p['dev']:.2e}，开口 {p['opening']}")

    # 候复核
    rows = {}
    for x, q in CAND.items():
        T = W_T_E(q, L4_C) @ E_T_I
        pos_err = abs(float(T[0, 3]) - x)
        op = T[:3, 0]; ax = T[:3, 2]
        tilt = math.degrees(math.acos(max(-1, min(1, float(op[0])))))
        ax_tilt = math.degrees(math.acos(max(-1, min(1, float(ax[2])))))
        rows[str(x)] = dict(pos_err=round(pos_err, 6),
                            opening=[round(float(v), 5) for v in op],
                            axis=[round(float(v), 5) for v in ax],
                            opening_tilt_deg=round(tilt, 4), axis_tilt_deg=round(ax_tilt, 4))
        print(f"  x={x}: 位置误差 {pos_err:.2e} mm，开口 {rows[str(x)]['opening']} "
              f"倾角 {tilt:.4f}°，轴倾角 {ax_tilt:.4f}°")

    out = dict(E_T_I=E_T_I.tolist(), reproduce=dict(pos_err=rep_pos, rot_err_deg=rep_rot, **oc),
               perturbations=pert, candidates=rows,
               note='E_T_I 由部署态求出后恒定；开口/轴线用矩阵变换，不再用角度公式')
    (HERE / 'out' / 'iface_se3.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
    print('\n写入 out/iface_se3.json')
