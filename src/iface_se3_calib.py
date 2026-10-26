"""SE(3) 校准（P0）：符号、CAD 特征点独立校核、特征点扰动、完整 XYZ 误差。"""
import math, json
from pathlib import Path
import importlib.util
import numpy as np

HERE = Path(__file__).resolve().parent
sp = importlib.util.spec_from_file_location('pl', str(HERE / 'planner.py'))
pl = importlib.util.module_from_spec(sp); sp.loader.exec_module(pl)

Q0 = [-1.6, -20.1, -38.63402497781479, -57.18221617233679]
L0 = 1600.0
CAND = {4250: [0, -23.451559, -44.957417, -57.2],
        4280: [0, -26.313807, -41.716171, -57.2],
        4230: [0, -21.978773, -46.727991, -57.2]}
L4C = 1400.0


def tip(q, L4):
    p = pl.chain3d(q, L4)[-1]
    return np.array([float(p[0]), 0.0, float(p[2])])


def R_Y(deg):
    t = math.radians(deg); c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def W_T_E(q, L4):
    T = np.eye(4); T[:3, :3] = R_Y(-q[3]); T[:3, 3] = tip(q, L4)
    return T


def ang(R):
    return math.degrees(math.acos(max(-1, min(1, (np.trace(R) - 1) / 2))))


# CAD 安装基准：接口系与世界系同向（法兰水平、卡箍竖直、开口 +X），原点=球心
W_T_I0 = np.eye(4); W_T_I0[:3, 3] = tip(Q0, L0)
E_T_E0 = W_T_E(Q0, L0)
E_T_I = np.linalg.inv(E_T_E0) @ W_T_I0

# —— 符号核对：E_T_I 的旋转对应 R_Y(?) ——
det = np.linalg.det(E_T_I[:3, :3])
# 由 R_Y(θ) 的 s 分量反解
theta = math.degrees(math.atan2(-E_T_I[2, 0], E_T_I[0, 0]))   # R_Y 中 R[2,0]=-sinθ
print(f"E_T_I 旋转 = R_Y({theta:+.6f}°)  det={det:.9f}")
print(f"符号按 R[2,0]=-sinθ 反解；旧文档写的正值有误")

# —— 独立 CAD 特征点 ——
# 取部署态下接口三个不共线特征点（世界系，来自 γ-2 装配基准）
FEAT_W = {
    'ball_center': np.array([4500.0, 0.0, 0.0]),
    'slot_out_x30': np.array([4530.0, 0.0, 0.0]),      # 开口方向 +X 30 mm
    'flange_up_z80': np.array([4500.0, 0.0, 80.0]),    # 腕柱轴 +Z 80 mm
    'clamp_side_y25': np.array([4500.0, 25.0, 100.0]), # 卡箍侧向
}

T_I0 = W_T_E(Q0, L0) @ E_T_I
err = {k: float(np.linalg.norm(T_I0[:3, :3] @ (v - FEAT_W['ball_center']) + T_I0[:3, 3] - v))
       for k, v in FEAT_W.items()}
print("\nCAD 特征点复现误差（mm）:", {k: f'{v:.2e}' for k, v in err.items()})
print("最大:", f"{max(err.values()):.2e}")

# —— 特征点扰动：刚性随动 + 距离保持 ——
def feat_after(q, L4):
    T = W_T_E(q, L4) @ E_T_I
    return {k: T[:3, :3] @ (v - FEAT_W['ball_center']) + T[:3, 3] for k, v in FEAT_W.items()}


base = feat_after(Q0, L0)
rows = []
for dq, dL in (((7, -5, 4, 6), 0), ((0, 0, 0, 12), 0), ((0, 0, 0, 0), -120)):
    q = [Q0[i] + dq[i] for i in range(4)]
    f = feat_after(q, L0 + dL)
    # 距离保持（刚性）
    dmax = 0
    keys = list(FEAT_W)
    for i in range(len(keys)):
        for j in range(i + 1, len(keys)):
            d0 = np.linalg.norm(base[keys[i]] - base[keys[j]])
            d1 = np.linalg.norm(f[keys[i]] - f[keys[j]])
            dmax = max(dmax, abs(d1 - d0))
    # 与 FK 预测的球心一致性
    fk_tip = tip(q, L0 + dL)
    rows.append(dict(dq=list(dq), dL=dL, rigid_dev_mm=float(dmax),
                     tip_err=float(np.linalg.norm(f['ball_center'] - fk_tip)),
                     aperture=[round(float(v), 4) for v in
                               (W_T_E(q, L0 + dL) @ E_T_I)[:3, 0]]))
for r in rows:
    print(f"  dq={r['dq']} dL={r['dL']:+d}: 特征点距离偏差 {r['rigid_dev_mm']:.2e} mm, "
          f"球心 vs FK {r['tip_err']:.2e} mm")

# —— 候选：完整 XYZ 误差 ——
print("\n候选（完整 XYZ）")
cand_out = {}
for x, q in CAND.items():
    T = W_T_E(q, L4C) @ E_T_I
    pos = T[:3, 3]
    d = pos - np.array([x, 0.0, 0.0])
    cand_out[str(x)] = dict(pos=[round(float(v), 4) for v in pos],
                            dxyz=[round(float(v), 6) for v in d],
                            norm=round(float(np.linalg.norm(d)), 6),
                            rot_deg=round(ang(T[:3, :3]), 6))
    print(f"  x={x}: 位置 ({pos[0]:.4f},{pos[1]:.4f},{pos[2]:.4f})  "
          f"误差 ({d[0]:+.2e},{d[1]:+.2e},{d[2]:+.2e}) 范数 {np.linalg.norm(d):.2e}  "
          f"旋转 {ang(T[:3, :3]):.4f}°")

out = dict(E_T_I=E_T_I.tolist(), E_T_I_angle_deg=round(theta, 6), det=round(float(det), 9),
           feature_err={k: float(v) for k, v in err.items()},
           perturbation=rows, candidates=cand_out,
           note='符号以 atan2 反解为准；特征点独立于程序构造的单位阵')
(HERE / 'out' / 'iface_se3_calib.json').write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding='utf8')
print('\n写入 out/iface_se3_calib.json')
