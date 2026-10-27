"""查看用户 SolidWorks 模型：真实投影生成 等轴测 + 正视 + 侧视 + 俯视。"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np, cadquery as cq
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_UniformAbscissa

plt.rcParams['font.sans-serif'] = ['Microsoft YaHei', 'SimHei']
plt.rcParams['axes.unicode_minus'] = False

sh = cq.importers.importStep("out/user_model/用户装配体.step").val()
sol = sorted(sh.Solids(), key=lambda s: -s.Volume())
colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']


def edges_of(shape, n=10):
    out = []
    for e in shape.Edges():
        try:
            a = BRepAdaptor_Curve(e.wrapped)
            u = GCPnts_UniformAbscissa(a, n)
            if not u.IsDone():
                continue
            pts = [a.Value(u.Parameter(i + 1)) for i in range(u.NbPoints())]
            out.append(np.array([[p.X(), p.Y(), p.Z()] for p in pts]))
        except Exception:
            pass
    return out


def draw(ax, direction, up, title, flat=True):
    d = np.array(direction, float); d /= np.linalg.norm(d)
    u = np.array(up, float); u -= d * (u @ d); u /= np.linalg.norm(u)
    r = np.cross(u, d)
    for k, s in enumerate(sol):
        for pl in edges_of(s):
            X = pl @ r; Y = pl @ u
            ax.plot(X, Y, color=colors[k % len(colors)], lw=0.55, alpha=0.9)
    ax.set_aspect('equal'); ax.set_title(title, fontsize=11, loc='left')
    ax.grid(True, lw=0.2, alpha=0.35); ax.tick_params(labelsize=7)
    if flat:
        for sp in ('top', 'right'):
            ax.spines[sp].set_visible(False)


fig = plt.figure(figsize=(15, 11), dpi=140)
fig.patch.set_facecolor('white')
ax1 = fig.add_subplot(2, 2, 1); draw(ax1, (1, 1, 0.55), (0, 0, 1), '等轴测（斜上方）')
ax2 = fig.add_subplot(2, 2, 2); draw(ax2, (1, 0, 0), (0, 0, 1), '正视图（沿 −X，Y–Z）')
ax3 = fig.add_subplot(2, 2, 3); draw(ax3, (0, 1, 0), (0, 0, 1), '侧视图（沿 −Y，X–Z）')
ax4 = fig.add_subplot(2, 2, 4); draw(ax4, (0, 0, 1), (0, 1, 0), '俯视图（沿 −Z，X–Y）')
handles = [plt.Line2D([], [], color=colors[i], lw=2,
                      label=f'实体{i}  V={s.Volume()/1e6:.1f} dm³') for i, s in enumerate(sol)]
fig.legend(handles=handles, loc='lower center', ncol=5, fontsize=9, frameon=False)
fig.suptitle('用户模型 装配体.SLDASM（转 STEP 后投影）  包络 5156 × 4600 × 3444 mm', fontsize=13, x=0.06, ha='left')
fig.tight_layout(rect=[0, 0.04, 1, 0.96])
fig.savefig('../renders/用户模型_四视图.png', bbox_inches='tight', facecolor='white')
print('saved')
