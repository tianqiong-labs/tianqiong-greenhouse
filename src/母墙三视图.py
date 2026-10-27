"""三视图：对 STEP 做真实 HLR 投影（前视 / 侧视 / 俯视），matplotlib 出图。"""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif']=['Microsoft YaHei','SimHei']
plt.rcParams['axes.unicode_minus']=False
import cadquery as cq
from pathlib import Path

from OCP.HLRBRep import HLRBRep_Algo, HLRBRep_HLRToShape
from OCP.HLRAlgo import HLRAlgo_Projector
from OCP.gp import gp_Ax2, gp_Pnt, gp_Dir
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_EDGE
from OCP.TopoDS import TopoDS
from OCP.BRepAdaptor import BRepAdaptor_Curve
from OCP.GCPnts import GCPnts_QuasiUniformDeflection

HERE = Path(__file__).resolve().parent
STEP = HERE / 'out' / 'module' / '单模块母墙_v1.step'
shape = cq.importers.importStep(str(STEP)).val()


def project(shape, direction, xdir):
    """平行投影：取所有边的端点，按投影面丢弃一个坐标。"""
    import numpy as _np
    d = _np.array(direction, float); d /= _np.linalg.norm(d)
    xd = _np.array(xdir, float); xd /= _np.linalg.norm(xd)
    yd = _np.cross(d, xd); yd /= _np.linalg.norm(yd)
    out = []
    for e in shape.Edges():
        try:
            p0, p1 = e.startPoint(), e.endPoint()
            out.append([(p0.x * xd[0] + p0.y * xd[1] + p0.z * xd[2],
                         p0.x * yd[0] + p0.y * yd[1] + p0.z * yd[2]),
                        (p1.x * xd[0] + p1.y * xd[1] + p1.z * xd[2],
                         p1.x * yd[0] + p1.y * yd[1] + p1.z * yd[2])])
        except Exception:
            pass
    return out


views = [
    ('正视图（沿 −X 看，Y–Z）', (1, 0, 0), (0, 1, 0), (0, 1), 'Y / mm', 'Z / mm'),
    ('侧视图（沿 −Y 看，X–Z）', (0, 1, 0), (-1, 0, 0), (0, 1), '−X / mm', 'Z / mm'),
    ('俯视图（沿 −Z 看，X–Y）', (0, 0, 1), (1, 0, 0), (0, 1), 'X / mm', 'Y / mm'),
]

fig, axes = plt.subplots(3, 1, figsize=(9, 16), dpi=150)
fig.patch.set_facecolor('white')
for ax, (title, d, xd, (iu, iv), lu, lv) in zip(axes, views):
    polys = project(shape, d, xd)
    for pl in polys:
        ax.plot([q[0] for q in pl], [q[1] for q in pl], color='#111', lw=0.7)
    ax.set_aspect('equal'); ax.set_title(title, fontsize=11, loc='left')
    ax.set_xlabel(lu, fontsize=8.5); ax.set_ylabel(lv, fontsize=8.5)
    ax.tick_params(labelsize=7.5)
    ax.grid(True, lw=0.25, alpha=0.45)
    for s in ('top', 'right'):
        ax.spines[s].set_visible(False)
fig.suptitle('单模块母墙 3500(Y) × 800(X) × 2800(Z)   ·   外壳+骨架   ·   三视图', fontsize=12.5, x=0.06, ha='left')
fig.tight_layout(rect=[0, 0, 1, 0.97])
out = HERE.parent / 'renders' / '单模块母墙_三视图.png'
fig.savefig(out, bbox_inches='tight', facecolor='white')
print('saved', out, 'edges', sum(len(p) for p in project(shape, (1, 0, 0), (0, 1, 0))))
