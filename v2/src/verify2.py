"""第二阶段补充审计：组件/刚体/关节口径、逐组件 STEP 比对、BR750 连接、承托头账目。"""
from pathlib import Path
import importlib.util, json
from collections import Counter, defaultdict
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, HERE / file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


kin = load('kin', 'kinematics.py')
poses = load('poses', 'poses.py')
A = poses.A
d = json.loads((OUT / 'assembly.json').read_text(encoding='utf8'))
comps, joints = d['components'], d['joints']

# ---------------------------------------------------------------- 1 口径
by_kind = Counter(c['kind'] for c in comps)
pairs, _ = poses.pairs_for(poses.DEPLOYED)
rigid_with_records = {c for _, _, c in pairs}
comp_ids = {c['id'] for c in comps}
implicit_nodes = sorted(rigid_with_records - comp_ids)
frames_only = sorted(c['id'] for c in comps if c['id'] not in rigid_with_records)
virtual = [j['id'] for j in joints if j.get('implementation') == 'declared']
physical = [j['id'] for j in joints if j.get('implementation') == 'physical']
fixed = [j['id'] for j in joints if j['type'] == 'fixed']
jtype = Counter(j['type'] for j in joints)

print('== 1 口径统计 ==')
print(f"功能组件 {len(comps)}（声明层）")
print(f"实际刚体节点 {len(rigid_with_records)} = 组件中有几何的 {len(comp_ids & rigid_with_records)} + 隐式连杆节点 {len(implicit_nodes)}")
print(f"纯坐标系/分组节点 {len(frames_only)}（无几何，仅承载关节链）")
print("隐式连杆节点示例:", implicit_nodes[:8])
print(f"关节 {len(joints)} = 转动 {jtype['revolute']} + 移动 {jtype['prismatic']} + 固定 {jtype['fixed']}")
print(f"实体机构 {len(physical)} / 方案件(declared) {len(virtual)} / 固定连接 {len(fixed)}")
print("组件类型:", dict(by_kind))
print("纯坐标系节点:", frames_only[:12], '...' if len(frames_only) > 12 else '')

# ---------------------------------------------------------------- 2 逐组件比对
import cadquery as cq
from scipy.spatial import cKDTree

w = cq.importers.importStep(str(OUT / 'deployed.step'))
solids = w.solids().vals()
scent = np.array([[s.Center().x, s.Center().y, s.Center().z] for s in solids])
svol = np.array([s.Volume() for s in solids])
tree = cKDTree(scent)

rows = []
for cid in sorted(rigid_with_records):
    pass

rows = []
for r, T, cid in pairs:
    sh = poses.m.shape_for(r)
    vol = sh.Volume(); cen = sh.Center()
    cent = np.array([cen.x, cen.y, cen.z])
    b = sh.BoundingBox()
    lo = np.array([b.xmin, b.ymin, b.zmin]); hi = np.array([b.xmax, b.ymax, b.zmax])
    dist, idx = tree.query(cent)
    rows.append(dict(component=cid, record=r['name'], volume_mm3=round(vol, 1),
                     fk_centroid=np.round(cent, 2).tolist(),
                     nearest_step_centroid=np.round(scent[idx], 2).tolist(),
                     centroid_gap_mm=round(float(dist), 4),
                     volume_gap_rel=round(abs(float(svol[idx]) - vol) / max(vol, 1e-9), 6),
                     fk_aabb=[np.round(lo, 1).tolist(), np.round(hi, 1).tolist()]))
gaps = np.array([r['centroid_gap_mm'] for r in rows])
vgap = np.array([r['volume_gap_rel'] for r in rows])
print('\n== 2 逐组件比对（与 deployed.step 最近实体）==')
print(f"比对构件 {len(rows)} 个；质心偏差 max {gaps.max():.3f} mm / mean {gaps.mean():.3f} mm；"
      f"体积相对偏差 max {vgap.max():.2e}")
print(f"质心偏差 >1 mm 的组件: {int((gaps > 1).sum())}；>5 mm: {int((gaps > 5).sum())}")

# 抽样顶点：取 3 个组件比对顶点集
sample_checks = []
for cid in ['M1_W01', 'M1_A01_link1', 'M1_A02_J1']:
    recs = [r for r, T, c in pairs if c == cid]
    if not recs:
        continue
    pts = []
    for r in recs:
        sh = poses.m.shape_for(r)
        v, _ = sh.tessellate(5.0, 0.5)
        pts += [[p.x, p.y, p.z] for p in v]
    pts = np.array(pts)
    if len(pts) == 0:
        continue
    b = poses.m.shape_for(recs[0]).BoundingBox()
    near = [s for s in solids
            if s.Center().x >= b.xmin and s.Center().x <= b.xmax
            and s.Center().y >= b.ymin and s.Center().y <= b.ymax
            and s.Center().z >= b.zmin and s.Center().z <= b.zmax]
    sample_checks.append(dict(component=cid, fk_vertices=len(pts),
                              fk_vertex_mean=np.round(pts.mean(0), 2).tolist(),
                              fk_vertex_bbox=[np.round(pts.min(0), 1).tolist(), np.round(pts.max(0), 1).tolist()],
                              step_solids_in_bbox=len(near)))

# ---------------------------------------------------------------- 3 BR750 连接
br = [j for j in joints if j['child'].startswith('BR')]
br_rows = []
for j in br:
    child = j['child']
    W = A.world(poses.DEPLOYED)
    p_deploy = W[child][:3, 3] if child in W else None
    Wt = A.world(poses.transport_q())
    p_trans = Wt[child][:3, 3] if child in Wt else None
    br_rows.append(dict(bridge=child, parent=j['parent'], type=j['type'],
                        deployed_xyz=np.round(p_deploy, 1).tolist() if p_deploy is not None else None,
                        transport_xyz=np.round(p_trans, 1).tolist() if p_trans is not None else None))
gap_centers = [3500.0 + k * 4250 + 375 for k in range(2)]
print('\n== 3 BR750 连接关系 ==')
for r, gy in zip(br_rows, gap_centers):
    print(f"  {r['bridge']}: parent={r['parent']}  部署 y={r['deployed_xyz'][1]:.0f} (空档中心 {gy:.0f})  "
          f"运输 y={r['transport_xyz'][1]:.0f} x={r['transport_xyz'][0]:.0f}")

# ---------------------------------------------------------------- 4 承托头账目
recs, meta = poses.m.build_state(modules=1)
arms = {a['id']: a for a in meta['arms']}
print('\n== 4 承托头账目（单模块 / 三模块）==')
print(f"单模块接口 {len(meta['interfaces'])}，承托 {sum(i['engaged'] for i in meta['interfaces'])}")
rel = poses.m.build_state(modules=1, compensate=1, gate=1, drop=1, retract=1, lower=1)[1]
rel_engaged = sum(i['engaged'] for i in rel['interfaces'])
print(f"释放态（2/5 号）承托 {rel_engaged}/{len(rel['interfaces'])} → 三模块 = {rel_engaged*3}/54")
released = [i['id'] for i in rel['interfaces'] if not i['engaged']]
print("脱离接口:", len(released), released[:6], '...')

report = dict(taxonomy=dict(components=len(comps), rigid_bodies=len(rigid_with_records),
                            frames_only=frames_only, joints=dict(jtype), physical=len(physical),
                            declared=len(virtual), fixed=len(fixed)),
              per_component=dict(count=len(rows), centroid_gap_max_mm=float(gaps.max()),
                                 centroid_gap_mean_mm=float(gaps.mean()),
                                 volume_gap_rel_max=float(vgap.max()),
                                 over_1mm=int((gaps > 1).sum()), rows=rows),
              rigid_nodes=sorted(rigid_with_records),
              vertex_samples=sample_checks, bridges=br_rows,
              carriers=dict(total=54, engaged_release_state=rel_engaged * 3,
                            released_interfaces=released * 3 if len(released) == 6 else released))
(OUT / 'verify2_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf8')
print('\nwritten out/verify2_report.json')
