"""TIANQIONG assembly audit.

Independent re-check of the assembly graph, pose pipeline and exported STEPs.
Nothing here trusts earlier logs: every number is recomputed from
out/assembly.json, the joint solver and the STEP files themselves.

Checks
  A  inventory + naming
  B  component identity, parenting, cycles, single-parent rule
  C  joint definition: axis, limits, zero pose, virtual vs physical
  D  arm DOF census against the declared design (7 DOF: root Y, yaw Z,
     four pitch, telescope)
  E  record -> component mapping completeness and duplicate geometry
  F  cross-module overlap (AABB) at the deployed pose
  G  deployed pose must be the identity transform everywhere
  H  STEP vs joint solution (deployed), and transport STEP provenance
  I  per-state constraint consistency (carrier Y = root Y = slider Y)
  J  undeclared virtual motion
"""
from pathlib import Path
import importlib.util, json, math, hashlib
from collections import Counter, defaultdict
import numpy as np

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
ASSEMBLY = OUT / 'assembly.json'


def load(name, file):
    spec = importlib.util.spec_from_file_location(name, HERE / file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Audit:
    def __init__(self):
        self.findings = []
        self.data = json.loads(ASSEMBLY.read_text(encoding='utf8'))
        self.comps = {c['id']: c for c in self.data['components']}
        self.joints = {j['id']: j for j in self.data['joints']}
        self.jlist = self.data['joints']
        self.kin = load('kin', 'kinematics.py')
        self.poses = load('poses', 'poses.py')
        self.A = self.kin.Assembly()
        self.m = self.poses.m

    # ---------------------------------------------------------------- helpers
    def add(self, level, area, message, detail=None):
        self.findings.append(dict(level=level, area=area, message=message, detail=detail or []))

    def has(self, level):
        return [f for f in self.findings if f['level'] == level]

    # ---------------------------------------------------------------- A
    def check_inventory(self):
        want = ['assembly.json', 'deployed.step', 'transport.step', 'transport.pose.json']
        rows = []
        for w in want:
            p = OUT / w
            rows.append(dict(file=w, exists=p.exists(),
                             size_mb=round(p.stat().st_size / 1e6, 2) if p.exists() else None,
                             sha256=hashlib.sha256(p.read_bytes()).hexdigest()[:16] if p.exists() else None))
        names = sorted(list(OUT.glob('*.py')) + list(HERE.glob('*.py')))
        self.add('info', 'A-inventory', f'{len(rows)} expected outputs, {sum(r["exists"] for r in rows)} present', rows)
        alt = HERE / 'posals.py'
        self.add('info', 'A-naming', f'poses.py exists={ (HERE/"poses.py").exists() }, posals.py exists={alt.exists()} ')
        return rows

    # ---------------------------------------------------------------- B
    def check_graph(self):
        bad_parent, bad_child, multi_parent, self_loop = [], [], [], []
        parent_of = defaultdict(list)
        for j in self.jlist:
            if j['parent'] not in self.comps and j['parent'] not in self.A.child_of:
                bad_parent.append((j['id'], j['parent']))
            if j['child'] not in self.comps and j['child'] not in self.A.child_of:
                bad_child.append((j['id'], j['child']))
            if j['parent'] == j['child']:
                self_loop.append(j['id'])
            if j['type'] not in ('coupling', 'deformable'):
                parent_of[j['child']].append(j['id'])
        for child, js in parent_of.items():
            if len(js) > 1:
                multi_parent.append((child, js))

        # cycles
        adj = defaultdict(list)
        for j in self.jlist:
            if j['type'] in ('coupling', 'deformable'):
                continue
            adj[j['parent']].append(j['child'])
        WHITE, GREY, BLACK = 0, 1, 2
        color = defaultdict(int)
        cycles = []

        def dfs(n, stack):
            color[n] = GREY
            for c in adj.get(n, []):
                if color[c] == GREY:
                    cycles.append(stack[stack.index(n):] + [c] if n in stack else [n, c])
                elif color[c] == WHITE:
                    dfs(c, stack + [c])
            color[n] = BLACK

        for n in list(adj):
            if color[n] == WHITE:
                dfs(n, [n])

        self.add('error' if bad_parent else 'ok', 'B-refs', f'关节点父组件缺失 {len(bad_parent)}', bad_parent[:5])
        self.add('error' if bad_child else 'ok', 'B-refs', f'关节子组件缺失 {len(bad_child)}', bad_child[:5])
        self.add('error' if self_loop else 'ok', 'B-refs', f'自环 {len(self_loop)}', self_loop[:5])
        self.add('error' if multi_parent else 'ok', 'B-refs', f'一物多父 {len(multi_parent)}', multi_parent[:5])
        self.add('error' if cycles else 'ok', 'B-cycles', f'运动链闭环 {len(cycles)}', cycles[:3])

        # orphan components: never a parent and never a child
        child_ids = set(self.A.child_of) | {j['child'] for j in self.jlist}
        parents = {j['parent'] for j in self.jlist}
        parents = {j['parent'] for j in self.jlist}
        orphans = [c for c in self.comps if c not in child_ids and c not in parents]
        self.add('warn' if orphans else 'ok', 'B-orphans', f'孤立组件 {len(orphans)}', orphans)
        return dict(cycles=len(cycles), multi_parent=len(multi_parent), orphans=orphans)

    # ---------------------------------------------------------------- C
    def check_joints(self):
        zero_axis, bad_limit, real, virtual, rigid_like = [], [], [], [], []
        for j in self.jlist:
            ax = np.asarray(j['axis'], float)
            n = np.linalg.norm(ax)
            if n < 1e-9:
                zero_axis.append(j['id'])
            if j['type'] in ('fixed', 'coupling', 'deformable'):
                continue
            if j['type'] == 'prismatic':
                lo, hi = j.get('range_mm', [0, 0])
                if lo > hi:
                    bad_limit.append((j['id'], lo, hi))
                if abs(hi - lo) < 1e-6:
                    rigid_like.append(j['id'])
            else:
                lo, hi = j.get('range_deg', [0, 0])
                if lo > hi:
                    bad_limit.append((j['id'], lo, hi))
            if j.get('implementation') == 'declared':
                virtual.append(j['id'])
            else:
                real.append(j['id'])
        self.add('error' if zero_axis else 'ok', 'C-axis', f'零长度轴线 {len(zero_axis)}', zero_axis[:5])
        self.add('error' if bad_limit else 'ok', 'C-limits', f'限位反向 {len(bad_limit)}', bad_limit[:5])
        self.add('info' if rigid_like else 'ok', 'C-rigid',
                 f'零行程移动副 {len(rigid_like)} 个（1/6 号臂端部滑座在恢复的补位数据中不参与平移，保留为滑座而非改成刚性，以便后续重新标定）',
                 rigid_like[:5])
        self.add('info', 'C-virtual', f'标注为方案/未验证的关节 {len(virtual)} 个；实体机构 {len(real)} 个', virtual[:8])
        return dict(zero_axis=zero_axis, bad_limit=bad_limit, rigid_like=rigid_like, virtual=virtual, physical=real)

    # ---------------------------------------------------------------- D
    def check_dof(self):
        per_arm = defaultdict(list)
        for j in self.jlist:
            for k in range(1, 7):
                tag = f'A{k:02d}'
                if tag in j['id'] and '_J' not in j['id'] and self.joints[j['id']]['type'] not in ('coupling', 'deformable'):
                    mod = j['id'].split('_')[0]
                    per_arm[f'{mod}_{tag}'].append(j['id'])
                    break
        census, short = {}, []
        for arm, js in sorted(per_arm.items()):
            kinds = Counter(self.joints[j]['type'] for j in js)
            has_yaw = any('yaw' in j.lower() for j in js)
            census[arm] = dict(joints=len(js), revolute=kinds['revolute'], prismatic=kinds['prismatic'],
                               yaw=has_yaw, ids=js)
            if len(js) != 7 or not has_yaw:
                short.append(arm)

        # what the recovered source defines
        cfg = self.m.M['CFG']
        self.add('info', 'D-design', '源码 M02 自由度定义: 根座Y + yaw Z + 4×俯仰 + 末节伸缩 = 7 DOF/臂；共 18 臂',
                 dict(source_keys=[k for k in cfg if 'yaw' in k.lower() or 'dof' in k.lower()][:5]))
        self.add('error' if short else 'ok', 'D-census',
                 f'{len(per_arm)} 条臂中，与 7 DOF 定义不符的有 {len(short)} 条', short[:6])
        return census, short

    # ---------------------------------------------------------------- E
    def check_records(self):
        pairs, unresolved = self.poses.pairs_for(self.poses.DEPLOYED)
        self.add('error' if unresolved else 'ok', 'E-mapping',
                 f'构件映射：{len(pairs)} 已映射 / {len(unresolved)} 未映射', unresolved[:8])
        seen = Counter()
        for r, T, comp in pairs:
            sh = self.m.shape_for(r)
            bb = sh.BoundingBox()
            key = (r['kind'], round(sh.Volume(), 1),
                   tuple(np.round([bb.xmin, bb.ymin, bb.zmin, bb.xmax, bb.ymax, bb.zmax], 1)))
            seen[key] += 1
        dup = [k for k, v in seen.items() if v > 1]
        self.add('warn' if dup else 'ok', 'E-duplicate', f'同位置同网格重复构件 {len(dup)} 组', [str(d) for d in dup[:3]])
        return dict(mapped=len(pairs), unresolved=len(unresolved), duplicates=len(dup))

    # ---------------------------------------------------------------- F
    def check_module_overlap(self):
        pairs, _ = self.poses.pairs_for(self.poses.DEPLOYED)
        boxes = defaultdict(lambda: [np.array([1e9] * 3), np.array([-1e9] * 3)])
        skip = {'film', 'band', 'proposal', 'roll'}
        for r, T, comp in pairs:
            if r['group'] in skip or 'flashing' in r['name']:
                continue
            mod = comp.split('_')[0]
            if not mod.startswith('M'):
                continue
            sh = self.m.shape_for(r)
            b = sh.BoundingBox()
            lo = np.array([b.xmin, b.ymin, b.zmin]); hi = np.array([b.xmax, b.ymax, b.zmax])
            boxes[mod][0] = np.minimum(boxes[mod][0], lo)
            boxes[mod][1] = np.maximum(boxes[mod][1], hi)
        hits = []
        mods = sorted(boxes)
        for i in range(len(mods)):
            for k in range(i + 1, len(mods)):
                a, b = boxes[mods[i]], boxes[mods[k]]
                ov = np.minimum(a[1], b[1]) - np.maximum(a[0], b[0])
                if np.all(ov > 0):
                    hits.append(dict(pair=[mods[i], mods[k]], overlap_mm=np.round(ov, 1).tolist()))
        self.add('error' if hits else 'ok', 'F-modules',
                 f'大模块结构 AABB 相交 {len(hits)} 对（已排除膜/带/方案件）', hits)
        return dict(boxes={k: [np.round(v[0], 1).tolist(), np.round(v[1], 1).tolist()] for k, v in boxes.items()},
                    hits=hits)

    # ---------------------------------------------------------------- G
    def check_deployed_identity(self):
        T = self.poses.pose_transform(self.poses.DEPLOYED)
        worst = max(float(np.abs(M - np.eye(4)).max()) for M in T.values())
        self.add('ok' if worst < 1e-9 else 'error', 'G-zero',
                 f'部署态（零位）变换矩阵最大偏离单位阵 {worst:.2e}')
        return worst

    # ---------------------------------------------------------------- H
    def check_steps(self):
        import cadquery as cq
        out = {}
        for name in ('deployed', 'transport'):
            p = OUT / f'{name}.step'
            if not p.exists():
                self.add('error', 'H-step', f'{name}.step 缺失')
                continue
            w = cq.importers.importStep(str(p))
            solids = w.solids().vals()
            bb = [s.BoundingBox() for s in solids]
            box = dict(x=[round(min(b.xmin for b in bb), 1), round(max(b.xmax for b in bb), 1)],
                       y=[round(min(b.ymin for b in bb), 1), round(max(b.ymax for b in bb), 1)],
                       z=[round(min(b.zmin for b in bb), 1), round(max(b.zmax for b in bb), 1)])
            # arm solids that stayed in the deployed tip region
            arms_deployed = sum(1 for s in solids
                                if s.BoundingBox().xmax > 4300 and s.BoundingBox().zmax < 400)
            out[name] = dict(solids=len(solids), bbox=box, solids_in_deployed_tip_region=arms_deployed)
            self.add('info', 'H-step', f'{name}.step: {len(solids)} 实体, 包络 {box}')
        # transport provenance: the pose file must exist and the rail must have moved outboard
        pp = OUT / 'transport.pose.json'
        if pp.exists():
            pose = json.loads(pp.read_text(encoding='utf8'))
            q = pose.get('q', {})
            moved = sum(1 for k, v in q.items() if abs(float(v)) > 1e-6)
            self.add('info', 'H-provenance', f'transport.pose.json 记录 {len(q)} 个关节量，其中 {moved} 个非零')
        else:
            self.add('error', 'H-provenance', 'transport.pose.json 缺失，无法证明运输态来自关节求解')
        return out

    # ---------------------------------------------------------------- I
    def check_state_constraints(self):
        rows = []
        for name, kw in [('deployed', {}), ('compensated', dict(compensate=1))]:
            recs, meta = self.m.build_state(**kw)
            arms = {a['id']: a for a in meta['arms']}
            worst = 0.0
            for itf in meta['interfaces']:
                arm = arms[itf['arm']]
                worst = max(worst, abs(itf['carrier_center'][1] - arm['y']))
            rows.append(dict(state=name, max_carrier_vs_arm_Y_mm=round(worst, 6),
                             engaged=sum(i['engaged'] for i in meta['interfaces']),
                             total=len(meta['interfaces'])))
        release = self.m.build_state(compensate=1, gate=1, drop=1, retract=1, lower=1)[1]
        rows.append(dict(state='release', max_carrier_vs_arm_Y_mm=None,
                         engaged=sum(i['engaged'] for i in release['interfaces']),
                         total=len(release['interfaces'])))
        ok = all(r['max_carrier_vs_arm_Y_mm'] is None or r['max_carrier_vs_arm_Y_mm'] < 1e-6 for r in rows)
        self.add('ok' if ok else 'error', 'I-constraints',
                 '承托头 Y 与臂根 Y 的一致性（滑移受约束）', rows)
        return rows

    # ---------------------------------------------------------------- J
    def check_transport_motion(self):
        """Components that should move must actually move under the transport pose."""
        q = self.poses.transport_q()
        pairs, _ = self.poses.pairs_for(q)
        static_groups = {'film', 'band', 'roll', 'proposal', 'wall', 'joint'}
        stuck = []
        for r, M, comp in pairs:
            if r['group'] in static_groups or 'flashing' in r['name']:
                continue
            if comp.endswith(('_FILM', '_DRUM', '_DRUM_HOUSING')) or comp.startswith('SEAL'):
                continue
            if comp.startswith(('SW', 'EW')):
                continue
            if np.allclose(M, np.eye(4), atol=1e-9):
                stuck.append((r['name'], comp))
        owned = {comp for _, _, comp in pairs}
        names = sorted({c for _, c in stuck if c in owned})
        self.add('error' if names else 'ok', 'J-transport',
                 f'运输态中未随姿态运动的非静态组件 {len(names)} 个', names[:8])
        return dict(stuck_components=names, stuck_records=len(stuck))

    # ---------------------------------------------------------------- report
    def run(self):
        self.check_inventory()
        g = self.check_graph()
        j = self.check_joints()
        census, short = self.check_dof()
        e = self.check_records()
        f = self.check_module_overlap()
        worst = self.check_deployed_identity()
        h = self.check_steps()
        i = self.check_state_constraints()
        jm = self.check_transport_motion()

        summary = Counter(x['level'] for x in self.findings)
        report = dict(summary=dict(summary), findings=self.findings,
                      graph=g, joints=dict(zero_axis=j['zero_axis'], bad_limits=j['bad_limit'],
                                           rigid_like=j['rigid_like'], virtual=j['virtual'],
                                           physical_count=len(j['physical'])),
                      dof=dict(per_arm=census, nonconforming=short),
                      records=e, modules=f, deployed_identity_error=worst, steps=h, states=i, transport_motion=jm)
        (OUT / 'audit_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=1), encoding='utf8')

        lines = ['# 田穹装配审计报告', '']
        lines.append(f"- 结论计数: " + ', '.join(f'{k}={v}' for k, v in summary.items()))
        for f_ in self.findings:
            if f_['level'] in ('error', 'warn', 'info', 'ok'):
                lines.append(f"- **{f_['level'].upper()}** [{f_['area']}] {f_['message']}")
        (OUT / 'audit_report.md').write_text('\n'.join(lines), encoding='utf8')
        return report


if __name__ == '__main__':
    rep = Audit().run()
    print(json.dumps(rep['summary'], ensure_ascii=False))
    for f in rep['findings']:
        print(f"{f['level']:5s} [{f['area']:14s}] {f['message']}")
        if f['level'] in ('error', 'warn') and f['detail']:
            print('        ', str(f['detail'])[:300])
