"""Joint-space evaluation for the TIANQIONG assembly.

Reads out/assembly.json, evaluates any joint vector to world transforms, and
validates the zero pose against the recovered source FK. Anything that moves in
the animation must come through this evaluator, so motion cannot drift away
from the assembly graph.

Convention: joint origins/axes are stored in WORLD coordinates at the zero pose
(this is how they are generated and verified against the recovered sources).
The evaluator composes:

    child_world = parent_world @ inv(Z_parent) @ Z_joint @ M(q)

where Z_joint is the zero-pose frame whose +Z is the joint axis, and M(q) is a
rotation (revolute) or translation (prismatic) about/along that local +Z.
"""
from pathlib import Path
import json, math
import numpy as np

HERE = Path(__file__).resolve().parent
ASM = HERE / 'out' / 'assembly.json'


def _rot_z(deg):
    t = math.radians(deg)
    M = np.eye(4)
    M[0, 0] = math.cos(t); M[0, 1] = -math.sin(t)
    M[1, 0] = math.sin(t); M[1, 1] = math.cos(t)
    return M


def _trans_z(d):
    M = np.eye(4); M[2, 3] = d
    return M


def frame(origin, axis):
    """Frame at origin whose +Z points along axis (world coords)."""
    a = np.asarray(axis, float)
    a = a / np.linalg.norm(a)
    z = np.array([0, 0, 1.0])
    if np.allclose(a, z):
        R = np.eye(3)
    elif np.allclose(a, -z):
        R = np.diag([1.0, -1.0, -1.0])
    else:
        v = np.cross(z, a)
        s = np.linalg.norm(v)
        c = float(np.dot(z, a))
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        R = np.eye(3) + K + (K @ K) * ((1 - c) / (s * s))
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = origin
    return M


class Assembly:
    def __init__(self, path=ASM):
        self.data = json.loads(Path(path).read_text(encoding='utf8'))
        self.joints = {j['id']: j for j in self.data['joints']}
        self.comps = {c['id']: c for c in self.data['components']}
        self.by_parent = {}
        for j in self.data['joints']:
            self.by_parent.setdefault(j['parent'], []).append(j)
        self.Z = {j['id']: frame(j['origin'], j['axis']) for j in self.data['joints']}
        # zero-pose world frame of each component: identity for roots, else the
        # frame of the joint that creates it
        self.child_of = {j['child']: j['id'] for j in self.data['joints']
                         if j['type'] not in ('coupling', 'deformable')}
    def zero_frame_inv(self, comp):
        jid = self.child_of.get(comp)
        return np.linalg.inv(self.Z[jid]) if jid else np.eye(4)

    def zero_value(self, j):
        if j['type'] == 'fixed':
            return 0.0
        return float(j.get('zero_deg', 0.0)) if j['type'] == 'revolute' else float(j.get('zero_mm', 0.0))

    def motion(self, j, q):
        if j['type'] in ('fixed', 'coupling', 'deformable'):
            return np.eye(4)
        val = q.get(j['id'], self.zero_value(j))
        return _rot_z(val) if j['type'] == 'revolute' else _trans_z(val)

    def world(self, q=None):
        """World transform for every joint child frame (plus roots)."""
        q = q or {}
        out = {}
        child_ids = set(self.child_of)
        roots = [c for c in self.comps if c not in child_ids]

        def walk(comp, world, depth=0):
            if depth > 60:
                raise RuntimeError('kinematic loop at ' + comp)
            out[comp] = world
            for j in self.by_parent.get(comp, []):
                if j['type'] in ('coupling', 'deformable'):
                    continue          # 闭链约束/柔性件：不作为树边
                nxt = world @ self.zero_frame_inv(comp) @ self.Z[j['id']] @ self.motion(j, q)
                walk(j['child'], nxt, depth + 1)

        for r in roots:
            walk(r, np.eye(4))
        return out

    def point(self, q, joint_id):
        """World origin of a joint frame under joint vector q."""
        W = self.world(q)
        j = self.joints[joint_id]
        parent = j['parent']
        child = W[parent] @ self.zero_frame_inv(parent) @ self.Z[joint_id] @ self.motion(j, q)
        return child[:3, 3]


def validate(path=ASM, verbose=True):
    """Zero pose must reproduce the recovered source FK joint points."""
    import importlib.util
    spec = importlib.util.spec_from_file_location('gh', HERE / 'greenhouse.py')
    gh = importlib.util.module_from_spec(spec); spec.loader.exec_module(gh)
    m = gh.m
    A = Assembly(path)
    worst, rows = 0.0, []
    for i in range(6):
        arm = f'M1_A{i+1:02d}'
        ref = m.arm_points(m.Y0[i], 0.0, 1600.0)
        for k in range(4):
            jid = f'{arm}_pitch{k+1}'
            got = A.point({}, jid)
            err = float(np.linalg.norm(got - ref[k]))
            worst = max(worst, err)
            rows.append((jid, np.round(got, 1).tolist(), np.round(ref[k], 1).tolist(), err))
    if verbose:
        for r in rows[:5]:
            print(f'  {r[0]:22s} fk={r[1]}  source={r[2]}  err={r[3]:.2e}')
    return worst, rows


if __name__ == '__main__':
    A = Assembly()
    print(f'components: {len(A.comps)}   joints: {len(A.joints)}')
    worst, _ = validate()
    print(f'\nzero-pose FK check: worst error = {worst:.9f} mm')
    # envelope check for the stowage pose
    q = {}
    for i in range(6):
        arm = f'M1_A{i+1:02d}'
        for k, ang in enumerate([-75, 75, -75, 75]):
            q[f'{arm}_pitch{k+1}'] = ang
        q[f'{arm}_telescope'] = -600.0
    W = A.world(q)
    pts = [A.point(q, f'M1_A01_pitch{k+1}') for k in range(4)]
    tip = A.point(q, 'M1_A01_telescope')
    print('stowage arm A01 joints (mm):')
    for k, p in enumerate(pts):
        print(f'  pitch{k+1} -> {np.round(p, 1).tolist()}')
    print('  telescope tip ->', np.round(tip, 1).tolist())
