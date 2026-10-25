"""Pose-driven geometry for TIANQIONG.

Every exported state is produced by evaluating the assembly joint graph, not by
hand-moving records. Records are tagged with the component they belong to, the
component's world transform comes from kinematics.Assembly, and rigid parts are
transformed by (world @ inverse(zero)). Arms therefore cannot drift: their links
are rigid bodies attached to joints that were validated against the source FK.

Usage:
    python poses.py --check
    python poses.py --export deployed transport
"""
import re
from pathlib import Path
import argparse, importlib.util, json, sys
import numpy as np
import cadquery as cq

HERE = Path(__file__).resolve().parent
OUT = HERE / 'out'
OUT.mkdir(exist_ok=True)


def _load(name, file):
    spec = importlib.util.spec_from_file_location(name, HERE / file)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


gh = _load('gh', 'greenhouse.py')
kin = _load('kin', 'kinematics.py')
m = gh.m
A = kin.Assembly()

# ------------------------------------------------------------------ mapping
PART_SUFFIX = {'_stem': 'head_lower', '_base': 'head_lower', '_magnet_aux': 'head_lower',
               '_sideL': 'head', '_sideR': 'head', '_gateL': 'gate_L_plate', '_gateR': 'gate_R_plate'}


def resolve(name):
    """记录名 -> 装配组件 id。顺序很关键：先识别新族（模块号直接开头），再剥双重前缀。"""
    # 新一代记录：M1_FILM / M1_BAND1 / M1_DRUM / M1_CLAMP / SEAL1 / M1_P03_slider01_body
    m_new = re.match(r'^(M\d+)_(FILM|BAND\d|BEAD\d|WEB\d|CLAMP|DRUM_HOUSING|DRUM)$', name)
    if m_new:
        mod, part = m_new.groups()
        if part.startswith('DRUM'):
            return f'{mod}_DRUM'
        if part == 'CLAMP':          # 前沿压条挂在锚轨上，随轨搬运
            return f'{mod}_P03_slot'
        return f'{mod}_W01'
    if name.startswith('SEAL'):
        head = name.split('_')[0]
        if name.endswith('_LAP'):        # 密封搭接的前沿压条挂在过渡轨上
            return 'BR' + head[4:]
        return head

    # 双重前缀：M<k>_M1_<part>，外层才是真实模块
    mo = re.match(r'^M(\d+)_(.*)$', name)
    outer = None
    if mo:
        outer, rest = mo.group(1), mo.group(2)
        name = rest
        if outer != '1' and name.startswith('M1_'):
            name = f'M{outer}_' + name[3:]

    if name.endswith('root_flashing_clamp'):
        return 'F04_flashing'
    if '_P03_slider' in name:
        head, _, idx = name.partition('_P03_slider')
        num = re.match(r'(\d+)', idx)
        return f'{head}_SLIDER{int(num.group(1))}' if num else f'{head}_P03_slot'
    if '_P03_' in name:
        return (name.split('_P03_')[0] + '_P03_anchor_group') if 'anchor' in name \
            else (name.split('_P03_')[0] + '_P03_slot')
    if name.startswith('F04_M') and '_J' in name:
        comp = name.split('F04_', 1)[1]
        if outer and outer != '1' and comp.startswith('M1_'):
            comp = f'M{outer}_' + comp[3:]
        for suf, sub in PART_SUFFIX.items():
            if comp.endswith(suf):
                return comp[:-len(suf)] + '_' + sub
        return comp
    if name.startswith('F04_') or name.startswith('BR') or name.startswith('SW') or name.startswith('ENDWALL'):
        if name.startswith('ENDWALL_1'):
            return 'EW_front'
        if name.startswith('ENDWALL_2'):
            return 'EW_rear'
        if name.startswith('F04_'):
            return 'F04_membrane'
        return name.split('_')[0]
    if name.endswith('_tel_outer'):
        return name[:-len('_tel_outer')] + '_telescope_outer'
    if name.endswith('_tel_inner'):
        return name[:-len('_tel_inner')] + '_telescope_inner'
    if name.rsplit('_', 1)[-1].startswith('link'):
        return name
    if '_A' in name:
        arm, _, last = name.rpartition('_')
        if last == 'tip':
            return f'{arm}_telescope_inner'
        if last.startswith('joint') and last[5:].isdigit():
            k = int(last[5:])
            return _arm_link(arm, k - 1)
    for cand in sorted(A.comps, key=len, reverse=True):
        if name.startswith(cand):
            return cand
    return None


def _arm_link(arm, incoming):
    """joint2..4 sit on the end of the previous link; joint1 sits on the arm root."""
    return arm if incoming == 0 else f'{arm}_link{incoming}'


def pose_transform(q):
    W = A.world(q)
    T = {c: (W[c] @ A.zero_frame_inv(c)) if c in W else np.eye(4)
         for c in set(A.comps) | set(A.child_of)}
    return T


def pairs_for(q):
    records = gh.house_records(gh.module_states())
    T = pose_transform(q)
    out, unresolved = [], []
    for r in records:
        comp = resolve(r['name'])
        if comp is None or comp not in T:
            unresolved.append(r['name'])
            continue
        out.append((r, T[comp], comp))
    return out, unresolved


def transform_shape(shape, T):
    """Rigid transform via OCCT gp_Trsf (CadQuery's Matrix treats rows as basis vectors)."""
    if np.allclose(T, np.eye(4)):
        return shape
    from OCP.gp import gp_Trsf
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    trsf = gp_Trsf()
    trsf.SetValues(*[float(v) for v in T[:3].reshape(-1)])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, trsf, True).Shape())


def export_step(q, name, drop_groups=(), drop_name_prefixes=()):
    pairs, unresolved = pairs_for(q)
    if drop_groups:
        pairs = [p for p in pairs if p[0]['group'] not in drop_groups]
    if drop_name_prefixes:
        pairs = [p for p in pairs if not any(p[0]['name'].split('_')[-1].startswith(x) or x in p[0]['name']
                                             for x in drop_name_prefixes)]
    asm = cq.Assembly(name=name)
    groups = {}
    for r, T, comp in pairs:
        g = r['group']
        groups.setdefault(g, cq.Assembly(name=g))
        shape = transform_shape(m.shape_for(r), T)
        groups[g].add(shape, name=r['name'], color=cq.Color(*m.color_for(r)))
    for g, sub in groups.items():
        asm.add(sub, name=g)
    path = OUT / f'{name}.step'
    asm.save(str(path), exportType='STEP')
    (OUT / f'{name}.pose.json').write_text(json.dumps(
        dict(name=name, q={k: round(v, 4) for k, v in q.items()},
             solids=len(pairs), dropped_groups=list(drop_groups),
             note='geometry only; no collision/strength/actuator verification'),
        ensure_ascii=False, indent=1), encoding='utf8')
    return path, len(pairs), unresolved


# ------------------------------------------------------------------ poses
DEPLOYED = {}

FOLD = [-75.0, 75.0, -75.0, 75.0]
A0 = np.array([-1.6, -20.1, -38.6, -57.2])


def fold_q(module_number=1):
    d = A0 - np.array(FOLD)
    rel = np.diff(np.r_[0.0, d])
    q = {}
    for k in range(6):
        arm = f'M{module_number}_A{k+1:02d}'
        for i in range(4):
            q[f'{arm}_pitch{i+1}'] = float(rel[i])
        q[f'{arm}_telescope'] = -600.0
    return q


def transport_q(modules=3):
    """Every module stows independently: folded arms, retracted anchors, rail in the outboard slot."""
    q = {}
    for i in range(modules):
        t = f'M{i+1}'
        q.update(fold_q(i + 1))
        q[f'{t}_P03_anchor_retract'] = 620.0
        q[f'{t}_P03_lift_Z'] = 2900.0
        q[f'{t}_P03_stow_X'] = 5520.0
        q[f'{t}_P03_slot_Z'] = -300.0
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--check', action='store_true')
    ap.add_argument('--export', nargs='*')
    args = ap.parse_args()

    if args.check or not args.export:
        pairs, unresolved = pairs_for(DEPLOYED)
        print(f'deployed: {len(pairs)} records mapped, {len(unresolved)} unresolved')
        for u in unresolved[:8]:
            print('   UNRESOLVED', u)
        # deployed pose must be identity for every component with records
        worst = 0.0
        for r, T, comp in pairs:
            worst = max(worst, float(np.abs(T - np.eye(4)).max()))
        print(f'max |T - I| at deployed: {worst:.2e}')
        qt = transport_q()
        W = A.world(qt)
        tip = A.point(qt, 'M1_A01_telescope')
        print('transport arm A01 tip:', np.round(tip, 1).tolist())
        print('transport rail frame z:', round(float(W['M1_P03_slot'][2, 3]), 1),
              ' x:', round(float(W['M1_P03_slot'][0, 3]), 1))

    for name in (args.export or []):
        q = DEPLOYED if name == 'deployed' else transport_q()
        drop = ('film', 'band', 'roll', 'proposal') if name == 'transport' else ()
        drop_names = ('ENDWALL',) if name == 'transport' else ()
        path, n, unresolved = export_step(q, name, drop, drop_names)
        print(f'{name}: {n} solids -> {path.name} ({path.stat().st_size/1e6:.1f} MB)')
        if unresolved:
            print(f'   {len(unresolved)} records unresolved, e.g. {unresolved[:3]}')


if __name__ == '__main__':
    main()
