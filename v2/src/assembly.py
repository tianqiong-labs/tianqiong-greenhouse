"""TIANQIONG assembly / joint graph builder.

Purpose: replace "records that happen to move" with an explicit articulated
assembly. Every moving part gets a named joint with a parent, an axis in the
parent frame, a limit, and a zero pose; every pose is then produced by FK from
those joints, so the animation cannot drift away from the assembly.

Coordinate system is the reviewed source system: mm, X outward (front), Y along
the house, Z up. Joint frames follow the recovered W01 / M02 / P03 datums.

Scope: geometry + kinematics only. No structural, actuator, collision-sweep or
safety case. Limits marked "declared" are design proposals, not certified.
"""
from pathlib import Path
import importlib.util, json, math
import numpy as np

ROOT = Path(__file__).resolve().parent
GH = ROOT / 'greenhouse.py'
_spec = importlib.util.spec_from_file_location('gh', GH)
gh = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(gh)
m = gh.m

OUT = ROOT / 'out'
OUT.mkdir(exist_ok=True)

P = m.P
MODULE_PITCH = P['module_length'] + P['small_wall_gap']      # 4250
Y0 = np.array(m.Y0, float)                                   # initial carriage Y
Y1 = np.array(m.Y1, float)                                   # compensated carriage Y
A0_DEG = np.rad2deg(m.A0)
FOLD_DEG = 75.0                                              # 下起手风琴, 已验证 ≤3022 mm
LINK_MM = [1150.0, 1380.0, 1520.0]
TEL_MIN, TEL_MAX = 1000.0, 1600.0
ROOT_Z = 2800.0
RAIL_X = 4620.0
ROOF_STANDOFF = P['roof_vertical_standoff']
BEAD_DROP = P['bead_drop']


def _v(x, y, z):
    return [round(float(x), 4), round(float(y), 4), round(float(z), 4)]


def build_module(module_index):
    """One large wall module: W01 datum + P03 rail + six 7-DOF arms + F04 carriers."""
    mod = module_index + 1
    dy = module_index * MODULE_PITCH
    tag = f'M{mod}'
    comps, joints = [], []

    comps.append(dict(id=f'{tag}_W01', kind='wall_module', role='large module base',
                      datum=dict(length_y=3500.0, thickness_x=600.0, height_z=2800.0),
                      note='recovered W01 source geometry; carriage rails at Z 2370/2670'))

    # ---- P03 front rail: Y travel, lift, outboard stowage -------------------
    comps.append(dict(id=f'{tag}_P03', kind='front_anchor_rail', role='front rail + 6 sliders + 4 anchors',
                      datum=dict(length_y=3500.0, main_rail_x=RAIL_X),
                      note='anchors retract into housings; rail is lifted by the arms (no independent lift actuator)'))
    joints.append(dict(id=f'{tag}_P03_travel_Y', type='prismatic', parent=f'{tag}_W01', child=f'{tag}_P03',
                       origin=_v(RAIL_X, dy, 0), axis=[0, 1, 0], range_mm=[0, 915.4],
                       driver='front slider group, synced with root carriages', implementation='physical',
                       note='declared travel = max(Y1-Y0) so both carriages reach the compensated datum'))
    joints.append(dict(id=f'{tag}_P03_anchor_retract', type='prismatic', parent=f'{tag}_P03_slot', child=f'{tag}_P03_anchor_group',
                       origin=_v(RAIL_X, dy + 1750, 0), axis=[0, 0, 1], range_mm=[-620, 0],
                       driver='anchor retraction (declared)', implementation='declared', note='4 rods, 620 mm retraction, design proposal'))
    joints.append(dict(id=f'{tag}_P03_lift_Z', type='prismatic', parent=f'{tag}_P03', child=f'{tag}_P03_lift',
                       origin=_v(RAIL_X, dy, 0), axis=[0, 0, 1], range_mm=[0, 2900],
                       driver='arm-driven lift (no independent actuator)', implementation='declared',
                       note='must clear the 2800 mm wall top before moving outboard'))
    joints.append(dict(id=f'{tag}_P03_stow_X', type='prismatic', parent=f'{tag}_P03_lift', child=f'{tag}_P03_stow',
                       origin=_v(RAIL_X, dy, 0), axis=[-1, 0, 0], range_mm=[0, 5520],
                       driver='arm-driven transfer to the outboard slot', implementation='declared',
                       note='outboard slot behind the mother wall (X < -600); path clearance NOT verified'))
    joints.append(dict(id=f'{tag}_P03_slot_Z', type='prismatic', parent=f'{tag}_P03_stow', child=f'{tag}_P03_slot',
                       origin=_v(RAIL_X, dy, 0), axis=[0, 0, 1], range_mm=[-300, 0],
                       driver='lowering into the outboard slot (declared)', implementation='declared',
                       note='outboard seat at Z 2600 (wall-top level): the only family that passes the carry IK; a 300 mm seat fails at every joint limit'))

    # ---- six arms ----------------------------------------------------------
    for i, y0 in enumerate(Y0):
        arm = f'{tag}_A{i+1:02d}'
        pts = m.arm_points(y0, 0.0, TEL_MAX)
        comps.append(dict(id=arm, kind='arm_assembly', role='4-section rib arm, 7 DOF',
                          datum=dict(root=_v(0, dy + y0, ROOT_Z), section_mm=LINK_MM,
                                     telescope_mm=[TEL_MIN, TEL_MAX])))
        joints.append(dict(id=f'{arm}_root_Y', type='prismatic', parent=f'{tag}_W01', child=arm,
                           origin=_v(0, dy + y0, ROOT_Z), axis=[0, 1, 0],
                           range_mm=sorted([round(float(Y1[i] - Y0[i]), 4), 0.0]),
                           zero_mm=0.0, driver='root carriage, synced with P03 sliders', implementation='physical',
                           note='double rail at Z 2370 / 2670; end carriages 1/6 have zero compensation travel in the recovered datum'))
        joints.append(dict(id=f'{arm}_yaw_Z', type='revolute', parent=arm, child=f'{arm}_yaw',
                           origin=_v(0, dy + y0, ROOT_Z), axis=[0, 0, 1],
                           range_deg=[-90, 90], zero_deg=0.0, implementation='physical',
                           driver='root yaw actuator (declared)', 
                           note='restores the 7th DOF of the recovered M02 definition; not used by the shelter poses'))
        parent = f'{arm}_yaw'
        for k in range(4):
            jid = f'{arm}_pitch{k+1}'
            joints.append(dict(id=jid, type='revolute', parent=parent,
                               child=f'{arm}_link{k+1}' if k < 3 else f'{arm}_telescope_outer',
                               origin=_v(*pts[0] if k == 0 else pts[k] * 0 + pts[k]),
                               axis=[0, 1, 0], range_deg=[-160, 160], zero_deg=0.0,
                               driver='pitch actuator (declared)', implementation='physical', note='pitch about Y; deployed source angles are baked into the joint frame, q is relative'))
            parent = f'{arm}_link{k+1}' if k < 3 else f'{arm}_telescope_outer'
        joints.append(dict(id=f'{arm}_telescope', type='prismatic', parent=f'{arm}_telescope_outer',
                           child=f'{arm}_telescope_inner', origin=_v(*pts[3]), axis=_v(*(np.array(pts[4]) - np.array(pts[3])) / np.linalg.norm(np.array(pts[4]) - np.array(pts[3]))),
                           range_mm=[TEL_MIN - TEL_MAX, 0], zero_mm=0.0,
                           driver='4th section extension (declared)', implementation='physical', note='1600 deployed, 1000 stowed'))

        # F04 carrier at each of the three joints
        for k in range(1, 4):
            jx, _, jz = pts[k]
            bz = jz + ROOF_STANDOFF - BEAD_DROP
            cid = f'{arm}_J{k}'
            comps.append(dict(id=cid, kind='membrane_carrier', role='sliding releasable carrier',
                              datum=dict(bead_dia=12.0, gate_stroke_mm=24.0, head_drop_mm=80.0)))
            mount_parent = f'{arm}_link{k}' if k < 4 else f'{arm}_telescope_outer'
            joints.append(dict(id=f'{cid}_mount', type='fixed', parent=mount_parent, child=cid,
                               origin=_v(jx, dy + y0, jz), axis=[0, 0, 1],
                               implementation='physical', driver='bolted carrier mount',
                               note='carrier is carried by the arm link at this joint line'))
            joints.append(dict(id=f'{cid}_slide_Y', type='prismatic', parent=cid, child=f'{cid}_head',
                               origin=_v(jx, dy + y0, bz), axis=[0, 1, 0], range_mm=[-25, 25],
                               driver='passive slide along the bead', implementation='physical', note='Y float limit 25 mm'))
            joints.append(dict(id=f'{cid}_head_drop', type='prismatic', parent=f'{cid}_head', child=f'{cid}_head_lower',
                               origin=_v(jx, dy + y0, bz), axis=[0, 0, -1], range_mm=[0, 80],
                               driver='release actuator (declared)', implementation='declared',
                               note='carries stem/base/magnet only; the lips stay with the head'))
            for side, sgn in (('L', -1), ('R', 1)):
                joints.append(dict(id=f'{cid}_gate_{side}', type='prismatic', parent=f'{cid}_head',
                                   child=f'{cid}_gate_{side}_plate',
                                   origin=_v(jx + sgn * 7.5, dy + y0, bz + 9.5), axis=[sgn, 0, 0],
                                   range_mm=[0, 24], driver='mechanical anti-detach gate (declared)', implementation='declared',
                                   note='closed slot 4 mm > web 1.5 mm and < bead 12 mm'))

    return comps, joints


def build_house(modules=3):
    comps, joints = [], []
    for k in range(modules):
        c, j = build_module(k)
        comps += c; joints += j
    # ---- 覆盖系统：每模块一张膜 + 母墙侧卷筒 + 模块间密封搭接 ----
    for k in range(modules):
        tag = f'M{k+1}'
        dy = k * MODULE_PITCH
        comps.append(dict(id=f'{tag}_FILM', kind='module_membrane_sheet',
                          role='one continuous sheet per large module (not cut at arm sections)',
                          datum=dict(thickness_mm=P['membrane_thickness'], stock_length_mm=P['membrane_stock_length'],
                                     winding='wall side'),
                          note='cross-section continuous; adjacent modules joined by a detachable seal lap'))
        comps.append(dict(id=f'{tag}_DRUM', kind='winding_drum',
                          role='wall-side roll-up drum', datum=dict(core_radius_mm=60.0, center_xz=[-210.0, 2860.0]),
                          note='integrated in the mother wall top; surplus winds here, not at the front rail'))
        joints.append(dict(id=f'{tag}_DRUM_wind', type='revolute', parent=f'{tag}_W01', child=f'{tag}_DRUM',
                           origin=_v(-210.0, dy, 2860.0), axis=[0, 1, 0], range_deg=[0, 5400], implementation='physical',
                           driver='wall-side winder (declared)', note='pays out / takes up the module sheet'))
        joints.append(dict(id=f'{tag}_FILM_bind', type='deformable', parent=f'{tag}_DRUM', child=f'{tag}_FILM',
                           origin=_v(-210.0, dy, 2860.0), axis=[0, 1, 0], implementation='declared',
                           driver='film winding (not a rigid body)',
                           note='membrane is generated geometrically from the wound length; not a rigid joint'))
        joints.append(dict(id=f'{tag}_WHEEL_lift', type='prismatic', parent=f'{tag}_W01', child=f'{tag}_WHEEL',
                           origin=_v(-300.0, dy + 1750, 0.0), axis=[0, 0, 1], range_mm=[0, 160],
                           implementation='physical', driver='vertical lift wheels (declared)',
                           note='wheels extend -> wall rises 160 mm; retract -> wall lands (source mobile_wall_lift)'))
        comps.append(dict(id=f'{tag}_WHEEL', kind='wheel_group', role='4 wheels on vertical lift',
                          datum=dict(lift_mm=160, wheel_centers_y=[345.0, 3155.0])))
        for i in range(6):
            comps.append(dict(id=f'{tag}_SLIDER{i+1}', kind='front_slider', role='P03 slider stays in the rail'))
            joints.append(dict(id=f'{tag}_SLIDER{i+1}_Y', type='prismatic', parent=f'{tag}_P03_slot', child=f'{tag}_SLIDER{i+1}',
                               origin=_v(RAIL_X, dy + float(Y0[i]), 0.0), axis=[0, 1, 0], range_mm=[0, 915.4],
                               implementation='physical', driver='synced with the root carriage'))
            joints.append(dict(id=f'{tag}_A{i+1:02d}_tip_coupling', type='coupling',
                               parent=f'{tag}_SLIDER{i+1}', child=f'{tag}_A{i+1:02d}_telescope_inner',
                               origin=_v(RAIL_X, dy + float(Y0[i]), 0.0), axis=[0, 0, 1], implementation='physical',
                               driver='quick-release tip coupling (declared)',
                               note='closed-loop grasp; must be released before retract/fold; slider stays in the rail'))
    for g in range(modules - 1):
        comps.append(dict(id=f'SEAL{g+1}', kind='seal_lap', role='detachable flexible seal between modules',
                          datum=dict(gap_y0=P['module_length'] + g * MODULE_PITCH, overlap_mm=100.0),
                          state_bindings=dict(deployed='gap datum (bolted lap)',
                                              transport=f'rides with M{g+1} large module',
                                              handling='seal is a separate part and can be removed/stacked on its own')))

    for g in range(modules - 1):
        joints.append(dict(id=f'SEAL{g+1}_fixed', type='fixed', parent=f'M{g+1}_W01', child=f'SEAL{g+1}',
                           origin=_v(0, P['module_length'] + g * MODULE_PITCH, 0), axis=[0, 0, 1],
                           implementation='physical', driver='bolted seal lap',
                           note='state-bound: deployed at the gap datum, transport rides with the left module, '
                                'and the seal can be removed and stacked on its own'))

    # infill + end walls + membrane
    for g in range(modules - 1):
        y = P['module_length'] + g * MODULE_PITCH
        comps.append(dict(id=f'SW{g+1}', kind='small_wall', role='750 mm infill between large modules',
                          datum=dict(width_x=600.0, length_y=750.0, height_z=2800.0, y0=y)))
        comps.append(dict(id=f'BR{g+1}', kind='bridge_rail', role='BR750 connector between modules',
                          datum=dict(y0=y, length_y=750.0)))
    total = modules * P['module_length'] + (modules - 1) * P['small_wall_gap']
    comps.append(dict(id='EW_front', kind='end_wall', role='transverse end wall with door',
                      datum=dict(y0=-80.0, thickness=80.0, door_mm=[1100, 2100])))
    comps.append(dict(id='EW_rear', kind='end_wall', role='transverse end wall',
                      datum=dict(y0=float(total), thickness=80.0)))
    comps.append(dict(id='F04_flashing', kind='wall_top_clamp', role='continuous root flashing clamp',
                      datum=dict(length_y=float(total), width_x=40.0, height_z=8.0),
                      note='house-level part, bolted to the left large module wall top'))
    joints.append(dict(id='F04_flashing_fixed', type='fixed', parent='M1_W01', child='F04_flashing',
                       origin=_v(-150, 0, 2804), axis=[0, 0, 1], implementation='physical',
                       driver='bolted flashing clamp'))
    comps.append(dict(id='F04_membrane', kind='continuous_membrane', role='single transparent sheet',
                      datum=dict(stock_length_mm=P['membrane_stock_length'], thickness_mm=P['membrane_thickness'],
                                 width_mm=modules * 3500 + (modules - 1) * 750 + 100, y0=-50.0),
                      note='NOT cut at arm sections; bands bonded at three joint lines'))
    joints.append(dict(id='EW_front_fixed', type='fixed', parent='M1_W01', child='EW_front',
                       origin=_v(0, -80, 0), axis=[0, 0, 1], implementation='physical',
                       driver='bolted end wall', note='dismantled before transport'))
    joints.append(dict(id='EW_rear_fixed', type='fixed', parent=f'M{modules}_W01', child='EW_rear',
                       origin=_v(0, float(total), 0), axis=[0, 0, 1], implementation='physical',
                       driver='bolted end wall', note='dismantled before transport'))
    for g in range(modules - 1):
        joints.append(dict(id=f'SW{g+1}_fixed', type='fixed', parent=f'M{g+1}_W01', child=f'SW{g+1}',
                           origin=_v(0, P['module_length'] + g * MODULE_PITCH, 0), axis=[0, 0, 1],
                           implementation='physical', driver='bolted infill module',
                           note='stays with the left large module'))
        joints.append(dict(id=f'BR{g+1}_fixed', type='fixed', parent=f'M{g+1}_P03_slot', child=f'BR{g+1}',
                           origin=_v(RAIL_X, P['module_length'] + g * MODULE_PITCH, 0), axis=[0, 0, 1],
                           implementation='physical', driver='bolted bridge rail',
                           note='travels with the front rail of the left module'))
    joints.append(dict(id='F04_wind', type='revolute', parent='EW_front', child='F04_membrane',
                       origin=_v(4860, -50, 140), axis=[0, 1, 0], range_deg=[0, 720],
                       driver='front roll-up shaft (declared)', implementation='declared', note='winding of thick beads NOT solved'))
    return comps, joints


def fold_pose():
    """Z-fold stowage pose: down-first accordion, no root lift. Verified envelope."""
    th = FOLD_DEG
    pat = [-th, th, -th, th]
    pts = m.M['fk'](0.0, np.deg2rad(pat), TEL_MIN / 1000.0) * 1000
    return dict(pattern_deg=pat, joints_mm=[[round(float(x), 1), round(float(z), 1)] for x, _, z in pts],
                envelope=dict(max_z_mm=round(float(pts[:, 2].max()), 1), min_z_mm=round(float(pts[:, 2].min()), 1),
                              intrusion_x_mm=round(float(pts[:, 0].max()), 1)))


def main():
    comps, joints = build_house(3)
    data = dict(schema='tianqiong.assembly/1', units='mm (STEP, assembly); m/rad (glTF, URDF)',
                axes='X outward, Y along house, Z up',
                source_baseline=dict(W01=m.W['P'], M02=m.M['CFG'], P03=m.R['P'],
                                     module_pitch_mm=MODULE_PITCH, arms_per_module=6, dof_per_arm=7),
                components=comps, joints=joints,
                implementation_counts=dict(physical=sum(j.get('implementation') == 'physical' for j in joints),
                                           declared=sum(j.get('implementation') == 'declared' for j in joints),),
                joint_counts=dict(prismatic=sum(j['type'] == 'prismatic' for j in joints),
                                  revolute=sum(j['type'] == 'revolute' for j in joints),
                                  total=len(joints)),
                fold_pose=fold_pose(),
                declared_not_verified=['structural strength', 'actuator sizing', 'continuous collision sweep',
                                       'bead/gate winding inside the roll', 'rail transfer path clearance',
                                       'transport tie-down and road envelope'],
                note='Geometry + declared joint limits only. No claim of structural or safety compliance.')
    p = OUT / 'assembly.json'
    p.write_text(json.dumps(data, ensure_ascii=False, indent=1), encoding='utf8')
    print(f'components {len(comps)}  joints {len(joints)}  '
          f'(prismatic {data["joint_counts"]["prismatic"]}, revolute {data["joint_counts"]["revolute"]})')
    print('fold envelope:', data['fold_pose']['envelope'])
    print('written', p, f'{p.stat().st_size/1024:.1f} KB')


if __name__ == '__main__':
    main()
