"""Bounded B-rep checks. Reports interferences rather than asserting safety."""
import json, math
import numpy as np
from model import OUT,P,W,M,R,Y0,Y1,A0,LF,build_state,timeline,shape_for,original_records,arm_points,dump

def bb(s):
    b=s.BoundingBox();return np.array([b.xmin,b.ymin,b.zmin]),np.array([b.xmax,b.ymax,b.zmax])
def intersects(a,b):return bool(np.all(np.minimum(a[1],b[1])-np.maximum(a[0],b[0])>1e-5))
def overlap(a,b):
    if not intersects(bb(a),bb(b)):return 0.
    return float(a.intersect(b).Volume())

def main():
    baseline,meta=build_state();film=shape_for(next(r for r in baseline if r['group']=='film'))
    base_shapes={r['name']:shape_for(r) for r in baseline}
    report={'scope':'Discrete B-rep checks of selected new F04 interactions; NOT continuous swept-volume, dynamics, soft-film or global safety certification',
            'sample_count':73,'sample_interval_s':.25,'checks':{},'interferences':[],'known_unverified':[]}
    # Rebuild original W01/M02/P03 totals independently through recovered CAD functions.
    orig,_=original_records(Y0)
    actual_by_group={}
    for r in orig:actual_by_group.setdefault(r['group'],[]).append(shape_for(r))
    source_arm=[M['make_arm_cad_assy'](p).toCompound() for p in M['state']('S0_SUPPORT')]
    source_vol=sum(s.Volume() for s in source_arm)
    rebuilt_arm=sum(s.Volume() for group in ['arm','joint','telescope'] for s in actual_by_group[group])
    rail=R['A'];R['cad_front'](rail)
    source_rail=rail.toCompound().Volume()
    rebuilt_rail=sum(s.Volume() for s in actual_by_group['rail'])
    source_wall=sum(p['solid'].Volume() for p in W['PARTS'])
    rebuilt_wall=sum(s.Volume() for s in actual_by_group['wall'])
    report['checks']['source_volume_consistency']={g:{'recovered_source_mm3':a,'rebuilt_mm3':b,'relative_error':abs(a-b)/a} for g,a,b in [('W01',source_wall,rebuilt_wall),('M02',source_vol,rebuilt_arm),('P03',source_rail,rebuilt_rail)]}
    report['checks']['endpoint_error_mm']=float(np.linalg.norm(arm_points(0)[-1,[0,2]]-[4500,0]))
    report['checks']['minimum_root_carriage_Y_gap_mm']=float(np.min(np.diff(Y1))-350)
    report['checks']['minimum_front_carriage_Y_gap_mm']=float(np.min(np.diff(Y1))-240)
    # New single sheet versus old assembly: no intended old-clamp usage.
    old_hits=[]
    for r in baseline:
        if not r['name'].startswith('F04_'):
            vol=overlap(film,base_shapes[r['name']])
            if vol>1e-3:old_hits.append({'part':r['name'],'overlap_mm3':vol})
    report['checks']['deployed_membrane_vs_original_components']=old_hits
    # New solid versus membrane. Bonded web/band and clamping contact are deliberate.
    new_hits=[]
    for r in baseline:
        if r['name'].startswith('F04_') and r['group'] not in ('film','band') and 'root_flashing' not in r['name']:
            vol=overlap(film,base_shapes[r['name']])
            if vol>1e-3:new_hits.append({'part':r['name'],'overlap_mm3':vol})
    report['checks']['deployed_membrane_vs_new_hardware']=new_hits
    # Three bead stations, carrier heads and gates checked through entire timeline.
    min_film=float('inf');min_bead=float('inf');min_web=float('inf');min_other_arm=float('inf')
    sampled_hits=[];wrist_hits=[];floor=[];root_front_sync=0.;interface_y=[]
    beads={k:base_shapes[f'F04_bead{k}'] for k in range(1,4)}
    webs={k:base_shapes[f'F04_web{k}'] for k in range(1,4)}
    for t,label,state in timeline():
        records,md=build_state(**state);lookup={r['name']:r for r in records}
        # Two released arms are symmetric; check A02 in B-rep, all six for metadata gaps.
        moving=[r for r in records if r['name'].startswith('F04_M1_A02_')]
        for r in moving:
            s=shape_for(r);k=int(r['name'].split('_J')[1][0])
            min_film=min(min_film,s.distance(film))
            bead=beads[k];web=webs[k]
            d=s.distance(bead);min_bead=min(min_bead,d);min_web=min(min_web,s.distance(web))
            for target,ts in [('bead',bead),('web',web),('film',film)]:
                vol=overlap(s,ts)
                if vol>1e-3:sampled_hits.append({'time_s':t,'part':r['name'],'target':target,'volume_mm3':vol})
        for a in md['arms']:
            if a['i'] not in (2,5):
                root_front_sync=max(root_front_sync,abs(a['points'][0][1]-a['points'][-1][1]))
            if min(p[2] for p in a['points']) < -1e-5:floor.append({'time_s':t,'arm':a['id'],'min_axis_z':min(p[2] for p in a['points'])})
        # Legacy spherical wrist + solid bracket is an existing joint proxy overlap,
        # not evidence of a modeled opening latch. Report its overlap throughout exit.
        tip=shape_for(lookup['M1_A02_tip'])
        bracket=shape_for(lookup['M1_P03_slider02_wrist_upright'])
        vol=overlap(tip,bracket)
        if vol>1e-3:wrist_hits.append({'time_s':t,'volume_mm3':vol})
        if abs(t-round(t))<1e-8:print('checked',t,flush=True)
    report['checks']['F04_head_motion']={'arm_exactly_checked':'A02; A05 is identical geometry shifted in Y','min_membrane_distance_mm':min_film,'min_bead_distance_mm':min_bead,'min_web_distance_mm':min_web,'positive_volume_collisions':sampled_hits}
    report['checks']['supported_root_front_Y_sync_error_mm']=root_front_sync
    report['checks']['negative_arm_axis_Z_samples']=floor
    report['checks']['legacy_wrist_bracket_overlap_samples']=wrist_hits
    report['checks']['last_pose_arm2_tip_mm']=build_state(compensate=1,gate=1,drop=1,retract=1,lower=1)[1]['arms'][1]['points'][-1]
    # Critical incomplete transition: 750-mm wall bay has NO new arm on bridge.
    support_y=np.r_[Y0,Y0+4250]
    report['checks']['two_module_max_rib_spacing_mm']=float(np.max(np.diff(support_y)))
    report['checks']['released_four_rib_max_spacing_mm']=float(np.max(np.diff(Y1[[0,2,3,5]])))
    report['known_unverified']=[
       'No flexible membrane stress, sag, wrinkling, finite fold radius or seam/weatherproof simulation.',
       'Single sheet across two modules spans 1450 mm between nearest ribs over 750 mm small-wall bay; not load verified.',
       'Bands/web/bead winding is NOT included in storage target: continuous bead is a rollability blocker.',
       'W01/M02 original joint and frame geometry contains simplified overlapping proxies; not a production mechanism.',
       'P03 wrist bracket is retained as an original solid proxy; opening latch and physically valid release path not engineered.',
       'F04 vertical carrier slide and gate actuation/positive locks are geometric DOF proposals; no drives or controls.',
       'No full deployment-from-storage trajectory; no rail transfer through/around mother wall; no folded arm packaging.',
       'Core bearings/mount envelopes are new add-ons; 3.6 m / 7.85 m roll shaft stiffness, motors and bearings not calculated.',
       'Original P03 membrane clamp is retained but NOT connected to F04 sheet. New front feed interface is provisional.',
       'Storage rail still uses deployed-anchor source geometry: anchor retraction and transport restraints unresolved.',
       '73 samples do not prove continuous swept-volume collision freedom; only listed pairs tested.',
       'No wind/snow/soil anchorage, static strength, fatigue, electrical interlocks or field safety validation.']
    report['summary']={'film_vs_original_no_positive_volume_overlap':not old_hits,'film_vs_new_hardware_no_positive_volume_overlap':not new_hits,
                       'sampled_head_bead_web_film_collision_free':not sampled_hits,'overall_safe_to_build_or_operate':False}
    dump('verification_report.json',report)
    print(json.dumps(report['summary']),flush=True)

if __name__=='__main__':main()
