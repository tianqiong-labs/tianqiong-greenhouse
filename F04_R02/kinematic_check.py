"""Independent URDF-chain FK of every recorded arm/time; no dynamics."""
import json,xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from model import OUT,build_state,shape_for,dump

tree=ET.parse(OUT/'M02_W01_六臂_7DOF.urdf');root=tree.getroot()
joints=root.findall('joint');rows=json.loads((OUT/'motion_trajectory.json').read_text(encoding='utf8'))
maxerr=0.;violations=[]
for row in rows:
    q=row['joint_state_SI'];poses={'world':np.eye(4)}
    for j in joints:
        parent=j.find('parent').get('link');child=j.find('child').get('link');o=j.find('origin')
        xyz=np.fromstring(o.get('xyz','0 0 0'),sep=' ');rpy=np.fromstring(o.get('rpy','0 0 0'),sep=' ')
        tf=np.eye(4);tf[:3,:3]=Rotation.from_euler('xyz',rpy).as_matrix();tf[:3,3]=xyz
        val=q.get(j.get('name'),0);motion=np.eye(4);kind=j.get('type')
        if kind!='fixed':
            axis=np.fromstring(j.find('axis').get('xyz'),sep=' ')
            if kind=='revolute':motion[:3,:3]=Rotation.from_rotvec(axis*val).as_matrix()
            elif kind=='prismatic':motion[:3,3]=axis*val
            lim=j.find('limit')
            if not float(lim.get('lower'))-1e-8<=val<=float(lim.get('upper'))+1e-8:violations.append({'time':row['time'],'joint':j.get('name'),'value':val})
        poses[child]=poses[parent]@tf@motion
    for a in row['arms']:
        p=poses[f"arm_{a['i']:02d}_tool_interface"][:3,3]*1000
        maxerr=max(maxerr,float(np.linalg.norm(p-np.array(a['points'][-1]))))
deployed,_=build_state();ground=[]
for r in deployed:
    if r['group'] in ('arm','telescope','joint'):
        z=shape_for(r).BoundingBox().zmin
        if z<-1e-5:ground.append({'part':r['name'],'zmin_mm':z})
dump('kinematic_validation.json',{'samples':len(rows),'arms_per_sample':6,'movable_joints':sum(j.get('type')!='fixed' for j in joints),'URDF_vs_analytical_max_tip_error_mm':maxerr,'joint_limit_violations':violations,'URDF_inertial_elements':len(root.findall('.//inertial')),'original_deployed_arm_geometry_below_soil_plane':ground,'caution':'Ground penetrations inherited from original Z=0 wrist datum; URDF visual/effort/velocity settings are placeholders, not safe controller limits. No contact/dynamics solver.'})
print({'FK_error_mm':maxerr,'limit_violations':len(violations),'original_arm_parts_below_ground':len(ground)},flush=True)
