#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""TIANQIONG M02 on W01: reusable telescopic 7-DOF arm and six arm geometry model.
Digital concept prototype; kinematics only, no load or true swept-volume verification.
W01 coordinates: x forward, y along greenhouse, z up; metres for animation/URDF, mm for CAD.
"""
from pathlib import Path
import numpy as np, math, json, csv, shutil, zipfile, copy, xml.etree.ElementTree as ET
import trimesh, cadquery as cq
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, FFMpegWriter
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go

ROOT=Path('/mnt/data')
OUT=ROOT/'田穹_M02_六机械臂数字样机'; OUT.mkdir(exist_ok=True)
W01=ROOT/'田穹_W01_母墙数字样机'; WPAR=json.loads((W01/'W01_装配接口与参数.json').read_text(encoding='utf8'))
CFG={'revision':'M02-A','lengths_fixed_m':[1.15,1.38,1.52], 'L4_min_m':1.0,'L4_max_m':1.6,
     'support_global_angles_deg':[-1.6,-20.1,-38.63402497781479,-57.18221617233679],
     'work_target_global_angles_deg':[-5.,-70.,0.,-105.], 'root_x_m':0.,'root_z_m':2.8,
     'initial_y_m':np.array(WPAR['dimensions_mm']['slide_centers_initial']).tolist(),
     'reconfigured_y_m':np.array(WPAR['dimensions_mm']['slide_centers_reconfigured']).tolist(),
     'deploy_width_m':4.5, 'module_length_m':3.5, 'wall_depth_m':0.6,
     'arm_link_radius_proxy_m':0.048,'work_arms':[2,5],
     'joint_count_per_arm':7, 'notes':'Not an executable control system or collision-certified design'}
CFG['initial_y_m']=[v/1000 for v in CFG['initial_y_m']];CFG['reconfigured_y_m']=[v/1000 for v in CFG['reconfigured_y_m']]
A0=np.deg2rad(CFG['support_global_angles_deg']); AW=np.deg2rad(CFG['work_target_global_angles_deg']); LF=np.array(CFG['lengths_fixed_m']);
plt.rcParams.update({'font.sans-serif':['Noto Sans CJK JP','DejaVu Sans'],'axes.unicode_minus':False})

def fk(y,angles=A0,ell=1.6,yaw=0):
    lengths=np.r_[LF,ell]; p=np.r_[0,np.cumsum(lengths*np.cos(angles))]; z=CFG['root_z_m']+np.r_[0,np.cumsum(lengths*np.sin(angles))]
    # rotate link paths after root about z; yaw=0 means paths in xz at same y
    return np.c_[p*np.cos(yaw), np.full(len(p),y)+p*np.sin(yaw), z]

def smooth(t):t=np.clip(t,0,1);return 6*t**5-15*t**4+10*t**3

def state(name):
    base=CFG['initial_y_m'] if name=='S0_SUPPORT' else CFG['reconfigured_y_m']
    out=[]
    for i,y in enumerate(base):
        work=name=='S2_WORK_TARGET' and (i+1 in CFG['work_arms'])
        out.append(dict(i=i+1,y=y,angles=(AW if work else A0).copy(),ell=1.0 if work else 1.6,work=work))
    return out

def shape_tube(a,b,r,rgba):
    start=np.asarray(a,dtype=float); end=np.asarray(b,dtype=float); delta=end-start;length=np.linalg.norm(delta)
    obj=trimesh.creation.cylinder(radius=r,height=length,sections=16)
    tf=trimesh.geometry.align_vectors([0,0,1],delta)
    obj.apply_transform(tf);obj.apply_translation((start+end)/2)
    obj.visual.face_colors=rgba
    return obj

def sphere_at(center,r,rgba):
    m=trimesh.creation.icosphere(subdivisions=1,radius=r);m.apply_translation(center);m.visual.face_colors=rgba;return m

def color(hexv,a=255):return [int(hexv[i:i+2],16) for i in (1,3,5)]+[a]
C={'arm':'#2B8B7C','work':'#B97B3D','telescope':'#6DA9BA','joint':'#203F45','front':'#A87E48','wall':'#9AB9AF'}

def arm_to_scene(sc,pose,include_interfaces=True):
    idx=pose['i']; js=fk(pose['y'],pose['angles'],pose['ell']); active=pose['work']
    col=color(C['work'] if active else C['arm'])
    for k in range(3):
        sc.add_geometry(shape_tube(js[k],js[k+1],.050,col),node_name=f'M02_R{idx:02d}_link{k+1}',geom_name=f'M02_R{idx:02d}_link{k+1}')
    p3=js[3];p4=js[4];v=(p4-p3)/np.linalg.norm(p4-p3)
    outer_end=p3+.98*v
    sc.add_geometry(shape_tube(p3,outer_end,.064,color(C['telescope'])),node_name=f'M02_R{idx:02d}_tel_outer')
    inner_start=p3+.78*v
    sc.add_geometry(shape_tube(inner_start,p4,.041,col),node_name=f'M02_R{idx:02d}_tel_inner')
    for k in range(4):sc.add_geometry(sphere_at(js[k],.073,color(C['joint'])),node_name=f'M02_R{idx:02d}_axis{k+1}')
    sc.add_geometry(sphere_at(js[-1],.069,color(C['work'] if active else C['front'])),node_name=f'M02_R{idx:02d}_tip')
    if include_interfaces:
        for k in [1,2,3]:
            # abstract bead gate coupling carried at hinge location; not actuated physics
            sc.add_geometry(sphere_at(js[k]+[0,0,.075],.032,color('#CDAA5A')),node_name=f'M02_R{idx:02d}_mem_gate{k}')
    return js

def load_wall_scene(name):
    wallname='W01_RECONFIGURED_完整装配.glb' if name!='S0_SUPPORT' else 'W01_DEPLOYED_完整装配.glb'
    original=trimesh.load(str(W01/wallname),force='scene')
    # existing wall in millimetres, new arm in metres
    original.apply_scale(.001)
    return original

def with_front_track(sc):
    # temporary placeholder of module 03; not represented as completed design
    mesh=trimesh.creation.box(extents=[.11,3.50,.09]);mesh.apply_translation([4.5,1.75,.018]);mesh.visual.face_colors=color(C['front']);sc.add_geometry(mesh,node_name='P03_front_rail_PLACEHOLDER')
    return sc

# CAD assemblies: exact six rigid arm geometries in current stance; kinematics reside in URDF/CSV, not STEP.
def cq_cylinder_between(p0,p1,rmm):
    a=cq.Vector(*(np.asarray(p0)*1000));b=cq.Vector(*(np.asarray(p1)*1000));dv=b-a
    return cq.Solid.makeCylinder(rmm,dv.Length,a,dv.normalized())

def make_arm_cad_assy(pose):
    idx=pose['i']; y=pose['y'];arm=cq.Assembly(name=f'TQ_M02_arm_{idx:02d}')
    js=fk(y,pose['angles'],pose['ell']);mc=cq.Color(.15,.49,.45)
    for k in range(3):arm.add(cq_cylinder_between(js[k],js[k+1],49),name=f'A{idx}_link_{k+1}',color=mc)
    a=js[3];b=js[4];v=(b-a)/np.linalg.norm(b-a)
    arm.add(cq_cylinder_between(a,a+.98*v,62),name=f'A{idx}_tel_outer',color=cq.Color(.37,.66,.72))
    arm.add(cq_cylinder_between(a+.78*v,b,38),name=f'A{idx}_tel_inner',color=mc)
    for k in range(4):
        arm.add(cq.Solid.makeSphere(73,cq.Vector(*(js[k]*1000))),name=f'A{idx}_joint_{k+1}',color=cq.Color(.16,.28,.29))
    arm.add(cq.Solid.makeSphere(67,cq.Vector(*(js[-1]*1000))),name=f'A{idx}_tip_connector',color=cq.Color(.76,.53,.26))
    return arm

# Save parametric CAD as 6 arms with separately named subassemblies (as static pose only).
assy=cq.Assembly(name='TIANQIONG_M02_6_ARM_SUPPORT')
for p in state('S0_SUPPORT'):
    # assembly nesting allows reuse as standard part
    assy.add(make_arm_cad_assy(p),name=f'M02_ARM_{p["i"]:02d}')
assy.save(str(OUT/'M02_六机械臂_展开装配.step'),exportType='STEP')
one=make_arm_cad_assy(state('S0_SUPPORT')[0]);one.save(str(OUT/'M02_单臂_四段伸缩结构.step'),exportType='STEP')

# 3D scenes W01+ six moving arms; front rail is explicitly future module placeholder.
for s in ['S0_SUPPORT','S1_RECONFIGURED','S2_WORK_TARGET']:
    sc=with_front_track(load_wall_scene(s))
    for p in state(s):arm_to_scene(sc,p)
    sc.export(OUT/f'M02_W01_{s}.glb')

# URDF 7DOF each: root Y prismatic, yaw Z, pitch around Y at starts of 4 links, terminal telescopic X.
robot=ET.Element('robot',{'name':'TIANQIONG_W01_M02_KINEMATICS'})

def link(name,body=None):
    el=ET.SubElement(robot,'link',name=name)
    if body:
        v=ET.SubElement(el,'visual');ET.SubElement(v,'origin',xyz=body['xyz'],rpy='0 0 0')
        ET.SubElement(ET.SubElement(v,'geometry'),'box',size=body['size'])
        ET.SubElement(ET.SubElement(v,'material',name=f'm_{name}'),'color',rgba=body.get('color','0.24 0.58 0.52 1'))
    # inertia dummy: not physically calibrated, explicitly marked in README
    ine=ET.SubElement(el,'inertial');ET.SubElement(ine,'mass',value='1');ET.SubElement(ine,'origin',xyz='0 0 0',rpy='0 0 0')
    ET.SubElement(ine,'inertia',ixx='.02',iyy='.02',izz='.02',ixy='0',ixz='0',iyz='0')
    return el

def joint(name,typ,pa,ch,xyz='0 0 0',rpy='0 0 0',axis=None,lim=None):
    el=ET.SubElement(robot,'joint',name=name,type=typ);ET.SubElement(el,'parent',link=pa);ET.SubElement(el,'child',link=ch);ET.SubElement(el,'origin',xyz=xyz,rpy=rpy)
    if axis:ET.SubElement(el,'axis',xyz=axis)
    if lim:
        ET.SubElement(el,'limit',lower=str(lim[0]),upper=str(lim[1]),effort='5000',velocity='1.0')

def fmt(vals):return ' '.join(f'{v:.7f}' for v in vals)

link('world')
link('w01_wall_proxy',{'xyz':'-0.3 1.75 1.4','size':'0.6 3.5 2.8','color':'.55 .68 .63 1'})
joint('wall_ground','fixed','world','w01_wall_proxy')
q_init={}
for i,y0 in enumerate(CFG['initial_y_m'],1):
    pref=f'arm_{i:02d}'
    slider=f'{pref}_root_slider';link(slider,{'xyz':'.085 0 -.29','size':'.07 .35 .41','color':'.76 .55 .29 1'})
    joint(f'{pref}_root_Y','prismatic','w01_wall_proxy',slider,xyz=fmt([0,y0,2.8]),axis='0 1 0',lim=[-.21,.21])
    yaw=f'{pref}_yaw';link(yaw);joint(f'{pref}_yaw_Z','revolute',slider,yaw,axis='0 0 1',lim=[-1.57,1.57])
    parent=yaw
    for k,L in enumerate(LF,1):
        now=f'{pref}_link_{k}'
        link(now,{'xyz':fmt([L/2,0,0]),'size':fmt([L,.1,.11])})
        xyz='0 0 0' if k==1 else fmt([LF[k-2],0,0]);joint(f'{pref}_pitch_{k}','revolute',parent,now,xyz=xyz,axis='0 1 0',lim=[-3.14,3.14]);parent=now
    # Fourth proximal 1m and telescoping extension 0..0.6m.
    now=f'{pref}_link_4_sleeve';link(now,{'xyz':'.50 0 0','size':'1.0 .13 .13','color':'.37 .66 .72 1'})
    joint(f'{pref}_pitch_4','revolute',parent,now,xyz=fmt([LF[-1],0,0]),axis='0 1 0',lim=[-3.14,3.14])
    link(f'{pref}_telescopic_rod',{'xyz':'-.10 0 0','size':'.22 .085 .085'})
    joint(f'{pref}_telescope_X','prismatic',now,f'{pref}_telescopic_rod',xyz='1.0 0 0',axis='1 0 0',lim=[0,.6])
    link(f'{pref}_tool_interface',{'xyz':'0 0 0','size':'.09 .10 .10','color':'.73 .49 .26 1'})
    joint(f'{pref}_tool_mount','fixed',f'{pref}_telescopic_rod',f'{pref}_tool_interface')
    angles=A0
    for k in range(4):q_init[f'{pref}_pitch_{k+1}']=float(-angles[k]+(angles[k-1] if k else 0))
    q_init[f'{pref}_root_Y']=0.0;q_init[f'{pref}_yaw_Z']=0.0;q_init[f'{pref}_telescope_X']=0.6
ET.indent(robot,space='  ')
ET.ElementTree(robot).write(OUT/'M02_W01_六臂_7DOF.urdf',encoding='utf-8',xml_declaration=True)
(OUT/'M02_六臂展开关节状态.json').write_text(json.dumps(q_init,indent=2),encoding='utf8')

# FK spot test both our direct FK and URDF product of joint transforms; lever arm static support closed chain represented only geometrically.
import scipy.spatial.transform as spt

def urdf_fk(yi,q,ell):
    # transform from root carriage to link4 tip for no yaw, matching URDF joint composition
    T=np.eye(4);angs=-q;parent=0.
    for k,L in enumerate([*LF,ell]):
        rel=angs[k]-parent;parent=angs[k]
        Q=np.eye(4);Q[:3,:3]=spt.Rotation.from_euler('y',rel).as_matrix();T=T@Q
        U=np.eye(4);U[0,3]=L;T=T@U
    out=T[:3,3]+np.array([0,yi,2.8]);return out
validate={}
for s in ['S0_SUPPORT','S1_RECONFIGURED','S2_WORK_TARGET']:
    ps=state(s);tt=np.array([fk(p['y'],p['angles'],p['ell'])[-1] for p in ps]); fktest=np.array([urdf_fk(p['y'],p['angles'],p['ell']) for p in ps]);
    validate[s]={'max_URDF_vs_analytical_FK_error_m':float(np.max(np.linalg.norm(tt-fktest,axis=1))),
       'tips_m':tt.round(6).tolist(),'roots_y_m':[p['y'] for p in ps], 'work_arm_ids':[p['i'] for p in ps if p['work']]}
    if s!='S2_WORK_TARGET':validate[s]['max_tip_distance_from_front_nominal_m']=float(np.max(np.linalg.norm(tt[:,[0,2]]-np.array([4.5,0]),axis=1)))
# Endpoint carries a 0.09m sleeve: real collision radius for crop area not checked.
validate['root_spacing']={'min_center_spacing_reconfigured_m':float(np.min(np.diff(CFG['reconfigured_y_m']))),
    'nominal_carriage_length_m':WPAR['dimensions_mm']['carriage_length_y']/1000,
    'min_carriage_gap_m':float(np.min(np.diff(CFG['reconfigured_y_m']))-WPAR['dimensions_mm']['carriage_length_y']/1000)}
validate['unverified']=['static assembly URDF has no front rail closure joint/contact solver','roof not physical soft-body; no wind loads','full motor envelopes and 3D swept collision not solved','S2 pose is a target only; path not certified','telescopic bearing and structural locks not dimensioned','stow geometry and transport clearance not validated','URDF inertials placeholders and no ros2_control']
(OUT/'M02_与W01接口及验证.json').write_text(json.dumps({'w01_source_parameters':str(W01/'W01_装配接口与参数.json'),'model':CFG,'validation':validate},indent=2,ensure_ascii=False),encoding='utf8')

# Motion state program: synchronized Y shift only; 121 frames; each root slider follows W01 exactly.
rows=[]
for k,t in enumerate(np.linspace(0,1,121)):
    a=smooth(t)
    for i,(v0,v1) in enumerate(zip(CFG['initial_y_m'],CFG['reconfigured_y_m']),1):
        rows.append({'time_s':round(t*6,4),'arm':i,'root_y_m':round((1-a)*v0+a*v1,7),'front_y_m':round((1-a)*v0+a*v1,7),
                     'yaw_rad':0,'q1_rad':float(q_init[f'arm_{i:02d}_pitch_1']),
                     'q2_rad':float(q_init[f'arm_{i:02d}_pitch_2']),
                     'q3_rad':float(q_init[f'arm_{i:02d}_pitch_3']),
                     'q4_rad':float(q_init[f'arm_{i:02d}_pitch_4']),
                     'telescopic_extension_m':0.6,'mode':'support_reconfig_only'})
with (OUT/'M02_六臂同步补位轨迹.csv').open('w',encoding='utf-8-sig',newline='') as f:
    wr=csv.DictWriter(f,fieldnames=list(rows[0]));wr.writeheader();wr.writerows(rows)

# Figure: axonometric renders of 3 states as shareable 3D views, separate from physical simulation.
def setup(ax,title,az=-61,elev=22):
    ax.set_title(title,fontsize=15,fontweight='bold',pad=10)
    ax.set_xlim(-.8,4.95);ax.set_ylim(-.12,3.63);ax.set_zlim(0,3.4)
    ax.set_xlabel('X 向棚前沿 / m');ax.set_ylabel('Y 沿棚长 / m');ax.set_zlabel('Z / m')
    ax.view_init(elev=elev,azim=az);ax.set_box_aspect((5.3,3.4,3.1),zoom=.92)
    ax.grid(False)

def draw(ax,name):
    st=state(name)
    # mother wall proxy + 2 rails + front line
    wallverts=np.array([[-.6,0,0],[0,0,0],[0,3.5,0],[-.6,3.5,0],[-.6,0,2.8],[0,0,2.8],[0,3.5,2.8],[-.6,3.5,2.8]])
    faces=[[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7],[4,5,6,7]]
    ax.add_collection3d(Poly3DCollection([wallverts[f] for f in faces],facecolor='#9EB9AE',edgecolor='#6C8A81',lw=.7,alpha=.54))
    for zz in [2.37,2.67]:ax.plot([.05,.05],[0,3.5],[zz,zz],color='#226F66',lw=3)
    ax.plot([4.5,4.5],[0,3.5],[.06,.06],color=C['front'],lw=6,alpha=.75)
    for p in st:
        points=fk(p['y'],p['angles'],p['ell']);cl=C['work'] if p['work'] else C['arm'];
        ax.plot(points[:,0],points[:,1],points[:,2],color=cl,lw=5,solid_capstyle='round')
        ax.scatter(points[1:4,0],points[1:4,1],points[1:4,2],color='#2A4E50',s=30)
        ax.scatter(*points[-1],color=C['front'],s=28)
        ax.text(-.08,p['y'],3.,str(p['i']),color='#244F49',fontsize=10,ha='center')

fig=plt.figure(figsize=(18,7));fig.patch.set_facecolor('#F7F9F8')
for idx,(s,t) in enumerate(zip(['S0_SUPPORT','S1_RECONFIGURED','S2_WORK_TARGET'],['S0 六臂全部支撑','S1 双轨同步补位','S2 2/5号作业目标'])):
    ax=fig.add_subplot(1,3,idx+1,projection='3d');setup(ax,t);draw(ax,s)
fig.suptitle('田穹 M02 + W01｜六机械臂模块装配（状态对照）',fontsize=21,fontweight='bold',y=.97)
fig.text(.5,.03,'说明：作业态是静态目标姿态，并非已验证的无碰撞转换轨迹。前沿轨道仅为下一模块的占位几何。',ha='center',fontsize=11,color='#596D66')
fig.savefig(OUT/'M02_W01_三状态装配对照.png',dpi=175,bbox_inches='tight');plt.close(fig)

fig=plt.figure(figsize=(14,8));ax=fig.add_subplot(projection='3d');setup(ax,'W01 + M02  |  六机械肋骨展开状态',az=-58,elev=27);draw(ax,'S0_SUPPORT');fig.savefig(OUT/'M02_W01_六臂总装轴测图.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig,axs=plt.subplots(2,2,figsize=(15,10));
for a in axs.flat:a.set_aspect('equal');a.set_facecolor('#F6F9F7');a.grid(alpha=.15)
# cross section
ax=axs[0,0];j=fk(.35,A0,1.6);ax.plot(j[:,0],j[:,2],'-o',lw=4,color=C['arm']);ax.fill_between(j[:,0],j[:,2],0,color='#A9D8C9',alpha=.2)
ax.add_patch(plt.Rectangle((-.6,0),.6,2.8,facecolor='#B1C6BC',alpha=.5));ax.plot([4.45,4.55],[.06,.06],color=C['front'],lw=8)
ax.set(xlim=(-.75,5),ylim=(-.2,3.15),xlabel='X 棚宽 / m',ylabel='Z 高度 / m',title='横截面：4 段伸缩式机械肋骨')
ax=axs[0,1];v0=np.array(CFG['initial_y_m']);v1=np.array(CFG['reconfigured_y_m'])
ax.scatter(v0,np.ones(6),s=100,color=C['arm']);ax.scatter(v1,np.zeros(6),s=100,color=C['front']);
for i in range(6):ax.plot([v0[i],v1[i]],[1,0],color='#8DA9A0');ax.text(v0[i],1.12,str(i+1),ha='center');ax.text(v1[i],-.13,str(i+1),ha='center')
ax.set(xlim=(0,3.5),ylim=(-.3,1.3),yticks=[0,1],yticklabels=['补位后','初始'],xlabel='Y 棚长 / m',title='六个根部滑座位置与运动行程')
ax=axs[1,0];xx=np.arange(1,5)
ax.bar(xx,LF.tolist()+[1.6],color=[C['arm']]*3+[C['telescope']]);ax.set(xticks=xx,xticklabels=['第一段','第二段','第三段','第四段'],ylabel='长度 / m',title='四段连杆展开长度');ax.set_ylim(0,1.9)
ax=axs[1,1];ax.axis('off')
rows=['母墙：3500 × 600 × 2800 mm','每个模块：6根伸缩机械臂','七自由度/臂：Y滑移 + 基座回转 + 4俯仰 + 末节伸缩','前三段：1.15 / 1.38 / 1.52 m','末节：1.00–1.60 m','末端名义位置：X=4.50 m, Z=0','根座补位最大行程：0.187 m','前沿锚轨：未设计，仅占位']
for n,r in enumerate(rows):ax.text(.03,.93-n*.112,r,fontsize=12,va='top',transform=ax.transAxes,color='#344B45')
fig.suptitle('田穹 M02｜单臂结构与 W01 接口设计参数',fontsize=20,fontweight='bold');fig.tight_layout(rect=[0,.02,1,.94]);fig.savefig(OUT/'M02_四段机械臂尺寸接口图.png',dpi=165,bbox_inches='tight');plt.close(fig)

# S0 -> S1 animation with end-following and maintaining joint angles. Quick 2D axonometric film.
fig=plt.figure(figsize=(10.5,7.2));ax=fig.add_subplot(111,projection='3d');
animframes=76
def frame(k):
    ax.cla();setup(ax,'M02 六机械臂同步补位（仅几何运动）')
    # render basic frame in blue-green
    ax.plot([0,0],[0,3.5],[2.8,2.8],color='#416C65',lw=9)
    ax.plot([4.5,4.5],[0,3.5],[0,0],color=C['front'],lw=5)
    t=smooth(k/(animframes-1))
    for i,(y0,y1) in enumerate(zip(CFG['initial_y_m'],CFG['reconfigured_y_m']),1):
        p=fk((1-t)*y0+t*y1,A0,1.6)
        ax.plot(p[:,0],p[:,1],p[:,2],color=C['arm'],lw=5)
        ax.plot(p[0,0],p[0,1],p[0,2],'o',color='#B38A49',markersize=7)
    ax.text2D(.04,.93,f'补位进度 {t:.0%}  /  最大移动 187 mm',transform=ax.transAxes,fontsize=12)
    return []
ani=FuncAnimation(fig,frame,frames=animframes,interval=75,blit=False)
try:ani.save(str(OUT/'M02_W01_六臂同步补位.mp4'),writer=FFMpegWriter(fps=20,bitrate=1600))
except Exception as e: print('VIDEO WARN',e)
plt.close(fig)

# Offline Plotly HTML, simplified layer that can switch scene states while keeping W01 geometry credible.
# Use WebGL Scatter3d line+marker for 6 arms, wall conceptual geometry via mesh3d.
fig=go.Figure()
for s in ['S0_SUPPORT','S1_RECONFIGURED','S2_WORK_TARGET']:
    for p in state(s):
        pp=fk(p['y'],p['angles'],p['ell'])
        fig.add_trace(go.Scatter3d(x=pp[:,0],y=pp[:,1],z=pp[:,2],mode='lines+markers',name=f'{s}:臂{p["i"]}',visible=(s=='S0_SUPPORT'),line=dict(color=C['work'] if p['work'] else C['arm'],width=12),marker=dict(size=5)))
N=6
walltrace=go.Mesh3d(x=[-.6,0,0,-.6,-.6,0,0,-.6],y=[0,0,3.5,3.5,0,0,3.5,3.5],z=[0,0,0,0,2.8,2.8,2.8,2.8],i=[0,0,0,1,1,1,2,2,3,3,4,4],j=[1,2,5,2,5,6,3,6,0,7,5,6],k=[2,3,4,6,4,5,7,7,4,4,6,7],opacity=.22,color='#8AA69C',name='母墙几何包络')
fig.add_trace(walltrace)
fig.add_trace(go.Scatter3d(x=[4.5,4.5],y=[0,3.5],z=[0,0],mode='lines',line=dict(width=14,color=C['front']),name='前沿锚轨占位'))
btn=[]
for idx,s in enumerate(['S0_SUPPORT','S1_RECONFIGURED','S2_WORK_TARGET']):
    viz=[False]*(N*3)+[True,True]
    for j in range(N):viz[idx*N+j]=True
    btn.append(dict(label={'S0_SUPPORT':'六臂展开','S1_RECONFIGURED':'同步补位','S2_WORK_TARGET':'双臂作业目标'}[s],method='update',args=[{'visible':viz},{'title':f'M02 + W01｜{s}  (概念运动学)'}]))
fig.update_layout(title='TIANQIONG M02 + W01｜六臂三维装配',scene=dict(xaxis_title='X 棚宽 /m',yaxis_title='Y 棚长 /m',zaxis_title='Z 高度 /m',aspectmode='manual',aspectratio=dict(x=1.45,y=1,z=.9)),updatemenus=[dict(type='buttons',direction='right',x=0.04,y=1.11,buttons=btn)],margin=dict(l=0,r=0,t=110,b=0),height=680)
fig.write_html(OUT/'M02_W01_三维交互模型.html',include_plotlyjs=True,full_html=True)

# documentation and zip
text='''# 田穹 W01 + M02-A 六机械臂模块数字样机\n\n## 从 W01 复用的精确装配基准\n- W01 坐标：X 朝棚内，Y 沿棚长，Z 朝上。原几何使用毫米，URDF/动画/GLB 使用米。\n- 根关节：(X=0,Z=2.8m)，六根机械臂的初始 Y 为 0.35、0.91、1.47、2.03、2.59、3.15m。\n- 根部双导轨 Z=2.37、2.67 m；中心距 0.30 m，归属 W01，M02 仅对接滑座。\n- 补位后的 Y：0.350、0.816667、1.283333、2.216667、2.683333、3.150m。\n\n## M02 建模\n- 同构六臂，每根7自由度：Y直线滑移(1)、基座偏航(1)、连杆俯仰(4)、第4段伸缩(1)。\n- 四段臂长 1.15 / 1.38 / 1.52 / 1.00～1.60m。\n- 展开姿态采用前一轮 M04/M06 已计算的四段绝对连杆角；与 W01 的 X=4.5m、Z=0 nominal front datum 匹配。\n- 三个GLB状态：六臂展开、根部六滑座与臂共同同步平移、2/5号静态作业目标。\n- 前沿锚轨当前以占位梁表示，并未将其与机械臂末端建成闭链滑动接触关节。\n- STEP 是静止的展开装配：含单臂CAD和六臂CAD，零件分别建模；不包含真实传动件或运动副。\n- URDF 是运动学树，**六根机械臂的滑座都以W01墙体为父节点**，所有随后的关节自然跟随根座平移。\n- URDF 惯量/质量均为占位值，禁止用于动力学/额定载荷结论。\n\n## 文件使用\n- M02_W01_S0_SUPPORT.glb：带实际W01模型的6臂展开总装，导入Blender可继续布置动画。\n- M02_W01_S1_RECONFIGURED.glb：W01滑座重排 + 六臂同步重排。\n- M02_W01_S2_WORK_TARGET.glb：第2、5号工作目标的静态几何姿态（并非安全路径）。\n- M02_六机械臂_展开装配.step：六臂实体CAD；W01的CAD另在W01资料包。\n- M02_单臂_四段伸缩结构.step：单臂静止实体CAD，便于继续编辑。\n- M02_W01_六臂_7DOF.urdf：ROS2关节树，可加载到robot_state_publisher/RViz进行姿态显示；未配置ros2_control。\n- M02_六臂展开关节状态.json：展开姿态对应的URDF相对关节角与伸缩值。\n- M02_六臂同步补位轨迹.csv：6秒、121帧的双轨同步几何插值。\n- M02_W01_三维交互模型.html：离线查看三种运动状态的轻量化三维示意。\n\n## 验证分级\n### 已进行\n1. W01根关节接口位置读取与单位换算。\n2. 四段机械臂解析FK与URDF关节变换对比。\n3. 展开姿态末端坐标检查（4.5, yi, 0）。\n4. 双轨同步Y平移：根部/末端相同位移；仅几何同步。\n5. 六个W01根滑块补位后不互换顺序；350mm滑座有约117mm最小名义间隙。\n\n### 尚未完成\n- 可折叠收纳姿态和真实电机尺寸下的六臂空间干涉。\n- 前沿锚轨闭链约束、运动中受力/安全锁止。\n- 连续棚膜18个接口、耐磨/承载及作业释放。\n- 第2/5号从支撑到工作目标的全程3D避障；以前模型发现作物区干涉，不能称为无碰撞。\n- 零部件重量、机械强度、关节扭矩、真实动力学。\n\n## 下一步\n以单臂 STEP/URDF 为基准，开展 M02-B 折叠姿态和六臂空间避碰设计；通过后进入模块03前沿锚轨的CAD装配。\n'''
(OUT/'README_M02_W01_模块装配说明.md').write_text(text,encoding='utf8')
(OUT/'生成脚本_build_tianqiong_m02.py').write_text(Path(__file__).read_text(encoding='utf8'),encoding='utf8')
package=ROOT/'田穹_M02_六机械臂与W01装配_设计包.zip'
with zipfile.ZipFile(package,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
    for f in OUT.iterdir():
        if f.is_file():z.write(f,arcname=f.name)
    z.write(W01/'W01_装配接口与参数.json',arcname='W01_装配接口与参数.json')
print('CREATED',package, 'files',len(list(OUT.iterdir())))
print('VALIDATION',json.dumps(validate,ensure_ascii=False)[:2200])
