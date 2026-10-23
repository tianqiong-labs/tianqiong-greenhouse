# -*- coding: utf-8 -*-
"""TIANQIONG W01 large movable mother-wall concept CAD.
Units: millimeters; coordinates strictly MATCH pre-existing M06:
  X = greenhouse outward direction (wall behind greenhouse is X=-600..0),
  Y = length along wall (0..3500), Z = vertical (0..2800).
Concept / animation geometry only; no load-bearing certification.
"""
from __future__ import annotations
from pathlib import Path
import math, json, zipfile, csv, os, warnings
import numpy as np
import cadquery as cq
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import plotly.graph_objects as go

ROOT=Path('/mnt/data'); OUT=ROOT/'田穹_W01_母墙数字样机'; OUT.mkdir(parents=True,exist_ok=True)
plt.rcParams.update({'font.sans-serif':['Noto Sans CJK JP','DejaVu Sans'],'axes.unicode_minus':False,'figure.dpi':145})
P={
 'length_y':3500.0,'thickness_x':600.0,'height_z':2800.0,'frame_tube':65.0,
 'slide_centers_initial':[350,910,1470,2030,2590,3150],
 'slide_centers_reconfigured':[350,816.6667,1283.3333,2216.6667,2683.3333,3150],
 'carriage_length_y':350., 'root_base_joints_x':0., 'root_base_joints_z':2800.,
 'root_lower_rail_z':2370., 'root_upper_rail_z':2670.,
 'wheel_radius':120.,'wheel_centers_y':[345.,3155.], 'wheel_centers_x':[-475.,-125.],
 'wheel_center_z_deployed_relative':280.,'wheel_center_z_mobile_relative':-40.,
 'mobile_wall_lift':160.,'wedge_projection_below_ground':60.,
 'module_gap_small_wall_y':750.0,
}
PALETTE={
 'frame':'#657E79','frame2':'#496764','shell':'#DCE8E2','shell2':'#A8C1B7',
 'rail':'#2C8C7D','carriage':'#D2A45E','root':'#2D6560',
 'wheel':'#303D43','hub':'#B5BEC0','lift':'#89969A','foot':'#866F4F',
 'connector':'#9A774E','placeholder':'#B2C6BC','storage':'#8E996B',
 'highlight':'#DEC27A'}

def rgb(key):
 s=PALETTE.get(key,key).lstrip('#'); return tuple(int(s[i:i+2],16) for i in (0,2,4))

def rgba(key,alpha=255): return (*rgb(key),alpha)

# One part is one independent CadQuery solid + positional metadata. Never merge kinematic parts.
PARTS=[]
def add(name,shape,group,color,transforms=None, visibility='all'):
 PARTS.append(dict(name=name,solid=shape,group=group,color=color,transforms=transforms or {}, visibility=visibility))

def bx(name,x0,x1,y0,y1,z0,z1,group,color,transforms=None,visibility='all'):
 if min(x1-x0,y1-y0,z1-z0)<=0: raise ValueError(name)
 solid=cq.Solid.makeBox(x1-x0,y1-y0,z1-z0,cq.Vector(x0,y0,z0))
 add(name,solid,group,color,transforms,visibility)

def cyl(name,r,length,xyz,axis,group,color,transforms=None):
 add(name,cq.Solid.makeCylinder(r,length,cq.Vector(*xyz),cq.Vector(*axis)),group,color,transforms)

# W01-01: structural skeleton, main rails along Y, structural posts and depth cross-members.
for ix,x in enumerate([-565.,-35.]):
 for level,z in enumerate([100.,2715.]):
  bx(f'F_long_{ix}_{level}',x-34,x+34,65,3435,z-49,z+49,'frame','frame')
 for j,y in enumerate([80.,915.,1750.,2585.,3420.]):
  bx(f'F_post_{ix}_{j}',x-34,x+34,y-35,y+35,157,2660,'frame','frame2')
 bx(f'F_middle_{ix}',x-30,x+30,90,3410,1440,1505,'frame','frame2')
for j,y in enumerate([80.,1750.,3420.]):
 for k,z in enumerate([125.,2690.]):
  bx(f'F_cross_depth_{j}_{k}',-565,-35,y-35,y+35,z-32,z+32,'frame','frame')
# extra root reinforcement behind guide tracks
bx('F_root_backing_plate',-78,-2,95,3405,2265,2760,'frame','frame2')
# structural ground contact - cross support beams above soil
for j,(ya,yb) in enumerate([(95,175),(515,2985),(3325,3405)]):
 bx(f'F_bottom_spreader_{j}',-490,-110,ya,yb,54,110,'frame','frame2')

# W01-02: removable insulated service panels, split into 5 along length; no planted crops inside wall.
for j in range(5):
 y0=95+j*660; y1=y0+646
 bx(f'P_outer_back_{j+1}',-608,-594,y0,y1,215,2595,'panels','shell')
 bx(f'P_front_lower_{j+1}',-6,6,y0,y1,220,2160,'panels','shell2')
# main lower access sill and roof coping
bx('P_wall_top_coping',-599,-40,70,3430,2760,2800,'panels','shell')
bx('P_wall_toe_seal',-598,-6,90,3410,25,76,'panels','shell2')

# W01-03: two continuous rails along Y with actual independent carriage bodies.
for z,key in [(P['root_lower_rail_z'],'lower'),(P['root_upper_rail_z'],'upper')]:
 bx(f'R_{key}_mount_plate',-17,30,120,3380,z-37,z+37,'root_rails','frame')
 bx(f'R_{key}_linear_guide',30,64,120,3380,z-16,z+16,'root_rails','rail')
 for i,y in enumerate([160.,3340.]):
  bx(f'R_{key}_end_stop_{i}',28,80,y-19,y+19,z-37,z+37,'root_rails','highlight')

# carriage dynamic location relative to starting position. 350mm in Y.
for idx,y in enumerate(P['slide_centers_initial'],1):
 group=f'carriage_{idx:02d}'
 bx(f'C{idx:02d}_carrier_body',69,105,y-175,y+175,2305,2718,group,'carriage')
 for key,z in [('lower',P['root_lower_rail_z']),('upper',P['root_upper_rail_z'])]:
  bx(f'C{idx:02d}_{key}_guide_shoe',60,93,y-139,y+139,z-37,z+37,group,'root')
 # root upper shoulder and attachment lug, root joint center is exactly (X=0,Y=y,Z=2800), M06 datum
 bx(f'C{idx:02d}_root_adapter',-19,75,y-75,y+75,2708,2780,group,'root')
 cyl(f'C{idx:02d}_joint_reference',28,18,(0,y,2782),(0,0,1),group,'highlight')

# W01-04: deployable wedges, shallow seal only, do not count as certified anti-uplift anchors.
for ix,(x0,x1) in enumerate([(-540,-385),(-245,-90)]):
 for j,(ya,yb) in enumerate([(115,175),(515,2985),(3325,3385)]):
  bx(f'G_foot_beam_{ix}_{j}',x0,x1,ya,yb,-20,52,'ground_contact','foot')
  # segment wedge around wheel bays to avoid motorized wheel penetration
  bx(f'G_wedge_lip_{ix}_{j}',(x0+x1)/2-12,(x0+x1)/2+12,ya,yb,-60,-20,'ground_contact','foot')
 for j,y in enumerate(P['wheel_centers_y']):
  # bay shutter: closes wheel opening in deployed mode; stows when wheels extend
  bx(f'G_wheel_bay_shutter_{ix}_{j}',x0,x1,y-170,y+170,-44,44,'bay_shutters','foot',{'mobile_dz':290})
# optional anchor sockets ONLY -- no soil anchors modeled as installed
for j,y in enumerate([580.,2920.]):
 bx(f'G_optional_anchor_socket_{j}',-390,-230,y-56,y+56,105,230,'anchor_reserve','storage')

# W01-05: four retractable wheel units, steering/tire assembly geometry simplified.
for a,x in enumerate(P['wheel_centers_x']):
 for b,y in enumerate(P['wheel_centers_y']):
  prefix=f'W{a+1}{b+1}'
  rz=P['wheel_center_z_deployed_relative']
  # housing fixed
  bx(f'{prefix}_housing',x-78,x+78,y-106,y+106,248,505,'wheel_housing','lift')
  # actuator telescopic leg + axle/wheel: these move down while module body lifts.
  bx(f'{prefix}_lift_rod',x-32,x+32,y-34,y+34,rz,rz+210,'wheel_actuator','hub', {'mobile_dz':-320})
  cyl(f'{prefix}_tire',P['wheel_radius'],74,(x-37,y,rz),(1,0,0),'wheel_actuator','wheel',{'mobile_dz':-320})
  cyl(f'{prefix}_hub',46,77,(x-38.5,y,rz),(1,0,0),'wheel_actuator','hub',{'mobile_dz':-320})

# W01-06: connector plates at end Y, four corner sockets, guide elements, standardized side interfaces.
for end,y in [('L',-12.),('R',3512.)]:
 for tier,z in [('low',420.),('mid',1400.),('upper',2420.)]:
  bx(f'J_{end}_{tier}_dock',-390,-210,y-29,y+29,z-63,z+63,'connectors','connector')
  cyl(f'J_{end}_{tier}_pin',19,42,(-300,y-21,z),(0,1,0),'connectors','highlight')
 # auxiliary pass-through plates (water, power, data still future-specific)
 bx(f'J_{end}_utility_bulkhead',-515,-427,y-30,y+30,810,1090,'connectors','root')

# W01-07: storage saddles on mother wall back; front-anchor rail stored along wall in transport.
for i,y in enumerate([380.,1750.,3120.]):
 bx(f'S_anchor_saddle_{i}',-707,-596,y-74,y+74,1210,1290,'storage','storage')
 bx(f'S_anchor_retaining_lip_{i}',-713,-687,y-74,y+74,1210,1390,'storage','connector')

assert len(PARTS)>100
# reference model interface points
manifest={
 'project':'TIANQIONG / 田穹','module':'W01 large structural mother-wall','revision':'W01-A','model_units':'mm',
 'coordinate_system':{'X': '棚宽方向，母墙正面 x=0，向棚内为正；墙体占 x=-600..0',
                      'Y': '棚长方向，模块占 y=0..3500，滑座沿 Y 平移',
                      'Z': '竖直向上，地面 z=0，母墙顶 z=2800'},
 'compatibility_note':'Use same axes as M06 model (M06 x=wall outward, y=wall length, z=up). Earlier chat had opposite X/Y verbal convention; geometry follows existing M06 assets.',
 'dimensions_mm':P,'state_definitions':{
 'DEPLOYED':'墙底 z=0, 轮组回收, 六根滑座初始位置','MOBILE':'墙体抬高160mm, 轮组下伸320mm（相对墙体）, 轮胎接地',
 'RECONFIGURED':'墙底着地, 2/5号工作臂滑座停放, 3/4号移动到新支撑位置（仅说明滑座目标）'},
 'root_joint_datums_mm':[{'arm':i+1,'x':0.0,'y':v,'z':2800.0} for i,v in enumerate(P['slide_centers_initial'])],
 'root_joint_datums_reconfigured_mm':[{'arm':i+1,'x':0.0,'y':v,'z':2800.0} for i,v in enumerate(P['slide_centers_reconfigured'])],
 'root_dual_rail_spacing_z_mm':300,
 'interfaces':{
 'six_arm_bases':'6x root connection datum (x=0,z=2800), translation along Y only',
 'adjacent_small_wall_left':'side couplers near y=0, no permanent ground plinth',
 'adjacent_small_wall_right':'side couplers near y=3500, intended for 0.75m small wall gap',
 'front_anchor_rail_storage':'back face saddles around x=-700,z=1250, along module length',
 'power_data_water':'end-wall bulkhead envelopes reserved, unengineered',
 'equipment_cavities':'battery HVAC water cooling are envelopes / reserved volumes; no installed mass assigned',
 'solar_wing':'mounting reserved on back/top, PV not installed in W01 to avoid false geometry agreement',
 'base_anchoring':'shallow wedge feet + movable wheel-bay shutters modeled; uplift anchor sockets provisional only'},
 'engineering_not_verified':['wall frame member strength and wind uplift','linear guide bearing moments or anti-fall lock','wheel lift actuator rating, ground bay-shutter interlocks and wall mass','wedge soil seal and ground settlement','inter-module connector strength','body finite element and combined six-arm bending','real six-arm folding swept volumes','PV wing collisions and flexible membrane reel clearance','long-haul transport clearances and load restraint'],
 'note':'CAD intended for concept competition digital twin and animation. Mechanisms are separate solids; no material certification or control ready.'
}
with open(OUT/'W01_装配接口与参数.json','w',encoding='utf-8') as f:json.dump(manifest,f,indent=2,ensure_ascii=False)

# CAD files and meshes: transform each subassembly with actual physical state
# Pre-tessellate each shape only once and translate vertices for state.
cache=[]
for p in PARTS:
 vs,fs=p['solid'].tessellate(2.5,0.24)
 v=np.array([(q.x,q.y,q.z) for q in vs],dtype=np.float64)
 faces=np.array(fs,dtype=np.int64)
 cache.append((v,faces))

def state_translation(p,state):
 z=160. if state=='MOBILE' else 0.
 if state=='MOBILE' and p['group']=='wheel_actuator': z-=320.
 if state=='MOBILE' and p['group']=='bay_shutters': z+=290.
 dy=0.
 if state=='RECONFIGURED' and p['group'].startswith('carriage_'):
  idx=int(p['group'].rsplit('_',1)[1])-1
  dy=P['slide_centers_reconfigured'][idx]-P['slide_centers_initial'][idx]
 return np.array([0.,dy,z])

def create_scene(state,all_groups=True):
 sc=trimesh.Scene()
 for p,(v,f) in zip(PARTS,cache):
  if not all_groups and p['group'] in ('panels', 'wheel_housing'): continue
  vs=v+state_translation(p,state)
  mesh=trimesh.Trimesh(vertices=vs,faces=f,process=False)
  mesh.visual.face_colors=np.array(rgba(p['color']),dtype=np.uint8)
  sc.add_geometry(mesh,node_name=p['name'],geom_name=p['name'])
 return sc
for state in ('DEPLOYED','MOBILE','RECONFIGURED'):
 sc=create_scene(state)
 path=OUT/f'W01_{state}_完整装配.glb';sc.export(str(path))
 print('GLB',state,os.path.getsize(path),'parts',len(sc.geometry),flush=True)
# Skeleton version
create_scene('DEPLOYED',False).export(str(OUT/'W01_FRAME_承重骨架.glb'))

# CADSTEP Assembly retains part names and components, not collapsed/fused.
assy=cq.Assembly(name='TIANQIONG_W01_MOTHER_WALL')
for p in PARTS:
 if p['group'] in ('wheel_actuator',): continue # user gets deployed wheels in second GLB, STEP to include full deployed version: actually include
 assy.add(p['solid'],name=p['name'],color=cq.Color(*(v/255 for v in rgb(p['color']))))
# add omitted wheel movable parts, as separate un-fused STEP bodies
for p in PARTS:
 if p['group']=='wheel_actuator':assy.add(p['solid'],name=p['name'],color=cq.Color(*(v/255 for v in rgb(p['color']))))
steppath=OUT/'W01_大型母墙_参数化装配.step'
assy.save(str(steppath),exportType='STEP')
print('STEP',os.path.getsize(steppath),flush=True)

# 2D engineering layout: ground-to-wall, front rail, plan, interface detail.
from matplotlib.patches import Rectangle, Circle, FancyBboxPatch, Polygon
C={name: '#'+PALETTE[name].strip('#') for name in PALETTE}
fig=plt.figure(figsize=(17.0,12.3))
axs=fig.subplots(2,2)
for ax in axs.ravel(): ax.set_aspect('equal');ax.set_facecolor('#F6F8F7');ax.grid(False)
ax=axs[0,0]
ax.add_patch(Rectangle((0,0),3.5,2.8,facecolor='#DDE8E2',edgecolor='#445E57',lw=2))
for yy in np.array([0,.7,1.4,2.1,2.8,3.5]):ax.plot([yy,yy],[0,2.8],c='#748B83',lw=1)
for zz in [0.1,1.45,2.68]: ax.plot([0,3.5],[zz,zz],c='#738A83',lw=1.2)
for zz in [2.37,2.67]:ax.plot([.12,3.38],[zz,zz],lw=5,c=C['rail'])
for i,yy in enumerate(np.array(P['slide_centers_initial'])/1000):
 ax.add_patch(Rectangle((yy-.175,2.30),.35,.42,facecolor=C['carriage'],edgecolor='#75603E',lw=1.1,alpha=.92))
 ax.text(yy,2.51,str(i+1),va='center',ha='center',fontsize=11,fontweight='bold')
ax.set_xlim(-.15,3.7);ax.set_ylim(-.1,3.07);ax.set_xlabel('Y：沿棚长 / m');ax.set_ylabel('Z：高度 / m');ax.set_title('① 面向棚内正视：六滑座 + 上下承重轨',fontweight='bold')
ax=axs[0,1]
ax.add_patch(Rectangle((-.6,0),.6,2.8,facecolor='#E4EEE9',edgecolor='#526A60',lw=2))
for zz in [.14,2.7]:ax.plot([-.6,0],[zz,zz],c='#526A60',lw=4)
for zz in [2.37,2.67]:ax.add_patch(Rectangle((0.025,zz-.016),.04,.032,color=C['rail']))
ax.add_patch(Rectangle((.065,2.30),.045,.42,color=C['carriage']))
ax.plot([0],[2.8],marker='o',c=C['highlight'],markersize=9)
for xx in [-.475,-.125]:
 ax.add_patch(Circle((xx,.28),.12,facecolor=C['wheel'],edgecolor='#F5F5F5',lw=1))
ax.plot([-.65,.28],[0,0],c='#887A60',lw=1.5)
ax.add_patch(Rectangle((-.50,-.06),.40,.06,color=C['foot']))
ax.set_xlim(-.78,.30);ax.set_ylim(-.2,3.08);ax.set_xlabel('X：向棚内为正 / m');ax.set_ylabel('Z：高度 / m');ax.set_title('② 模块截面：墙体深度与落地轮组',fontweight='bold')
ax=axs[1,0]
ax.add_patch(Rectangle((0,-.6),3.5,.6,facecolor='#E1EAE5',edgecolor='#4C685E',lw=2))
ax.plot([.12,3.38],[.07,.07],c=C['rail'],lw=5)
for i,yy in enumerate(np.array(P['slide_centers_initial'])/1000):
 ax.add_patch(Rectangle((yy-.175,.037),.35,.065,facecolor=C['carriage'],edgecolor='#654D30'))
 ax.text(yy,-.30,str(i+1),fontsize=10,ha='center')
for yy in [0,3.5]:ax.add_patch(Rectangle((yy-.04,-.38),.08,.15,facecolor=C['connector']))
for yy in [.345,3.155]:
 for xx in [-.475,-.125]: ax.add_patch(Circle((yy,xx),.10,color='#616E69'))
ax.set_xlim(-.20,3.7);ax.set_ylim(-.8,.25);ax.set_xlabel('Y：沿棚长 / m');ax.set_ylabel('X：墙体厚度 / m');ax.set_title('③ 顶视：六滑座、四轮组和两端接口',fontweight='bold')
ax=axs[1,1]
ys=np.array(P['slide_centers_initial'])/1000; yr=np.array(P['slide_centers_reconfigured'])/1000
for i in range(6):
 ax.plot([ys[i],yr[i]],[1,0],c='#91A59C',alpha=.7,lw=1.2)
 ax.scatter(ys[i],[1],s=140,color=C['rail'])
 ax.scatter(yr[i],[0],s=140,color=C['carriage'] if i in(1,4) else C['root'])
 ax.text(ys[i],1.12,str(i+1),ha='center',fontsize=10)
 ax.text(yr[i],-.13,str(i+1),ha='center',fontsize=10)
ax.set_xlim(0,3.5);ax.set_ylim(-.28,1.38);ax.set_yticks([0,1],['补位后','初始']);ax.set_xlabel('Y：沿棚长 / m');ax.set_title('④ 六滑座位置状态：2、5 号转为作业臂',fontweight='bold')
fig.suptitle('田穹 W01｜大型母墙模块设计基准与装配坐标',fontsize=23,fontweight='bold')
fig.text(.5,.016,'CAD 单位 mm｜W01 大型墙模块 3500 × 600 × 2800｜机械臂根关节对接基准 (X=0, Z=2800)',ha='center',fontsize=12,color='#56695F')
fig.tight_layout(rect=(.03,.035,.98,.95)); fig.savefig(OUT/'W01_四视结构和六滑座布局.png',dpi=175,bbox_inches='tight'); plt.close(fig)

# Render axonometric CAD (glass/panels etc) via simple fast trimesh polygonal box visual, few polygons.
from matplotlib.colors import to_rgba
from mpl_toolkits.mplot3d import Axes3D

def box_vertices_faces(s):
 bb=s.BoundingBox(); x0,x1=bb.xmin,bb.xmax;y0,y1=bb.ymin,bb.ymax;z0,z1=bb.zmin,bb.zmax
 v=np.array([[x0,y0,z0],[x1,y0,z0],[x1,y1,z0],[x0,y1,z0],[x0,y0,z1],[x1,y0,z1],[x1,y1,z1],[x0,y1,z1]])/1000
 return v,[[0,1,2,3],[4,5,6,7],[0,1,5,4],[1,2,6,5],[2,3,7,6],[3,0,4,7]]

def show3d(ax,state,panels=True,elev=25,az=-53):
 ax.view_init(elev=elev,azim=az)
 plist=[];colors=[]
 for p in PARTS:
  if not panels and p['group']=='panels':continue
  v,fs=box_vertices_faces(p['solid'])
  delta=state_translation(p,state)/1000;v=v+delta
  # wheel cylinders use a bounding box polygon as coarse preview; GLB has proper curved cylinders.
  for fi in fs:
   plist.append(v[fi]); col=list(to_rgba(PALETTE[p['color']],alpha=0.75 if p['group']=='panels' else .96)); colors.append(col)
 col=Poly3DCollection(plist,facecolors=colors,edgecolor=(.29,.34,.32,.2),linewidths=.14,zsort='average')
 ax.add_collection3d(col)
 ax.set_xlim(-.85,.43); ax.set_ylim(-.15,3.65);ax.set_zlim(-.20,3.15)
 ax.set_box_aspect((1.5,3.6,3.0));ax.set_xlabel('X / m');ax.set_ylabel('Y / m');ax.set_zlabel('Z / m');ax.tick_params(labelsize=8)

fig=plt.figure(figsize=(15.5,9.0)); ax=fig.add_subplot(111,projection='3d');show3d(ax,'DEPLOYED',panels=True,elev=24,az=-63)
ax.set_title('田穹 W01｜大型母墙装配轴测（部署落地状态）',fontsize=18,pad=15)
fig.tight_layout();fig.savefig(OUT/'W01_母墙装配轴测图.png',dpi=170,bbox_inches='tight');plt.close(fig)
fig=plt.figure(figsize=(16.4,8.2))
for j,state in enumerate(['DEPLOYED','MOBILE']):
 ax=fig.add_subplot(1,2,j+1,projection='3d');show3d(ax,state,True,elev=18,az=-68)
 ax.set_title('部署：墙底落地、轮组回收' if j==0 else '迁移：墙体升起、轮组伸出',fontsize=14)
fig.suptitle('田穹 W01｜升降轮组两状态对照',fontsize=19,fontweight='bold')
fig.tight_layout();fig.savefig(OUT/'W01_部署与移动状态.png',dpi=160,bbox_inches='tight');plt.close(fig)
fig=plt.figure(figsize=(15.5,9.0));ax=fig.add_subplot(111,projection='3d');show3d(ax,'DEPLOYED',False,24,-66)
ax.set_title('田穹 W01｜承重骨架与根部双轨（去除服务盖板）',fontsize=18)
fig.tight_layout();fig.savefig(OUT/'W01_承重框架及双导轨.png',dpi=160,bbox_inches='tight');plt.close(fig)

# Offline interactive 3D HTML based on CAD tessellations, grouped by physical subsystem.
# Plotly use distinct traces per groups per state, dropdown switches visibility.
group_names=list(dict.fromkeys(p['group'] for p in PARTS))
fig=go.Figure()
for state in ['DEPLOYED','MOBILE','RECONFIGURED']:
 for group in group_names:
  vlist=[]; flist=[]; facecols=[];offset=0
  for p,(v,f) in zip(PARTS,cache):
   if p['group']!=group:continue
   vv=(v+state_translation(p,state))/1000
   vlist.append(vv)
   flist.append(f+offset)
   offset+=len(vv)
  if not vlist: continue
  vs=np.vstack(vlist);faces=np.vstack(flist)
  color=PALETTE[next(p['color'] for p in PARTS if p['group']==group)]
  fig.add_trace(go.Mesh3d(x=vs[:,0],y=vs[:,1],z=vs[:,2],i=faces[:,0],j=faces[:,1],k=faces[:,2],
            flatshading=True,color=color,opacity=.84 if group=='panels' else 1.0,
            name=group,visible=state=='DEPLOYED',showlegend=state=='DEPLOYED',lighting=dict(ambient=.65,diffuse=.75),
            hovertemplate=f'{state}: {group}<extra></extra>'))
N=len(group_names)
buttons=[]
for j,(key,label) in enumerate([('DEPLOYED','部署／落地'),('MOBILE','迁移／升轮'),('RECONFIGURED','六滑座补位')]):
 flags=[False]*(3*N)
 for q in range(j*N,(j+1)*N): flags[q]=True
 buttons.append(dict(label=label,method='update',args=[{'visible':flags},{'title':f'田穹 W01｜{label}（仅几何状态）'}]))
fig.update_layout(title='田穹 W01｜部署／迁移／滑座补位',scene=dict(aspectmode='data',
  xaxis_title='X 向棚内 (m)',yaxis_title='Y 沿棚长 (m)',zaxis_title='Z 高度 (m)',
  camera=dict(eye=dict(x=2.2,y=-2.6,z=1.5))),
  updatemenus=[dict(type='buttons',direction='right',buttons=buttons,x=.01,y=1.03,xanchor='left',yanchor='bottom')],
  margin=dict(l=0,r=0,t=75,b=0),height=780,legend=dict(orientation='h',y=-.12),paper_bgcolor='#F8FAF9')
html=OUT/'W01_可交互三维装配_三状态.html'
fig.write_html(str(html),include_plotlyjs=True,full_html=True)
print('HTML',os.path.getsize(html),flush=True)

# Concise engineering record & links
with open(OUT/'README_从W01到六臂装配.md','w',encoding='utf-8') as f:
 f.write('''# 田穹 TIANQIONG｜W01 大型母墙数字样机\n\n版本 W01-A（数字概念模型，非实体制造图）\n\n## 坐标及对接基准\n\n- 与既有 M06 模型保持相同坐标：**X 从母墙向棚内，Y 沿暖棚长度，Z 向上**，单位毫米。\n- 墙体几何范围：X=-600..0，Y=0..3500，Z=0..2800。\n- 机械臂基座根关节对接点 (X=0, Z=2800)，Y 初始位置：350、910、1470、2030、2590、3150。\n- 双承重导轨上下中心高 Z=2370 和 2670，间距 300 mm。\n- 补位后 Y：350、816.667、1283.333、2216.667、2683.333、3150。\n- 根部滑座为350 mm长的几何代理，实际承重及避碰仍未验证。\n\n## 组成\n\n1. 承重框架：前后纵梁、立柱、横撑、根部安装背板。\n2. 双根部导轨：同一母墙顶部沿 Y 方向安装两条上下承重导轨。\n3. 六根独立滑动基座：可以沿 Y 平移，带有机械臂根关节的标准对接基准。\n4. 收轮落地：四轮升降机构 + 浅楔形接地裙边；轮组穿越底边处增设可动密封挡板（几何占位），**并未校核土壤地锚**。\n5. 模块化围护板：标准可拆卸盖板；墙体内部没有种植仓。\n6. 两端连接件：连接小墙模块预留位置，连接强度未校核。\n7. 前沿锚轨收纳卡座：布置在母墙背侧；仍需要核查与光伏翼板空间冲突。\n\n## 动作逻辑与后续仿真\n\n- DEPLOYED：墙体底部贴地，车轮回收，六个根座处于均匀初始位置。\n- MOBILE：四个车轮的轮轴相对于墙体下降 320 mm，同时墙体相对地面抬高160 mm。\n- RECONFIGURED：墙体保持落地，六根滑座沿 Y 前后补位，为第2、5根臂脱离作业准备。\n- 展开动作只是运动学几何提案，**轮组驱动、结构强度、滑块额定动载及制动互锁尚未验证**。\n\n## 动力学/承载提醒\n\n上一轮举例根部悬臂弯矩约471 N·m；双导轨中心距300 mm时，力偶单边载荷量级约1.57 kN，\n此估算是特定假设下的力偶，不是轴承额定载荷，更不能将六个这样的值简单视作整墙的风荷载。\n还需计算墙体抗倾覆/抗拔、地基接触压强、侧墙连接强度、移动时重心和升降轮组功率。\n\n## 文件\n\n- `W01_大型母墙_参数化装配.step`：CAD多零件、保留零件名称。\n- `W01_DEPLOYED_完整装配.glb`、`W01_MOBILE_完整装配.glb`、`W01_RECONFIGURED_完整装配.glb`：三状态三维场景。\n- `W01_FRAME_承重骨架.glb`：隐藏盖板，观察承重框架。\n- `W01_可交互三维装配_三状态.html`：单文件离线交互式三维场景。\n- `W01_四视结构和六滑座布局.png`：工程布置示意。\n- `W01_装配接口与参数.json`：供六机械臂及仿真程序读取的接口坐标。\n- `build_tianqiong_w01.py`：参数化 CAD 生成源码；修改顶部尺寸后可重新生成。\n\n## 下一模块\n\n模块 02：六根机械臂，使用根关节对接点 (0, yi, 2800)，确认其根部滑座与交错折叠包络；随后加入前沿锚轨、膜面。\n''')

# zip only designed deliverables
archive=ROOT/'田穹_W01_大型母墙数字样机_设计包.zip'
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
 for file in OUT.iterdir():
  if file.is_file():z.write(file,arcname='W01/'+file.name)
 z.write(Path(__file__),arcname='W01/build_tianqiong_w01.py')
print('ALL DONE',len(PARTS),'parts','ZIP',os.path.getsize(archive),flush=True)
