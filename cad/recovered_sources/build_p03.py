#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""田穹 P03-A / W01 M02-compatible kinematic front rail conceptual CAD.
Units: CAD mm, scene metres. Includes 6 independently retained rail carriages and
an optional 750-mm connector segment. NOT a dimensioned load-bearing design.
"""
import math,json,csv,zipfile,shutil, copy,os
from pathlib import Path
import numpy as np
import cadquery as cq
import trimesh
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle,FancyArrowPatch,Circle
from matplotlib.animation import FuncAnimation, FFMpegWriter
import plotly.graph_objects as go

ROOT=Path('/mnt/data'); OUT=ROOT/'田穹_P03_前沿锚轨数字样机'; OUT.mkdir(exist_ok=True)
WPAR=json.loads((ROOT/'田穹_W01_母墙数字样机/W01_装配接口与参数.json').read_text(encoding='utf8'))
Y0=np.array(WPAR['dimensions_mm']['slide_centers_initial'],float)
Y1=np.array(WPAR['dimensions_mm']['slide_centers_reconfigured'],float)
P=dict(project='TIANQIONG',revision='P03-A',unit='mm',datum=dict(x='from mother-wall towards greenhouse front',y='along greenhouse length',z='up; soil plane z=0'),
       rail_length_y=3500,rail_body_center_x=4620,rail_section_x=120,rail_body_base_z=32,rail_body_top_z=145,
       nominal_arm_wrist=dict(x=4500,z=0),carriage_y_length=240,carriage_center_z=175,
       initial_y=Y0.tolist(),reconfigured_y=Y1.tolist(),carriage_y_limit=[220,3280],
       anchor_y=[260,1200,2300,3240],anchor_x=4665,anchor_extended_bottom_z=-160,anchor_stowed_bottom_z=22,
       seal_x=4700,storage_x=-750,storage_z=1250,bridge_y_length=750,
       load_notes='Concept geometry; no stiffness, stress, soil pull-out or kinematic lifting path validated.')
assert min(np.diff(Y1))>P['carriage_y_length']

# CAD geometry helper in mm
A=cq.Assembly(name='TIANQIONG_P03_3500_FRONT_ANCHOR_RAIL')
CO={'beam':cq.Color(.31,.39,.40),'guide':cq.Color(.23,.56,.53),'slide':cq.Color(.72,.54,.26),
    'wrist':cq.Color(.80,.83,.82),'anchor':cq.Color(.45,.48,.53),'seal':cq.Color(.25,.54,.45),'lock':cq.Color(.89,.47,.19)}

def cbox(assy,name,center,size,color):
    obj=cq.Workplane('XY').box(*map(float,size)).val().translate(tuple(map(float,center)))
    assy.add(obj,name=name,color=CO.get(color,cq.Color(.5,.5,.5)))
    return obj

def ccyl(assy,name,center,r,h,color):
    obj=cq.Solid.makeCylinder(float(r),float(h),cq.Vector(center[0],center[1],center[2]-h/2),cq.Vector(0,0,1))
    assy.add(obj,name=name,color=CO.get(color,cq.Color(.5,.5,.5)))
    return obj

def cad_front(assy,ys=Y0,anchors_deployed=True,brackets=True,prefix='P03'):
    cx=P['rail_body_center_x'];L=P['rail_length_y']
    # multipart rail profile runs parallel Y, nominal support at rear side x=4500 z=0
    cbox(assy,prefix+'_base_web',(cx,1750,49),(120,L,34),'beam')
    for i,x in enumerate([cx-48,cx+48]):
        cbox(assy,f'{prefix}_upright_{i+1}',(x,1750,96),(18,L,72),'guide')
        cbox(assy,f'{prefix}_upper_retainer_{i+1}',(x+(11 if i==0 else -11),1750,140),(38,L,13),'guide')
    cbox(assy,prefix+'_outer_soil_seal',(4700,1750,12),(30,L,18),'seal')
    cbox(assy,prefix+'_front_membrane_clamp',(4485,1750,24),(24,L,21),'seal')
    # positive alignment tongues / socket plates intentionally not fully engineered
    cbox(assy,prefix+'_end_socket_L',(cx,38,92),(132,76,70),'lock')
    cbox(assy,prefix+'_end_socket_R',(cx,3462,92),(132,76,70),'lock')
    for i,y in enumerate(ys,1):
        key=f'{prefix}_slider{i:02d}'
        cbox(assy,key+'_body',(cx,y,171),(92,240,27),'slide')
        # simple roller representations
        for j,dy in enumerate([-83,83]):
            ccyl(assy,key+f'_roller{j+1}',(cx,y+dy,153),18,16,'wrist')
        cbox(assy,key+'_positive_latch',(cx+4,y,194),(38,58,18),'lock')
        if brackets:
            # upward offset bracket: preserve legacy M02 wrist x=4500 z=0
            cbox(assy,key+'_wrist_upright',(4500,y,97),(25,32,194),'wrist')
            cbox(assy,key+'_upper_offset_beam',(4560,y,193),(120,33,24),'wrist')
            ccyl(assy,key+'_wrist_datum',(4500,y,18),15,36,'lock')
    for i,y in enumerate(P['anchor_y'],1):
        cbox(assy,f'{prefix}_anchor_housing_{i}',(4665,y,69),(80,100,88),'anchor')
        bottom=P['anchor_extended_bottom_z'] if anchors_deployed else P['anchor_stowed_bottom_z']
        h=45-bottom
        cbox(assy,f'{prefix}_anchor_rod_{i}',(4665,y,(bottom+45)/2),(22,22,h),'anchor')
        # wedges are simplified blunt ground pins; not detailed penetration profile
    for i,y in enumerate([350,1750,3150],1):
        cbox(assy,f'{prefix}_handling_eye_{i}',(cx+44,y,220),(38,64,52),'lock')

cad_front(A)
A.save(str(OUT/'P03_3500前沿锚轨_六滑块装配.step'),exportType='STEP')

B=cq.Assembly(name='P03_BR750_SMALL_WALL_TRANSITION_CONNECTOR')
# separate bridging segment for 0.75m small wall intervals; no travelling arm on this section
cx=P['rail_body_center_x'];
cbox(B,'bridge_web',(cx,375,49),(120,750,34),'beam')
for i,x in enumerate([cx-48,cx+48]):
    cbox(B,f'bridge_side_{i}',(x,375,96),(18,750,72),'guide')
    cbox(B,f'bridge_retainer_{i}',(x+(11 if i==0 else -11),375,140),(38,750,13),'guide')
cbox(B,'bridge_soil_seal',(4700,375,12),(30,750,18),'seal')
cbox(B,'bridge_front_membrane_clamp',(4485,375,24),(24,750,21),'seal')
for y in [34,716]:cbox(B,f'bridge_dock_{y}',(cx,y,96),(135,64,76),'lock')
B.save(str(OUT/'P03_BR750小墙过渡锚轨.step'),exportType='STEP')

# Add P03 CAD as separate assembly to original W01 M02 STEP (static deployed pose)
full=cq.Assembly(name='TIANQIONG_W01_M02_P03_STATIC_ASSEMBLY')
try:
    previous=cq.importers.importStep(str(ROOT/'田穹_M02_六机械臂数字样机/M02_W01_完整展开CAD总装.step'))
    full.add(previous.val(),name='W01_M02_prev_static_full',color=cq.Color(.50,.65,.61))
    full.add(A,name='P03_rail_docked')
    full.save(str(OUT/'P03_W01_M02_前沿锚轨完整总装.step'),exportType='STEP')
    print('STEP full assembly created')
except Exception as e:
    print('FULL STEP FAILED:',type(e).__name__,str(e)[:400])

# Real-state GLBs with P03 boxes (metres). Keep original M02-W01 detailed geometry.
C={'beam':[64,88,91,255],'guide':[40,136,122,255],'slide':[191,143,69,255],
   'wrist':[190,205,203,255],'anchor':[106,111,119,255],'seal':[56,142,112,255],'lock':[223,127,62,255]}
def mesh_box(sc,name,center,size,col):
    g=trimesh.creation.box(extents=np.array(size)/1000)
    g.apply_translation(np.array(center)/1000);g.visual.face_colors=C[col]
    sc.add_geometry(g,node_name=name,geom_name=name)

def rail_visual(scene,state,ys=None):
    # front rail (supported, reclustered, undocked), or visual nominal storage position
    ys=np.array(P['initial_y'] if ys is None else ys,float)
    store=state=='STORAGE_GOAL'
    if store:
        # shift from main center x=4620/z_nom=~125 to W01 storage saddle at x=-750,z=1250
        dx=P['storage_x']-P['rail_body_center_x']; dz=P['storage_z']-108
    else:dx=dz=0
    def b(n,xyz,dim,c):mesh_box(scene,'P03_'+state+'_'+n,(xyz[0]+dx,xyz[1],xyz[2]+dz),dim,c)
    b('base',(4620,1750,49),(120,3500,34),'beam')
    for i,x in enumerate([4572,4668]):
        b(f'guide{i}',(x,1750,96),(18,3500,72),'guide')
        b(f'lip{i}',(x+(11 if i==0 else -11),1750,140),(38,3500,13),'guide')
    b('membrane_front_clamp',(4485,1750,24),(24,3500,21),'seal')
    b('seal',(4700,1750,12),(30,3500,18),'seal')
    for i,y in enumerate(ys,1):
        b(f'cart_{i}',(4620,y,171),(92,240,27),'slide')
        b(f'cart_lock_{i}',(4624,y,194),(38,58,18),'lock')
        # in deployed support, bracket reaches *nominal* M02 wrist at ground.
        if not store and not (state=='WORK_UNDOCK' and i in [2,5]):
            b(f'wrist_upright_{i}',(4500,y,97),(25,32,194),'wrist')
            b(f'wrist_top_{i}',(4560,y,193),(120,33,24),'wrist')
    if not store:
        for i,y in enumerate(P['anchor_y'],1):
            b(f'anchor_housing_{i}',(4665,y,69),(80,100,88),'anchor')
            b(f'anchor_shank_{i}',(4665,y,-57.5),(22,22,205),'anchor')
    for i,y in enumerate([350,1750,3150],1):
        b(f'lift_datum_{i}',(4664,y,220),(38,64,52),'lock')
    return scene

src=OUT/'_M02_S0.glb'
for state,fn in [('SUPPORT','_M02_S0.glb'),('RECONFIGURED','_M02_S1.glb'),('WORK_UNDOCK','_M02_S2.glb')]:
    sc=trimesh.load(str(OUT/fn),force='scene')
    # M02 predecessor has P03 placeholder rail, remove it by importing only unplaceholder geometries (done via graph deletion)
    for n in list(sc.graph.nodes_geometry):
        if 'P03_front_rail_PLACEHOLDER' in n:
            sc.delete_geometry(sc.graph[n][1])
    rail_visual(sc,state,Y0 if state=='SUPPORT' else Y1)
    sc.export(str(OUT/f'P03_W01_M02_{state}.glb'))
# storage milestone is not a solved arm trajectory; show wall only plus docked rail
s0=trimesh.load(str(OUT/'_M02_S0.glb'),force='scene')
for n in list(s0.graph.nodes_geometry):
    if n.startswith('M02_R') or 'P03_front_rail_PLACEHOLDER' in n:
        s0.delete_geometry(s0.graph[n][1])
rail_visual(s0,'STORAGE_GOAL',Y0)
s0.export(str(OUT/'P03_W01_STORAGE_GOAL_only.glb'))

# 2D engineering image maps and section
plt.rcParams.update({'font.sans-serif':['Noto Sans CJK JP','DejaVu Sans'],'axes.unicode_minus':False})
fig,ax=plt.subplots(1,2,figsize=(15.2,6.5),gridspec_kw={'width_ratios':[1.75,1]})
fig.patch.set_facecolor('#f9fbf9')
a=ax[0];a.set_xlim(-.25,3.75);a.set_ylim(-.37,.65);a.set_aspect('equal');a.set_title('P03 3500 mm 前沿锚轨：俯视滑块布置',weight='bold')
a.add_patch(Rectangle((0,-.12),3.5,.24,facecolor='#516669',edgecolor='#263d3e'))
for i,y in enumerate(Y0,1):
    a.add_patch(Rectangle((y/1000-.12,-.16),.24,.32,facecolor='#c69754',edgecolor='#6c593d',lw=1.4))
    a.text(y/1000,0,i.__str__(),ha='center',va='center',fontsize=10,weight='bold',color='white')
for i,y in enumerate(P['anchor_y']):
    a.add_patch(Rectangle((y/1000-.05,.17),.10,.12,facecolor='#a0a5ae',edgecolor='#59636d'))
    a.text(y/1000,.33,'锚'+str(i+1),ha='center',fontsize=9)
a.annotate('',(.0,-.28),(3.5,-.28),arrowprops={'arrowstyle':'<->','color':'#374b48','lw':1.6})
a.text(1.75,-.33,'3500 mm / 单个大型母墙模块',ha='center',fontsize=10)
a.annotate('滑块不跨越模块端部',xy=(3.25,.03),xytext=(2.25,.48),arrowprops={'arrowstyle':'->','color':'#47645e'},fontsize=9)
a.set_xticks(np.arange(0,3.501,.5));a.set_yticks([]);a.set_xlabel('Y / m：棚长方向');a.spines[['top','right','left']].set_visible(False)

b=ax[1]; b.set_title('前沿接口：X-Z 剖面（概念尺寸）',weight='bold');b.set_aspect('equal');b.set_xlim(4410,4790);b.set_ylim(-205,285)
b.axhline(0,color='#9a8060',lw=2);b.text(4760,-13,'土壤表面 Z=0',ha='right',fontsize=8,color='#78664f')
for boxxy,wh,color,label in [((4560,32),(120,34),'#526467','主梁'),((4570,60),(20,72),'#518e85','导轨'),((4650,60),(20,72),'#518e85','导轨'),((4574,158),(92,27),'#c89a55','滑块'),((4487,5),(27,20),'#50977e','膜端密封')]:
    b.add_patch(Rectangle(boxxy,*wh,facecolor=color,edgecolor='#334543'))
b.add_patch(Rectangle((4492,0),20,190,facecolor='#bdcfc9',edgecolor='#6e8278'))
b.add_patch(Rectangle((4500,181),120,18,facecolor='#bdcfc9',edgecolor='#6e8278'))
b.add_patch(Rectangle((4654,-160),22,220,facecolor='#808997',edgecolor='#4f5a61'))
b.add_patch(Rectangle((4691,2),18,20,facecolor='#528d73',edgecolor='#2f725b'))
b.plot(4500,0,'o',color='#e08132',ms=7);b.text(4460,15,'M02 末端\n(4500,0)',ha='center',fontsize=9)
b.annotate('滑块正向锁止',xy=(4620,177),xytext=(4705,243),arrowprops={'arrowstyle':'->','lw':1.2},fontsize=9)
b.annotate('伸缩地锚',xy=(4664,-115),xytext=(4720,-175),arrowprops={'arrowstyle':'->','lw':1.2},fontsize=9)
b.annotate('离土封闭唇',xy=(4700,16),xytext=(4690,80),arrowprops={'arrowstyle':'->','lw':1.2},fontsize=9)
b.set_xlabel('X / mm（向前沿）');b.set_ylabel('Z / mm');b.spines[['top','right']].set_visible(False)
fig.tight_layout(pad=2);fig.savefig(OUT/'P03_滑块布置与接口剖面.png',dpi=180);plt.close(fig)

# multi-state 3D true coordinates simplified representation of W01 and actual M02 arm FK
angles=np.deg2rad([-1.6,-20.1,-38.63402497781479,-57.18221617233679]);lengths=np.array([1.15,1.38,1.52,1.6])
angles_work=np.deg2rad([-5,-70,0,-105]);len_work=np.array([1.15,1.38,1.52,1.0])
def fk(y,ang,lens):
    return np.array([[0,y,2.8]]+[[sum(lens[:k]*np.cos(ang[:k])),y,2.8+sum(lens[:k]*np.sin(ang[:k]))] for k in range(1,5)])

def draw3d(ax,config):
    # x front, y longitudinal, z up
    ax.plot([-0.6,0,0,-0.6,-0.6],[0,0,3.5,3.5,0],[0,0,0,0,0],color='#8fa8a1',lw=2)
    for y in [0,3.5]:ax.plot([-.6,-.6,-.6,-.6,-.6],[y,y,y,y,y],[0,.93,1.86,2.8,0],alpha=.0)
    for y in np.linspace(0,3.5,7):ax.plot([-.59,-.59], [y,y],[0,2.8],color='#91a8a2',lw=2,alpha=.6)
    for z in [0,2.8]:ax.plot([-.6,-.6],[0,3.5],[z,z],color='#6f948a',lw=3)
    for i,y in enumerate(config['ys'] if config['mode']!='STORAGE' else [],1):
        iswork=config['mode']=='WORK' and i in [2,5]
        v=fk(y,angles_work if iswork else angles,len_work if iswork else lengths)
        ax.plot(v[:,0],v[:,1],v[:,2],lw=3.3,color='#d18635' if iswork else '#287e77',marker='o',ms=2)
    if config['mode']=='STORAGE':
        ax.plot([-.75,-.75],[0,3.5],[1.25,1.25],lw=6,color='#b08347')
        ax.text(-.8,1.1,1.5,'轨道收纳位置',color='#9d712c',fontsize=8)
    else:
        ax.plot([4.62,4.62],[0,3.5],[.11,.11],lw=6,color='#b08347')
        for y in config['ys']:ax.scatter(4.62,y,.17,c='#c2934c',s=24)
        for y in np.array(P['anchor_y'])/1000:ax.plot([4.66,4.66],[y,y],[.04,-.16],color='#777f87',lw=2)
    ax.set(xlim=(-.9,5.2),ylim=(-.2,3.7),zlim=(-.2,3.15),xlabel='X: 棚宽 / m',ylabel='Y: 棚长 / m',zlabel='Z / m')
    ax.view_init(elev=24,azim=-64);ax.set_box_aspect((6,3.8,3.1),zoom=.95)
    ax.tick_params(labelsize=7);ax.grid(alpha=.15)

fig=plt.figure(figsize=(14,10));fig.patch.set_facecolor('#fbfcfb')
for j,(label,config) in enumerate([('① 六臂支撑 / 六车锁定',dict(mode='SUPPORT',ys=Y0/1000)),('② 六臂补位 / 前沿车随动',dict(mode='RECONF',ys=Y1/1000)),('③ 第2、5臂作业 / 滑块留轨',dict(mode='WORK',ys=Y1/1000)),('④ 锚轨贴墙收纳目标 / 机械臂路径未求解',dict(mode='STORAGE',ys=Y0/1000))]):
    ax=fig.add_subplot(2,2,j+1,projection='3d');draw3d(ax,config);ax.set_title(label,fontsize=11,pad=8,weight='bold')
fig.suptitle('田穹 P03 × W01 母墙 × M02 六臂：模块装配与状态关系',fontsize=16,weight='bold')
fig.subplots_adjust(left=.01,right=.99,bottom=.02,top=.94,wspace=.03,hspace=.10)
fig.savefig(OUT/'P03_W01_M02_四状态集成示意.png',dpi=160);plt.close(fig)

# interactive Plotly: simplified explicit geometry with complete arm shapes and rail states; no unproven transition animation
fig=go.Figure()
fig.add_trace(go.Mesh3d(x=[-.6,0,0,-.6,-.6,0,0,-.6],y=[0,0,3.5,3.5,0,0,3.5,3.5],z=[0,0,0,0,2.8,2.8,2.8,2.8],
                         i=[0,0,0,4,4,4,0,1,2,3,0,1],j=[1,2,3,5,6,7,1,2,3,0,5,6],k=[2,3,0,6,7,4,5,6,7,4,6,7],color='#8eaaa2',opacity=.36,name='W01母墙',hoverinfo='name'))
NAMES=['SUPPORT','RECONFIGURED','WORK_UNDOCK','STORAGE_GOAL']
for mode in NAMES:
    ys=Y0/1000 if mode in ['SUPPORT','STORAGE_GOAL'] else Y1/1000
    visible=(mode=='SUPPORT')
    if mode!='STORAGE_GOAL':
        rx=4.62; rz=.11
    else:rx=-.75;rz=1.25
    fig.add_trace(go.Scatter3d(x=[rx,rx],y=[0,3.5],z=[rz,rz],mode='lines',line=dict(width=15,color='#af823f'),name=f'{mode}:锚轨',visible=visible,hovertemplate='P03锚轨<extra></extra>'))
    fig.add_trace(go.Scatter3d(x=[rx]*6,y=ys,z=[rz+.075]*6,mode='markers+text',marker=dict(size=6,color='#c39a54'),text=[str(i) for i in range(1,7)],textposition='top center',name=f'{mode}:滑座',visible=visible))
    if mode!='STORAGE_GOAL':
        for i,y in enumerate(ys,1):
            work=(mode=='WORK_UNDOCK' and i in [2,5]); pts=fk(y,angles_work if work else angles,len_work if work else lengths)
            fig.add_trace(go.Scatter3d(x=pts[:,0],y=pts[:,1],z=pts[:,2],mode='lines+markers',line=dict(width=8,color='#ca8240' if work else '#2b897d'),marker=dict(size=4),name=f'{mode}:臂{i}',visible=visible))
    else:
        fig.add_trace(go.Scatter3d(x=[],y=[],z=[],mode='lines',name='收纳目标未计算机械臂姿态',visible=visible))
tracesPerMode=9
buttons=[dict(label=nm,method='update',args=[{'visible': [True]+[(i//tracesPerMode)==j for i in range(4) for _ in range(tracesPerMode)]}, {'title':f'田穹 P03：{nm}'}]) for j,nm in enumerate(NAMES)]
# above tracing mode has 8 traces (rail+sliders+6arm) vs 3 traces storage; use explicit ranges instead
start=1; mode_ranges=[]
# inspect actual traces using name-prefix so don't rely on constant N
for nm in NAMES:
    ids=[idx for idx,tr in enumerate(fig.data) if idx>0 and str(tr.name).startswith(nm+':') or (nm=='STORAGE_GOAL' and '收纳目标未计算' in str(tr.name))]
    mode_ranges.append(ids)
buttons=[dict(label={'SUPPORT':'六臂支撑','RECONFIGURED':'同步补位','WORK_UNDOCK':'2/5臂作业目标','STORAGE_GOAL':'锚轨收纳目标'}[nm],method='update',args=[{'visible':[i==0 or i in mode_ranges[j] for i in range(len(fig.data))]}, {'title':f'田穹 W01+M02+P03｜{nm}'}]) for j,nm in enumerate(NAMES)]
fig.update_layout(title='田穹 W01+M02+P03｜六臂支撑',updatemenus=[dict(buttons=buttons,direction='down',x=.01,y=1.12,xanchor='left',yanchor='top')],height=720,
                  scene=dict(xaxis_title='X：棚宽 m',yaxis_title='Y：棚长 m',zaxis_title='Z：高度 m',aspectmode='manual',aspectratio=dict(x=1.6,y=1,z=.85),camera=dict(eye=dict(x=1.55,y=-1.75,z=1.2))),showlegend=False,
                  margin=dict(l=0,r=0,b=0,t=60),annotations=[dict(text='几何状态切换，不代表已求解完整脱轨／抬轨路径',x=.5,y=1.01,xref='paper',yref='paper',showarrow=False,font=dict(size=12,color='#8b5d31'))])
fig.write_html(str(OUT/'P03_W01_M02_交互式装配.html'),include_plotlyjs=True,full_html=True)

# animated actual 6 carriages in sync on fixed rail: state S0 -> S1 (geometry only)
fig,ax=plt.subplots(figsize=(12,3.9));fig.patch.set_facecolor('#fbfcfb')
ax.set_xlim(0,3.5);ax.set_ylim(-.4,.6);ax.set_yticks([]);ax.set_xlabel('沿前沿锚轨 Y / m');ax.spines[['top','left','right']].set_visible(False)
ax.add_patch(Rectangle((0,-.14),3.5,.28,facecolor='#526e69'))
for y in np.array(P['anchor_y'])/1000:ax.plot([y,y],[.19,.34],color='#6f7780',lw=7)
patches=[Rectangle((v/1000-.12,-.17),.24,.34,facecolor='#cb994d',edgecolor='#6d532f') for v in Y0]
for p in patches:ax.add_patch(p)
title=ax.text(.02,.51,'六滑块处于初始状态',fontsize=14,weight='bold')
lab=[ax.text(v/1000,0,str(i),color='white',weight='bold',fontsize=11,ha='center',va='center') for i,v in enumerate(Y0,1)]
def ease(t):return 6*t**5-15*t**4+10*t**3
def update(frame):
    t=ease(frame/100); ys=(Y0*(1-t)+Y1*t)/1000
    for i,p in enumerate(patches):p.set_x(ys[i]-.12);lab[i].set_x(ys[i])
    title.set_text(f'根部主动、前沿被动：六滑块同步补位  {frame:.0f}%')
    return patches+lab+[title]
ani=FuncAnimation(fig,update,frames=101,interval=45,blit=True)
ani.save(str(OUT/'P03_六滑块同步补位演示.mp4'),writer=FFMpegWriter(fps=25,bitrate=1300))
plt.close(fig)

# State/kinematic constraints and CSV with matching M02 root slider y
csvpath=OUT/'P03_六滑块同步位移与锁止状态.csv'
with csvpath.open('w',encoding='utf-8-sig',newline='') as f:
    w=csv.writer(f);w.writerow(['phase','t_s','arm_id','root_y_mm','front_slider_y_mm','front_slider_locked','wrist_coupled','anchors_extended'])
    for k in range(101):
        t=k/100; e=ease(t)
        for i in range(6):w.writerow(['RECONFIGURE',round(4*t,3),i+1,round(Y0[i]+(Y1[i]-Y0[i])*e,4),round(Y0[i]+(Y1[i]-Y0[i])*e,4),int(k==0 or k==100),1,1])
    for i in range(6):w.writerow(['WORK_TARGET_NOT_PATH',4.1,i+1,Y1[i],Y1[i],1,int(i not in [1,4]),1])

P['computed_checks']={'six_carriages_initial_y':Y0.tolist(),'six_carriages_reconfigured_y':Y1.tolist(),'minimum_target_slider_center_pitch_mm':round(float(min(np.diff(Y1))),4),'minimum_nominal_slider_body_gap_mm':round(float(min(np.diff(Y1))-P['carriage_y_length']),4),'maximum_slider_travel_mm':round(float(max(abs(Y1-Y0))),4),'tip_datum_reaches_legacy_m02':True,'anchor_foot_depth_below_soil_mm':160,'small_wall_bridge_length_mm':750,'dynamic_rail_pickup_path_solved':False}
P['safety_notes']=['All 6 carriages remain captured in P03 front rail during arms 2/5 release','Wrist datum x=4500,z=0 is a legacy M02 abstraction; offset bracket is a model compatibility concept, not fabricated joint','Soil stake deployment and rail segment alignment must be physically checked','Rail lifting/storage state is a visual milestone only; six-arm collision-free synchronized lifting trajectory has NOT been solved','Anchors and carriage locking cannot be released under unverified wind/structural load','Each large wall section has its own rail; 750 mm small-wall gaps require BR750 segments, included as separate STEP','P03 glb/W01 M02 original placeholder rail was removed before adding actual P03 components.']
P['interfaces']={'M02_wrist_mm':{'x':4500,'z':0,'y':Y0.tolist()},'front_slider':{'x':4620,'z':175,'direction':'Y'},'bridge_small_wall':{'part':'P03_BR750','length_mm':750},'wall_storage_saddles':{'x':-750,'z':1250,'y':[350,1750,3150]},'anchor_extension':{'count':4,'deployed_bottom_z':-160,'retracted_bottom_z':22}}
(OUT/'P03_装配接口与参数.json').write_text(json.dumps(P,ensure_ascii=False,indent=2),encoding='utf8')
(OUT/'README_P03_结构与约束说明.md').write_text('''# 田穹 P03-A｜前沿滑移锚轨数字样机\n\n## 数据来源\n复用 W01 母墙 `W01_装配接口与参数.json` 和 M02 前一阶段 `M02_W01_完整展开CAD总装.step`、三姿态 GLB。\n坐标 X=母墙向棚内，Y=沿棚长，Z=向上；STEP 用毫米，GLB 与动画采用米。\n\n## 零件和接口\n- P03 前沿导轨长3500 mm，几何位于 X≈4620，主体落地附近。六个被动移动滑块按 M02 根座位置同步运行（初始 350/910/1470/2030/2590/3150 mm）。\n- M02 旧末端几何基准为 X=4500、Z=0。新增上跨式连接支架，使该名义基准连接到地面上方的轨道滑块；**连接支架未做实物强度、干涉或关节载荷验证**。\n- 六个前沿滑块中，2、5号机械臂去作业时，滑块留在轨道上。此时接口脱开，不是滑块消失。\n- 轨道具有 4 个伸缩式地锚、前沿柔性密封条、三处搬运吊点和两端连接座。\n- 相邻大墙之间小墙连接区约750 mm，单独建模 `P03_BR750小墙过渡锚轨.step` 补齐前沿；小墙区域不供机械臂跨越。\n- 轨道收纳目标中心 x≈-750,z≈1250，沿母墙长度方向水平固定。由机械臂吊起并转移的无碰撞路径尚未求解。\n\n## 四类概念状态\n- SUPPORT：6臂连接轨道、地锚伸出、母墙着地\n- RECONFIGURED：6个前沿滑块根随6个母墙根部滑座沿Y同步调整，前沿导轨本体固定\n- WORK_UNDOCK：第2/5臂脱离前沿滑块，滑块不离轨；机械臂目标姿态来自 M02，**非安全运动路径**\n- STORAGE_GOAL：锚轨贴近母墙外侧收纳位，仅示意目标几何，未绑定真实M02搬运关节轨迹\n\n## 交付与限制\nSTEP 为静态几何装配零件；GLB 为静态状态场景；HTML 可切换四状态；CSV 用于同步运动动画。MP4 只演示6滑块随动。\n未验证：轨道抗弯、锚固抗拔、插土阻力、轨道接头连接、腕部偏置支架强度、前沿闭链约束、真实六臂抬轨路径及收卷棚膜干涉。\n''',encoding='utf8')

# build delivery zip only including relevant deliverables (GLBs small, html large)
zipname=ROOT/'田穹_P03_前沿锚轨_与W01_M02集成设计包.zip'
with zipfile.ZipFile(zipname,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=7) as z:
    for f in sorted(OUT.iterdir()):
        if f.is_file() and not f.name.startswith('_'):
            z.write(f,arcname='P03/'+f.name)
print('DONE',zipname.stat().st_size, 'files=',len(list(OUT.iterdir())))
