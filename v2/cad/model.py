"""Source-aligned W01/M02/P03 rebuild and editable F04 concept, millimetres.
Only reviewed geometry AST prefixes of the recovered sources are executed.
STEP is named B-rep geometry; edit parameters and rerun for parametric edits.
No source binary identity, CAD motion constraints, membrane FEA or safety claim.
"""
from pathlib import Path
import ast, hashlib, json, math, shutil, struct, sys
import numpy as np
import cadquery as cq
import trimesh
from scipy.spatial.transform import Rotation

OUT = Path(__file__).resolve().parent
P = json.loads((OUT/'parameters.json').read_text(encoding='utf8'))
assert P['module_length']==3500 and P['small_wall_gap']==750, 'Original module dimensions are fixed source datums; change recovered geometry and interfaces together.'
SOURCE = OUT/'recovered_sources'
COLORS = {'wall':[.49,.62,.58,1], 'arm':[.15,.49,.45,1],
          'joint':[.16,.28,.29,1], 'telescope':[.37,.66,.72,1],
          'rail':[.40,.49,.50,1], 'film':[.42,.81,.91,.25],
          'band':[.13,.62,.64,.85], 'head':[.88,.65,.29,1],
          'gate':[.94,.38,.18,1], 'magnet':[.48,.43,.72,1],
          'roll':[.49,.56,.57,1], 'proposal':[.69,.49,.75,.55]}
CATALOG = {}
SHAPES = {}

def dump(name, obj):
    (OUT/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2),encoding='utf8')

def load_reviewed_geometry(filename, marker, replacements, env):
    text=(SOURCE/filename).read_text(encoding='utf8').split(marker)[0]
    for before,after in replacements:
        if before not in text: raise ValueError('source marker changed: '+before)
        text=text.replace(before,after)
    text=text.replace("matplotlib.use('Agg')",'')
    tree=ast.parse(text,filename)
    # Source-only imports intentionally removed; no historical plotting, files,
    # paths, network, shell operations, output commands or later code run here.
    tree.body=[n for n in tree.body if not isinstance(n,(ast.Import,ast.ImportFrom))]
    exec(compile(tree,filename,'exec'),env)
    return env

def initialize_sources():
    env={'np':np,'math':math,'cq':cq,'json':json,'Path':Path}
    w=load_reviewed_geometry('build_tianqiong_w01.py','# reference model interface points',[
        ("ROOT=Path('/mnt/data'); OUT=ROOT/'田穹_W01_母墙数字样机'; OUT.mkdir(parents=True,exist_ok=True)",''),
        ("plt.rcParams.update({'font.sans-serif':['Noto Sans CJK JP','DejaVu Sans'],'axes.unicode_minus':False,'figure.dpi':145})",'')],env.copy())
    m=load_reviewed_geometry('build_tianqiong_m02.py','# Save parametric CAD',[
        ("ROOT=Path('/mnt/data')",''),
        ("OUT=ROOT/'田穹_M02_六机械臂数字样机'; OUT.mkdir(exist_ok=True)",''),
        ("W01=ROOT/'田穹_W01_母墙数字样机'; WPAR=json.loads((W01/'W01_装配接口与参数.json').read_text(encoding='utf8'))",''),
        ("plt.rcParams.update({'font.sans-serif':['Noto Sans CJK JP','DejaVu Sans'],'axes.unicode_minus':False})",'')],dict(env,WPAR={'dimensions_mm':w['P']}))
    p=load_reviewed_geometry('build_p03.py','cad_front(A)',[
        ("ROOT=Path('/mnt/data'); OUT=ROOT/'田穹_P03_前沿锚轨数字样机'; OUT.mkdir(exist_ok=True)",''),
        ("WPAR=json.loads((ROOT/'田穹_W01_母墙数字样机/W01_装配接口与参数.json').read_text(encoding='utf8'))",'')],dict(env,WPAR={'dimensions_mm':w['P']}))
    return w,m,p

W,M,R=initialize_sources()
Y0=np.array(W['P']['slide_centers_initial']);Y1=np.array(W['P']['slide_centers_reconfigured'])
A0=M['A0']; LF=M['LF']*1000

def color_tuple(c): return list(c.toTuple())

def mesh_data(shape):
    v,f=shape.tessellate(1.2,.22)
    return {'vertices':[[x.x,x.y,x.z] for x in v], 'faces':[list(t) for t in f]}

def register(key, shape, color):
    if key not in CATALOG:
        CATALOG[key]=dict(mesh_data(shape),color=color)
        SHAPES[key]=shape
    return key

def mesh_record(name,group,key,position=(0,0,0)):
    return dict(name=name,group=group,kind='mesh',mesh=key,c=list(position),q=[0,0,0,1],size=[1,1,1])

def box(name,group,c,size):return dict(name=name,group=group,kind='box',c=list(c),size=list(size))
def cylinder(name,group,a,b,r):return dict(name=name,group=group,kind='cylinder',p=list(a),q=list(b),r=float(r))
def hemisphere(name,c,r):return dict(name=name,group='joint',kind='hemisphere',c=list(c),r=r)

for part in W['PARTS']:
    register('W01_'+part['name'],part['solid'],[v/255 for v in W['rgb'](part['color'])]+[1])
original_rail=cq.Assembly(name='P03')
R['cad_front'](original_rail)
for node in original_rail.traverse():
    name,part=node
    if isinstance(part.obj,cq.Shape):register(name,part.obj,color_tuple(part.color))

def arm_points(y,angle_delta=0,ell=1600):
    return M['fk'](y/1000,A0+np.deg2rad(angle_delta),ell/1000)*1000

def original_records(ys,release=0,lower=0,modules=1):
    records=[];arms=[]
    for module in range(modules):
        shift=module*(P['module_length']+P['small_wall_gap']);prefix=f'M{module+1}_'
        for part in W['PARTS']:
            dy=0
            if part['group'].startswith('carriage_'):
                i=int(part['group'][-2:])-1;dy=ys[i]-Y0[i]
            records.append(mesh_record(prefix+'W01_'+part['name'],'wall','W01_'+part['name'],[0,shift+dy,0]))
        for key in list(CATALOG):
            if not key.startswith('P03_'):continue
            dy=0
            if '_slider' in key:
                i=int(key.split('_slider')[1][:2])-1;dy=ys[i]-Y0[i]
            records.append(mesh_record(prefix+key,'rail',key,[0,shift+dy,0]))
        for i,y in enumerate(ys):
            active=i in (1,4)
            delta=P['released_root_pitch_delta_deg']*lower if active else 0
            ell=1600-600*release if active else 1600
            pts=arm_points(y+shift,delta,ell)
            ident=prefix+f'A{i+1:02d}'
            for j in range(3):records.append(cylinder(ident+f'_link{j+1}','arm',pts[j],pts[j+1],49))
            vec=(pts[4]-pts[3])/ell
            records.append(cylinder(ident+'_tel_outer','telescope',pts[3],pts[3]+980*vec,62))
            records.append(cylinder(ident+'_tel_inner','arm',pts[3]+780*vec,pts[4],38))
            for j in range(4):records.append(hemisphere(ident+f'_joint{j+1}',pts[j],73))
            records.append(hemisphere(ident+'_tip',pts[4],67))
            arms.append(dict(id=ident,module=module+1,i=i+1,points=pts.tolist(),y=float(y+shift),ell=float(ell),delta_deg=delta))
    if modules==2:
        # Exact BR750 source geometry, repositioned to y=3500..4250.
        b=cq.Assembly(name='BR750');cx=4620
        R['cbox'](b,'bridge_web',(cx,375,49),(120,750,34),'beam')
        for i,x in enumerate([cx-48,cx+48]):
            R['cbox'](b,f'bridge_side_{i}',(x,375,96),(18,750,72),'guide')
            R['cbox'](b,f'bridge_retainer_{i}',(x+(11 if i==0 else -11),375,140),(38,750,13),'guide')
        R['cbox'](b,'bridge_soil_seal',(4700,375,12),(30,750,18),'seal')
        R['cbox'](b,'bridge_front_membrane_clamp',(4485,375,24),(24,750,21),'seal')
        for y in [34,716]:R['cbox'](b,f'bridge_dock_{y}',(cx,y,96),(135,64,76),'lock')
        for name,n in b.traverse():
            if isinstance(n.obj,cq.Shape):
                key=register('BR750_'+name,n.obj,color_tuple(n.color))
                records.append(mesh_record('BR750_'+name,'rail',key,[0,3500,0]))
    return records,arms

def offset_polyline(pts,amount):
    pts=np.array(pts,float);d=np.diff(pts,axis=0);d/=np.linalg.norm(d,axis=1)[:,None]
    n=np.c_[-d[:,1],d[:,0]];out=[]
    for i,p in enumerate(pts):
        v=n[0] if i==0 else n[-1] if i==len(pts)-1 else (n[i-1]+n[i])/(1+np.dot(n[i-1],n[i]))
        out.append(p+amount*v)
    return np.array(out)

def sheet_shape(profile,y0,width,thickness):
    a=offset_polyline(profile,thickness/2);b=offset_polyline(profile,-thickness/2)
    poly=np.vstack([a,b[::-1]])
    # XZ plane has -Y normal: negative extrusion keeps original Y-positive axis.
    return cq.Workplane('XZ',origin=(0,y0,0)).polyline(poly.tolist()).close().extrude(-width).val()

def sheet_record(name,group,profile,y0,width,thickness):
    return dict(name=name,group=group,kind='sheet',profile=np.array(profile).tolist(),y0=y0,width=width,thickness=thickness)

def path_length(p):return float(np.linalg.norm(np.diff(p,axis=0),axis=1).sum())

def spiral_exact(start,center,length,radius,pitch):
    """Piecewise spiral with exact polyline arclength; no elastic stretching."""
    start=np.array(start,float);center=np.array(center,float)
    theta0=math.atan2(*(start-center)[::-1]);points=[start];s=0;theta=0
    while s<length-1e-8:
        theta+=.045;r=radius-pitch*theta/(2*math.pi)
        if r<10:raise ValueError('coil core exhausted')
        p=center+r*np.array([math.cos(theta0+theta),math.sin(theta0+theta)])
        d=np.linalg.norm(p-points[-1])
        if s+d>length:p=points[-1]+(p-points[-1])*(length-s)/d;d=length-s
        points.append(p);s+=d
    return np.array(points)

def deployed_profile():
    js=arm_points(0)[:,[0,2]];js[:,1]+=P['roof_vertical_standoff']
    start=[[-150,2800],[-150,js[0,1]]]
    profile=np.vstack([start,js,[P['front_guide_x'],280],[P['front_guide_x'],P['front_guide_z']]])
    surplus=P['membrane_stock_length']-path_length(profile)
    center=[P['front_guide_x']+P['coil_start_radius'],P['front_guide_z']]
    coil=spiral_exact(profile[-1],center,surplus,P['coil_start_radius'],P['coil_pitch'])
    return np.vstack([profile,coil[1:]]),js,surplus,center

def band_segment(profile,index,halfwidth):
    p=profile[index];a=profile[index-1]-p;b=profile[index+1]-p
    return np.array([p+a/np.linalg.norm(a)*halfwidth,p,p+b/np.linalg.norm(b)*halfwidth])

def build_state(compensate=0,gate=0,drop=0,retract=0,lower=0,modules=1,storage=False):
    ys=Y0+(Y1-Y0)*compensate
    records,arms=original_records(ys,retract,lower,modules)
    width=modules*3500+(modules-1)*750+2*P['side_margin'];y0=-P['side_margin']
    profile,roof,surplus,center=deployed_profile()
    interfaces=[]
    if storage:
        # Endpoint study only: neither arm folding nor transfer through W01 is solved.
        records=[r for r in records if r['group']=='wall']
        arms=[]
        for key in list(CATALOG):
            if key.startswith('P03_'):
                records.append(mesh_record('STORAGE_'+key,'rail',key,[-5370,0,1142]))
        center=P['storage_coil_center_xz'];rad=P['storage_coil_outer_radius']
        profile=spiral_exact([center[0]-rad,center[1]],center,P['membrane_stock_length'],rad,P['coil_pitch'])
        surplus=P['membrane_stock_length']
        records.append(sheet_record('F04_CONTINUOUS_SINGLE_SHEET','film',profile,y0,width,P['membrane_thickness']))
        # Bands and bead/web are not wound into this storage model: explicit incomplete study.
        records.append(cylinder('F04_storage_core','roll',[*center[:1],y0,center[1]],[center[0],y0+width,center[1]],90))
    else:
        records.append(sheet_record('F04_CONTINUOUS_SINGLE_SHEET','film',profile,y0,width,P['membrane_thickness']))
        for k in range(1,4):
            seg=band_segment(roof,k,P['band_width']/2)
            seg=offset_polyline(seg,P['membrane_thickness']/2+P['band_thickness']/2)
            records.append(sheet_record(f'F04_band{k}','band',seg,y0,width,P['band_thickness']))
            x,z=roof[k];bz=z-P['bead_drop']
            records.append(cylinder(f'F04_bead{k}','band',[x,y0,bz],[x,y0+width,bz],P['bead_radius']))
            records.append(box(f'F04_web{k}','band',[x,y0+width/2,(bz+6+z-.5)/2],[P['web_thickness'],width,z-.5-bz-6]))
        for arm in arms:
            active=arm['i'] in (2,5);g=gate if active else 0;d=drop if active else 0
            for k in range(1,4):
                pt=np.array(arm['points'][k]);x,y,z=pt
                bz=z+P['roof_vertical_standoff']-P['bead_drop']-P['carrier_release_drop']*d
                ident=f"F04_{arm['id']}_J{k}"
                # Carrier U-channel: Y sliding is distinct from two X-opening retention lips.
                records.append(box(ident+'_stem','head',[x,y,(z+65+bz-15)/2],[18,26,bz-15-z-65]))
                records.append(box(ident+'_base','head',[x,y,bz-12.5],[28,50,5]))
                for sign,suffix in [(-1,'L'),(1,'R')]:
                    records.append(box(ident+'_side'+suffix,'head',[x+10.5*sign,y,bz-1],[5,50,18]))
                    records.append(box(ident+'_gate'+suffix,'gate',[x+sign*(7.5+P['gate_half_stroke']*g),y,bz+9.5],[11,50,3]))
                records.append(box(ident+'_magnet_aux','magnet',[x,y,bz-17],[12,16,3]))
                interfaces.append(dict(id=ident,arm=arm['id'],joint=k,engaged=not(active and gate>0),gate=g,drop=d,
                    carrier_center=[x,y,bz],bead_datum=[float(roof[k,0]),y,float(roof[k,1]-P['bead_drop'])]))
        records.append(cylinder('F04_front_roll_core','roll',[center[0],y0-55,center[1]],[center[0],y0+width+55,center[1]],48))
        # Bearings OUTSIDE the continuous sheet width, not blocks penetrating the coil.
        # Long shaft deflection and cantilever attachment loads are NOT solved.
        for end,y in enumerate([y0-32,y0+width+32]):
            outer=cq.Solid.makeCylinder(58,30,cq.Vector(center[0],y-15,center[1]),cq.Vector(0,1,0))
            bore=cq.Solid.makeCylinder(49,30,cq.Vector(center[0],y-15,center[1]),cq.Vector(0,1,0))
            key=register(f'F04_bearing_{width}_{end}',outer.cut(bore),COLORS['proposal'])
            records.append(mesh_record(f'F04_roll_bearing_{end}','proposal',key))
            records.append(box(f'F04_roll_pedestal_{end}','proposal',[center[0],y,65],[36,30,66]))
            records.append(box(f'F04_roll_mount_{end}','proposal',[4740,y,45],[240,30,16]))
            anchor_y=150 if end==0 else width+y0-150
            records.append(box(f'F04_roll_mount_Y_{end}','proposal',[4620,(y+anchor_y)/2,45],[32,abs(y-anchor_y),16]))
        records.append(box('F04_W01_root_flashing_clamp','head',[-150,width/2+y0,2804],[40,width,8]))
    meta=dict(arms=arms,interfaces=interfaces,modules=modules,surplus=surplus,stock_length=path_length(profile),storage=storage,
       membrane_sheets=1,bands=0 if storage else 3,profile=profile.tolist(),
       note='Storage target ONLY; arms omitted; bead and reinforcement winding NOT solved' if storage else 'Rigid geometric envelope, not flexible-film or safety simulation')
    return records,meta

KEYS={
 '01_deployed':({},'部署基准 · 18 个承托接口'),
 '02_compensated':(dict(compensate=1),'根座 / 前滑块同步补位 · 18 接口'),
 '03_gates_open':(dict(compensate=1,gate=1,drop=1),'2/5 号：机械门打开、承托头下降'),
 '04_release_candidate':(dict(compensate=1,gate=1,drop=1,retract=1,lower=1),'2/5 号脱离候选 · 12 接口承托'),
 '05_two_modules_BR750':(dict(modules=2),'双母墙 + 750 mm 过渡 · 一整张连续膜'),
 '06_storage_TARGET_ONLY':(dict(storage=True),'收纳目标：臂折叠、整条嵌条卷绕及转运路径未解')}

def smooth(t):t=np.clip(t,0,1);return float(6*t**5-15*t**4+10*t**3)
def timeline():
    result=[]
    for t in np.linspace(0,18,73):
        state=dict(compensate=smooth(t/6),gate=smooth((t-6)/2),drop=smooth((t-8)/2),retract=smooth((t-10)/4),lower=smooth((t-14)/4))
        label='同步滑移补位' if t<=6 else '2/5 号打开机械防脱门' if t<=8 else '承托头下降退出嵌条' if t<=10 else '第 4 段缩回，腕部解耦候选' if t<=14 else '根俯仰下降候选（非安全认证）'
        result.append((float(t),label,state))
    return result

def shape_for(r):
    if r['kind']=='mesh':return SHAPES[r['mesh']].translate(tuple(r['c']))
    if r['kind']=='box':return cq.Workplane('XY').box(*r['size']).val().translate(tuple(r['c']))
    if r['kind']=='hemisphere':return cq.Solid.makeSphere(r['r'],cq.Vector(*r['c']))
    if r['kind']=='cylinder':
        a=cq.Vector(*r['p']);d=cq.Vector(*r['q'])-a
        return cq.Solid.makeCylinder(r['r'],d.Length,a,d.normalized())
    return sheet_shape(r['profile'],r['y0'],r['width'],r['thickness'])

def color_for(r):return CATALOG[r['mesh']]['color'] if r['kind']=='mesh' else COLORS[r['group']]

def assembly(records,name):
    a=cq.Assembly(name=name);groups={}
    for r in records:
        group=r['group'];groups.setdefault(group,cq.Assembly(name=group))
        groups[group].add(shape_for(r),name=r['name'],color=cq.Color(*color_for(r)))
    for group,b in groups.items():a.add(b,name=group)
    return a

def export_step(records,name):
    a=assembly(records,name);a.save(str(OUT/(name+'.step')),exportType='STEP')

def unit_mesh(r):
    if r['kind']=='mesh':return CATALOG[r['mesh']],r['mesh']
    if r['kind']=='sheet':
        key=r['name']+'_'+hashlib.sha256(json.dumps([r['profile'],r['width'],r['y0'],r['thickness']]).encode()).hexdigest()[:12]
        if key not in CATALOG:CATALOG[key]=dict(mesh_data(shape_for(r)),color=color_for(r))
        return CATALOG[key],key
    key='primitive_'+r['kind']
    if key not in CATALOG:
        if r['kind']=='box':s=cq.Workplane('XY').box(1,1,1).val()
        elif r['kind']=='hemisphere':s=cq.Solid.makeSphere(1)
        else:s=cq.Solid.makeCylinder(1,1,cq.Vector(0,0,-.5))
        register(key,s,[1,1,1,1])
    return CATALOG[key],key

def transform(r):
    if r['kind']=='mesh':return r['c'],[0,0,0,1],[1,1,1]
    if r['kind']=='sheet':return [0,0,0],[0,0,0,1],[1,1,1]
    if r['kind']=='box':return r['c'],[0,0,0,1],r['size']
    if r['kind']=='hemisphere':return r['c'],[0,0,0,1],[r['r']]*3
    a=np.array(r['p']);b=np.array(r['q']);d=b-a;length=np.linalg.norm(d)
    rot=trimesh.geometry.align_vectors([0,0,1],d/length)[:3,:3]
    return ((a+b)/2).tolist(),Rotation.from_matrix(rot).as_quat().tolist(),[r['r'],r['r'],float(length)]

def export_glb(frames,name):
    """Same mesh catalog as STEP tessellation; root converts CAD mm/Z-up to glTF m/Y-up."""
    binary=bytearray()
    doc={'asset':{'version':'2.0','generator':'TIANQIONG F04 R02 source-aligned B-rep exporter'},'scene':0,
         'scenes':[{'nodes':[0]}],'nodes':[{'name':'CAD_mm_Z_up_to_gltf_m_Y_up','rotation':[-math.sqrt(.5),0,0,math.sqrt(.5)],'scale':[.001]*3,'children':[]}],
         'meshes':[],'materials':[],'accessors':[],'bufferViews':[],'buffers':[]}
    def acc(data,kind,ctype=5126,bounds=False):
        arr=np.array(data,np.float32 if ctype==5126 else np.uint32)
        while len(binary)%4:binary.append(0)
        start=len(binary);binary.extend(arr.tobytes());vi=len(doc['bufferViews'])
        doc['bufferViews'].append({'buffer':0,'byteOffset':start,'byteLength':arr.nbytes})
        a={'bufferView':vi,'componentType':ctype,'count':len(arr),'type':kind}
        if bounds:
            x=arr.reshape(len(arr),-1);a.update(min=x.min(axis=0).tolist(),max=x.max(axis=0).tolist())
        doc['accessors'].append(a);return len(doc['accessors'])-1
    meshcache={};matcache={}
    def mesh(r):
        data,key=unit_mesh(r);col=color_for(r);cachekey=(key,tuple(col))
        if cachekey in meshcache:return meshcache[cachekey]
        ck=tuple(col)
        if ck not in matcache:
            matcache[ck]=len(doc['materials']);doc['materials'].append({'name':r['group'],'doubleSided':True,'alphaMode':'BLEND' if col[3]<1 else 'OPAQUE','pbrMetallicRoughness':{'baseColorFactor':col,'metallicFactor':.15,'roughnessFactor':.55}})
        tm=trimesh.Trimesh(vertices=data['vertices'],faces=data['faces'],process=False)
        mi=len(doc['meshes']);doc['meshes'].append({'name':key,'primitives':[{'attributes':{'POSITION':acc(tm.vertices,'VEC3',bounds=True),'NORMAL':acc(tm.vertex_normals,'VEC3')},'indices':acc(tm.faces.reshape(-1),'SCALAR',5125),'material':matcache[ck]}]});meshcache[cachekey]=mi;return mi
    animated=len(frames)>1
    if animated:
        doc['animations']=[{'name':'Sync_Open_Drop_Retract_Lower_CANDIDATE','samplers':[],'channels':[]}]
        times=acc([f[0] for f in frames],'SCALAR',bounds=True)
    lookups=[{r['name']:r for r in records} for _,records in frames]
    for r in frames[0][1]:
        name_=r['name'];ni=len(doc['nodes']);trs=transform(r)
        doc['nodes'].append({'name':name_,'mesh':mesh(r),'translation':trs[0],'rotation':trs[1],'scale':trs[2]});doc['nodes'][0]['children'].append(ni)
        if animated:
            values=[transform(l[name_]) for l in lookups]
            for j,(path,kind) in enumerate([('translation','VEC3'),('rotation','VEC4'),('scale','VEC3')]):
                vals=np.array([v[j] for v in values])
                if path=='rotation':
                    for k in range(1,len(vals)):
                        if np.dot(vals[k-1],vals[k])<0:vals[k]*=-1
                if np.max(np.abs(vals-vals[0]))>1e-8:
                    an=doc['animations'][0];si=len(an['samplers']);an['samplers'].append({'input':times,'output':acc(vals,kind),'interpolation':'LINEAR'});an['channels'].append({'sampler':si,'target':{'node':ni,'path':path}})
    doc['buffers']=[{'byteLength':len(binary)}]
    raw=json.dumps(doc,separators=(',',':')).encode();raw+=b' '*((-len(raw))%4);binary+=b'\0'*((-len(binary))%4)
    blob=struct.pack('<4sII',b'glTF',2,28+len(raw)+len(binary))+struct.pack('<I4s',len(raw),b'JSON')+raw+struct.pack('<I4s',len(binary),b'BIN\0')+binary
    (OUT/(name+'.glb')).write_bytes(blob)

def main():
    keys={};shapes_report={}
    for key,(args,label) in KEYS.items():
        records,meta=build_state(**args);print('Building',key,len(records),flush=True)
        export_step(records,key);export_glb([(0,records)],key)
        shapes=[shape_for(r) for r in records]
        film=shapes[next(i for i,r in enumerate(records) if r['group']=='film')]
        shapes_report[key]={'components':len(records),'invalid_components':[r['name'] for r,s in zip(records,shapes) if not s.isValid()],
            'membrane_solids':len(film.Solids()),'membrane_volume_mm3':film.Volume(),'membrane_arclength_mm':meta['stock_length'],
            'engaged_interfaces':sum(x['engaged'] for x in meta['interfaces']),'total_interfaces':len(meta['interfaces'])}
        dump(key+'_assembly.json',{'label':label,'parameters':args,'metadata':meta,'components':records})
        keys[key]={'label':label,'records':records,'meta':meta}
    baseline,_=original_records(Y0)
    export_step(baseline,'00_W01_M02_P03_SOURCE_REBUILD');export_glb([(0,baseline)],'00_W01_M02_P03_SOURCE_REBUILD')
    export_step([r for r in keys['01_deployed']['records'] if r['name'].startswith('F04_')],'F04_only')
    # Named originals retained separately; independently reusable imported subassemblies.
    for group,name in [('wall','W01_SOURCE_REBUILD'),('arm','M02_SOURCE_REBUILD'),('rail','P03_SOURCE_REBUILD')]:
        groups={'arm','telescope','joint'} if group=='arm' else {group}
        export_step([r for r in baseline if r['group'] in groups],name)
    frames=[];traj=[]
    for t,label,state in timeline():
        rs,meta=build_state(**state);frames.append({'time':t,'label':label,'records':rs,'meta':meta})
        joint_state={}
        for a in meta['arms']:
            angles=A0+np.deg2rad(a['delta_deg']);prefix=f"arm_{a['i']:02d}"
            joint_state[prefix+'_root_Y']=(a['y']-Y0[a['i']-1])/1000
            joint_state[prefix+'_yaw_Z']=0.
            for k in range(4):joint_state[prefix+f'_pitch_{k+1}']=float(-angles[k]+(angles[k-1] if k else 0))
            joint_state[prefix+'_telescope_X']=(a['ell']-1000)/1000
        traj.append({'time':t,'state':state,'arms':meta['arms'],'interfaces':meta['interfaces'],'joint_state_SI':joint_state})
    export_glb([(f['time'],f['records']) for f in frames],'F04_sync_release_CANDIDATE_18s')
    dump('motion_trajectory.json',traj);dump('BRep_validation.json',shapes_report)
    import xml.etree.ElementTree as ET
    source_text=(SOURCE/'build_tianqiong_m02.py').read_text(encoding='utf8')
    urdf_block=source_text.split('# URDF 7DOF each:')[1].split('# FK spot test')[0]
    urdf_block=urdf_block[urdf_block.index("robot=ET.Element"):]
    # This previously reviewed block writes only its generated URDF and initial state.
    ue=dict(M,ET=ET,OUT=OUT)
    exec(compile(urdf_block,'recovered_M02_URDF_block','exec'),ue)
    robot=ue['robot']
    for link in robot.findall('link'):
        for inertial in link.findall('inertial'):link.remove(inertial)
    ET.ElementTree(robot).write(OUT/'M02_W01_六臂_7DOF.urdf',encoding='utf8',xml_declaration=True)
    dump('assembly_relations.json',{
       'format':'Descriptive kinematic relationships, not embedded STEP mates or executable controller',
       'units':{'CAD':'mm','URDF_and_joint_trajectory':'metres and radians','glTF':'metres, Y up'},
       'hierarchy':['W01 fixed wall -> root Y carriage -> yaw Z -> pitch Y 1/2/3/4 -> L4 prismatic X','P03 fixed rail -> Y carriage -> original wrist bracket proxy','F04 single membrane -> 3 bonded bands/webs/beads -> 18 Y-sliding releasable carriers'],
       'arms':{'count_per_large_module':6,'DOF_each':7,'DOF_order':['root_Y','yaw_Z','pitch_1','pitch_2','pitch_3','pitch_4','telescope_X'],'L4_mm':[1000,1600]},
       'support_constraint':'When engaged: root Y = P03 carriage Y = carrier Y. Wrist datum X4500/Z0; no yaw during sync.',
       'F04_carrier':{'count':18,'slide_axis':'Y, along continuous bead','positive_retention':'two X-opening lips, closed center slot 4 mm > web 1.5 mm and < bead diameter 12 mm','release':'both lips open 24 mm each; head lowers 80 mm before arm motion','aux_magnet':'alignment only, no retention credit'},
       'animation_sequence_s':{'0-6':'all root and front Y carriages synchronous quintic shift','6-8':'open six carrier gates on arms 2/5','8-10':'lower six heads 80 mm','10-14':'L4 of 2/5 retracts 1600 -> 1000; original wrist latch unresolved','14-18':'root pitch 2/5 decreases 6 degrees; other relative pitches fixed'},
       'release_preconditions_NOT_IMPLEMENTED':['wind/load safe envelope','remaining 4 ribs load verified','positive gate-open sensing','wrist latch release sensing','head-stroke lock sensing','collision guard and emergency stop'],
       'cross_module':{'large_wall_pitch_mm':4250,'small_wall_gap_mm':750,'continuous_membrane_width_two_modules_mm':7850,'BR750':'no travelling arm on connector','support_rib_gap_over_transition_mm':1450},
       'storage_status':'TARGET ONLY: no arm folding, no bead winding and no rail transfer trajectory. Rail anchors still in deployed source state.',
       'URDF':'Recovered original kinematic chain. Dummy masses/inertias removed; visual link boxes and effort/velocity limits remain placeholders, not dynamics-ready.'})
    # Reuse one catalog per HTML; all viewer primitives match the STEP geometry.
    for data in list(keys.values())+frames:
        for r in data['records']:
            md,key=unit_mesh(r)
            if key not in CATALOG:CATALOG[key]=dict(md,color=color_for(r))
            r['viewerMesh']=key;r['trs']=transform(r);r['color']=color_for(r)
    (OUT/'viewer_data.js').write_text('window.TQ_DATA='+json.dumps({'keys':keys,'frames':frames,'catalog':CATALOG,'colors':COLORS},separators=(',',':'),ensure_ascii=False)+';',encoding='utf8')
    dump('source_alignment.json',{'source':'Recovered historical source text, not original binary STEP/ZIP','axes':P['units'],
        'W01':W['P'],'M02':M['CFG'],'P03':R['P'],'source_hashes':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in SOURCE.glob('*.py')},
        'changes':['F04 add-on membrane + 3 bands + 18 heads','P03 original clamp remains; external F04 coil proposed','Storage target uses original anchor-rod shape and is not transport ready'],
        'preserved':['W01 geometry including wheel bays','M02 exact CAD cylinders and original hemisphere joint proxies','P03 CAD carriages, brackets and 4 anchors','BR750 geometry and module pitch 4250']})
    print('R02 build complete',flush=True)

if __name__=='__main__':main()
