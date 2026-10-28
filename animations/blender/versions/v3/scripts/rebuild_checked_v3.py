"""Native Z-up source-CAD reconstruction, with independently checked poses."""
import bpy, json, math, sys
from pathlib import Path
from mathutils import Matrix, Vector

BASE=Path(__file__).resolve().parents[1]
SRC=Path(r'C:\Users\12753\Documents\Codex\2026-10-23\referenced-chatgpt-conversation-this-is-an\outputs\TIANQIONG_F04_R02')
if (BASE/'source_cad'/'01_deployed.glb').is_file(): SRC=BASE/'source_cad'
OUT=BASE/'outputs'
WORK=BASE/'work'
FRAME_DIR=WORK/'frames_v3_checked'
BLEND=OUT/'田穹_F04_R02_V3_坐标与关节校正.blend'
REPORT=OUT/'田穹_F04_R02_V3_核查记录.json'
META=json.loads((SRC/'01_deployed_assembly.json').read_text(encoding='utf-8'))
ARMS=META['metadata']['arms']
REST=[Vector(p)/1000 for p in ARMS[0]['points']]
ANGLES=[math.atan2((REST[i+1]-REST[i]).z,(REST[i+1]-REST[i]).x) for i in range(4)]
LENGTHS=[1.15,1.38,1.52,1.6]
FPS=24
END=816

def trans(p): return Matrix.Translation(Vector(p))
def unit(a): return Vector((math.cos(a),0,math.sin(a)))
def smooth(a,b,t):
    v=max(0,min(1,(t-a)/(b-a)))
    return v*v*v*(10+v*(-15+6*v))
def bbox(obj):
    points=[obj.matrix_world@Vector(v) for v in obj.bound_box]
    return [[min(p[i] for p in points) for i in range(3)], [max(p[i] for p in points) for i in range(3)]]
def curves(action):
    if hasattr(action,'fcurves'): return list(action.fcurves)
    result=[]
    for layer in action.layers:
        for strip in layer.strips:
            for bag in strip.channelbags: result.extend(bag.fcurves)
    return result
def linear_animation():
    for action in bpy.data.actions:
        for fc in curves(action):
            for key in fc.keyframe_points: key.interpolation='LINEAR'

def material(name,color,metal=.25,rough=.4,alpha=1):
    m=bpy.data.materials.new(name)
    m.diffuse_color=(*color,alpha)
    m.use_nodes=True
    p=m.node_tree.nodes.get('Principled BSDF')
    p.inputs['Base Color'].default_value=(*color,1)
    p.inputs['Metallic'].default_value=metal
    p.inputs['Roughness'].default_value=rough
    p.inputs['Alpha'].default_value=alpha
    if alpha<1:
        m.surface_render_method='BLENDED'
        m.use_transparency_overlap=False
    return m

def import_frozen():
    bpy.ops.import_scene.gltf(filepath=str(SRC/'01_deployed.glb'))
    bpy.context.view_layer.update()
    original=[o for o in bpy.context.scene.objects if o.type=='MESH']
    snapshots=[]
    for obj in original:
        snapshots.append((obj,obj.name,obj.matrix_world.copy(),bbox(obj),[obj.matrix_world@v.co for v in obj.data.vertices]))
    frozen={}
    errors=[]
    for obj,name,matrix,bb,vertices in snapshots:
        # Importer has ALREADY restored CAD Z-up and metres. Bake its world
        # transform exactly once, without any manual axis conversion.
        obj.parent=None
        obj.data=obj.data.copy()
        obj.data.transform(matrix)
        obj.matrix_world=Matrix.Identity(4)
        obj.data.update()
        frozen[name]=obj
        errors.extend((a-v.co).length for a,v in zip(vertices,obj.data.vertices))
    for o in list(bpy.context.scene.objects):
        if o.type=='EMPTY': bpy.data.objects.remove(o,do_unlink=True)
    bpy.context.view_layer.update()
    maximum=max(errors)
    assert maximum<2e-5, maximum
    rail=bbox(frozen['M1_P03_base_web'])
    wall=bbox(frozen['M1_W01_F_post_0_0'])
    assert abs((rail[1][1]-rail[0][1])-3.5)<1e-5
    assert abs(rail[0][2]-.032)<1e-5
    assert abs(rail[1][2]-.066)<1e-5
    assert abs((wall[1][2]-wall[0][2])-2.503)<1e-5
    return frozen,{'source_meshes':len(original),'bake_max_vertex_error_m':maximum,'rail_bbox_m':rail,'wall_post_bbox_m':wall}

def classify_and_materials(source):
    mats={
        'wall':material('W01 | pearl graphite',(.30,.34,.34),.32,.42),
        'arm':material('M02 | satin alloy',(.26,.42,.43),.50,.28),
        'telescope':material('M02 | brushed steel',(.46,.55,.54),.60,.23),
        'joint':material('M02 | dark joints',(.07,.15,.16),.32,.3),
        'rail':material('P03 | rail',(.20,.28,.29),.48,.30),
        'head':material('F04 | carriers',(.16,.30,.31),.35,.3),
        'gate':material('F04 | orange gates',(.95,.24,.045),.18,.38),
        'proposal':material('F04 | proposal mounts',(.38,.21,.44),.3,.35),
        'magnet':material('F04 | magnets',(.045,.055,.055),.25,.3),
        'band':material('F04 | reinforcement',(.15,.32,.32),.22,.40),
        'roll':material('F04 | reel',(.42,.47,.47),.45,.28),
        'film':material('F04 | transparent film',(.23,.67,.72),.02,.28,.14),
    }
    for name,obj in source.items():
        old=obj.data.materials[0].name.split('.')[0]
        obj.data.materials.clear()
        obj.data.materials.append(mats.get(old,mats['wall']))
        # Cylinders from CAD already have meaningful vertex normals. Keep them.
        if old in ('arm','telescope','joint','roll'):
            for p in obj.data.polygons: p.use_smooth=True

def make_modules(source):
    records={}
    collection=bpy.data.collections.new('CAD | 3 modules - 18 arms')
    bpy.context.scene.collection.children.link(collection)
    # Immutable source names and frozen mesh data prevent inherited transforms
    # or source-object renames from changing later copies.
    for name,obj in source.items():
        if name.endswith('_tel_inner'):
            i=int(name[4:6])
            start=Vector(ARMS[i-1]['points'][3])/1000+.780*unit(ANGLES[3])
            # Express the actual imported rod in an axial local frame, so
            # ordinary Blender scale animates its length without matrix shear.
            obj.data.transform(Matrix.Rotation(ANGLES[3],4,'Y')@trans(-start))
    items=[(name,obj.data) for name,obj in source.items()]
    for m in (1,2,3):
        for name,data in items:
            suffix=name[3:] if name.startswith('M1_') else name
            label=f'M{m}_{suffix}'
            obj=bpy.data.objects.new(label,data)
            collection.objects.link(obj)
            obj.location=(0,(m-1)*4.25,0)
            obj['source_node']=name
            obj['module']=m
            obj['geometry_origin']='F04 R02 STEP tessellation via original GLB'
            obj.rotation_mode='QUATERNION'
            records[(m,name)]=obj
    for obj in source.values(): bpy.data.objects.remove(obj,do_unlink=True)
    return records

def pose(m,i,t):
    start=4.6+(m-1)*.20+(i-1)*.06
    deploy=smooth(start,10.8+(m-1)*.20+(i-1)*.06,t)
    compact=[math.radians(a) for a in (-88,88,-88,88)]
    aa=[a+(b-a)*deploy for a,b in zip(compact,ANGLES)]
    ell=1.0+.6*deploy
    # F04 R02 source candidate: opening, lowering, retracting, then -6 deg.
    work=m==2 and i in (2,5)
    release=(smooth(20.1,22.1,t)-smooth(26.2,28.6,t)) if work else 0
    lower=(smooth(22.1,24.0,t)-smooth(25.8,28.0,t)) if work else 0
    if work:
        ell-=.6*release
        aa=[a-math.radians(6)*lower for a in aa]
    y=ARMS[i-1]['y']/1000+(m-1)*4.25
    pts=[Vector((0,y,2.8))]
    for j,a in enumerate(aa): pts.append(pts[-1]+unit(a)*(ell if j==3 else LENGTHS[j]))
    return pts,aa,ell

def bind_index(name):
    if '_link' in name: return int(name[-1])-1
    if '_joint' in name: return int(name[-1])-1
    if name.endswith(('_tel_outer','_tel_inner','_tip')): return 3
    return None

def object_matrix(m,name,t):
    lane=Vector((0,(m-1)*4.25,0))
    # Placement only along shelf length, with no wall crossing.
    shifts={1:Vector((0,-1.7,0)),2:Vector((0,0,0)),3:Vector((0,1.7,0))}
    move=shifts[m]*(1-smooth(.5,4.4,t))
    shift=trans(move)
    if name.startswith('M1_A'):
        i=int(name[4:6]); j=bind_index(name)
        points,aa,ell=pose(m,i,t)
        restpts=[Vector(p)/1000 for p in ARMS[i-1]['points']]
        rotation=Matrix.Rotation(ANGLES[j]-aa[j],4,'Y')
        if name.endswith('_tip'):
            result=trans(points[4])@rotation@trans(-restpts[4])
        elif name.endswith('_tel_inner'):
            # Source inner rod starts 780 mm from J4 and ends at the wrist.
            # Its exposed length changes, while J4 and the outer tube stay put.
            newstart=points[3]+.780*unit(aa[3])
            ratio=(ell-.780)/(1.600-.780)
            result=trans(newstart)@Matrix.Rotation(-aa[3],4,'Y')@Matrix.Diagonal((ratio,1,1,1))
        else:
            result=trans(points[j])@rotation@trans(-restpts[j])
        return shift@result
    if name.startswith('F04_M1_A'):
        i=int(name[8:10]); j=int(name.split('_J')[1][0])
        points,aa,ell=pose(m,i,t)
        rest=Vector(ARMS[i-1]['points'][j])/1000
        drop=gate=0
        if m==2 and i in (2,5):
            gate=smooth(18.4,19.4,t)-smooth(28.9,29.5,t)
            drop=.080*(smooth(19.4,20.1,t)-smooth(28.6,28.9,t))
        delta=points[j]-rest+Vector((0,0,-drop))
        if name.endswith('_gateL'): delta.x-=.024*gate
        if name.endswith('_gateR'): delta.x+=.024*gate
        return shift@trans(delta)
    film_names=('F04_CONTINUOUS_SINGLE_SHEET','F04_band','F04_bead','F04_web')
    if any(name.startswith(s) for s in film_names):
        # Concept affine unfolding of the actual CAD sheet and reinforcements.
        # This is disclosed as a mesh deformation, not simulated winding.
        progress=.004+.996*smooth(12+(m-1)*.18,17.4+(m-1)*.18,t)
        center=Vector((4.86,0,.14))
        s=Matrix.Diagonal((progress,1,progress,1))
        return shift@trans(lane)@trans(center)@s@trans(-center)
    return shift@trans(lane)

def animate(records):
    def insert(obj,matrix,frame):
        obj.matrix_world=matrix
        obj.keyframe_insert(data_path='location',frame=frame)
        obj.keyframe_insert(data_path='rotation_quaternion',frame=frame)
        obj.keyframe_insert(data_path='scale',frame=frame)
    for (m,name),obj in records.items():
        previous=None
        lastkey=0
        for f in range(1,END+1):
            matrix=object_matrix(m,name,(f-1)/24)
            changed=previous is None or max(abs(matrix[a][b]-previous[a][b]) for a in range(4) for b in range(4))>1e-8
            if changed:
                if previous is not None and lastkey<f-1: insert(obj,previous,f-1)
                insert(obj,matrix,f)
                lastkey=f
            elif f==END:
                insert(obj,matrix,f)
            previous=matrix
    linear_animation()

def add_scene():
    scene=bpy.context.scene
    scene.world=bpy.data.worlds.new('Neutral studio')
    scene.world.use_nodes=True
    bg=scene.world.node_tree.nodes['Background']
    bg.inputs['Color'].default_value=(.25,.28,.31,1)
    bg.inputs['Strength'].default_value=.55
    ground=material('Studio ground',(.34,.37,.36),.0,.8)
    bpy.ops.mesh.primitive_plane_add(size=200,location=(2,6,-.12))
    bpy.context.object.name='ENV | ground - non CAD'
    bpy.context.object.data.materials.append(ground)
    for name,position,energy,size in (
        ('Key',(3,-1,11),2000,10),
        ('Fill',(9,8,8),2200,9),
        ('Rim',(-4,6,7),1800,10),
    ):
        bpy.ops.object.light_add(type='AREA',location=position)
        light=bpy.context.object; light.name='LIGHT | '+name
        light.data.energy=energy; light.data.shape='DISK';light.data.size=size
        light.rotation_euler=(Vector((2.2,6,1.3))-light.location).to_track_quat('-Z','Y').to_euler()
    bpy.ops.object.light_add(type='SUN',location=(4,4,10))
    bpy.context.object.rotation_euler=(math.radians(22),math.radians(-25),math.radians(15))
    bpy.context.object.data.energy=1.2
    bpy.context.object.data.angle=math.radians(8)
    bpy.ops.object.camera_add(location=(15,-10,10))
    camera=bpy.context.object; camera.name='CAMERA | inspection'
    camera.data.type='ORTHO'; camera.data.ortho_scale=16.2
    camera.rotation_euler=(Vector((2.1,5.8,1.5))-camera.location).to_track_quat('-Z','Y').to_euler()
    scene.camera=camera
    return camera

def production_camera(camera):
    shots=[
        (1,(16,-11,10.5),(2.1,5.8,1.4),18.0),
        (112,(15,-10,10),(2.1,5.8,1.4),16.2),
        (288,(14,-9,9),(2.1,5.8,1.4),15.7),
        (430,(14,-9,9),(2.1,5.8,1.4),15.7),
        (490,(11,1.5,6.3),(2.5,6,1.55),8.4),
        (650,(11,.8,5.9),(2.5,6,1.55),8.4),
        (724,(15,-10,10),(2.1,5.8,1.4),16.2),
        (816,(15,-10,10),(2.1,5.8,1.4),16.2),
    ]
    for frame,position,target,scale in shots:
        camera.location=position
        camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
        camera.data.ortho_scale=scale
        camera.keyframe_insert(data_path='location',frame=frame)
        camera.keyframe_insert(data_path='rotation_euler',frame=frame)
        camera.data.keyframe_insert(data_path='ortho_scale',frame=frame)
    linear_animation()

def work_highlights(records):
    palette={}
    for (m,name),obj in records.items():
        if m!=2 or not name.startswith(('M1_A02_','M1_A05_')): continue
        original=obj.data.materials[0]
        if original.name not in palette:
            mat=original.copy(); mat.name='WORK HIGHLIGHT | '+original.name
            p=mat.node_tree.nodes.get('Principled BSDF')
            base=tuple(p.inputs['Base Color'].default_value)
            for frame,color in ((1,base),(430,base),(465,(.85,.28,.045,1)),(660,(.85,.28,.045,1)),(710,base),(816,base)):
                p.inputs['Base Color'].default_value=color
                p.inputs['Base Color'].keyframe_insert(data_path='default_value',frame=frame)
            palette[original.name]=mat
        obj.material_slots[0].link='OBJECT'
        obj.material_slots[0].material=palette[original.name]

def overlay(camera):
    mat=bpy.data.materials.new('HUD | ink');mat.use_nodes=True
    nt=mat.node_tree;nt.nodes.clear()
    output=nt.nodes.new('ShaderNodeOutputMaterial');em=nt.nodes.new('ShaderNodeEmission')
    em.inputs[0].default_value=(.018,.055,.060,1);nt.links.new(em.outputs[0],output.inputs[0])
    fontpath=Path(r'C:\Windows\Fonts\msyh.ttc')
    font=bpy.data.fonts.load(str(fontpath)) if fontpath.exists() else None
    def txt(name,body,y):
        data=bpy.data.curves.new(name,'FONT');data.body=body;data.size=1
        if font:data.font=font
        obj=bpy.data.objects.new(name,data);bpy.context.scene.collection.objects.link(obj)
        obj.parent=camera;obj.location=(0,0,-2)
        obj.data.materials.append(mat)
        for path,index,expression in [('location',0,'-.46*s'),('location',1,f'{y}*s'),('scale',0,'.014*s'),('scale',1,'.014*s'),('scale',2,'.014*s')]:
            d=obj.driver_add(path,index).driver;d.expression=expression
            var=d.variables.new();var.name='s';var.type='SINGLE_PROP'
            var.targets[0].id_type='CAMERA';var.targets[0].id=camera.data;var.targets[0].data_path='ortho_scale'
        return obj
    txt('HUD | project','田穹  TIANQIONG  /  F04 R02',.244)
    txt('HUD | type','真实 CAD · 三模块 18 臂 · 概念关键帧演示',-.258)

def independent_rest_check(records):
    bpy.context.scene.frame_set(816);bpy.context.view_layer.update()
    final={key:[obj.matrix_world@v.co for v in obj.data.vertices] for key,obj in records.items()}
    before=set(bpy.data.objects)
    bpy.ops.import_scene.gltf(filepath=str(SRC/'01_deployed.glb'))
    bpy.context.view_layer.update()
    raw=[obj for obj in bpy.data.objects if obj not in before and obj.type=='MESH']
    maximum=0;count=0
    # Imported nodes may receive .001 suffixes due to existing M1 objects.
    for obj in raw:
        name=obj.name.rsplit('.',1)[0] if obj.name.rsplit('.',1)[-1].isdigit() else obj.name
        assert (1,name) in final,name
        vertices=[obj.matrix_world@v.co for v in obj.data.vertices]
        for m in (1,2,3):
            lane=Vector((0,(m-1)*4.25,0))
            for old,new in zip(vertices,final[(m,name)]):
                maximum=max(maximum,(new-lane-old).length);count+=1
    for obj in list(bpy.data.objects):
        if obj not in before:bpy.data.objects.remove(obj,do_unlink=True)
    assert maximum<2e-5,maximum
    return {'independent_source_vertex_comparisons':count,'independent_rest_max_vertex_error_m':maximum}

def save_still(name,frame,camera,location,target,scale):
    scene=bpy.context.scene;scene.frame_set(frame)
    camera.location=location
    camera.rotation_euler=(Vector(target)-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.ortho_scale=scale
    scene.render.filepath=str(OUT/name)
    bpy.ops.render.render(write_still=True)

def validate(records):
    bpy.context.scene.frame_set(816)
    bpy.context.view_layer.update()
    checks=[]
    maxjoint=0
    for (m,name),obj in records.items():
        if name.startswith('M1_A') and '_joint' in name:
            i=int(name[4:6]); j=int(name[-1])-1
            expected=Vector(ARMS[i-1]['points'][j])/1000+Vector((0,(m-1)*4.25,0))
            # Frozen data origin is zero; the raw hemisphere lowest center
            # location is preserved in its mesh. Compare the matrix directly.
            delta=max(abs(obj.matrix_world[a][b]-(trans((0,(m-1)*4.25,0)))[a][b]) for a in range(4) for b in range(4))
            maxjoint=max(maxjoint,delta)
    assert maxjoint<1e-5,maxjoint
    for frame in (1,100,200,288,430,520,576,672,816):
        bpy.context.scene.frame_set(frame)
        bpy.context.view_layer.update()
        for m in (1,2,3):
            for i in range(1,7):
                # Verify consecutive link endpoints in world space; this is
                # geometry continuity, not a collision clearance assertion.
                rest=[Vector(p)/1000 for p in ARMS[i-1]['points']]
                objs=[records[(m,f'M1_A{i:02d}_{s}')] for s in ('link1','link2','link3','tel_outer')]
                for j in range(3):
                    a=objs[j].matrix_world@rest[j+1]
                    b=objs[j+1].matrix_world@rest[j+1]
                    distance=(a-b).length
                    assert distance<.002,(frame,m,i,j,distance)
                    checks.append(distance)
    return {'joint_rest_matrix_max_error':maxjoint,'chain_endpoint_max_error_m':max(checks),'chain_checks':len(checks),'checked_frames':[1,100,200,288,430,520,576,672,816]}

def main():
    OUT.mkdir(exist_ok=True);FRAME_DIR.mkdir(parents=True,exist_ok=True)
    bpy.ops.wm.read_factory_settings(use_empty=True)
    scene=bpy.context.scene
    scene.render.engine='BLENDER_EEVEE'
    scene.render.resolution_x=1280;scene.render.resolution_y=720;scene.render.resolution_percentage=100
    scene.render.image_settings.file_format='PNG'
    scene.render.fps=24;scene.frame_start=1;scene.frame_end=END
    scene.eevee.taa_render_samples=64
    scene.view_settings.view_transform='AgX'
    scene.view_settings.look='AgX - Medium High Contrast'
    scene.view_settings.exposure=-.35
    source,audit=import_frozen()
    classify_and_materials(source)
    records=make_modules(source)
    animate(records)
    camera=add_scene()
    audit.update(validate(records))
    audit.update(independent_rest_check(records))
    audit.update({'basis':'Blender GLB importer already restored metres/Z-up; no extra rotation','source':str(SRC/'01_deployed.glb'),'arms':18,'modules':3,'pitch_m':4.25,'animation_class':'concept keyframes; film affine unfolding; no collision or winding simulation'})
    REPORT.write_text(json.dumps(audit,ensure_ascii=False,indent=2),encoding='utf-8')
    print('CHECKED_AUDIT '+json.dumps(audit))
    save_still('田穹_V3_正确CAD总览.png',816,camera,(15,-10,10),(2.1,5.8,1.4),16.2)
    save_still('田穹_V3_中央模块作业核查.png',576,camera,(11,1.5,6.3),(2.5,6.0,1.55),8.4)
    save_still('田穹_V3_侧向坐标核查.png',816,camera,(2.2,-18,1.5),(2.2,6,1.5),6.6)
    save_still('田穹_V3_展开中间态核查.png',200,camera,(15,-10,10),(2.1,5.8,1.4),16.2)
    scene.frame_set(816)
    camera.location=(15,-10,10)
    camera.rotation_euler=(Vector((2.1,5.8,1.4))-camera.location).to_track_quat('-Z','Y').to_euler()
    camera.data.ortho_scale=16.2
    work_highlights(records)
    production_camera(camera)
    overlay(camera)
    scene.frame_set(816)
    scene.render.filepath=str(FRAME_DIR/'frame_')
    scene['source_glb']=str(SRC/'01_deployed.glb')
    scene['coordinate_basis']='native Blender importer Z-up, X into greenhouse, Y along length'
    scene['animation_class']='Concept keyframes; sheet affine unfolding; unverified folding and collisions'
    scene['audit_report']=str(REPORT)
    bpy.ops.wm.save_as_mainfile(filepath=str(BLEND))

if __name__=='__main__': main()
