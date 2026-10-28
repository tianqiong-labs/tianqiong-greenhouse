"""V4 upper enclosure; native V3 CAD in metres/Z-up, static deployed skeleton."""
import bpy, json, math
from pathlib import Path
from mathutils import Vector, Matrix
from mathutils.bvhtree import BVHTree
P=Path(__file__).resolve().parent
OUT=P/'output_v4_1'; OUT.mkdir(exist_ok=True)
META=json.loads((P/'source/01_deployed_assembly.json').read_text(encoding='utf-8'))
REST=[Vector((p[0]/1000,0,p[2]/1000)) for p in META['metadata']['arms'][0]['points']]
bpy.ops.wm.read_factory_settings(use_empty=True)
scene=bpy.context.scene
def mat(name,c,alpha=1,metal=.1):
 m=bpy.data.materials.new(name); m.diffuse_color=(*c,alpha); m.use_nodes=True
 n=m.node_tree.nodes.get('Principled BSDF'); n.inputs['Base Color'].default_value=(*c,1); n.inputs['Alpha'].default_value=alpha; n.inputs['Metallic'].default_value=metal; n.inputs['Roughness'].default_value=.32
 if alpha<1: m.surface_render_method='BLENDED'; m.use_transparency_overlap=False
 return m
white=mat('Pearl white structure',(.72,.79,.78),metal=.35)
green=mat('Green drive and edge locks',(.025,.32,.19),metal=.3)
rubber=mat('Continuous EPDM seals',(.035,.065,.06))
film=mat('Transparent PE film',(.46,.79,.78),.26)
band=mat('Reinforced flexible overlap',(.18,.46,.37),.65)
def mesh(name,verts,faces,material):
 me=bpy.data.meshes.new(name); me.from_pydata(verts,[],faces); me.update()
 ob=bpy.data.objects.new(name,me); scene.collection.objects.link(ob); ob.data.materials.append(material); return ob
def box(name,loc,dim,material):
 bpy.ops.mesh.primitive_cube_add(size=1,location=loc); o=bpy.context.object; o.name=name; o.dimensions=dim; bpy.ops.object.transform_apply(location=False,rotation=False,scale=True); o.data.materials.append(material); return o
def tube(name,pts,r,material):
 cu=bpy.data.curves.new(name,'CURVE'); cu.dimensions='3D'; cu.bevel_depth=r; cu.bevel_resolution=3
 sp=cu.splines.new('POLY'); sp.points.add(len(pts)-1)
 for a,b in zip(sp.points,pts): a.co=(*b,1)
 o=bpy.data.objects.new(name,cu); scene.collection.objects.link(o); cu.materials.append(material); return o
def cylinder(name,loc,r,length,material):
 bpy.ops.mesh.primitive_cylinder_add(vertices=48,radius=r,depth=length,location=loc,rotation=(math.pi/2,0,0)); o=bpy.context.object; o.name=name; o.data.materials.append(material); return o

# Preserve source vertices and coordinates, replacing only the old F04 system.
bpy.ops.import_scene.gltf(filepath=str(P/'source/01_deployed.glb')); bpy.context.view_layer.update()
source=[]; removed=[]
for o in list(scene.objects):
 if o.type!='MESH': continue
 if o.name.startswith('F04'):
  removed.append(o.name); bpy.data.objects.remove(o,do_unlink=True); continue
 mw=o.matrix_world.copy(); o.parent=None; o.data=o.data.copy(); o.data.transform(mw); o.matrix_world=Matrix.Identity(4)
 o.data.materials.clear(); o.data.materials.append(white); source.append(o)
for o in list(scene.objects):
 if o.type=='EMPTY': bpy.data.objects.remove(o,do_unlink=True)
for m in (1,2):
 for o in source:
  c=o.copy(); c.data=o.data; scene.collection.objects.link(c); c.name=o.name.replace('M1_',f'M{m+1}_'); c.location.y=m*4.25

# Rounded outward offset: film sits on new continuous rib shoes, above joints.
offset=.110
normals=[]
for a,b in zip(REST,REST[1:]):
 d=(b-a).normalized(); normals.append(Vector((-d.z,0,d.x)))
path=[Vector((-.05,0,3.02)),REST[0]+offset*normals[0]]
for i in range(1,4):
 a=math.atan2(normals[i-1].x,normals[i-1].z); b=math.atan2(normals[i].x,normals[i].z)
 for j in range(17):
  ang=a+(b-a)*j/16; path.append(REST[i]+offset*Vector((math.sin(ang),0,math.cos(ang))))
end=REST[-1]+offset*normals[-1]
last_tangent=(REST[-1]-REST[-2]).normalized()
end+=last_tangent*((.32-end.z)/last_tangent.z)
path.extend([end,Vector((4.80,0,.32))])
# Front interface rises clear of rail/soil; final vertical skirt closes below it.
dist=[0.0]
for a,b in zip(path,path[1:]): dist.append(dist[-1]+(b-a).length)
L=dist[-1]
def at(s):
 s=max(0,min(L,s))
 for i in range(1,len(path)):
  if s<=dist[i]: return path[i-1].lerp(path[i],(s-dist[i-1])/(dist[i]-dist[i-1]))
 return path[-1].copy()
def normal(s):
 d=(at(min(L,s+.0001))-at(max(0,s-.0001))).normalized()
 return Vector((-d.z,0,d.x))
N=240
full=[at(L*i/N) for i in range(N+1)]
def normal_at(i):
 d=(full[min(N,i+1)]-full[max(0,i-1)]).normalized()
 return Vector((-d.z,0,d.x))
def sheet(name,y0,y1,material,pts=full):
 vs=[(p.x,y,p.z) for p in pts for y in (y0,y1)]
 return mesh(name,vs,[(2*i,2*i+1,2*i+3,2*i+2) for i in range(len(pts)-1)],material)
modules={}
for m in range(3):
 c=bpy.data.collections.new(f'M{m+1} | one mother wall - one module')
 scene.collection.children.link(c); modules[m]=c
for o in list(scene.objects):
 for m in range(3):
  if o.name.startswith(f'M{m+1}_'):
   for c in list(o.users_collection): c.objects.unlink(o)
   modules[m].objects.link(o)
connectors=bpy.data.collections.new('Module connections | roof only')
scene.collection.children.link(connectors)
def move_to(objects,collection,module):
 for o in objects:
  for c in list(o.users_collection): c.objects.unlink(o)
  collection.objects.link(o); o['module_id']=module
def drive(key,control,step):
 d=key.driver_add('value').driver
 d.expression=f'max(0,1-abs(p*30-{step}))'
 v=d.variables.new(); v.name='p'; v.targets[0].id=control; v.targets[0].data_path='["deploy"]'
animated=[]
for m in range(3):
 before=set(scene.objects)
 control=bpy.data.objects.new(f'M{m+1}_DEPLOY_CONTROL',None); scene.collection.objects.link(control)
 control['deploy']=1.0; control.id_properties_ui('deploy').update(min=0,max=1,description='0 = roof film stored; 1 = roof film deployed')
 for f,v in ((1,0.0),(1+120*m,0.0),(121+120*m,1.0),(361,1.0)):
  control['deploy']=v; control.keyframe_insert(data_path='["deploy"]',frame=f)
 lo=m*4.25-.05; hi=m*4.25+3.55
 o=sheet(f'V4_M{m+1}_roof_film',lo,hi,film); o['thickness_m']=.00015
 o.shape_key_add(name='Basis')
 for step in range(31):
  progress=max(.0001,step/30); key=o.shape_key_add(name=f'Deploy_{step:02d}')
  for i in range(N+1):
   q=at(L*progress*i/N)
   for j,y in enumerate((lo,hi)): key.data[2*i+j].co=(q.x,y,q.z)
  key.value=0; drive(key,control,step)
 # Guide shoes sit directly under film; the original arm remains articulated.
 for arm in META['metadata']['arms']:
  y=arm['y']/1000+m*4.25
  tube(f'M{m+1}_rib_shoe_{arm["i"]}',[(p.x-.014*normal_at(i).x,y,p.z-.014*normal_at(i).z) for i,p in enumerate(full)],.014,white)
  for a,b,n in zip(REST,REST[1:],normals):
   mid=(a+b)/2
   tube('Rib saddle standoff',[(mid.x+.045*n.x,y,mid.z+.045*n.z),(mid.x+.096*n.x,y,mid.z+.096*n.z)],.012,green)
 # Side captive tracks with twin contacting lips.
 for y in (lo,hi):
  tube('Edge track base',[(p.x,y,p.z-.025) for p in full],.022,white)
  tube('Edge EPDM lip',[(p.x,y,p.z+.001) for p in full],.003,rubber)
 # Recess the bar ends 20 mm inside the captive edge seals.
 head=cylinder(f'M{m+1}_travelling_pull_bar',(0,(lo+hi)/2,0),.014,hi-lo-.04,green)
 reel=cylinder(f'M{m+1}_top_reel',(-.26,(lo+hi)/2,2.97),.080,hi-lo,green)
 bpy.ops.object.select_all(action='DESELECT')
 bpy.context.view_layer.objects.active=head; head.select_set(True)
 bpy.ops.object.transform_apply(location=True,rotation=True,scale=True)
 base=[v.co.copy() for v in head.data.vertices]; head.shape_key_add(name='Basis')
 for step in range(31):
  q=at(L*step/30)+.028*normal(L*step/30)
  key=head.shape_key_add(name=f'Position_{step:02d}')
  for v,co in zip(key.data,base): v.co=co+Vector((q.x,0,q.z))
  drive(key,control,step)
 for axis in (0,1):
  d=reel.driver_add('scale',axis).driver
  d.expression=f'sqrt(.08**2+.00015*{L}*(1-p)/pi)/.08'
  v=d.variables.new(); v.name='p'; v.targets[0].id=control; v.targets[0].data_path='["deploy"]'
 d=reel.driver_add('rotation_euler',1).driver
 d.expression=f'2*pi*(sqrt(.08**2+.00015*{L}*(1-p)/pi)-.08)/.00015'
 v=d.variables.new(); v.name='p'; v.targets[0].id=control; v.targets[0].data_path='["deploy"]'
 # Longitudinal supports provision the load path; release/load validation follows later.
 for j in range(1,4):
  q=REST[j]+.11*normals[j-1]
  cylinder(f'M{m+1}_roof_load_spreader_{j}',(q.x,(lo+hi)/2,q.z-.032),.014,hi-lo,white)
  for arm in META['metadata']['arms']:
   y=arm['y']/1000+m*4.25
   tube(f'M{m+1}_detachable_saddle_{arm["i"]}_{j}',[(REST[j].x,y,REST[j].z+.073),(q.x,y,q.z-.046)],.009,green)
 # Actual exit web from reel upper tangent to fixed guide.
 sheet('Reel exit web',lo,hi,film,[Vector((-.26,0,3.05)),path[0]])
 cylinder('Exit guide roller',(-.05,(lo+hi)/2,3.005),.015,hi-lo,white)
 box('Reel hood roof',(-.27,(lo+hi)/2,3.12),(.5,hi-lo+.10,.035),white)
 box('Reel hood back',(-.51,(lo+hi)/2,2.99),(.035,hi-lo+.10,.26),white)
 for y in (lo-.03,hi+.03):
  box('Bearing end plate',(-.26,y,2.98),(.43,.035,.25),white)
  box('Wall reel support',(-.32,y,2.85),(.38,.05,.15),green)
 box('Exit upper wiping seal',(-.05,(lo+hi)/2,3.026),(.025,hi-lo,.012),rubber)
 box('Exit lower wiping seal',(-.05,(lo+hi)/2,3.014),(.025,hi-lo,.012),rubber)
 cylinder(f'M{m+1}_front_roof_lock',(path[-1].x,(lo+hi)/2,path[-1].z-.01),.025,hi-lo+.08,green)
 sheet(f'M{m+1}_rear_top_apron',lo,hi,band,[path[0],Vector((-.52,0,3.02)),Vector((-.56,0,2.68))])
 move_to(set(scene.objects)-before,modules[m],m+1)
connection_start=set(scene.objects)
# Connector roof panels bridge the 650 mm uncovered bands, overlap 120 mm each side.
for lo,hi in ((3.43,4.32),(7.68,8.57)):
 sheet('Flexible inter-module bridge',lo,hi,band,[p+Vector((0,0,.004)) for p in full])
 for y in (lo,hi): tube('Bridge compression seam',[(p.x,y,p.z+.006) for p in full],.009,green)
 sheet('Bridge rear top apron',lo,hi,band,[path[0],Vector((-.52,0,3.02)),Vector((-.56,0,2.68))])
move_to(set(scene.objects)-connection_start,connectors,0)

# Independent distance checks against imported fixed hardware mesh triangles.
verts=[]; faces=[]
for o in source:
 base=len(verts); verts.extend([o.matrix_world@v.co for v in o.data.vertices]); faces.extend([tuple(base+i for i in f.vertices) for f in o.data.polygons])
bvh=BVHTree.FromPolygons(verts,faces)
minimum=1e9; worst=None; head_clearance=1e9
for k in range(801):
 q=at(L*k/800)
 for y in [-.05,0,.15,.35,.91,1.47,2.03,2.59,3.15,3.4,3.55]:
  v=Vector((q.x,y,q.z)); hit=bvh.find_nearest(v)
  if hit and hit[3]<minimum: minimum=hit[3]; worst=list(v)
  headhit=bvh.find_nearest(v+.028*normal(L*k/800))
  if headhit: head_clearance=min(head_clearance,headhit[3]-.014)
report={'basis':'V3 native deployed GLB, 3 modules, 18 arms; replaced historical F04 only',
 'roof_path_length_m':L,'film_thickness_m':.00015,'rib_offset_m':offset,
 'sampled_clearance_to_source_hardware_m':minimum,'worst_sample_m':worst,
 'samples':801*11,'leading_bar_radius_m':.014,'leading_bar_normal_offset_m':.028,'sampled_leading_bar_clearance_m':head_clearance,
 'module_bridge_overlap_m':.12,'end_walls_included':False,'modules':3,'reels':3,'independent_films':3,
 'removed_legacy_F04_objects':len(removed),
 'limits':'Static deployed skeleton; sampled source-hardware clearance, not continuous solid CCD. Seals modeled; airtightness needs physical test. Roof only; ends intentionally open. No arm folding/Mode A validation.'}
assert minimum>.015, report
assert head_clearance>0, report
(OUT/'geometry_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')

# Studio and inspection views.
scene.world=bpy.data.worlds.new('Studio'); scene.world.use_nodes=True; scene.world.node_tree.nodes['Background'].inputs[0].default_value=(.38,.44,.46,1); scene.world.node_tree.nodes['Background'].inputs[1].default_value=.65
box('Ground',(2,6,-.18),(200,200,.1),mat('Ground',(.19,.24,.24)))
for loc,power,size in [((1,-3,11),2300,8),((7,7,10),2700,9),((-5,6,7),1800,8)]:
 bpy.ops.object.light_add(type='AREA',location=loc); o=bpy.context.object; o.data.energy=power; o.data.shape='DISK'; o.data.size=size; o.rotation_euler=(Vector((2,6,1.3))-o.location).to_track_quat('-Z','Y').to_euler()
bpy.ops.object.camera_add(); cam=bpy.context.object; scene.camera=cam; cam.data.type='ORTHO'
scene.render.engine='CYCLES'; scene.cycles.samples=24
scene.render.resolution_x=1440; scene.render.resolution_y=900; scene.render.resolution_percentage=100
scene.render.image_settings.file_format='PNG'; scene.render.fps=24; scene.frame_start=1; scene.frame_end=361
def camera(loc,target,scale):
 cam.location=loc; cam.rotation_euler=(Vector(target)-cam.location).to_track_quat('-Z','Y').to_euler(); cam.data.ortho_scale=scale
for a in bpy.data.actions:
 for layer in a.layers:
  for strip in layer.strips:
   for bag in strip.channelbags:
    for fc in bag.fcurves:
     for k in fc.keyframe_points: k.interpolation='LINEAR'
scene.frame_set(361); camera((15,-13,11),(2,5.9,1.4),15.3)
scene['V4_SCOPE']='Upper enclosure revision; stationary deployed V3 skeleton; frames 1-361 = independent sequential module roof deployment; ends omitted'
bpy.ops.wm.save_as_mainfile(filepath=str(OUT/'TIANQIONG_V4_1_upper_modules.blend'))
for name,frame,loc,target,scale in [
 ('01_upper_complete',361,(15,-13,11),(2,5.9,1.4),15.3),
 ('02_independent_modules',181,(15,-13,11),(2,5.9,1.4),15.3),
 ('03_side_fit',361,(2,-18,1.5),(2,6,1.5),6.6),
 ('04_top_reel',61,(4,-4,6),(-.1,1.7,2.9),5.0)]:
 scene.frame_set(frame); camera(loc,target,scale); scene.render.filepath=str(OUT/(name+'.png')); bpy.ops.render.render(write_still=True)
print('V4_REPORT',json.dumps(report))
