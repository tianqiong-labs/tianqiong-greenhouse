import bpy,json
from pathlib import Path
P=Path(__file__).resolve().parent/'output_v4_1'
bpy.ops.wm.open_mainfile(filepath=str(P/'TIANQIONG_V4_1_upper_modules.blend'))
s=bpy.context.scene
films=sorted([o for o in s.objects if o.name.endswith('_roof_film')],key=lambda o:o.name)
reels=sorted([o for o in s.objects if o.name.endswith('_top_reel')],key=lambda o:o.name)
assert len(films)==len(reels)==3
assert not any('End gable' in o.name or 'skirt' in o.name.lower() for o in s.objects)
spans=[]
for f in range(1,362):
 s.frame_set(f); dg=bpy.context.evaluated_depsgraph_get(); row=[]
 for o in films:
  e=o.evaluated_get(dg); me=e.to_mesh(); xs=[v.co.x for v in me.vertices]; row.append(max(xs)-min(xs)); e.to_mesh_clear()
 spans.append(row)
assert all(spans[i+1][j]>=spans[i][j]-1e-5 for i in range(360) for j in range(3))
assert spans[180][0]>4.84 and 1<spans[180][1]<4 and spans[180][2]<.002
assert all(abs(v-4.85)<.001 for v in spans[-1])
for m,o in enumerate(reels):
 assert abs(o.location.x+.26)<1e-5 and abs(o.location.z-2.97)<1e-5
 assert abs(o.location.y-(1.75+4.25*m))<1e-5
# Isolated control test: freeze animation, change only M2's deployment property.
controls=[bpy.data.objects[f'M{m}_DEPLOY_CONTROL'] for m in (1,2,3)]
for o in controls: o.animation_data_clear(); o['deploy']=1.0; o.update_tag()
controls[1]['deploy']=.5; controls[1].update_tag(); bpy.context.view_layer.update()
dg=bpy.context.evaluated_depsgraph_get(); independent=[]
for o in films:
 e=o.evaluated_get(dg); me=e.to_mesh(); xs=[v.co.x for v in me.vertices]; independent.append(max(xs)-min(xs)); e.to_mesh_clear()
assert independent[0]>4.84 and 1<independent[1]<4 and independent[2]>4.84
report={'modules':3,'mother_wall_groups':3,'top_reels':3,'independent_roof_films':3,'frames_checked':361,'end_walls_included':False,'frame181_spans_m':spans[180],'isolated_M2_control_spans_m':independent,'reel_centres_verified':True,'result':'PASS'}
(P/'assembly_animation_check.json').write_text(json.dumps(report,indent=2),encoding='utf-8'); print(report)
