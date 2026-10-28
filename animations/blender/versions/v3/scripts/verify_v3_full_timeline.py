import bpy,importlib.util,json
from pathlib import Path

base=Path(__file__).resolve().parents[1]
builder_path=Path(__file__).with_name('rebuild_checked_v3.py')
if not builder_path.is_file():builder_path=base/'work'/'rebuild_checked_v3.py'
spec=importlib.util.spec_from_file_location('checked_builder',str(builder_path))
builder=importlib.util.module_from_spec(spec);spec.loader.exec_module(builder)
nearby_report=Path(bpy.data.filepath).with_name('田穹_F04_R02_V3_核查记录.json')
if nearby_report.is_file():builder.REPORT=nearby_report
scene=bpy.context.scene
records={(int(o['module']),o['source_node']):o for o in scene.objects if 'source_node' in o}
maximum=0.0;telmax=0.0;count=0
for frame in range(1,817):
    scene.frame_set(frame)
    bpy.context.view_layer.update()
    t=(frame-1)/24
    for m in (1,2,3):
        for i in range(1,7):
            rest=[builder.Vector(p)/1000 for p in builder.ARMS[i-1]['points']]
            objs=[records[(m,f'M1_A{i:02d}_{s}')] for s in ('link1','link2','link3','tel_outer')]
            for j in range(3):
                delta=((objs[j].matrix_world@rest[j+1])-(objs[j+1].matrix_world@rest[j+1])).length
                maximum=max(maximum,delta);count+=1
                assert delta<.00002,(frame,m,i,j,delta)
            inner=records[(m,f'M1_A{i:02d}_tel_inner')]
            tip=records[(m,f'M1_A{i:02d}_tip')]
            center=tip.matrix_world@rest[4]
            endpoint=inner.matrix_world@builder.Vector((max(v[0] for v in inner.bound_box),0,0))
            delta=(endpoint-center).length
            telmax=max(telmax,delta)
            assert delta<.00002,(frame,m,i,delta)
report=json.loads(builder.REPORT.read_text(encoding='utf-8'))
report['full_timeline_frames_checked']=816
report['full_timeline_chain_checks']=count
report['full_timeline_chain_max_error_m']=maximum
report['full_timeline_telescope_tip_max_error_m']=telmax
builder.REPORT.write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
print('FULL_TIMELINE_AUDIT '+json.dumps({k:v for k,v in report.items() if k.startswith('full_')}))
