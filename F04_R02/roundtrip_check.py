"""Read exported STEP and GLB back independently; compare counts/units/bounds."""
import json
import numpy as np
import cadquery as cq
import trimesh
from model import OUT,KEYS,build_state,shape_for,original_records,Y0,dump

def bbox(shapes):
    b=[s.BoundingBox() for s in shapes]
    return np.array([[min(v.xmin for v in b),min(v.ymin for v in b),min(v.zmin for v in b)],
                     [max(v.xmax for v in b),max(v.ymax for v in b),max(v.zmax for v in b)]])

def main():
    report={}
    for key,(args,label) in KEYS.items():
        records,_=build_state(**args);expected=[shape_for(r) for r in records]
        imported=cq.importers.importStep(str(OUT/(key+'.step'))).val();solids=imported.Solids()
        ev=sum(s.Volume() for s in expected);iv=sum(s.Volume() for s in solids)
        scene=trimesh.load(str(OUT/(key+'.glb')),force='scene')
        # Standard glTF m/Y-up back to CAD mm/Z-up.
        scene.apply_transform([[1000,0,0,0],[0,0,-1000,0],[0,1000,0,0],[0,0,0,1]])
        expected_bounds=bbox(expected);step_bounds=bbox(solids)
        report[key]={'exported_record_count':len(records),'STEP_solid_count':len(solids),'STEP_valid':imported.isValid(),
                     'GLB_mesh_node_count':len(scene.graph.nodes_geometry),'STEP_relative_volume_error':abs(iv-ev)/ev,
                     'STEP_bbox_max_error_mm':float(np.max(np.abs(step_bounds-expected_bounds))),
                     'GLB_bbox_max_error_mm':float(np.max(np.abs(scene.bounds-expected_bounds))),
                     'expected_CAD_bbox_mm':expected_bounds.tolist(),'GLB_uses_standard_metres_Y_up':True}
        print(key,report[key]['STEP_solid_count'],report[key]['GLB_bbox_max_error_mm'],flush=True)
    dump('STEP_GLB_roundtrip_validation.json',report)

if __name__=='__main__':main()
