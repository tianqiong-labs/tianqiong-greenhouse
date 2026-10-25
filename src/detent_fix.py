"""锁销修正：半环开 Ø6.4 止退孔；销沿 X 退回 28 mm。重算并覆写 z76_section.json 的 detent 字段。"""
import cadquery as cq, json
from pathlib import Path
OUT = Path(__file__).resolve().parent/'out'/'cad'
PIN_R, HOLE_R, PIN_Y, PIN_Z = 3.0, 3.2, 28.0, 100.0

def half(sign=1, s=0.0, hole=True):
    ring=(cq.Workplane("XY").workplane(offset=90).circle(32).extrude(20)
          .cut(cq.Workplane("XY").workplane(offset=90).circle(15.5).extrude(20)))
    h=ring.intersect(cq.Workplane("XY").workplane(offset=90).rect(200,40).extrude(20).translate((0,sign*20,0)))
    if hole:
        h=h.cut(cq.Workplane("YZ").workplane(offset=-30).circle(HOLE_R).extrude(22).translate((0,PIN_Y,PIN_Z)))
    return h.translate((0,sign*s,0))

def pin(dx=0.0):
    return cq.Workplane("YZ").workplane(offset=-27).circle(PIN_R).extrude(16).translate((dx,PIN_Y,PIN_Z))

def v(a,b):
    try: return round(a.val().intersect(b.val()).Volume(),2)
    except: return 0.0

h=half(); h0=half(hole=False)
rows=dict(
  pin_vs_hole_rest=v(h,pin(0.0)),
  hole_removed=v(h0,pin(0.0)),
  push2_pin=v(h.translate((0,2,0)),pin(0.0)),
  push2_retracted=v(h.translate((0,2,0)),pin(-28.0)),
  push20_pin=v(h.translate((0,20,0)),pin(0.0)),
  push20_retracted=v(h.translate((0,20,0)),pin(-28.0)),
)
for k,val in rows.items(): print(f"  {k:22s} {val:8.2f} mm3")
ok = rows['push2_pin']>1 and rows['push2_retracted']<0.1 and rows['push20_retracted']<0.1
print("  结论:", "止退成立且退让充分" if ok else "仍有残余干涉")
p=OUT/'z76_section.json'; d=json.loads(p.read_text(encoding='utf8'))
d['detent']=dict(rows, corrected='销沿X退回28mm；半环开Ø6.4孔', ok=ok)
p.write_text(json.dumps(d,ensure_ascii=False,indent=1),encoding='utf8')
asm=cq.Assembly(name='G2_DETENT_FIX')
asm.add(half(1),name='HALF_R'); asm.add(half(-1),name='HALF_L'); asm.add(pin(0.0),name='PIN_R')
asm.save(str(OUT/'gamma2_detent_fix.step')); print("导出 gamma2_detent_fix.step")
