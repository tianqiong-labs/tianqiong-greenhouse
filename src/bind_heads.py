"""承托头/闸门/磁件按父关节刚体绑定：T_head(t) = T_joint(t) ∘ T_local（含旋转）。"""
import importlib.util, json
from pathlib import Path
import numpy as np
anim = importlib.import_module('anim'); packlib = importlib.import_module('packlib')
m = anim.m; HERO = anim.HERO
pl = importlib.import_module('planner'); Y_COMP=[float(x) for x in pl.Y_WORK]

def quat_from_R(R):
    t=np.trace(R)
    if t>0:
        s=np.sqrt(t+1.0)*2; w=0.25*s
        return np.array([(R[2,1]-R[1,2])/s,(R[0,2]-R[2,0])/s,(R[1,0]-R[0,1])/s,w])
    i=int(np.argmax(np.diag(R)))
    if i==0:
        s=np.sqrt(1+R[0,0]-R[1,1]-R[2,2])*2
        return np.array([0.25*s,(R[0,1]+R[1,0])/s,(R[0,2]+R[2,0])/s,(R[2,1]-R[1,2])/s])
    if i==1:
        s=np.sqrt(1+R[1,1]-R[0,0]-R[2,2])*2
        return np.array([(R[0,1]+R[1,0])/s,0.25*s,(R[1,2]+R[2,1])/s,(R[0,2]-R[2,0])/s])
    s=np.sqrt(1+R[2,2]-R[0,0]-R[1,1])*2
    return np.array([(R[0,2]+R[2,0])/s,(R[1,2]+R[2,1])/s,0.25*s,(R[1,0]-R[0,1])/s])

def R_from(a,b):
    a=a/np.linalg.norm(a); b=b/np.linalg.norm(b); v=np.cross(a,b); c=float(a@b)
    if np.linalg.norm(v)<1e-9: return np.eye(3) if c>0 else -np.eye(3)
    K=np.array([[0,-v[2],v[1]],[v[2],0,-v[0]],[-v[1],v[0],0]])
    return np.eye(3)+K+K@K*(1/(1+c))

def qmul(q1,q2):
    x1,y1,z1,w1=q1; x2,y2,z2,w2=q2
    return np.array([w1*x2+x1*w2+y1*z2-z1*y2, w1*y2-x1*z2+y1*w2+z1*x2,
                     w1*z2+x1*y2-y1*x2+z1*w2, w1*w2-x1*x2-y1*y2-z1*z2])

def bind(idx, q0, base_parts):
    y_abs=HERO*anim.PITCH+Y_COMP[idx]
    _, pts0 = packlib.hero_arm(f'M{HERO+1}_M1_', idx, y_abs, np.asarray(q0[:4],float), float(q0[4]), m)
    joints={f'J{k+1}':pts0[k] for k in range(4)}
    out={'base_pts':pts0,'joints':joints,'parts':[]}
    for p in base_parts:
        c0=np.asarray(p['c'],float)
        jm=min(joints, key=lambda k: float(np.linalg.norm(joints[k]-c0)))
        out['parts'].append(dict(name=p['name'], j=jm, c0=c0, quat0=list(p['quat']), scale=list(p['scale']),
                                 off=c0-joints[jm]))
    return out

def apply(bindinfo, q, frame_moves):
    y_abs=HERO*anim.PITCH+Y_COMP[dict(zip((1,4),(1,4)))[0]]  # 由调用方覆盖
    return None
