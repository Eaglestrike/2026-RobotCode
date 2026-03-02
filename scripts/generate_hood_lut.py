#!/usr/bin/env python3
"""
Generate distance → RPM / Hood Angle / TOF lookup table (10 points).
For each distance, pick the RPS giving the highest non-clamped hood angle.
"""

import math

G = 9.80665
K_DRAG = 0.024
SHOOTER_HEIGHT = 0.554035
HUB_HEIGHT = 72.0 * 0.0254
REL_Z = HUB_HEIGHT - SHOOTER_HEIGHT

DT = 0.01
T_MAX = 5.0
TOL = 0.05
MAX_ITER = 20
HIGH_ARC = True

HOOD_MIN, HOOD_MAX = 26.0, 75.11
CLAMP_THRESH = 74.5

_A, _B, _C = -0.00276490, 0.38986381, -2.25855714
def bv(rps): return max(_A*rps*rps + _B*rps + _C, 0.0)

class V:
    __slots__ = ('x','y','z')
    def __init__(s,x=0.,y=0.,z=0.): s.x,s.y,s.z=x,y,z
    def __add__(s,o): return V(s.x+o.x,s.y+o.y,s.z+o.z)
    def __sub__(s,o): return V(s.x-o.x,s.y-o.y,s.z-o.z)
    def __mul__(s,k): return V(s.x*k,s.y*k,s.z*k)
    def __rmul__(s,k): return s.__mul__(k)
    def dot(s,o): return s.x*o.x+s.y*o.y+s.z*o.z
    def norm(s): return math.sqrt(s.dot(s))

def acc(v,kd):
    a=V(0,0,-G); sp=v.norm()
    if sp>1e-12 and kd: a=a-v*(kd*sp)
    return a

def rk4(r,v,h,kd):
    d1=acc(v,kd); v2=v+d1*(h*.5); d2=acc(v2,kd)
    v3=v+d2*(h*.5); d3=acc(v3,kd); v4=v+d3*h; d4=acc(v4,kd)
    return r+(v+v2*2+v3*2+v4)*(h/6.), v+(d1+d2*2+d3*2+d4)*(h/6.)

def closest(pv,rp,kd):
    r,v,t=V(),pv,0.
    rel=r-rp; q=rel.dot(v)
    while t<T_MAX:
        rn,vn=rk4(r,v,DT,kd); tc=t+DT
        rc=rn-(rp); qc=rc.dot(vn)
        ok=(not HIGH_ARC) or vn.z<=0
        if ok and q<=0 and qc>=0:
            tau=DT*(-q)/(qc-q+1e-12); tau=max(0.,min(tau,DT))
            rs,vs=(rk4(r,v,tau,kd) if tau>1e-15 else (r,v))
            return rs-rp, t+tau
        r,v,t,q=rn,vn,tc,qc
    return r-rp, t

def vac(R,v0):
    Rxy=math.sqrt(R.x**2+R.y**2); ph=math.atan2(R.y,R.x)
    if Rxy<1e-12: return math.pi/4,ph
    v2=v0*v0; disc=v2*v2-G*(G*Rxy*Rxy+2*R.z*v2)
    if disc<0: return math.atan2(R.z,Rxy),ph
    s=math.sqrt(disc)
    th=math.atan((v2+s)/(G*Rxy) if HIGH_ARC else (v2-s)/(G*Rxy))
    return (th if math.isfinite(th) else math.atan2(R.z,Rxy)), ph

def resid(th,rp,v0,kd):
    d=V(math.cos(th),0,math.sin(th))*v0
    rm,ts=closest(d,rp,kd)
    return rm.norm(), ts

def solve(dist,v0):
    if v0<=0: return None
    rp=V(dist,0,REL_Z)
    th,_=vac(rp,v0); th=max(-math.pi/2,min(math.pi/2,th))
    miss,tof=resid(th,rp,v0,K_DRAG)
    bt,bm,btof=th,miss,tof; step=0.1
    for _ in range(MAX_ITER):
        if miss<=TOL: break
        h=1e-4
        mp,_=resid(th+h,rp,v0,K_DRAG); mm,_=resid(th-h,rp,v0,K_DRAG)
        gr=(mp-mm)/(2*h); dt=-step*gr; a,ok=1.,False
        for _ in range(8):
            nt=max(-math.pi/2,min(math.pi/2,th+a*dt))
            nm,ntof=resid(nt,rp,v0,K_DRAG)
            if nm<miss: th,miss,tof=nt,nm,ntof; ok=True; break
            a*=0.5
        if ok and miss<bm: bt,bm,btof=th,miss,tof
        if not ok: step*=0.5
        if step<1e-8: break
    deg=max(HOOD_MIN,min(HOOD_MAX,math.degrees(bt)))
    return deg, bm, btof

def main():
    dists = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 9.5]
    results = []

    for dist in dists:
        best = None
        for rps in range(20, 76, 1):  # coarse sweep: 20–75 rps, step 1
            v0 = bv(rps)
            if v0 <= 0: continue
            res = solve(dist, v0)
            if res is None: continue
            angle, miss, tof = res
            if miss > 0.15: continue
            if angle >= CLAMP_THRESH: continue
            if best is None or angle > best[1]:
                best = (rps, angle, tof, miss)

        if best:
            rps, angle, tof, miss = best
            results.append((dist, rps*60., angle, tof))
        else:
            print(f"// WARNING: {dist}m - no valid non-clamped solution found")

    print("// Auto-generated hood lookup table (high-arc)")
    print(f"// Hub={HUB_HEIGHT:.4f}m, Shooter={SHOOTER_HEIGHT}m, ΔZ={REL_Z:.4f}m, kDrag={K_DRAG}")
    print()
    for dist, rpm, angle, tof in results:
        print(f"RPM_MAP.put({dist:.1f}, {rpm:.0f});")
    print()
    for dist, rpm, angle, tof in results:
        print(f"HOOD_MAP.put({dist:.1f}, Rotation2d.fromDegrees({angle:.1f}));")
    print()
    for dist, rpm, angle, tof in results:
        print(f"TOF_MAP.put({dist:.1f}, {tof:.3f});")
    print()
    print(f"// {'Dist':>5} | {'RPM':>6} | {'RPS':>5} | {'v0(m/s)':>7} | {'Hood':>6} | {'TOF(s)':>6} | {'miss(m)':>7}")
    for dist, rpm, angle, tof in results:
        rps=rpm/60.; v0=bv(rps)
        r=solve(dist,v0); miss=r[1] if r else 0
        print(f"// {dist:>5.1f} | {rpm:>6.0f} | {rps:>5.1f} | {v0:>7.2f} | {angle:>5.1f}° | {tof:>6.3f} | {miss:>7.3f}")

if __name__=="__main__":
    main()
