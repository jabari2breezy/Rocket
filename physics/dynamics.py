"""A readable RK4 vertical-flight model. CG is recalculated at each derivative evaluation."""
from math import pi, sqrt, exp
from physics.atmosphere import isa, gravity
from physics.motor import thrust_at, mass_flow, total_impulse
from design.efficiency import score

NOSE = {"Von Kármán":.09,"Parabolic":.11,"Ogive":.14,"Elliptical":.18,"Conical":.25}

def _aero(design, altitude, velocity, time):
    env=isa(altitude); rel=velocity-design.get("wind",0)*0.12
    mach=abs(rel)/env["sound_speed"]; re=max(1e4,env["density"]*abs(rel)*design["length"]/1.8e-5)
    base=NOSE.get(design["nose"],.14)+.037/re**.2+.018*design["fin_count"]
    # Clamp only the Gaussian exponent; extreme failed flights must not overflow numerically.
    delta=abs((mach-1)/.16)
    transonic=1+2.5*exp(-min(700, delta*delta))
    cd=base*transonic
    area=pi*(design["diameter"]/2)**2
    drag=.5*env["density"]*rel*abs(rel)*cd*area
    return drag,mach,cd,.5*env["density"]*rel*rel

def _margin(design, prop_remaining):
    # Simple Barrowman-style rollup: fin CN moves CP aft; propellant shift changes CG.
    d=design["diameter"]; L=design["length"]; fin=design["fin_span"]
    cp=(.48*L + design["fin_position"]*(fin/d)*.14)/(1+(fin/d)*.14)
    dry=design["dry_mass"]+design.get("ballast",0); cg_dry=.43*L - design.get("ballast",0)/max(dry,.001)*.22*L
    cg=(dry*cg_dry+prop_remaining*.69*L)/(dry+prop_remaining)
    return (cp-cg)/d, cp, cg

def simulate(design):
    curve=design["curve"]; burn=curve[-1][0]; isp=design["isp"]; prop0=design["prop_mass"]
    dry=design["dry_mass"]+design.get("ballast",0); t=0.; h=0.; v=0.; prop=prop0; phase="powered"; data=[]
    max_q=max_mach=0; min_margin=99; max_alt=0; unstable=False
    # RK4 state is (altitude, velocity, remaining propellant); use a small step during burn.
    def deriv(state, now):
      alt,vel,p=state; p=max(0.0,p); thrust=thrust_at(curve,now) if phase=="powered" and p>0 else 0
      drag,mach,cd,q=_aero(design,alt,vel,now); mass=dry+p
      return (vel,(thrust-drag-mass*gravity(alt))/mass,-mass_flow(thrust,isp)), (thrust,mach,cd,q,mass)
    while t < 240:
      dt=.01 if phase=="powered" else .05
      state=(h,v,prop); k1,info=deriv(state,t)
      k2,_=deriv(tuple(state[i]+k1[i]*dt/2 for i in range(3)),t+dt/2)
      k3,_=deriv(tuple(state[i]+k2[i]*dt/2 for i in range(3)),t+dt/2)
      k4,_=deriv(tuple(state[i]+k3[i]*dt for i in range(3)),t+dt)
      h += dt*(k1[0]+2*k2[0]+2*k3[0]+k4[0])/6; v += dt*(k1[1]+2*k2[1]+2*k3[1]+k4[1])/6; prop=max(0,prop+dt*(k1[2]+2*k2[2]+2*k3[2]+k4[2])/6)
      if t >= burn and phase=="powered": phase="coast"
      if phase=="coast" and v <= 0 and h>5: phase="recovery"
      if phase=="recovery":
        env=isa(h); A=pi*(design["chute_diameter"]/2)**2; chute=.5*env["density"]*v*abs(v)*1.15*A
        v += chute/(dry+prop)*dt # drag is upward for descent
      margin,cp,cg=_margin(design,prop); min_margin=min(min_margin,margin); unstable |= margin<1
      max_q=max(max_q,info[3]); max_mach=max(max_mach,info[1]); max_alt=max(max_alt,h)
      if len(data)<1800 and int(t*20)%2==0: data.append([round(t,2),round(max(h,0),1),round(v,1),round(info[1],3),round(info[3],0),round(margin,2),round(info[0],1)])
      t+=dt
      if h<=0 and t>burn+2: break
    impulse=total_impulse(curve); initial=dry+prop0; theoretical=isp*9.80665*__import__('math').log(initial/dry)
    achieved=max(v for _,_,v,*_ in data) if data else 0
    flight={"apogee":max_alt,"max_mach":max_mach,"max_q":max_q,"min_margin":min_margin,"unstable":unstable,"landing_speed":abs(v),"liftoff_tw":thrust_at(curve,0.05)/(initial*9.80665),"drag_loss":max(0,1-achieved/max(theoretical,1)),"impulse":impulse,"burn_time":burn,"theoretical_dv":theoretical}
    return {"series":data,"flight":{k:round(v,2) if isinstance(v,float) else v for k,v in flight.items()},"score":score(flight,design),"cp":round(cp,3),"cg":round(cg,3)}
