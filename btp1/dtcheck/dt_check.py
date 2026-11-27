import re, subprocess, sys, os, numpy as np
from scipy.io import loadmat
os.chdir(os.path.dirname(os.path.abspath(__file__)))
base=open("controllers/params_nominal.py").read()
over={"LEADER_FIELD":'"arccos"',"SPEED_LAW":'"epsfix"',"EPS_FIX":"0.05","DELTA_MODE":'"full"',"SMC_LAW":'"sat"',"MU":"np.pi / 4","EPS_SAT":"1.0","RESULTS_DIR":'"res"'}
os.makedirs("res",exist_ok=True)
for dt in (0.01,0.005,0.0025,0.00125):
    src=base
    for k,v in {**over,"DT":str(dt),"TAG":f'"_dt{dt}"'}.items():
        src,n=re.subn(rf"^{k} = .*$",f"{k} = {v}",src,flags=re.M); assert n==1,k
    open("controllers/params.py","w").write(src)
    tag=subprocess.run([sys.executable,"controllers/run_pal_ours.py"],capture_output=True,text=True,check=True).stdout.splitlines()[0]
    m=loadmat(f"res/{tag}.mat",squeeze_me=True); t=m["t"]; ss=t>50
    print(dt, "rms_ey_mm %.4f"%(1e3*np.sqrt(np.mean(m["ey"][ss]**2))), "rms_D_mm %.4f"%(1e3*np.sqrt(np.mean(m["ex"][ss]**2+m["ey"][ss]**2))), "ts %.2f"%m["fol_ts"], "rms_d_mm %.3f"%(1e3*np.sqrt(np.mean(m['d'][ss]**2))))
