import re, subprocess, sys, os, json
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "phase2"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
FILES = ("basak.py", "run_ours.py")


def setp(src, key, val):
    new, n = re.subn(rf"^{key} = .*$", f"{key} = {val}", src, flags=re.M)
    assert n == 1, key
    return new


def run(fn, over):
    src = base
    for k, v in {**over, "RESULTS_DIR": f'"{RES}"'}.items():
        src = setp(src, k, v)
    open("controllers/params.py", "w").write(src)
    out = subprocess.run([sys.executable, f"controllers/{fn}"], capture_output=True, text=True, check=True, env=env).stdout
    return out.splitlines()[0]


def metrics(tag):
    m = loadmat(os.path.join(RES, tag + ".mat"), squeeze_me=True, simplify_cells=True)
    p = m["params"]
    t, dt, tss = m["t"], float(p["DT"]), 50.0
    eps = float(p["EPS_DEN"] if p["SPEED_LAW"] == "guard" else p["EPS_FIX"])
    ss = t > tss
    D = np.hypot(m["ex"], m["ey"])
    inband = (D > 0.05) & (D < 1.0)
    rate = None
    idx = np.flatnonzero(inband)
    if idx.size:
        start = idx[0]
        stop = start
        while stop + 1 < len(D) and inband[stop + 1]:
            stop += 1
        seg = slice(start, stop + 1)
        if (stop - start + 1) * dt >= 1.0:
            rate = float(-np.polyfit(t[seg], np.log(D[seg]), 1)[0])
    return {
        "mean_ex": float(m["ex"][ss].mean()),
        "rms_ey": float(np.sqrt(np.mean(m["ey"][ss] ** 2))),
        "max_ey": float(np.abs(m["ey"]).max()),
        "final_D": float(D[ss].mean()),
        "t_clipped": float(m["fol_t_clipped"]),
        "reg_active": float(dt * np.count_nonzero(np.abs(m["den"][ss]) < eps)),
        "decay": rate,
        "eps": eps,
    }


configs = []
for e in (0.1, 0.01, 0.001):
    configs.append(("guard", 1.0, e))
for e in (0.1, 0.05, 0.01):
    configs.append(("epsfix", 1.0, e))
for kd in (0.5, 2.0):
    configs.append(("guard", kd, 0.01))
for kd in (0.5, 2.0):
    configs.append(("epsfix", kd, 0.05))

rows = []
try:
    for law, kd, eps in configs:
        epskey = "EPS_DEN" if law == "guard" else "EPS_FIX"
        for fn in FILES:
            tag = run(fn, {"SPEED_LAW": f'"{law}"', "K_D": kd, epskey: eps,
                           "TAG": f'"_p2_{law}_eps{eps:g}_kd{kd:g}"'})
            rows.append({"id": f"V{len(rows) + 1}", "file": fn, "law": law, "kd": kd, "tag": tag,
                         "part": "A" if kd == 1.0 else "B", **metrics(tag)})
finally:
    open("controllers/params.py", "w").write(base)

json.dump(rows, open(os.path.join(OUT, "phase2_rows.json"), "w"), indent=1)


def f(x, fmt):
    return "" if x is None else format(x, fmt)


lines = ["| Run | File | Law | K_D | ε | Mean e_x (m) | RMS e_y (m) | Max \\|e_y\\| (m) | Final D (m) | Time clipped (s) | Reg. active (s) | D decay rate (1/s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    lines.append(f"| {r['id']} | {r['file']} | {'ε-fix' if r['law'] == 'epsfix' else 'guard'} | {r['kd']:g} | {r['eps']:g} | "
                 f"{r['mean_ex']:.2e} | {r['rms_ey']:.2e} | {r['max_ey']:.3f} | {r['final_D']:.2e} | "
                 f"{r['t_clipped']:.2f} | {r['reg_active']:.2f} | {f(r['decay'], '.3f')} |")
note = ("\nMean e_x, RMS e_y, final D and reg. active use t > T_SS = 50 s; max |e_y| and time clipped use the whole run. "
        "Decay rate: −slope of ln D vs t over the first contiguous stretch with 0.05 m < D < 1 m (blank if under 1 s).\n")
open(os.path.join(OUT, "phase2_table.md"), "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))
