import re, subprocess, sys, os, json
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "phase4"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
FIELDS = (("arctan", "run_ours.py"), ("arccos", "run_pal_ours.py"))
K_L = 1.0 / (2 / (3 * np.sqrt(3)) * 20.0)
C_L = 1.0 / (0.300 * 20.0)


def setp(src, key, val):
    new, n = re.subn(rf"^{key} = .*$", f"{key} = {val}", src, flags=re.M)
    assert n == 1, key
    return new


def run(fn, over):
    src = base
    for k, v in {**over, "K_L": repr(K_L), "C_L": repr(C_L), "RESULTS_DIR": f'"{RES}"'}.items():
        src = setp(src, k, v)
    open("controllers/params.py", "w").write(src)
    out = subprocess.run([sys.executable, f"controllers/{fn}"], capture_output=True, text=True, check=True, env=env).stdout
    return out.splitlines()[0]


def metrics(tag):
    m = loadmat(os.path.join(RES, tag + ".mat"), squeeze_me=True, simplify_cells=True)
    p = m["params"]
    t = m["t"]
    ss = t > 50.0
    free = (m["v_i"] > p["V_MIN"] + 1e-9) & (m["v_i"] < p["V_MAX"] - 1e-9)
    kap = np.abs(m["chidot_i"] / m["v_i"])
    return {
        "ts": float(m["fol_ts"]),
        "rms_ey": float(np.sqrt(np.mean(m["ey"][ss] ** 2))),
        "effort": float(m["fol_effort"]),
        "kappa_max_free": float(kap[free].max()),
        "max_chi_t_deg": float(m["fol_max_chi_t_deg"]),
        "t_clipped": float(m["fol_t_clipped"]),
        "leader_ts": float(m["leader_ts"]),
    }


configs = [(f, fn, r, 5.0) for r in (10.0, 20.0, 30.0) for f, fn in FIELDS]
configs += [(f, fn, 20.0, 15.0) for f, fn in FIELDS]

rows = []
try:
    for field, fn, rmin, ey0 in configs:
        tag = run(fn, {"R_MIN": rmin, "EY0": ey0, "TAG": f'"_p4"'})
        rows.append({"id": f"F{len(rows) + 1}", "field": field, "file": fn, "rmin": rmin, "ey0": ey0,
                     "tag": tag, **metrics(tag)})
finally:
    open("controllers/params.py", "w").write(base)

json.dump(rows, open(os.path.join(OUT, "phase4_rows.json"), "w"), indent=1)

lines = ["| Run | Follower field | r_min (m) | e_y0 (m) | Settling (s) | RMS e_y (m) | Effort | Max curvature, unclipped (1/m) | Max \\|χ̃_i\\| (deg) | Clipped (s) | Leader settling (s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    lines.append(f"| {r['id']} | {r['field']} | {r['rmin']:g} | {r['ey0']:g} | {r['ts']:.2f} | {r['rms_ey']:.2e} | {r['effort']:.1f} | "
                 f"{r['kappa_max_free']:.3f} | {r['max_chi_t_deg']:.1f} | {r['t_clipped']:.2f} | {r['leader_ts']:.2f} |")
note = ("\nLeader fixed in every run: arctan field with the r_min = 20 m gain. Only the follower field gain follows r_min. "
        "RMS e_y uses t > 50 s. Settling: |e_y| stays within 0.2 m. Max curvature excludes samples at a speed limit.\n")
open(os.path.join(OUT, "phase4_table.md"), "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))
