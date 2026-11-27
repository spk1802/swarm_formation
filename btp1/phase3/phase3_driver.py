import re, subprocess, sys, os, json
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "phase3"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
LEVELS = (("uncorrected", "basak.py", "full"), ("pointwise", "run_ours.py", "pointwise"), ("full", "run_ours.py", "full"))


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


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def metrics(tag):
    m = loadmat(os.path.join(RES, tag + ".mat"), squeeze_me=True, simplify_cells=True)
    t = m["t"]
    ss, late = t > 50.0, t > 20.0
    a, b = m["omega"] * 3.0, 5.0 - m["omega"] * 3.0
    delta_pred = np.arctan2(a, b)
    rel = wrap(m["chi_i"] - m["chi_l"])
    return {
        "ts": float(m["fol_ts"]),
        "rms_ey": float(np.sqrt(np.mean(m["ey"][ss] ** 2))),
        "max_ey_late": float(np.abs(m["ey"][late]).max()),
        "mean_ex": float(m["ex"][ss].mean()),
        "effort": float(m["fol_effort"]),
        "kappa_max": float(m["fol_kappa_max"]),
        "max_chi_t_deg": float(m["fol_max_chi_t_deg"]),
        "max_rel_err_rad": float(np.abs(rel - delta_pred)[late].max()),
        "t_clipped": float(m["fol_t_clipped"]),
    }


rows = []
try:
    for level, fn, mode in LEVELS:
        tag = run(fn, {"DELTA_MODE": f'"{mode}"', "TAG": '"_p3"'})
        rows.append({"id": f"H{len(rows) + 1}", "level": level, "file": fn, "tag": tag, **metrics(tag)})
finally:
    open("controllers/params.py", "w").write(base)

json.dump(rows, open(os.path.join(OUT, "phase3_rows.json"), "w"), indent=1)

lines = ["| Run | Level | Settling (s) | RMS e_y (m) | Max \\|e_y\\| t>20 s (m) | Mean e_x (m) | Effort | Max curvature (1/m) | Max \\|χ̃_i\\| (deg) | Max \\|χ_i−χ_l−Δ\\| t>20 s (rad) | Clipped (s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    lines.append(f"| {r['id']} | {r['level']} | {r['ts']:.2f} | {r['rms_ey']:.2e} | {r['max_ey_late']:.2e} | {r['mean_ex']:.2e} | "
                 f"{r['effort']:.1f} | {r['kappa_max']:.3f} | {r['max_chi_t_deg']:.1f} | {r['max_rel_err_rad']:.1e} | {r['t_clipped']:.2f} |")
note = "\nRMS e_y and mean e_x use t > 50 s. Settling: |e_y| stays within 0.2 m. Δ is the value predicted from ω(t).\n"
open(os.path.join(OUT, "phase3_table.md"), "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))
