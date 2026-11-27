import re, subprocess, sys, os, json
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "final"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}

BASELINE = ("basak.py", {"LEADER_FIELD": '"arctan"', "SPEED_LAW": '"guard"', "EPS_DEN": "0.01", "SMC_LAW": '"sign_pow"'})
CHOSEN = ("run_pal_ours.py", {"LEADER_FIELD": '"arccos"', "SPEED_LAW": '"epsfix"', "EPS_FIX": "0.05",
                              "DELTA_MODE": '"full"', "SMC_LAW": '"sat"', "MU": "np.pi / 4", "EPS_SAT": "1.0"})
CASES = (("nominal", {}), ("start15", {"EY0": "15.0"}), ("noise1", {"CHI_NOISE": "np.deg2rad(1.0)"}))


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


def load(tag):
    return loadmat(os.path.join(RES, tag + ".mat"), squeeze_me=True, simplify_cells=True)


def metrics(m, ref=None):
    t = m["t"]
    ss, late = t > 50.0, t > 20.0
    r = {
        "leader_ts": float(m["leader_ts"]),
        "ts": float(m["fol_ts"]),
        "rms_ey": float(np.sqrt(np.mean(m["ey"][ss] ** 2))),
        "max_ey_late": float(np.abs(m["ey"][late]).max()),
        "rms_D": float(np.sqrt(np.mean(m["ex"][ss] ** 2 + m["ey"][ss] ** 2))),
        "effort": float(m["fol_effort"]),
        "t_clipped": float(m["fol_t_clipped"]),
        "v_std_late": float(np.std(m["v_i"][late])),
    }
    if ref is not None:
        r["jitter"] = float(np.sqrt(np.mean((m["chidot_i"][late] - ref["chidot_i"][late]) ** 2)))
    return r


rows, clean = [], {}
try:
    for case, extra in CASES:
        for ctrl, (fn, over) in (("baseline", BASELINE), ("chosen", CHOSEN)):
            tag = run(fn, {**over, **extra, "TAG": f'"_final_{ctrl}_{case}"'})
            m = load(tag)
            if case == "nominal":
                clean[ctrl] = m
            rows.append({"id": f"C{len(rows) + 1}", "controller": ctrl, "case": case, "tag": tag,
                         **metrics(m, clean[ctrl] if case == "noise1" else None)})
finally:
    open("controllers/params.py", "w").write(base)

json.dump(rows, open(os.path.join(OUT, "final_rows.json"), "w"), indent=1)


def f(x, fmt):
    return "–" if x is None or (isinstance(x, float) and np.isnan(x)) else format(x, fmt)


lines = ["| Run | Controller | Case | Leader settling (s) | Follower settling (s) | RMS e_y (m) | Max \\|e_y\\| t>20 s (m) | RMS D (m) | Effort | Clipped (s) | Std v_i t>20 s (m/s) | Jitter (rad/s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    lines.append(f"| {r['id']} | {r['controller']} | {r['case']} | {r['leader_ts']:.2f} | {f(r['ts'], '.2f')} | {r['rms_ey']:.2e} | "
                 f"{r['max_ey_late']:.2e} | {r['rms_D']:.2e} | {r['effort']:.1f} | {r['t_clipped']:.2f} | {r['v_std_late']:.3f} | {f(r.get('jitter'), '.4f')} |")
note = ("\nBaseline: Basak follower (Δ = 0, arctan field, sign-pow η = π/4), guard speed law ε = 0.01, arctan leader. "
        "Chosen: full Δ, arccos follower field (r_min = 20 m), saturation μ = π/4 / ε_s = 1 rad, ε-fix speed law ε = 0.05, arccos leader. "
        "nominal: e_y0 = 5 m; start15: e_y0 = 15 m; noise1: nominal + 1° white heading noise on the follower. "
        "RMS values use t > 50 s. Jitter: RMS χ̇_i(noisy) − χ̇_i(nominal, same controller) for t > 20 s.\n")
open(os.path.join(OUT, "final_table.md"), "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))
