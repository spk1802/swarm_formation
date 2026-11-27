import re, subprocess, sys, os, json
import numpy as np
from scipy.io import loadmat

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "phase5"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}
FN = "run_pal_ours.py"
GAINS = {"pi/8": np.pi / 8, "pi/4": np.pi / 4, "pi/2": np.pi / 2}


def setp(src, key, val):
    new, n = re.subn(rf"^{key} = .*$", f"{key} = {val}", src, flags=re.M)
    assert n == 1, key
    return new


def run(over):
    src = base
    for k, v in {**over, "RESULTS_DIR": f'"{RES}"'}.items():
        src = setp(src, k, v)
    open("controllers/params.py", "w").write(src)
    out = subprocess.run([sys.executable, f"controllers/{FN}"], capture_output=True, text=True, check=True, env=env).stdout
    return out.splitlines()[0]


def load(tag):
    return loadmat(os.path.join(RES, tag + ".mat"), squeeze_me=True, simplify_cells=True)


def settle_below(t, x, thr):
    out = np.abs(x) > thr
    if not out.any():
        return 0.0
    last = np.nonzero(out)[0][-1]
    return float(t[last + 1]) if last + 1 < len(t) else None


def metrics(m, ref=None):
    t = m["t"]
    ss, late = t > 50.0, t > 20.0
    r = {
        "ts": float(m["fol_ts"]),
        "rms_ey": float(np.sqrt(np.mean(m["ey"][ss] ** 2))),
        "effort": float(m["fol_effort"]),
        "max_chi_t_deg": float(m["fol_max_chi_t_deg"]),
        "t_chi_1e3": settle_below(t, m["chi_t_i"], 1e-3),
        "t_clipped": float(m["fol_t_clipped"]),
    }
    if ref is not None:
        r["jitter"] = float(np.sqrt(np.mean((m["chidot_i"][late] - ref["chidot_i"][late]) ** 2)))
    return r


rows = []
clean = {}
try:
    for law, gkey, pname in [(l, g, "ETA" if l == "sign_pow" else "MU") for l in ("sign_pow", "sat") for g in GAINS]:
        tag = run({"LEADER_FIELD": '"arctan"', "SMC_LAW": f'"{law}"', pname: repr(GAINS[gkey]),
                   "TAG": f'"_p5_{law}_g{gkey.replace("/", "")}_n0"'})
        m = load(tag)
        clean[(law, gkey)] = m
        rows.append({"id": f"R{len(rows) + 1}", "law": law, "gain": gkey, "noise_deg": 0.0, "tag": tag, **metrics(m)})
    for sig in (0.5, 1.0, 2.0):
        for law in ("sign_pow", "sat"):
            pname = "ETA" if law == "sign_pow" else "MU"
            tag = run({"LEADER_FIELD": '"arctan"', "SMC_LAW": f'"{law}"', pname: repr(GAINS["pi/4"]),
                       "CHI_NOISE": repr(np.deg2rad(sig)), "TAG": f'"_p5_{law}_gpi4_n{sig:g}"'})
            m = load(tag)
            rows.append({"id": f"R{len(rows) + 1}", "law": law, "gain": "pi/4", "noise_deg": sig, "tag": tag,
                         **metrics(m, clean[(law, "pi/4")])})
finally:
    open("controllers/params.py", "w").write(base)

json.dump(rows, open(os.path.join(OUT, "phase5_rows.json"), "w"), indent=1)


def f(x, fmt):
    return "–" if x is None else format(x, fmt)


lines = ["| Run | Law | Gain (η or μ) | Heading noise σ (deg) | Settling (s) | RMS e_y (m) | Effort | Max \\|χ̃_i\\| (deg) | t(\\|χ̃_i\\| < 1e-3) (s) | Turn-rate jitter (rad/s) | Clipped (s) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for r in rows:
    lines.append(f"| {r['id']} | {r['law']} | {r['gain']} | {r['noise_deg']:g} | {r['ts']:.2f} | {r['rms_ey']:.2e} | {r['effort']:.1f} | "
                 f"{r['max_chi_t_deg']:.1f} | {f(r['t_chi_1e3'], '.2f')} | {f(r.get('jitter'), '.4f')} | {r['t_clipped']:.2f} |")
note = ("\nFollower: arccos field, r_min = 20 m, full Δ, ε-fix speed law. Leader: arctan, unchanged. Sat band ε_s = 1 rad, p = 3/5. "
        "Noise: white Gaussian on the follower's measured course each 5 ms step, seed 0. "
        "RMS e_y uses t > 50 s. Jitter: RMS of χ̇_i(noisy) − χ̇_i(noise-free, same law and gain) for t > 20 s.\n")
open(os.path.join(OUT, "phase5_table.md"), "w").write("\n".join(lines) + "\n" + note)
print("\n".join(lines))
