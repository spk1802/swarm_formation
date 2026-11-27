import os
import re
import subprocess
import sys
import numpy as np
from scipy.io import loadmat

HERE = os.path.dirname(os.path.abspath(__file__))
CTRL = os.path.join(HERE, "controllers")
sys.path.insert(0, CTRL)
FILES = ["basak.py", "run_ours.py", "run_pal.py", "run_pal_ours.py"]
ok = True


def report(name, passed, info=""):
    global ok
    ok &= passed
    print(f"[{'PASS' if passed else 'FAIL'}] {name}  {info}")


def functions_of(fn):
    src = open(os.path.join(CTRL, fn)).read().split("\nn = int(round")[0]
    ns = {}
    exec(compile(src, fn, "exec"), ns)
    return ns


ns = functions_of("basak.py")

e = np.linspace(-40, 40, 4001)
e = e[np.abs(e) > 1e-3]
h = 1e-6
for kind in ("arctan", "arccos"):
    phi, dphi, g = ns[f"phi_{kind}"], ns[f"dphi_{kind}"], 0.15
    fd = np.array([(phi(x + h, g) - phi(x - h, g)) / (2 * h) for x in e])
    an = np.array([dphi(x, g) for x in e])
    err = np.max(np.abs(fd - an))
    report(f"{kind} field derivative vs finite difference", err < 1e-5, f"max err {err:.1e}")

worst = 0.0
for xq, yq in [(0, -30), (75, 60), (140, -10), (220, 25)]:
    xs = xq
    for _ in range(30):
        xs = ns["closest_point"](xq, yq, xs)
    g = (xs - xq) + (ns["f"](xs) - yq) * ns["fp"](xs)
    worst = max(worst, abs(g))
report("closest point is normal to the tangent", worst < 1e-8, f"max |g| {worst:.1e}")

rng = np.random.default_rng(0)
worst = 0.0
for _ in range(20):
    xl, yl, chl = rng.normal(size=3) * 5
    xi, yi = xl + rng.normal() * 8, yl + rng.normal() * 8
    chi, vi, om = rng.uniform(-3, 3), rng.uniform(1, 5), rng.uniform(-0.5, 0.5)
    vgl = ns["V_GL"]

    def errs(s):
        return np.array(ns["formation_errors"](xi + s * vi * np.cos(chi), yi + s * vi * np.sin(chi),
                                               xl + s * vgl * np.cos(chl), yl + s * vgl * np.sin(chl),
                                               chl + s * om))
    fd = (errs(h) - errs(-h)) / (2 * h)
    ex, ey = errs(0)
    exact = vi * np.sin(chi - chl) - om * (ns["GX"] + ex)
    worst = max(worst, abs(fd[1] - exact))
report("exact e_y rate vs finite difference", worst < 1e-6, f"max err {worst:.1e}")

leader = {}
for fn in FILES:
    out = subprocess.run([sys.executable, os.path.join(CTRL, fn)], cwd=HERE, capture_output=True, text=True)
    if out.returncode != 0:
        report(f"{fn} runs", False, out.stderr.strip().splitlines()[-1])
        continue
    tag = out.stdout.splitlines()[0]
    vals = dict(re.findall(r"(leader_\w+)\s+(\S+)", out.stdout))
    leader[fn] = (tag, vals)
    report(f"{fn} runs", True, tag)

if len(leader) == len(FILES):
    ref = leader[FILES[0]][1]
    same = all(v[1] == ref for v in leader.values())
    report("leader metrics identical in all four files", same, str(ref))

    m = loadmat(os.path.join(HERE, ns["RESULTS_DIR"], leader["run_ours.py"][0] + ".mat"), squeeze_me=True)
    if str(m["params"]["HEAD"]) == "full":
        ss = m["t"] >= 50
        rel = (m["chi_i"] - m["chi_l"] + np.pi) % (2 * np.pi) - np.pi
        gap = np.max(np.abs(rel[ss] - m["Delta"][ss]))
        report("steady chi_i - chi_l matches Delta (run_ours, full)", gap < 1e-3, f"max gap {gap:.1e} rad")

print("\nALL PASSED" if ok else "\nSOME CHECKS FAILED")
