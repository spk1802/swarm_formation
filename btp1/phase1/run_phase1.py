import re, subprocess, sys, os, json

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
os.chdir(ROOT)
OUT = "phase1"
RES = os.path.join(OUT, "results")

base = open("controllers/params_nominal.py").read()
env = {**os.environ, "PYTHONDONTWRITEBYTECODE": "1"}

def setp(src, key, val):
    return re.sub(rf"^{key} = .*$", f"{key} = {val}", src, flags=re.M)

def run(over):
    src = base
    for k, v in {**over, "RESULTS_DIR": f'"{RES}"'}.items():
        src = setp(src, k, v)
    open("controllers/params.py", "w").write(src)
    out = subprocess.run([sys.executable, "controllers/basak.py"], capture_output=True, text=True, check=True, env=env).stdout
    tag = out.splitlines()[0]
    m = dict((k, float(v)) for k, v in re.findall(r"(leader_\w+)\s+(\S+)", out))
    return tag, m

rows = []
try:
    for rmin in (20.0, 10.0, 30.0):
        for field in ("arctan", "arccos"):
            tag, m = run({"LEADER_FIELD": f'"{field}"', "R_MIN": rmin})
            rows.append({"part": "A", "field": field, "rmin": rmin, "y0": 30, "h0": 0, "tag": tag, **m})
    for y in (15, 30, 60):
        for h in (0, 90, 180):
            if y == 30 and h == 0:
                continue
            for field in ("arctan", "arccos"):
                tag, m = run({"LEADER_FIELD": f'"{field}"', "YL0": -float(y),
                              "CHI_L0": f"np.deg2rad({h}.0)", "TAG": f'"_y{y}_h{h}"'})
                rows.append({"part": "B", "field": field, "rmin": 20.0, "y0": y, "h0": h, "tag": tag, **m})
finally:
    open("controllers/params.py", "w").write(base)
json.dump(rows, open(os.path.join(RES, "phase1_rows.json"), "w"), indent=1)

lines = ["| Run | Field | r_min (m) | Start y_l0 (m) | χ_l0 (deg) | Settling time (s) | Effort | Max curvature (1/m) | Under 1/r_min? | RMS d, t>50 s (mm) |",
         "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]
for i, r in enumerate(rows, 1):
    lim = 1 / r["rmin"]
    lines.append(f"| L{i} | {r['field']} | {r['rmin']:g} | −{r['y0']} | {r['h0']} | {r['leader_ts']:.2f} | "
                 f"{r['leader_effort']:.1f} | {r['leader_kappa_max']:.3f} | {'yes' if r['leader_kappa_max'] <= lim else 'no'} "
                 f"({lim:.3f}) | {r['leader_rms_d']*1000:.2f} |")
open(os.path.join(OUT, "table_runs.md"), "w").write("\n".join(lines) + "\n")
print("\n".join(lines))
