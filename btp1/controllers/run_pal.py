import os
import numpy as np
from scipy.io import savemat
from params import *

NAME = "pal"
HEAD = "none"


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def sat(z):
    return np.clip(z, -1.0, 1.0)


w = 2 * np.pi / L


def f(x):
    return A * np.sin(w * x)


def fp(x):
    return A * w * np.cos(w * x)


def fpp(x):
    return -A * w**2 * np.sin(w * x)


def kappa(x):
    return fpp(x) / (1 + fp(x)**2) ** 1.5


def phi_arctan(e, k):
    return -CHI_INF * (2 / np.pi) * np.arctan(k * e)


def dphi_arctan(e, k):
    return -CHI_INF * (2 / np.pi) * k / (1 + (k * e)**2)


def phi_arccos(e, c):
    u = np.exp(-c * abs(e))
    return -np.sign(e) * np.arcsin(1 - u)


def dphi_arccos(e, c):
    u = np.exp(-c * abs(e))
    return -c * u / np.sqrt(2 * u - u**2)


def closest_point(xq, yq, x_guess):
    x = x_guess
    for _ in range(NEWTON_ITER):
        g = (x - xq) + (f(x) - yq) * fp(x)
        gp = 1 + fp(x)**2 + (f(x) - yq) * fpp(x)
        x = x - g / gp
    return x


def leader_field(d):
    if LEADER_FIELD == "arctan":
        return phi_arctan(d, K_L), dphi_arctan(d, K_L)
    if LEADER_FIELD == "arccos":
        return phi_arccos(d, C_L), dphi_arccos(d, C_L)
    raise ValueError(LEADER_FIELD)


def leader_law(xl, yl, chi_l, x_guess):
    xs = closest_point(xl, yl, x_guess)
    chi_p = np.arctan2(fp(xs), 1.0)
    d = -(xl - xs) * np.sin(chi_p) + (yl - f(xs)) * np.cos(chi_p)
    k = kappa(xs)

    th = wrap(chi_l - chi_p)
    d_dot = V_GL * np.sin(th)
    chi_p_dot = k * V_GL * np.cos(th) / (1 - k * d)

    phi, dphi = leader_field(d)
    chi_d = chi_p + phi
    chi_d_dot = chi_p_dot + dphi * d_dot
    chi_t = wrap(chi_l - chi_d)
    u = chi_d_dot - BETA * sat(chi_t / EPS)
    return u, xs, d, chi_t


def formation_errors(xi, yi, xl, yl, chi_l):
    dx, dy = xi - xl, yi - yl
    ex = dx * np.cos(chi_l) + dy * np.sin(chi_l) - GX
    ey = -dx * np.sin(chi_l) + dy * np.cos(chi_l) - GY
    return ex, ey


def speed_guard(num, den):
    if abs(den) < EPS_DEN:
        den = EPS_DEN if den >= 0 else -EPS_DEN
    return num / den


def speed_epsfix(num, den, v_blend):
    return (num * den + v_blend * EPS_FIX**2) / (den**2 + EPS_FIX**2)


def speed(ex, ey, rel, omega):
    num = V_GL * ex - K_D * (ex**2 + ey**2) - omega * (ex * GY - ey * GX)
    den = ex * np.cos(rel) + ey * np.sin(rel)
    if SPEED_LAW == "guard":
        v = speed_guard(num, den)
    elif SPEED_LAW == "epsfix":
        v_req = np.hypot(V_GL - omega * GY, omega * GX)
        v_blend = v_req if (HEAD != "none" and v_req > 1e-3) else V_GL
        v = speed_epsfix(num, den, v_blend)
    else:
        raise ValueError(SPEED_LAW)
    return float(np.clip(v, V_MIN, V_MAX)), den


def reaching(chi_t):
    if SMC_LAW == "sign_pow":
        return -ETA * np.sign(chi_t) * np.abs(chi_t) ** P
    if SMC_LAW == "sat":
        return -MU * sat(chi_t / EPS_SAT)
    raise ValueError(SMC_LAW)


def carousel(omega, omega_dot):
    a, b = omega * GX, V_GL - omega * GY
    Delta = np.arctan2(a, b)
    if DELTA_MODE == "pointwise":
        return Delta, 0.0
    if DELTA_MODE == "full":
        return Delta, omega_dot * V_GL * GX / (a**2 + b**2 + 1e-9)
    raise ValueError(DELTA_MODE)


def follower_law(xi, yi, chi_i, xl, yl, chi_l, omega, omega_dot):
    ex, ey = formation_errors(xi, yi, xl, yl, chi_l)
    rel = wrap(chi_i - chi_l)
    v, den = speed(ex, ey, rel, omega)
    ey_dot = v * np.sin(rel) - omega * (GX + ex)

    Delta, Delta_dot = 0.0, 0.0
    chi_d = chi_l + Delta + phi_arccos(ey, C_ARCCOS)
    chi_d_dot = omega + Delta_dot + dphi_arccos(ey, C_ARCCOS) * ey_dot
    chi_t = wrap(chi_i - chi_d)
    u = chi_d_dot + reaching(chi_t)
    return u, v, ex, ey, chi_t, Delta, den


def autopilot(chi, u):
    chi_c = wrap(chi + u / ALPHA)
    return ALPHA_TRUE * wrap(chi_c - chi)


n = int(round(T_END / DT)) + 1
t = np.arange(n) * DT
H = {k: np.zeros(n) for k in [
    "xl", "yl", "chi_l", "omega", "d", "chi_t_l",
    "xi", "yi", "chi_i", "chidot_i", "u_i", "v_i", "ex", "ey", "chi_t_i", "Delta", "den"]}

xl, yl, chi_l = XL0, YL0, CHI_L0
ox, oy = GX + EX0, GY + EY0
xi = xl + ox * np.cos(chi_l) - oy * np.sin(chi_l)
yi = yl + ox * np.sin(chi_l) + oy * np.cos(chi_l)
chi_i = wrap(chi_l + CHI_I_OFFSET0)
x_star = XL0
omega_prev = None
rng = np.random.default_rng(NOISE_SEED)

for k in range(n):
    uL, x_star, d, chi_tl = leader_law(xl, yl, chi_l, x_star)
    omega = autopilot(chi_l, uL)
    omega_dot = 0.0 if omega_prev is None else (omega - omega_prev) / DT
    omega_prev = omega

    chi_meas = wrap(chi_i + rng.normal(0.0, CHI_NOISE)) if CHI_NOISE > 0 else chi_i
    uF, v_i, ex, ey, chi_ti, Delta, den = follower_law(xi, yi, chi_meas, xl, yl, chi_l, omega, omega_dot)
    chidot_i = autopilot(chi_i, uF)

    for key, val in (("xl", xl), ("yl", yl), ("chi_l", chi_l), ("omega", omega), ("d", d),
                     ("chi_t_l", chi_tl), ("xi", xi), ("yi", yi), ("chi_i", chi_i),
                     ("chidot_i", chidot_i), ("u_i", uF), ("v_i", v_i), ("ex", ex), ("ey", ey),
                     ("chi_t_i", chi_ti), ("Delta", Delta), ("den", den)):
        H[key][k] = val

    xl += DT * V_GL * np.cos(chi_l)
    yl += DT * V_GL * np.sin(chi_l)
    chi_l = wrap(chi_l + DT * omega)
    xi += DT * v_i * np.cos(chi_i)
    yi += DT * v_i * np.sin(chi_i)
    chi_i = wrap(chi_i + DT * chidot_i)


def settling_time(err):
    out = np.abs(err) > SETTLE_BAND
    if not out.any():
        return 0.0
    last = np.nonzero(out)[0][-1]
    return float(t[last + 1]) if last + 1 < n else np.nan


ss = t >= T_SS
clipped = (H["v_i"] <= V_MIN + 1e-9) | (H["v_i"] >= V_MAX - 1e-9)
metrics = {
    "leader_ts": settling_time(H["d"]),
    "leader_effort": np.sum((V_GL * H["omega"]) ** 2) * DT,
    "leader_kappa_max": np.max(np.abs(H["omega"])) / V_GL,
    "leader_rms_d": np.sqrt(np.mean(H["d"][ss] ** 2)),
    "fol_ts": settling_time(H["ey"]),
    "fol_max_ey": np.max(np.abs(H["ey"])),
    "fol_rms_ey": np.sqrt(np.mean(H["ey"][ss] ** 2)),
    "fol_mean_ex": np.mean(H["ex"][ss]),
    "fol_effort": np.sum((H["v_i"] * H["chidot_i"]) ** 2) * DT,
    "fol_kappa_max": np.max(np.abs(H["chidot_i"] / H["v_i"])),
    "fol_max_chi_t_deg": np.rad2deg(np.max(np.abs(H["chi_t_i"]))),
    "fol_t_clipped": np.sum(clipped) * DT,
}

run_params = {
    "V_GL": V_GL, "A": A, "L": L, "XL0": XL0, "YL0": YL0, "CHI_L0": CHI_L0,
    "GX": GX, "GY": GY, "EX0": EX0, "EY0": EY0, "CHI_I_OFFSET0": CHI_I_OFFSET0,
    "BETA": BETA, "EPS": EPS, "ETA": ETA, "P": P, "MU": MU, "EPS_SAT": EPS_SAT,
    "K_D": K_D, "EPS_DEN": EPS_DEN, "EPS_FIX": EPS_FIX, "V_MIN": V_MIN, "V_MAX": V_MAX,
    "R_MIN": R_MIN, "K_ARCTAN": K_ARCTAN, "C_ARCCOS": C_ARCCOS, "K_L": K_L, "C_L": C_L,
    "LEADER_FIELD": LEADER_FIELD, "SPEED_LAW": SPEED_LAW, "SMC_LAW": SMC_LAW, "HEAD": HEAD,
    "ALPHA": ALPHA, "ALPHA_TRUE": ALPHA_TRUE, "CHI_NOISE": CHI_NOISE, "NOISE_SEED": NOISE_SEED, "DT": DT, "T_END": T_END,
}

os.makedirs(RESULTS_DIR, exist_ok=True)
tag = (f"{NAME}_{HEAD}_L-{LEADER_FIELD}_{SPEED_LAW}_{SMC_LAW}"
       f"_rmin{R_MIN:g}_chi{np.rad2deg(CHI_I_OFFSET0):g}_off{EY0:g}{TAG}")
savemat(os.path.join(RESULTS_DIR, tag + ".mat"),
        {"t": t, **H, **metrics, "params": run_params, "name": NAME})

print(tag)
for key, val in metrics.items():
    print(f"  {key:18s} {val:.4g}")
