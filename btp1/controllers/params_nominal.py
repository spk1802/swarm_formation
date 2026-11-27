import numpy as np

A = 30.0
L = 300.0

V_GL = 5.0
XL0 = 0.0
YL0 = -30.0
CHI_L0 = np.deg2rad(0.0)

BETA = np.pi / 4
EPS = 1.0

GX, GY = 3.0, 3.0

EX0 = 0.0
EY0 = 5.0
CHI_I_OFFSET0 = np.deg2rad(0.0)

ETA = np.pi / 4
P = 3 / 5
MU = np.pi / 4
EPS_SAT = 1.0

V_MIN, V_MAX = 0.5, 8.0
K_D = 1.0
EPS_DEN = 0.01
EPS_FIX = 0.05

CHI_INF = np.pi / 2
R_MIN = 20.0

K_ARCTAN = 1.0 / (2 / (3 * np.sqrt(3)) * R_MIN)
C_ARCCOS = 1.0 / (0.300 * R_MIN)

LEADER_FIELD = "arctan"
K_L = K_ARCTAN
C_L = C_ARCCOS

SPEED_LAW = "epsfix"
SMC_LAW = "sign_pow"
DELTA_MODE = "full"

ALPHA = 1.65
ALPHA_TRUE = 1.65

CHI_NOISE = 0.0
NOISE_SEED = 0

DT = 0.005
T_END = 60.0
T_SS = 50.0
NEWTON_ITER = 8
SETTLE_BAND = 0.2

RESULTS_DIR = "nominal/results"
TAG = ""
