import numpy as np

REAL = 1
lcoa_mode = 1
SIMUL_TIME = 10
CPU_MIN = 9
CPU_MAX = 15
FU_MAX = 30
FB_MAX = 30
MAX_TASK = 30
DELAY_MIN = 2
DELAY_MAX = 5
MAX_BUS = 30
NUM_BUS = 20
NUM_UAV = 1
NUM_TASK = 10
FU = 3
FB = 9
BANDWIDTH = 2*10**6
LOOP_COUNT = 5
MAX_DISTANCE = 250
MAX_LCOA = 200

NUM_PATH = 20  # 버스 운행경로별로 지나는 정류장(지점)의 개수
MAP_SIZE = 1000  # MAP크기

omega1 = 1
omega2 = 1
epsilon_u = 0.1
lamda = 0.5

X, Y, Z = 550, 400, 100
BUS_POS = [100, 200, 300, 400, 500, 600, 700, 800, 900]

uavs_original = []
buses_original = []
buses_original2 = []
bus_simul = []

task_original = []
sm = np.ones((MAX_TASK, NUM_UAV))
cm = np.ones((MAX_TASK, NUM_UAV))
dm = np.ones((MAX_TASK, NUM_UAV))

Distance = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
P_ub = [[2 for j in range(MAX_BUS)] for i in range(NUM_UAV)] # 전송 파워 (W)
R_ub = [[1 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
W_ub = [[BANDWIDTH for j in range(MAX_BUS)] for i in range(NUM_UAV)] # 대역폭 (Hz)

alpha_0 = 10 ** ((-50.0) / 10)  # 1m 참조 거리에서의 수신 파워 (-50dB를 와트로 변환)
Noise = 10 ** ((-100.0 - 30) / 10)  # 노이즈 파워 (-100dBm를 와트로 변환)