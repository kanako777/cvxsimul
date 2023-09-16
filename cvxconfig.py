import numpy as np

SIMUL_TIME = 10
CPU_MIN = 6
CPU_MAX = 9
FU_MAX = 30
FB_MAX = 30
TASK_SIZE_MAX = 100
DELAY_MAX = 10
MAX_BUS = 20
NUM_BUS = 4 # 운행하는 버스의 대수(버스별로 자기 노선(path)을 가짐
NUM_UAV = 1  # UAV의 개수
NUM_TASK = 10
FU = 3
FB = 9
BANDWIDTH = 1*10**6
LOOP_COUNT = 5
MAX_DISTANCE = 250

NUM_PATH = 100  # 버스 운행경로별로 지나는 정류장(지점)의 개수
MAP_SIZE = 1000  # MAP크기

omega1 = 1
omega2 = 1
epsilon_u = 0.1
lamda = 0.5

X, Y, Z = 500, 500, 100
BUS_POS = [100, 200, 300, 400, 500, 600, 700, 800, 900]

Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
P_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)] # 전송 파워 (W)
R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)] # 대역폭 (Hz)

uavs_original = []
buses_original = []
bus_simul = []


task_original = []
sm = np.ones((NUM_TASK, NUM_UAV))
cm = np.ones((NUM_TASK, NUM_UAV))
dm = np.ones((NUM_TASK, NUM_UAV))

alpha_0 = 10 ** ((-50.0) / 10)  # 1m 참조 거리에서의 수신 파워 (-50dB를 와트로 변환)
Noise = 10 ** ((-100.0 - 30) / 10)  # 노이즈 파워 (-100dBm를 와트로 변환)
