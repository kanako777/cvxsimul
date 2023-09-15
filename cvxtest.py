import numpy as np
import cvxpy as cvx
import time
from cvxpy import *
import math
import random
import time
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.font_manager
from cvxpy import *
from cvxgraph import *
from cvxconfig import *
from cvxutil import *

class UAV:
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = z
        self.cpu = FU

class BUS:
    def __init__(self, id, path):
        self.id = id
        self.path = path
        self.reset()

    def reset(self):
        self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]
        #self.x = BUS_POS[count]
        #self.y = BUS_POS[count]

    def move(self):
        self.path_index += 1
        if self.path_index == len(self.path):
            self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]

class TASK:
    def __init__(self, id, size, cycle, delay):
        self.id = id
        self.size = size
        self.cycle = cycle
        self.delay = delay


paths = []

for i in range(NUM_BUS):
    path = [(random.randint(0, MAP_SIZE), random.randint(0, MAP_SIZE))]
    while len(path) < NUM_PATH:
        x, y = path[-1]
        next_x = random.randint(x - random.randint(1, 50), x + random.randint(1, 50))
        next_y = random.randint(y - random.randint(1, 50), y + random.randint(1, 50))
        if next_x>0 and next_x <MAP_SIZE and next_y>0 and next_y < MAP_SIZE:
            path.append((next_x, next_y))

    paths.append(path)

# UAV 생성
uavs_original = []
for i in range(NUM_UAV):
    uavs_original.append(UAV(i, X, Y, Z))

# BUS 생성
buses_original = []
for i in range(NUM_BUS):
    buses_original.append(BUS(i, paths[i]))
    bus_count +=1

# UAV와 BUS간의 전송률 계산
alpha_0 = 10 ** ((-50.0) / 10)  # 1m 참조 거리에서의 수신 파워 (-50dB를 와트로 변환)
Noise = 10 ** ((-100.0 - 30) / 10)  # 노이즈 파워 (-100dBm를 와트로 변환)

for i in range(NUM_UAV):
    for j in range(NUM_BUS):

        Distance[i][j] = math.dist((uavs_original[0].x,uavs_original[0].y,uavs_original[0].z), (buses_original[j].x,buses_original[j].y,0))

        # 전송률 계산 (Shannon Capacity 공식 사용)
        SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j]**2)
        R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

        # 결과 출력
        print("거리 :", Distance[i][j], "m ", " 전송률 :", R_ub[i][j], "Gbps")


MODE = 1

if MODE == 1: # UAV의 CPU를 증가시켜가며 실험

    system_cost = [0 for j in range((FU_MAX + 1) // 3)]
    k_index = 0
    make_task(1, 5)

    for k in range(3, FU_MAX+1, 3):

        print("STEP : ", k_index)
        result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(k) # 제안 알고리즘

        result2, fum2 = uav_only_algorithm(k) # uav only 알고리즘

        result3, rho_bm3, fbm3, mum3 = bus_only_algorithm(k)  # bus only 알고리즘

        result4, rho_um4, rho_bm4, fum4, fbm4, mum4 = equal_ratio_algorithm(k)  # bus only 알고리즘

        system_cost[k_index] = result, result2, result3, result4
        print("Proposed : ", system_cost[k_index][0], "UAV Only : ", system_cost[k_index][1], "Bus Only : ", system_cost[k_index][2], "Equal Ratio : ", system_cost[k_index][3])
        k_index += 1


if MODE == 2: # TASK의 data size를 증가시켜가며 테스트

    system_cost = [0 for j in range((TASK_SIZE_MAX + 1) // 5)]
    k_index = 0

    for k in range(5, TASK_SIZE_MAX + 1, 5):

        make_task(k, k)

        print("STEP : ", k_index)
        result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘

        result2, fum2 = uav_only_algorithm(FU)  # uav only 알고리즘

        result3, rho_bm3, fbm3, mum3 = bus_only_algorithm(FU)  # bus only 알고리즘

        result4, rho_um4, rho_bm4, fum4, fbm4, mum4 = equal_ratio_algorithm(FU)  # bus only 알고리즘

        system_cost[k_index] = result, result2, result3, result4
        print("Proposed : ", system_cost[k_index][0], "UAV Only : ", system_cost[k_index][1], "Bus Only : ",
              system_cost[k_index][2], "Equal Ratio : ", system_cost[k_index][3])
        k_index += 1


#ratio_graph(rho_um, rho_bm, Distance) # UAV와 BUS의 ratio 그래프 그리기