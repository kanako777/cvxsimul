import seaborn as sns
import numpy as np
import cvxpy as cvx
import time
from cvxpy import *
import math
import random
import time
from copy import deepcopy
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager
import itertools
from cvxpy import *
from cvxconfig import *

rho_um = cvx.Variable([NUM_TASK, NUM_UAV], pos=True)
rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
fum = cvx.Variable([NUM_TASK, NUM_UAV])
fbm = cvx.Variable([NUM_TASK, NUM_BUS])
mum = cvx.Variable(NUM_TASK)

tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

def dbm_to_watt(dbm):
    return 10 ** (dbm / 10) / 1000

def watt_to_dbm(watt):
    return 10 * math.log10(1000 * watt)

class UAV:
    def __init__(self, id, x, y, z):
        self.id = id
        self.x = x
        self.y = y
        self.z = random.randint(20, 100)
        self.cpu = FU
        #print("UAV위치", (self.x, self.y, self.z))

class BUS:
    def __init__(self, id, path, cpu):
        self.id = id
        self.path = path
        self.reset()
        self.cpu = cpu
        self.distance = 0

    def reset(self):
        self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]

    def move(self):
        self.path_index += 1
        if self.path_index == len(self.path):
            self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]

class BUS2:
    def __init__(self, id, x, y, t, cpu):
        self.id = id
        self.path_index = 0
        self.x = x
        self.y = y
        self.path = [[x, y, t]]
        self.cpu = cpu
        self.distance = 0
        self.timetable = np.zeros(12)
        self.timetable[t] = 1

    def move(self):
        self.path_index += 1
        if self.path_index == len(self.path):
            self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]

def make_bus(real, num_bus=NUM_BUS, CPU_MIN=CPU_MIN, CPU_MAX=CPU_MAX):

    bus_count = 0
    bus_count2 = 0
    bus_num = MAX_BUS
    paths = []

    if real:
        simul_time = 12
        with open("./realbus2.txt", "r") as fp:
            t = 0
            while t < simul_time:
                path = []
                line = fp.readline()
                poslst = line.split('/')
                for pos in poslst:

                    bus_route, bus_id, xx, yy = pos.split(',')
                    x = float(xx)
                    y = float(yy)

                    find = 0

                    for b in range(bus_count2):
                        if bus_id == buses_original2[b].id:
                            find = 1
                            find_number = b

                    if find==0:
                        buses_original2.append(BUS2(bus_id, x, y, t, random.randint(CPU_MIN,CPU_MAX)))
                        bus_count2 += 1
                    else:
                        if buses_original2[find_number].timetable[t]==0:
                            buses_original2[find_number].path.append([x, y, t])
                            buses_original2[find_number].timetable[t]=1

                    path.append((x, y))

                while len(path) < bus_num:
                    path.append((-1000, -1000))

                paths.append(path)
                t += 1

        paths = np.array(paths).transpose((1, 0, 2))
        num_bus = len(paths)

    else:
        for i in range(num_bus):
            path = [(random.randint(0, MAP_SIZE), random.randint(0, MAP_SIZE))]
            while len(path) < NUM_PATH:
                x, y = path[-1]
                next_x = random.randint(x - random.randint(1, 50), x + random.randint(1, 50))
                next_y = random.randint(y - random.randint(1, 50), y + random.randint(1, 50))
                if next_x > 0 and next_x < MAP_SIZE and next_y > 0 and next_y < MAP_SIZE:
                    path.append((next_x, next_y))

            paths.append(path)

    # BUS 생성
    for i in range(num_bus):
        buses_original.append(BUS(i, paths[i], random.randint(CPU_MIN,CPU_MAX)))
        bus_count += 1

    for i in range(bus_count2):
        print("BUS ID:", buses_original2[i].id, "CPU:", buses_original2[i].cpu, "위치:", (buses_original2[i].x, buses_original2[i].y))


def make_bus2(real, num_bus=NUM_BUS, mode=1):

    bus_count = 0
    paths = []

    if real:
        simul_time = 10
        with open("./buspos.txt", "r") as fp:
            t = 0
            while t < simul_time:
                path = []
                line = fp.readline()
                poslst = line.split('/')[:-1]
                for pos in poslst:
                    x, y = np.array(pos.split(','), dtype=np.float32)
                    path.append((x, y))

                paths.append(path)
                t += 1

        paths = np.array(paths).transpose((1, 0, 2))
        num_bus = len(paths)

    else:
        for i in range(num_bus):
            path = [(random.randint(0, MAP_SIZE), random.randint(0, MAP_SIZE))]
            while len(path) < NUM_PATH:
                x, y = path[-1]
                next_x = random.randint(x - random.randint(1, 50), x + random.randint(1, 50))
                next_y = random.randint(y - random.randint(1, 50), y + random.randint(1, 50))
                if next_x > 0 and next_x < MAP_SIZE and next_y > 0 and next_y < MAP_SIZE:
                    path.append((next_x, next_y))

            paths.append(path)

    # BUS 생성
    for i in range(num_bus):
        if i<2:
            if mode:
                buses_original.append(BUS(i, paths[i], random.randint(9,10)))
            else:
                buses_original.append(BUS(i, paths[i], random.randint(4, 6)))
        elif i<4:
            if mode:
                buses_original.append(BUS(i, paths[i], random.randint(11,12)))
            else:
                buses_original.append(BUS(i, paths[i], random.randint(8,10)))
        else:
            if mode:
                buses_original.append(BUS(i, paths[i], random.randint(13, 15)))
            else:
                buses_original.append(BUS(i, paths[i], random.randint(12, 15)))

        bus_count += 1

def cal_distance(): # UAV와 BUS간의 전송률 계산

    NUM_BUS = len(buses_original)

    for i in range(NUM_UAV):

        for j in range(NUM_BUS):

            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original[j].x, buses_original[j].y, 0))
            buses_original[j].distance = round(Distance[i][j])

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

            # 결과 출력
            # print("거리 :", Distance[i][j], "m ", " 전송률 :", R_ub[i][j], "Gbps")

# TASK 생성
def make_task(start_size, end_size, min_cycle, max_cycle, num_task=NUM_TASK):

    NUM_TASK = num_task

    for i in range(NUM_TASK):
        sm[i] = random.randint(start_size, end_size) / 1E3 # 1~20Mbits (Gbits 단위로 변환)
        cm[i] = sm[i] * random.randint(min_cycle, max_cycle) # 200 cycles per bit (Gcycls 단위로 변환)
        dm[i] = random.randint(DELAY_MIN, DELAY_MAX) / 10

        print("TASK", i+1, " Size:", sm[i]*1E3, "Mbits", "Cycles:", cm[i])

def make_task2(start_size, end_size, min_cycle, max_cycle):

    for i in range(NUM_TASK):
        if i<3:
            sm[i] = random.randint(3, 5) / 1E3 # 1~20Mbits (Gbits 단위로 변환)
        elif i<6:
            sm[i] = random.randint(6, 8) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        else:
            sm[i] = random.randint(9, 10) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        cm[i] = sm[i] * random.randint(min_cycle, max_cycle) # 200 cycles per bit (Gcycls 단위로 변환)
        dm[i] = random.randint(DELAY_MIN, DELAY_MAX) / 10

    sm[0] = 4 / 1E3
    sm[1] = 5 / 1E3
    sm[2] = 5 / 1E3
    sm[3] = 8 / 1E3
    sm[4] = 10 / 1E3
    sm[5] = 10 / 1E3
    sm[6] = 13 / 1E3
    sm[7] = 15 / 1E3
    sm[8] = 15 / 1E3
    sm[9] = 15 / 1E3

    for i in range(NUM_TASK):
        print("TASK", i+1, " Size:", sm[i]*1E3, "Mbits", "Cycles:", cm[i])

def proposed_algorithm(k=FU, o1=omega1, o2=omega2, numbus=NUM_BUS):

    omega1 = o1
    omega2 = o2

    NUM_BUS = numbus

    rho_um = cvx.Variable([NUM_TASK, NUM_UAV], pos=True)
    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fum = cvx.Variable([NUM_TASK, NUM_UAV])
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])
    mum = cvx.Variable(NUM_TASK)

    tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

    FU = k
    FB = 0

    for b in range(NUM_BUS):
        FB+=buses_original[b].cpu

    t_um = [0 for _ in range(NUM_TASK)]
    t_bm = [0 for _ in range(NUM_TASK)]
    t_tx = [0 for _ in range(NUM_TASK)]

    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_BUS / NUM_TASK

    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            t_um[m] += rho_um_k[m, u] * cm[m] / f_u_k[m, u]

            for b in range(NUM_BUS):
                t_bm[m] += rho_bm_k[m, b] * cm[m] / f_b_k[m, b]
                t_tx[m] += rho_bm_k[m, b] * sm[m] / R_ub[u][b]

        if t_um[m] > (t_bm[m] + t_tx[m]):
            mum_k[m] = mum_k[m] * t_um[m]
        else:
            mum_k[m] = mum_k[m] * (t_bm[m] + t_tx[m])

    loop = 1

    while (loop <= LOOP_COUNT):

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = [0 for _ in range(NUM_TASK)]
        e_tx_cost = [0 for _ in range(NUM_TASK)]
        t_um_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]

        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    # t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    # t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m] = cm[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost[m] += epsilon_u * cm[m] * (
                (rho_um[m, u] * f_u_k[m, u] ** 2 + rho_um_k[m, u] * fum[m, u] ** 2)) + (
                                            0.5 * tou_rho_um[m, u] * (rho_um[m, u] - rho_um_k[m, u]) ** 2) + (
                                            0.5 * tou_f_um[m, u] * (fum[m, u] - f_u_k[m, u]) ** 2)

            P2_time_cost += omega1 * mum[m]
            P2_energy_cost += omega2 * (e_um_cost[m] + e_tx_cost[m])

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <= FU] + \
            [0 <= fbm] + \
            [rho_um + cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= rho_um, rho_um <= 1] + \
            [0 <= rho_bm, rho_bm <= 1]

        for b in range(NUM_BUS):
            bus_fb = buses_original[b].cpu
            constraints += [cvxpy.sum(fbm[:, b:b + 1:], axis=0, keepdims=True) <= bus_fb]

        for m in range(NUM_TASK):
            constraints += [0 <= mum[m], mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve(solver=SCS)

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum, NUM_BUS


def proposed_algorithm2(k=FU, fb=0, lcoa_mode=1, simul_time=0, distance=MAX_DISTANCE):

    time_t = simul_time

    MAX_BUS = len(buses_original2)
    MAX_DISTANCE = distance
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    bus_simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            if buses_original2[j].timetable[time_t] ==1:

                abs_x = abs(uavs_original[0].x - buses_original2[j].x)
                abs_y = abs(uavs_original[0].y - buses_original2[j].y)
                DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original2[j].x, buses_original2[j].y, 0))

                if lcoa_mode:
                    if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                        bus_simul.append(buses_original2[j])

                else:
                    if DMAT[i][j] <= MAX_DISTANCE:
                        bus_simul.append(buses_original2[j])

    NUM_BUS = len(bus_simul)

    if NUM_BUS ==0:
        print("UAV 주위에 버스가 없음")
        return None, None, None, None, None, None, None

    if fb:
        bus_simul[0].cpu = fb

    Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    P_ub = [[2 for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 전송 파워 (W)
    R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 대역폭 (Hz)

    for i in range(NUM_UAV):
        for j in range(NUM_BUS):
            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (bus_simul[j].x, bus_simul[j].y, 0))
            bus_simul[j].distance = round(Distance[i][j])
            # Distance[i][j] = random.randint(distance-50, distance)

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

            # 결과 출력
            print("BUS ID:", bus_simul[j].id, ", CPU:", bus_simul[j].cpu, ", 위치:", (bus_simul[j].x, bus_simul[j].y),
                  ", 거리 :", round(Distance[i][j]), "m", ", 전송률 :", round(R_ub[i][j] * 1000, 2), "Mbps")

    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])

    FU = k
    FB = 0

    for b in range(NUM_BUS):
        FB += bus_simul[b].cpu

    t_um = [0 for _ in range(NUM_TASK)]
    t_bm = [0 for _ in range(NUM_TASK)]
    t_tx = [0 for _ in range(NUM_TASK)]

    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS))
    for b in range(NUM_BUS):
        f_b_k[:,b:b+1:] = bus_simul[b].cpu / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            t_um[m] += rho_um_k[m, u] * cm[m] / f_u_k[m, u]

            for b in range(NUM_BUS):
                t_bm[m] += rho_bm_k[m, b] * cm[m] / f_b_k[m, b]
                t_tx[m] += rho_bm_k[m, b] * sm[m] / R_ub[u][b]

        if t_um[m] > (t_bm[m] + t_tx[m]):
            mum_k[m] = mum_k[m] * t_um[m]
        else:
            mum_k[m] = mum_k[m] * (t_bm[m] + t_tx[m])

    loop = 1

    while (loop <= LOOP_COUNT):

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = [0 for _ in range(NUM_TASK)]
        e_tx_cost = [0 for _ in range(NUM_TASK)]
        t_um_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]
        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    # t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    # t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m] = cm[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost[m] = epsilon_u * cm[m] * (
                (rho_um[m, u] * f_u_k[m, u] ** 2 + rho_um_k[m, u] * fum[m, u] ** 2)) + (
                                           0.5 * tou_rho_um[m, u] * (rho_um[m, u] - rho_um_k[m, u]) ** 2) + (
                                           0.5 * tou_f_um[m, u] * (fum[m, u] - f_u_k[m, u]) ** 2)

            P2_time_cost += omega1 * mum[m]
            P2_energy_cost += omega2 * (e_um_cost[m] + e_tx_cost[m])

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <= FU] + \
            [0 <= fbm] + \
            [rho_um + cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= rho_um, rho_um <= 1] + \
            [0 <= rho_bm, rho_bm <= 1]

        for b in range(NUM_BUS):
            bus_fb = bus_simul[b].cpu
            constraints += [cvxpy.sum(fbm[:, b:b + 1:], axis=0, keepdims=True) <= bus_fb]

        for m in range(NUM_TASK):
            constraints += [0 <= mum[m], mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve(solver=SCS)

        if (rho_um.value is None):
            print("최적해를 구할 수 없음")
            return None, None, None, None, None, None, None

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum, NUM_BUS


def uav_only_algorithm(k=FU, lcoa_mode=0, distance=MAX_DISTANCE):

    MAX_BUS = len(buses_original)
    MAX_DISTANCE = distance
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    bus_simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            abs_x = abs(uavs_original[0].x - buses_original[j].x)
            abs_y = abs(uavs_original[0].y - buses_original[j].y)
            DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z), (buses_original[j].x, buses_original[j].y, 0))

            if lcoa_mode:
                if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                    bus_simul.append(buses_original[j])

            else:
                if DMAT[i][j] <= MAX_DISTANCE:
                    bus_simul.append(buses_original[j])

    NUM_BUS = len(bus_simul)

    Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    P_ub = [[2 for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 전송 파워 (W)
    R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 대역폭 (Hz)

    for i in range(NUM_UAV):
        for j in range(NUM_BUS):
            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (bus_simul[j].x, bus_simul[j].y, 0))
            # Distance[i][j] = random.randint(distance-50, distance)

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])

    FU = k

    t_um = [0 for _ in range(NUM_TASK)]
    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            t_um[m] += rho_um_k[m, u] * cm[m] / f_u_k[m, u]
        mum_k[m] = mum_k[m] * t_um[m]

    loop = 1

    while (loop <= LOOP_COUNT):

        e_um_cost = [0 for _ in range(NUM_TASK)]
        t_um_cost = [0 for _ in range(NUM_TASK)]

        P2_energy_cost = 0
        P2_time_cost = 0

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m] = cm[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost[m] = epsilon_u * cm[m] * (
                (rho_um[m, u] * f_u_k[m, u] ** 2 + rho_um_k[m, u] * fum[m, u] ** 2)) + (
                                        0.5 * tou_rho_um[m, u] * (rho_um[m, u] - rho_um_k[m, u]) ** 2) + (
                                        0.5 * tou_f_um[m, u] * (fum[m, u] - f_u_k[m, u]) ** 2)

            P2_time_cost += omega1 * mum[m]
            P2_energy_cost += omega2 * (e_um_cost[m])

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)

        constraints = \
            [0 <= fum, cvxpy.sum(fum) <= FU] + \
            [0 <= fbm, fbm <=0] + \
            [1 <= rho_um, rho_um <= 1] + \
            [0 <= rho_bm, rho_bm <= 0]

        for m in range(NUM_TASK):
            constraints += [0 <= mum[m], mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()

        if (rho_um.value is None):
            print("최적해를 구할 수 없음")
            return None, None, None, None, None, None, None, None

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum, NUM_BUS, e_um_cost

def bus_only_algorithm(k=FU, lcoa_mode=0, distance=MAX_DISTANCE):

    MAX_BUS = len(buses_original)
    MAX_DISTANCE = distance
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    bus_simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            abs_x = abs(uavs_original[0].x - buses_original[j].x)
            abs_y = abs(uavs_original[0].y - buses_original[j].y)
            DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                   (buses_original[j].x, buses_original[j].y, 0))

            if lcoa_mode:
                if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                    bus_simul.append(buses_original[j])

            else:
                if DMAT[i][j] <= MAX_DISTANCE:
                    bus_simul.append(buses_original[j])

    NUM_BUS = len(bus_simul)

    Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    P_ub = [[2 for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 전송 파워 (W)
    R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 대역폭 (Hz)

    for i in range(NUM_UAV):
        for j in range(NUM_BUS):
            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (bus_simul[j].x, bus_simul[j].y, 0))
            # Distance[i][j] = random.randint(distance-50, distance)

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])

    FU = k
    FB = 0

    for b in range(NUM_BUS):
        FB += bus_simul[b].cpu

    t_bm = [0 for _ in range(NUM_TASK)]
    t_tx = [0 for _ in range(NUM_TASK)]

    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_BUS)
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_BUS / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            for b in range(NUM_BUS):
                t_bm[m] += rho_bm_k[m, b] * cm[m] / f_b_k[m, b]
                t_tx[m] += rho_bm_k[m, b] * sm[m] / R_ub[u][b]

        mum_k[m] = mum_k[m] * (t_bm[m] + t_tx[m])

    loop = 1

    while (loop<=LOOP_COUNT) :

        P2_energy_cost = 0
        P2_time_cost = 0

        e_tx_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]
        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost[m] += rho_bm[m,b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

            P2_time_cost += omega1 * mum[m]
            P2_energy_cost += omega2 * (e_tx_cost[m])

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= rho_um, rho_um <= 0] + \
            [0 <= fum, fum <= 0] + \
            [0 <= rho_bm, rho_bm <= 1] + \
            [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= fbm]

        for b in range(NUM_BUS):
            bus_fb = bus_simul[b].cpu
            constraints += [cvxpy.sum(fbm[:, b:b+1:], axis=0, keepdims=True) <= bus_fb]

        for m in range(NUM_TASK):
            constraints += [0<= mum[m], mum[m] <= dm[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()

        if (rho_bm.value is None):
            print("최적해를 구할 수 없음")
            return None, None, None, None, None, None

        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    #return result, rho_bm, fbm, mum
    return result, rho_um, rho_bm, fum, fbm, mum

def fixed_algorithm(k=FU, lcoa_mode=0, distance=MAX_DISTANCE):

    MAX_BUS = len(buses_original)
    MAX_DISTANCE = distance
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    bus_simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            abs_x = abs(uavs_original[0].x - buses_original[j].x)
            abs_y = abs(uavs_original[0].y - buses_original[j].y)
            DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                   (buses_original[j].x, buses_original[j].y, 0))

            if lcoa_mode:
                if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                    bus_simul.append(buses_original[j])

            else:
                if DMAT[i][j] <= MAX_DISTANCE:
                    bus_simul.append(buses_original[j])

    NUM_BUS = len(bus_simul)

    Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    P_ub = [[2 for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 전송 파워 (W)
    R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 대역폭 (Hz)

    for i in range(NUM_UAV):
        for j in range(NUM_BUS):
            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (bus_simul[j].x, bus_simul[j].y, 0))
            # Distance[i][j] = random.randint(distance-50, distance)

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])

    FU = k
    FB = 0

    for b in range(NUM_BUS):
        FB += bus_simul[b].cpu

    t_um = [0 for _ in range(NUM_TASK)]
    t_bm = [0 for _ in range(NUM_TASK)]
    t_tx = [0 for _ in range(NUM_TASK)]

    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_BUS / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            t_um[m] += rho_um_k[m, u] * cm[m] / f_u_k[m, u]

            for b in range(NUM_BUS):
                t_bm[m] += rho_bm_k[m, b] * cm[m] / f_b_k[m, b]
                t_tx[m] += rho_bm_k[m, b] * sm[m] / R_ub[u][b]

        if t_um[m] > (t_bm[m] + t_tx[m]):
            mum_k[m] = mum_k[m] * t_um[m]
        else:
            mum_k[m] = mum_k[m] * (t_bm[m] + t_tx[m])

    loop = 1

    while (loop <= LOOP_COUNT):

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = [0 for _ in range(NUM_TASK)]
        e_tx_cost = [0 for _ in range(NUM_TASK)]
        t_um_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]
        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    # t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    # t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m] = cm[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost[m] = epsilon_u * cm[m] * (
                (rho_um[m, u] * f_u_k[m, u] ** 2 + rho_um_k[m, u] * fum[m, u] ** 2)) + (
                                           0.5 * tou_rho_um[m, u] * (rho_um[m, u] - rho_um_k[m, u]) ** 2) + (
                                           0.5 * tou_f_um[m, u] * (fum[m, u] - f_u_k[m, u]) ** 2)

            P2_time_cost += omega1 * mum[m]
            P2_energy_cost += omega2 * (e_um_cost[m] + e_tx_cost[m])

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <=FU] + \
            [0 <= fbm] + \
            [0.5 <= rho_um, rho_um <= 0.5] + \
            [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 0.5] + \
            [0 <= rho_bm, rho_bm <= 0.5]

        for b in range(NUM_BUS):
            bus_fb = bus_simul[b].cpu
            constraints += [cvxpy.sum(fbm[:, b:b+1:], axis=0, keepdims=True) <= bus_fb]

        for m in range(NUM_TASK):
            constraints += [0<= mum[m], mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()


        if (rho_um.value is None):
            print("최적해를 구할 수 없음")
            return None, None, None, None, None, None

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum

def busmap(X_STEP=5, Y_STEP=5, distance=MAX_DISTANCE):

    MAX_BUS = len(buses_original)
    MAP_X_STEP = MAP_SIZE / X_STEP
    MAP_Y_STEP = MAP_SIZE / Y_STEP

    BUS_MAP = [[0 for j in range(Y_STEP)] for i in range(X_STEP)]

    for j in range(MAX_BUS):

        x = buses_original[j].x // MAP_X_STEP
        y = buses_original[j].y // MAP_Y_STEP

        BUS_MAP[int(y)][int(x)] +=1

    return BUS_MAP

def draw_map(X, Y, buses_original):
    # 버스 경로를 파일로 프린트
    fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(8, 4), gridspec_kw={"wspace": 0.0, "hspace": 0.0}, sharex=False, sharey=True, constrained_layout=True)

    filename = "map3.png"
    lonmin, lonmax, latmin, latmax = (-400,1350,-100,1100) # just example

    image_extent = (lonmin, lonmax, latmin, latmax)

    ax1.imshow(plt.imread(filename), extent=image_extent)
    ax2.imshow(plt.imread(filename), extent=image_extent)

    x = []
    y = []
    for bus in buses_original:
        for i in range(len(bus.path)):
            tx, ty = bus.path[i]
            x.append(tx)
            y.append(ty)

    ax1.scatter(x, y, s=100, alpha=0.5)
    #ax1.scatter(X, Y, s=150, marker="X", color='r', alpha=1)
    ax1.set_ylim((-100, 1100))
    ax1.set_xlim((-100, 1100))

    sns.kdeplot(x=x, y=y, cmap=plt.cm.GnBu, fill=True, levels=30, ax=ax2)
    #ax2.scatter(X, Y, s=150, marker="X", color='r', alpha=1)
    ax2.set_ylim((-100, 1100))
    ax2.set_xlim((-100, 1100))

    num = [0, 1, 2, 3, 4]
    for i in num:
        for j in num:
            # test = patches.Rectangle((j * 200, i * 200), 200, 200, linewidth=0.5, edgecolor='g', facecolor='none', linestyle=':')
            # ax1.add_patch(test)
            pass

    for i in num:
        for j in num:
            test = patches.Rectangle((j * 200, i * 200), 200, 200, linewidth=0.5, edgecolor='g', facecolor='none',
                                     linestyle=':')
            ax2.add_patch(test)

    # for i in range(len(POSITION)):
    # x, y = POSITION[i]
    # ax2.text(x * 200 + 10, y*200+160, "P{}".format(9-i), dict(size=15))

    plt.show()
    plt.savefig("./graphs/map")
    plt.clf()


def count_bus(lcoa_mode=0):

    time_t = 0
    MAX_BUS = len(buses_original2)
    MAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    temp = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):
            if (buses_original2[j].timetable[time_t]==1):
                abs_x = abs(uavs_original[0].x - buses_original2[j].x)
                abs_y = abs(uavs_original[0].y - buses_original2[j].y)
                MAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original2[j].x, buses_original2[j].y, 0))

                if lcoa_mode:
                    if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                        temp.append(buses_original2[j])

                else:
                    if MAT[i][j] <= MAX_DISTANCE:
                        temp.append(buses_original2[j])

    NUM_BUS = len(temp)
    return NUM_BUS