import math
import random
import time
import numpy as np
import cvxpy as cvx
import seaborn as sns
import matplotlib.pyplot as plt
import matplotlib.patches as patches
import matplotlib.font_manager
import itertools
import mosek
from cvxpy import *
from copy import deepcopy
from cvxpy import *
from cvxconfig import *


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
        # print("UAV위치", (self.x, self.y, self.z))


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
        self.reset()
        self.id = id
        self.path_index = 0
        self.x = x
        self.y = y
        self.path = [[x, y, t]]
        self.cpu = cpu
        self.distance = 0
        self.timetable = np.zeros(12)
        self.timetable[t] = 1

    def reset(self):
        self.id = 0
        self.path_index = 0
        self.path = []
        self.cpu = 0
        self.distance = 0
        self.timetable = np.zeros(12)

    def move(self):
        self.path_index += 1
        if self.path_index == len(self.path):
            self.path_index = 0
        self.location = self.path[self.path_index]
        self.x = self.location[0]
        self.y = self.location[1]


def cal_bus():
    lcoa_buses = np.zeros(12)
    total_buses = np.zeros(12)
    MAX_BUS = len(buses_original2)
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]

    for time_t in range(12):

        for i in range(NUM_UAV):
            for j in range(MAX_BUS):

                if buses_original2[j].timetable[time_t] == 1:

                    abs_x = abs(uavs_original[0].x - buses_original2[j].x)
                    abs_y = abs(uavs_original[0].y - buses_original2[j].y)
                    DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                           (buses_original2[j].x, buses_original2[j].y, 0))

                    if lcoa_mode:
                        if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                            lcoa_buses[time_t] += 1

                    else:
                        if DMAT[i][j] <= MAX_DISTANCE:
                            lcoa_buses[time_t] += 1

                    total_buses[time_t] += 1

    print(total_buses)
    print(lcoa_buses)


def cal_distance():  # UAV와 BUS간의 전송률 계산

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


def make_bus(real, num_bus=NUM_BUS, input=INPUT_FILE, CPU_MIN=CPU_MIN, CPU_MAX=CPU_MAX):
    fp = 0
    bus_count = 0
    bus_num = MAX_BUS
    paths = []

    if real:
        simul_time = 12
        with open("./busdata/" + str(input) + ".txt", "r") as fp:

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

                    for b in range(bus_count):
                        if bus_id == buses_original2[b].id:
                            find = 1
                            find_number = b

                    if find == 0:
                        buses_original2.append(BUS2(bus_id, x, y, t, random.randint(CPU_MIN, CPU_MAX)))
                        bus_count += 1
                    else:
                        if buses_original2[find_number].timetable[t] == 0:
                            buses_original2[find_number].path.append([x, y, t])
                            buses_original2[find_number].timetable[t] = 1

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
        buses_original.append(BUS(i, paths[i], random.randint(CPU_MIN, CPU_MAX)))

    fp.close()


# TASK 생성
def make_task(start_size, end_size, min_cycle, max_cycle, num_task=NUM_TASK):
    NUM_TASK = num_task

    for i in range(NUM_TASK):
        sm[i] = random.randint(start_size, end_size) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        cm[i] = sm[i] * random.randint(min_cycle, max_cycle)  # 200 cycles per bit (Gcycls 단위로 변환)
        dm[i] = random.randint(DELAY_MIN, DELAY_MAX) / 10

        print("TASK", i + 1, " Size:", sm[i] * 1E3, "Mbits", "Cycles:", cm[i], "Delay:", dm[i], "sec")


def make_task2(start_size, end_size, min_cycle, max_cycle, num_task=NUM_TASK):
    for i in range(NUM_TASK):
        if i < 3:
            sm[i] = random.randint(3, 5) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        elif i < 6:
            sm[i] = random.randint(6, 8) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        else:
            sm[i] = random.randint(9, 10) / 1E3  # 1~20Mbits (Gbits 단위로 변환)
        cm[i] = sm[i] * random.randint(min_cycle, max_cycle)  # 200 cycles per bit (Gcycls 단위로 변환)
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
        print("TASK", i + 1, " Size:", sm[i] * 1E3, "Mbits", "Cycles:", cm[i])


def proposed_algorithm2(k=FU, fb=0, lcoa_mode=1, simul_time=0, num_task=NUM_TASK, last_task=0, distance=MAX_LCOA,
                        Delay_Constraint=1):
    remain_system_cost1 = 0
    remain_system_cost2 = 0

    time_t = simul_time
    time_t = simul_time
    NUM_TASK = num_task - last_task

    sm2 = np.ones((NUM_TASK, NUM_UAV))
    cm2 = np.ones((NUM_TASK, NUM_UAV))
    dm2 = np.ones((NUM_TASK, NUM_UAV))

    for i in range(NUM_TASK):
        sm2[i] = sm[i + last_task]
        cm2[i] = cm[i + last_task]
        dm2[i] = dm[i + last_task]

    tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

    bus_simul = []
    MAX_BUS = len(buses_original2)

    if lcoa_mode == 1:
        MAX_LCOA = distance
    else:
        MAX_DISTANCE = distance

    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            if buses_original2[j].timetable[time_t] == 1:

                abs_x = abs(uavs_original[0].x - buses_original2[j].x)
                abs_y = abs(uavs_original[0].y - buses_original2[j].y)
                DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original2[j].x, buses_original2[j].y, 0))

                if lcoa_mode:
                    if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                        bus_simul.append(buses_original2[j])

                # else:
                # if DMAT[i][j] <= MAX_DISTANCE:
                # bus_simul.append(buses_original2[j])

    NUM_BUS = len(bus_simul)
    # print("버스대수 : ", NUM_BUS)

    # mode=2 실험을 위해서 필요함(삭제하면 안됨)
    # bus_simul[0].cpu = 15
    # bus_simul[0].x = 740
    # bus_simul[0].y = 590

    # bus_simul[1].cpu = 13
    # bus_simul[1].x = 450
    # bus_simul[1].y = 450

    # bus_simul[2].cpu = 10
    # bus_simul[3].cpu = 15

    if NUM_BUS == 0:
        print("UAV 주위에 버스가 없음")
        return None, None, None, None, None, None, None, None, None, NUM_BUS, 0

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
            # print("BUS ID:", bus_simul[j].id, ", CPU:", bus_simul[j].cpu, ", 위치:", (bus_simul[j].x, bus_simul[j].y), ", 거리 :", round(Distance[i][j]), "m", ", 전송률 :", round(R_ub[i][j] * 1000, 2), "Mbps")

    solution = np.ones(NUM_TASK)

    for t_count in range(NUM_TASK, 0, -1):

        rho_um = cvx.Variable([t_count, NUM_UAV], pos=True)
        rho_bm = cvx.Variable([t_count, NUM_BUS], pos=True)
        fum = cvx.Variable([t_count, NUM_UAV])
        fbm = cvx.Variable([t_count, NUM_BUS])
        mum = cvx.Variable(t_count)

        FU = k
        FB = 0

        for b in range(NUM_BUS):
            FB += bus_simul[b].cpu

        t_um = [0 for _ in range(t_count)]
        t_bm = [0 for _ in range(t_count)]
        t_tx = [0 for _ in range(t_count)]

        rho_um_k = np.ones((t_count, NUM_UAV))
        rho_bm_k = np.ones((t_count, NUM_BUS)) / NUM_BUS
        # f_u_k = np.ones((t_count, NUM_UAV)) * FU / t_count
        if k <= CPU_MIN:
            f_u_k = np.ones((t_count, NUM_UAV)) * FU / t_count
        else:
            f_u_k = np.ones((t_count, NUM_UAV))
        f_b_k = np.ones((t_count, NUM_BUS))
        for b in range(NUM_BUS):
            f_b_k[:, b:b + 1:] = bus_simul[b].cpu / t_count
        mum_k = np.ones(t_count)

        loop = 1

        while (loop <= LOOP_COUNT):

            P2_energy_cost = 0
            P2_time_cost = 0

            e_um_cost = [0 for _ in range(t_count)]
            e_tx_cost = [0 for _ in range(t_count)]
            t_um_cost = [0 for _ in range(t_count)]
            t_bm_cost = [0 for _ in range(t_count)]
            t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(t_count)]

            for m in range(t_count):
                for u in range(NUM_UAV):
                    for b in range(NUM_BUS):
                        # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                        # t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                        # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                        e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm2[m] / R_ub[u][b]

                        # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                        # t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                        t_bus_cost[m][b] = rho_bm[m, b] * sm2[m] / R_ub[u][b] + cm2[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                        power(inv_pos(f_b_k[m, b]), 3) * (
                                        inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                    # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                    t_um_cost[m] = cm2[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                    # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                    e_um_cost[m] = epsilon_u * cm2[m] * (
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

            for m in range(t_count):
                constraints += [0 <= mum[m]]
                if (Delay_Constraint == 1):
                    constraints += [mum[m] <= dm2[m]]
                constraints += [mum[m] >= t_um_cost[m]]
                for b in range(NUM_BUS):
                    constraints += [mum[m] >= t_bus_cost[m][b]]

            prob = cvx.Problem(obj, constraints)
            result = prob.solve(solver=ECOS)

            if (rho_um.value is None):
                solution[t_count - 1] = 0
                break

            rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
            rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
            f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
            f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
            mum_k = lamda * mum.value + (1 - lamda) * mum_k

            loop += 1

        if solution[t_count - 1] == 1:
            if t_count < NUM_TASK:  # 모든 task를 처리하지 못했다면

                for r in range(NUM_TASK, t_count, -1):  # 처리하지 못한 task에 대해 system cost를 더하기
                    remain_system_cost1 += 1

                cm_remain_task = np.sum(cm[t_count:NUM_TASK:])
                delay_remain_task = cm_remain_task / FU
                energy_remain_task = epsilon_u * delay_remain_task * FU ** 3
                remain_system_cost2 = round(omega1 * delay_remain_task + omega2 * energy_remain_task, 3)

            return bus_simul, result, remain_system_cost1, remain_system_cost2, rho_um, rho_bm, fum, fbm, mum, NUM_BUS, t_count


def uav_only_algorithm(k=FU, fb=0, lcoa_mode=1, simul_time=0, num_task=NUM_TASK, last_task=0, distance=MAX_LCOA,
                       Delay_Constraint=1):
    remain_system_cost1 = 0
    remain_system_cost2 = 0

    FU = k
    time_t = simul_time
    NUM_TASK = num_task - last_task

    sm2 = np.ones((NUM_TASK, NUM_UAV))
    cm2 = np.ones((NUM_TASK, NUM_UAV))
    dm2 = np.ones((NUM_TASK, NUM_UAV))

    for i in range(NUM_TASK):
        sm2[i] = sm[i + last_task]
        cm2[i] = cm[i + last_task]
        dm2[i] = dm[i + last_task]

    tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

    solution = np.ones(NUM_TASK)

    for t_count in range(NUM_TASK, 0, -1):
        rho_um = cvx.Variable([t_count, NUM_UAV], pos=True)
        fum = cvx.Variable([t_count, NUM_UAV])
        mum = cvx.Variable(t_count)

        t_um = [0 for _ in range(NUM_TASK)]

        rho_um_k = np.ones((t_count, NUM_UAV))
        # f_u_k = np.ones((t_count, NUM_UAV))
        if k <= CPU_MIN:
            f_u_k = np.ones((t_count, NUM_UAV)) * FU / t_count
        else:
            f_u_k = np.ones((t_count, NUM_UAV))

        mum_k = np.ones(t_count)

        for m in range(t_count):
            for u in range(NUM_UAV):
                t_um[m] += rho_um_k[m, u] * cm2[m] / f_u_k[m, u]
            mum_k[m] = mum_k[m] * t_um[m]

        loop = 1

        while (loop <= LOOP_COUNT):

            e_um_cost = [0 for _ in range(t_count)]
            t_um_cost = [0 for _ in range(t_count)]

            P2_energy_cost = 0
            P2_time_cost = 0

            for m in range(t_count):
                for u in range(NUM_UAV):
                    # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                    t_um_cost[m] = cm2[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                    # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                    e_um_cost[m] = epsilon_u * cm2[m] * (
                        (rho_um[m, u] * f_u_k[m, u] ** 2 + rho_um_k[m, u] * fum[m, u] ** 2)) + (
                                           0.5 * tou_rho_um[m, u] * (rho_um[m, u] - rho_um_k[m, u]) ** 2) + (
                                           0.5 * tou_f_um[m, u] * (fum[m, u] - f_u_k[m, u]) ** 2)

                P2_time_cost += omega1 * mum[m]
                P2_energy_cost += omega2 * (e_um_cost[m])

            P2 = P2_time_cost + P2_energy_cost
            obj = cvx.Minimize(P2)

            constraints = \
                [0 <= fum, cvxpy.sum(fum) <= FU] + \
                [rho_um == 1]

            for m in range(t_count):
                constraints += [0 <= mum[m]]
                constraints += [mum[m] >= t_um_cost[m]]
                if (Delay_Constraint == 1):
                    constraints += [mum[m] <= dm2[m]]

            prob = cvx.Problem(obj, constraints)
            result = prob.solve(solver=ECOS)

            if (rho_um.value is None):
                solution[t_count - 1] = 0
                break

            rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
            f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
            mum_k = lamda * mum.value + (1 - lamda) * mum_k

            loop += 1

        if solution[t_count - 1] == 1:
            remain_system_cost1 = 0
            remain_system_cost2 = 0

            for r in range(NUM_TASK, t_count, -1):  # 처리하지 못한 task에 대해 system cost를 더하기
                remain_system_cost1 += 1

            cm_remain_task = np.sum(cm[t_count:NUM_TASK:])
            delay_remain_task = cm_remain_task / FU
            energy_remain_task = epsilon_u * delay_remain_task * FU ** 3
            remain_system_cost2 = round(omega1 * delay_remain_task + omega2 * energy_remain_task, 3)

            return result, remain_system_cost1, remain_system_cost2, rho_um, fum, mum, NUM_BUS, e_um_cost, t_count


def bus_only_algorithm(k=FU, fb=0, lcoa_mode=1, simul_time=0, num_task=NUM_TASK, last_task=0, distance=MAX_LCOA,
                       Delay_Constraint=1):
    remain_system_cost1 = 0
    remain_system_cost2 = 0
    time_t = simul_time
    NUM_TASK = num_task - last_task

    sm2 = np.ones((NUM_TASK, NUM_UAV))
    cm2 = np.ones((NUM_TASK, NUM_UAV))
    dm2 = np.ones((NUM_TASK, NUM_UAV))

    for i in range(NUM_TASK):
        sm2[i] = sm[i + last_task]
        cm2[i] = cm[i + last_task]
        dm2[i] = dm[i + last_task]

    tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

    bus_simul = []
    MAX_BUS = len(buses_original2)

    if lcoa_mode == 1:
        MAX_LCOA = distance
    else:
        MAX_DISTANCE = distance

    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            if buses_original2[j].timetable[time_t] == 1:

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

    if NUM_BUS == 0:
        print("UAV 주위에 버스가 없음")
        return None, None, None, None, None, None, 0

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

    solution = np.ones(NUM_TASK)

    for t_count in range(NUM_TASK, 0, -1):

        rho_bm = cvx.Variable([t_count, NUM_BUS], pos=True)
        fbm = cvx.Variable([t_count, NUM_BUS])
        mum = cvx.Variable(t_count)

        FU = k
        FB = 0

        for b in range(NUM_BUS):
            FB += bus_simul[b].cpu

        t_bm = [0 for _ in range(t_count)]
        t_tx = [0 for _ in range(t_count)]

        rho_bm_k = np.ones((t_count, NUM_BUS)) * 1 / (NUM_BUS)
        f_b_k = np.ones((t_count, NUM_BUS)) * FB / NUM_BUS / t_count
        mum_k = np.ones(t_count)

        for m in range(t_count):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    t_bm[m] += rho_bm_k[m, b] * cm2[m] / f_b_k[m, b]
                    t_tx[m] += rho_bm_k[m, b] * sm2[m] / R_ub[u][b]

            mum_k[m] = mum_k[m] * (t_bm[m] + t_tx[m])

        loop = 1

        while (loop <= LOOP_COUNT):

            P2_energy_cost = 0
            P2_time_cost = 0

            e_tx_cost = [0 for _ in range(t_count)]
            t_bm_cost = [0 for _ in range(t_count)]
            t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(t_count)]

            for m in range(t_count):
                for u in range(NUM_UAV):
                    for b in range(NUM_BUS):
                        # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                        e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm2[m] / R_ub[u][b]

                        # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                        t_bus_cost[m][b] = rho_bm[m, b] * sm2[m] / R_ub[u][b] + cm2[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                        power(inv_pos(f_b_k[m, b]), 3) * (
                                        inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                P2_time_cost += omega1 * mum[m]
                P2_energy_cost += omega2 * (e_tx_cost[m])

            P2 = P2_time_cost + P2_energy_cost

            obj = cvx.Minimize(P2)
            constraints = \
                [0 <= rho_bm, rho_bm <= 1] + \
                [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
                [0 <= fbm]

            for b in range(NUM_BUS):
                bus_fb = bus_simul[b].cpu
                constraints += [cvxpy.sum(fbm[:, b:b + 1:], axis=0, keepdims=True) <= bus_fb]

            for m in range(t_count):
                constraints += [0 <= mum[m]]
                if (Delay_Constraint == 1):
                    constraints += [mum[m] <= dm2[m]]

                for b in range(NUM_BUS):
                    constraints += [mum[m] >= t_bus_cost[m][b]]

            prob = cvx.Problem(obj, constraints)
            result = prob.solve(solver=ECOS)

            if (rho_bm.value is None):
                solution[t_count - 1] = 0
                break

            rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
            f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
            mum_k = lamda * mum.value + (1 - lamda) * mum_k

            loop += 1

        if solution[t_count - 1] == 1:
            remain_system_cost1 = 0
            remain_system_cost2 = 0

            for r in range(NUM_TASK, t_count, -1):  # 처리하지 못한 task에 대해 system cost를 더하기
                remain_system_cost1 += 1

            cm_remain_task = np.sum(cm[t_count:NUM_TASK:])
            delay_remain_task = cm_remain_task / FU
            energy_remain_task = epsilon_u * delay_remain_task * FU ** 3
            remain_system_cost2 = round(omega1 * delay_remain_task + omega2 * energy_remain_task, 3)

            return result, remain_system_cost1, remain_system_cost2, rho_bm, fbm, mum, t_count


def fixed_algorithm(kk=FU, fb=0, lcoa_mode=1, simul_time=0, num_task=NUM_TASK, last_task=0, distance=MAX_LCOA,
                    Delay_Constraint=1):
    remain_system_cost1 = 0
    remain_system_cost2 = 0
    time_t = simul_time
    NUM_TASK = num_task - last_task

    sm2 = np.ones((NUM_TASK, NUM_UAV))
    cm2 = np.ones((NUM_TASK, NUM_UAV))
    dm2 = np.ones((NUM_TASK, NUM_UAV))

    for i in range(NUM_TASK):
        sm2[i] = sm[i + last_task]
        cm2[i] = cm[i + last_task]
        dm2[i] = dm[i + last_task]

    tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1
    bus_simul = []
    MAX_BUS = len(buses_original2)

    if lcoa_mode == 1:
        MAX_LCOA = distance
    else:
        MAX_DISTANCE = distance

    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            if buses_original2[j].timetable[time_t] == 1:

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

    if NUM_BUS == 0:
        print("UAV 주위에 버스가 없음")
        return None, None, None, None, None, None, None, None, 0

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

    solution = np.ones(NUM_TASK)

    for t_count in range(NUM_TASK, 0, -1):

        # rho_um = cvx.Variable([t_count, NUM_UAV], pos=True)
        rho_um = np.ones((t_count, NUM_UAV)) * 0.5
        rho_bm = cvx.Variable([t_count, NUM_BUS], pos=True)
        fum = cvx.Variable([t_count, NUM_UAV], pos=True)
        fbm = cvx.Variable([t_count, NUM_BUS], pos=True)
        mum = cvx.Variable(t_count, pos=True)

        FU = kk
        FB = 0

        for b in range(NUM_BUS):
            FB += bus_simul[b].cpu

        t_um = [0 for _ in range(t_count)]
        t_bm = [0 for _ in range(t_count)]
        t_tx = [0 for _ in range(t_count)]

        rho_um_k = np.ones((t_count, NUM_UAV)) * 0.5
        rho_bm_k = np.ones((t_count, NUM_BUS)) * 0.5 / NUM_BUS
        # f_u_k = np.ones((t_count, NUM_UAV)) * FU / t_count
        if kk <= CPU_MIN:
            f_u_k = np.ones((t_count, NUM_UAV)) * FU / t_count
        else:
            f_u_k = np.ones((t_count, NUM_UAV))
        f_b_k = np.ones((t_count, NUM_BUS))
        for b in range(NUM_BUS):
            f_b_k[:, b:b + 1:] = bus_simul[b].cpu / t_count
        mum_k = np.ones(t_count)

        loop = 1

        while (loop <= LOOP_COUNT):

            P2_energy_cost = 0
            P2_time_cost = 0

            e_um_cost = [0 for _ in range(t_count)]
            e_tx_cost = [0 for _ in range(t_count)]
            t_um_cost = [0 for _ in range(t_count)]
            t_bm_cost = [0 for _ in range(t_count)]
            t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(t_count)]

            for m in range(t_count):
                for u in range(NUM_UAV):
                    for b in range(NUM_BUS):
                        # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                        # t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                        # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                        e_tx_cost[m] += rho_bm[m, b] * P_ub[u][b] * sm2[m] / R_ub[u][b]

                        # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                        # t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                        t_bus_cost[m][b] = rho_bm[m, b] * sm2[m] / R_ub[u][b] + cm2[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                        power(inv_pos(f_b_k[m, b]), 3) * (
                                        inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                    # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                    t_um_cost[m] = cm2[m] * (0.5 * (
                            power(rho_um[m, u] + inv_pos(fum[m, u]), 2) - power(rho_um_k[m, u], 2) - power(
                        inv_pos(f_u_k[m, u]), 2) - (rho_um_k[m, u] * (rho_um[m, u] - rho_um_k[m, u])) + power(
                        inv_pos(f_u_k[m, u]), 3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m, u]))))

                    # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                    e_um_cost[m] = epsilon_u * cm2[m] * (
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
                [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 0.5] + \
                [0 <= rho_bm, rho_bm <= 0.5]

            for b in range(NUM_BUS):
                bus_fb = bus_simul[b].cpu
                constraints += [cvxpy.sum(fbm[:, b:b + 1:], axis=0, keepdims=True) <= bus_fb]

            for m in range(t_count):
                constraints += [0 <= mum[m]]
                if (Delay_Constraint == 1):
                    constraints += [mum[m] <= dm2[m]]
                constraints += [mum[m] >= t_um_cost[m]]
                for b in range(NUM_BUS):
                    constraints += [mum[m] >= t_bus_cost[m][b]]

            prob = cvx.Problem(obj, constraints)
            if NUM_TASK == 50:
                result = prob.solve(solver=MOSEK)
            else:
                result = prob.solve(solver=ECOS)

            if (rho_bm.value is None):
                solution[t_count - 1] = 0
                break

            # rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
            # f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
            # f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
            # mum_k = lamda * mum.value + (1 - lamda) * mum_k

            loop += 1

        if solution[t_count - 1] == 1:
            remain_system_cost1 = 0
            remain_system_cost2 = 0

            for r in range(NUM_TASK, t_count, -1):  # 처리하지 못한 task에 대해 system cost를 더하기
                remain_system_cost1 += 1

            cm_remain_task = np.sum(cm[t_count:NUM_TASK:])
            delay_remain_task = cm_remain_task / FU
            energy_remain_task = epsilon_u * delay_remain_task * FU ** 3
            remain_system_cost2 = round(omega1 * delay_remain_task + omega2 * energy_remain_task, 3)

            return result, remain_system_cost1, remain_system_cost2, rho_um, rho_bm, fum, fbm, mum, t_count


def busmap(X_STEP=5, Y_STEP=5, distance=MAX_DISTANCE):
    MAX_BUS = len(buses_original)
    MAP_X_STEP = MAP_SIZE / X_STEP
    MAP_Y_STEP = MAP_SIZE / Y_STEP

    BUS_MAP = [[0 for j in range(Y_STEP)] for i in range(X_STEP)]

    for j in range(MAX_BUS):
        x = buses_original[j].x // MAP_X_STEP
        y = buses_original[j].y // MAP_Y_STEP

        BUS_MAP[int(y)][int(x)] += 1

    return BUS_MAP


def draw_map(X, Y, buses_original):
    # 버스 경로를 파일로 프린트
    fig, (ax1, ax2) = plt.subplots(ncols=2, figsize=(8, 4), gridspec_kw={"wspace": 0.0, "hspace": 0.0}, sharex=False,
                                   sharey=True, constrained_layout=True)

    filename = "map3.png"
    lonmin, lonmax, latmin, latmax = (-400, 1350, -100, 1100)  # just example

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
    # ax1.scatter(X, Y, s=150, marker="X", color='r', alpha=1)
    ax1.set_ylim((-100, 1100))
    ax1.set_xlim((-100, 1100))

    sns.kdeplot(x=x, y=y, cmap=plt.cm.GnBu, fill=True, levels=30, ax=ax2)
    # ax2.scatter(X, Y, s=150, marker="X", color='r', alpha=1)
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


def count_bus(lcoa_mode=1, simul_time=0):
    time_t = simul_time
    MAX_BUS = len(buses_original2)
    DMAT = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]
    simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            if buses_original2[j].timetable[time_t] == 1:

                abs_x = abs(uavs_original[0].x - buses_original2[j].x)
                abs_y = abs(uavs_original[0].y - buses_original2[j].y)
                DMAT[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original2[j].x, buses_original2[j].y, 0))

                if lcoa_mode:
                    if (abs_x.value <= MAX_LCOA and abs_y.value <= MAX_LCOA):
                        simul.append(buses_original2[j])

                # else:
                # if DMAT[i][j] <= MAX_DISTANCE:
                # simul.append(buses_original2[j])

    NUM_BUS = len(simul)
    return NUM_BUS