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
from cvxconfig import *


rho_um = cvx.Variable([NUM_TASK, NUM_UAV], pos=True)
rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
fum = cvx.Variable([NUM_TASK, NUM_UAV])
fbm = cvx.Variable([NUM_TASK, NUM_BUS])
mum = cvx.Variable(NUM_TASK)

tou_rho_um = np.ones((NUM_TASK, NUM_UAV)) * 1
tou_f_um = np.ones((NUM_TASK, NUM_UAV)) * 1

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

# BUS와 UAV 생성
def make_bus(num_bus=NUM_BUS):
    bus_count = 0
    paths = []

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
        buses_original.append(BUS(i, paths[i]))
        bus_count += 1

    print(buses_original)

def cal_distance(): # UAV와 BUS간의 전송률 계산

    for i in range(NUM_UAV):

        for j in range(NUM_BUS):

            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original[j].x, buses_original[j].y, 0))

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

            # 결과 출력
            print("거리 :", Distance[i][j], "m ", " 전송률 :", R_ub[i][j], "Gbps")

# TASK 생성
def make_task(start_size, end_size, min_cycle, max_cycle):

    for i in range(NUM_TASK):
        sm[i] = random.randint(start_size, end_size) / 1E3 # 1~20Mbits (Gbits 단위로 변환)
        cm[i] = sm[i] * random.randint(min_cycle, max_cycle) # 200 cycles per bit (Gcycls 단위로 변환)
        dm[i] = DELAY_MAX  # 10 seconds

        print("TASK", i+1, " Size:", sm[i]*1E3, "Mbits", "Cycles:", cm[i])

def dbm_to_watt(dbm):
    return 10 ** (dbm / 10) / 1000

def watt_to_dbm(watt):
    return 10 * math.log10(1000 * watt)

def proposed_algorithm(k=FU):

    FU = k
    t_um = 0
    t_bm = 0
    t_tx = 0
    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            for b in range(NUM_BUS):
                t_um += rho_um_k[m,u] * cm[m] / f_u_k[m,u]
                t_bm += rho_bm_k[m,b] * cm[m] / f_b_k[m,b]
                t_tx += rho_bm_k[m,b] * sm[m] / R_ub[u][b]

    if t_um > (t_bm + t_tx):
        mum_k *= t_um
    else :
        mum_k *= (t_bm + t_tx)

    loop = 1

    while (loop<=LOOP_COUNT) :

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = 0
        e_tx_cost = 0

        t_um_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]

        #t_um_cost = 0
        #t_bm_cost = 0
        t_tx_cost = 0

        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    #t_tx_cost = rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost += rho_bm[m,b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    #t_bm_cost = cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m,b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m]= cm[m] * ( 0.5 * (power(rho_um[m,u] + inv_pos(fum[m,u]),2) - power(rho_um_k[m,u],2) - power(inv_pos(f_u_k[m,u]),2) - ( rho_um_k[m,u] * (rho_um[m, u] - rho_um_k[m,u]) ) + power(inv_pos(f_u_k[m,u]),3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m,u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost = epsilon_u * cm[m] * (( rho_um[m,u] * f_u_k[m,u]**2 + rho_um_k[m,u] * fum[m,u]**2 )) + ( 0.5 * tou_rho_um[m,u] * (rho_um[m,u] - rho_um_k[m,u])**2 ) + ( 0.5 * tou_f_um[m,u] * (fum[m,u] - f_u_k[m,u])**2 )

            P2_time_cost += omega1 * mum[m]

        P2_energy_cost = omega2 * (e_um_cost + e_tx_cost)

        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <=FU] + \
            [0 <= fbm, cvxpy.sum(fbm, axis=0, keepdims=True) <=FB] + \
            [rho_um + cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= rho_um, rho_um <= 1] + \
            [0 <= rho_bm, rho_bm <= 1]
            # [mum <= dm] + \
            # [mum >= t_um_cost] + \
            # [mum >= t_bm_cost + t_tx_cost]

        for m in range(NUM_TASK):
            constraints += [mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve(solver=SCS)

        if loop % 10 == -1:
        #if 1:
            np.set_printoptions(precision=3)

            print("Iteration : ", loop)
            print("Status : ", prob.status)
            print(rho_um.value)
            print(rho_bm.value)
            print(fum.value)
            print(fbm.value)
            print(mum.value)
            print(result)
            print("")

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum

def uav_only_algorithm(k):

    FU = k
    loop = 1

    while (loop <= LOOP_COUNT):

        e_um_cost = 0
        t_um_cost = 0

        P2_energy_cost = 0
        P2_time_cost = 0

        for m in range(NUM_TASK):

            for u in range(NUM_UAV):

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost += cm[m] * inv_pos(fum[m, u])

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost += epsilon_u * cm[m] * power(fum[m, u],2)

            #P2_time_cost += omega1 * t_um_cost
            #P2_energy_cost += omega2 * e_um_cost

        P2 = omega1 * t_um_cost + omega2 * e_um_cost
        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <= FU] + \
            [t_um_cost <= DELAY_MAX]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()

        if loop % 10 == -1:
        #if 1:
            np.set_printoptions(precision=3)

            print("Iteration : ", loop)
            print("Status : ", prob.status)
            print(fum.value)
            print(result, end=" ")

        loop += 1

    return result, fum, t_um_cost

def bus_only_algorithm(k):

    t_um = 0
    t_bm = 0
    t_tx = 0

    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_TASK
    mum_k = np.ones(NUM_TASK)

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            for b in range(NUM_BUS):
                t_bm += rho_bm_k[m, b] * cm[m] / f_b_k[m, b]
                t_tx += rho_bm_k[m, b] * sm[m] / R_ub[u][b]

    mum_k *= (t_bm + t_tx)

    loop = 1

    while (loop<=LOOP_COUNT) :

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = 0
        e_tx_cost = 0

        t_um_cost = 0
        t_tx_cost = 0
        t_bm_cost = 0

        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):

            for u in range(NUM_UAV):

                for b in range(NUM_BUS):

                    ####### START (for_b) #########
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    #t_tx_cost += rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost += rho_bm[m,b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    #t_bm_cost += cm[m] * (0.5 * ( power(rho_bm[m, u],2) + power(inv_pos(fbm[m, u]),2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m, u] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m, u]) - inv_pos(f_b_k[m, b])))))
                    #t_bm_cost += cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))
                    ####### END (for_b) ###########

            P2_time_cost += omega1 * mum[m]

        P2_energy_cost = omega2 * (e_tx_cost)
        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fbm, cvxpy.sum(fbm, axis=0, keepdims=True) <=FB] + \
            [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= rho_bm, rho_bm <= 1]

        for m in range(NUM_TASK):
            constraints += [mum[m] <= dm[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()

        if loop % 10 == -1:
        #if 1:
            np.set_printoptions(precision=3)

            print("Iteration : ", loop)
            print("Status : ", prob.status)
            print(rho_bm.value)
            print(fbm.value)
            print(mum.value)
            print(result)
            print("")

        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_bm, fbm, mum

def fixed_algorithm(k):

    t_um = 0
    t_bm = 0
    t_tx = 0
    FU = k
    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_TASK
    mum_k = DELAY_MAX

    loop = 1

    while (loop<=LOOP_COUNT) :

        P2_energy_cost = 0
        P2_time_cost = 0

        e_um_cost = 0
        e_tx_cost = 0

        t_um_cost = [0 for _ in range(NUM_TASK)]
        t_bm_cost = [0 for _ in range(NUM_TASK)]

        # t_um_cost = 0
        # t_bm_cost = 0
        t_tx_cost = 0

        t_bus_cost = [[0 for _ in range(NUM_BUS)] for _ in range(NUM_TASK)]

        for m in range(NUM_TASK):
            for u in range(NUM_UAV):
                for b in range(NUM_BUS):


                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    #t_tx_cost += rho_bm[m,b] * sm[m] / R_ub[u][b]

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost += rho_bm[m,b] * P_ub[u][b] * sm[m] / R_ub[u][b]

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    #t_bm_cost += cm[m] * (0.5 * ( power(rho_bm[m, u],2) + power(inv_pos(fbm[m, u]),2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m, u] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m, u]) - inv_pos(f_b_k[m, b])))))
                    #t_bm_cost += cm[m] * (0.5 * (power( rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))

                    t_bus_cost[m][b] = rho_bm[m, b] * sm[m] / R_ub[u][b] + cm[m] * (0.5 * (
                                power(rho_bm[m, b] + inv_pos(fbm[m, b]), 2) - power(rho_bm_k[m, b], 2) - power(
                            inv_pos(f_b_k[m, b]), 2) - (rho_bm_k[m, b] * (rho_bm[m, b] - rho_bm_k[m, b])) + (
                                            power(inv_pos(f_b_k[m, b]), 3) * (
                                                inv_pos(fbm[m, b]) - inv_pos(f_b_k[m, b])))))


                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost[m]= cm[m] * ( 0.5 * (power(rho_um[m,u] + inv_pos(fum[m,u]),2) - power(rho_um_k[m,u],2) - power(inv_pos(f_u_k[m,u]),2) - ( rho_um_k[m,u] * (rho_um[m, u] - rho_um_k[m,u]) ) + power(inv_pos(f_u_k[m,u]),3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m,u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost = epsilon_u * cm[m] * (( rho_um[m,u] * f_u_k[m,u]**2 + rho_um_k[m,u] * fum[m,u]**2 )) + ( 0.5 * tou_rho_um[m,u] * (rho_um[m,u] - rho_um_k[m,u])**2 ) + ( 0.5 * tou_f_um[m,u] * (fum[m,u] - f_u_k[m,u])**2 )

            P2_time_cost += omega1 * mum[m]

        P2_energy_cost = omega2 * (e_um_cost + e_tx_cost)
        P2 = P2_time_cost + P2_energy_cost

        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <=FU] + \
            [0 <= fbm, cvxpy.sum(fbm, axis=0, keepdims=True) <=FB] + \
            [rho_um == 0.5] + \
            [cvxpy.sum(rho_bm, axis=1, keepdims=True) == 0.5]

        for m in range(NUM_TASK):
            constraints += [mum[m] <= dm[m]]
            constraints += [mum[m] >= t_um_cost[m]]
            for b in range(NUM_BUS):
                constraints += [mum[m] >= t_bus_cost[m][b]]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve()

        if loop % 10 == -1:
        #if 1:
            np.set_printoptions(precision=3)

            print("Iteration : ", loop)
            print("Status : ", prob.status)
            print(rho_um.value)
            print(rho_bm.value)
            print(fum.value)
            print(fbm.value)
            print(mum.value)
            print(result)
            print("")

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum

def ratio_graph(rho_um, rho_bm, Distance):

    # creating the dataset
    xaxis = np.arange(1, NUM_TASK+1, 1)
    x = np.zeros((NUM_TASK, NUM_UAV))
    y = np.zeros((NUM_TASK, NUM_BUS))

    for m in range(NUM_TASK):
        for b in range(NUM_BUS):
            y[m][b] = rho_bm[m][b].value

    for m in range(NUM_TASK):
        for u in range(NUM_UAV):
            x[m][u] = rho_um[m][u].value

    y1 = y.transpose()
    x1 = x.transpose()

    plt.bar(xaxis, x1[0], label="UAV")
    bottom = x1[0]

    for b in range(NUM_BUS):
        plt.bar(xaxis, y1[b], bottom=bottom, label="BUS" + str(b) + " : " + str(round(Distance[0][b],1)) +"m")
        bottom += y1[b]

    plt.legend(loc='best')
    plt.legend(frameon=True)

    plt.xlabel("Task")
    plt.ylabel("Offloading ratio.")
    plt.xticks(xaxis)
    plt.show()

def proposed2(NUM_BUS=NUM_BUS, k=FU):

    Distance_MAX = [[0 for j in range(MAX_BUS)] for i in range(NUM_UAV)]

    bus_simul = []

    for i in range(NUM_UAV):
        for j in range(MAX_BUS):

            Distance_MAX[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (buses_original[j].x, buses_original[j].y, 0))

            if Distance_MAX[i][j] <= 500:
                bus_simul.append(buses_original[j])

    NUM_BUS = len(bus_simul)

    Distance = [[0 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    P_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 전송 파워 (W)
    R_ub = [[1 for j in range(NUM_BUS)] for i in range(NUM_UAV)]
    W_ub = [[BANDWIDTH for j in range(NUM_BUS)] for i in range(NUM_UAV)]  # 대역폭 (Hz)

    for i in range(NUM_UAV):
        for j in range(NUM_BUS):
            Distance[i][j] = math.dist((uavs_original[0].x, uavs_original[0].y, uavs_original[0].z),
                                       (bus_simul[j].x, bus_simul[j].y, 0))

            # 전송률 계산 (Shannon Capacity 공식 사용)
            SNR = (P_ub[i][j] * alpha_0) / (Noise * Distance[i][j] ** 2)
            R_ub[i][j] = W_ub[i][j] * math.log2(1 + SNR) / 1E9  # Gbps

            # 결과 출력
            print("BUS ID:", j, "거리 :", Distance[i][j], "m ", " 전송률 :", R_ub[i][j], "Gbps")

    rho_um = cvx.Variable([NUM_TASK, NUM_UAV], pos=True)
    rho_bm = cvx.Variable([NUM_TASK, NUM_BUS], pos=True)
    fum = cvx.Variable([NUM_TASK, NUM_UAV])
    fbm = cvx.Variable([NUM_TASK, NUM_BUS])
    mum = cvx.Variable()

    FU = k
    rho_um_k = np.ones((NUM_TASK, NUM_UAV)) * 1 / (NUM_UAV + NUM_BUS)
    rho_bm_k = np.ones((NUM_TASK, NUM_BUS)) * 1 / (NUM_UAV + NUM_BUS)
    f_u_k = np.ones((NUM_TASK, NUM_UAV)) * FU / NUM_TASK
    f_b_k = np.ones((NUM_TASK, NUM_BUS)) * FB / NUM_TASK
    mum_k = DELAY_MAX

    loop = 1

    while (loop<=LOOP_COUNT) :

        e_um_cost = 0
        e_tx_cost = 0

        t_um_cost = 0
        t_tx_cost = 0
        t_bm_cost = 0

        for m in range(NUM_TASK):

            for u in range(NUM_UAV):

                for b in range(NUM_BUS):

                    ####### START (for_b) #########
                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 걸리는 시간 (t_tx) 계산 : 식(9)
                    t_tx_cost += rho_bm[m,b] * sm[m] / R_ub[u][b]
                    #t_tx_cost += rho_bm[m,b] * sm[m] / (W_ub[u, b] * math.log2(1 + SNR[u][b]) / 1E9)

                    # task m 처리를 위해 UAV -> bus b로 데이터를 보내는 데 필요한 에너지(e_tx) 계산 : 식(10)
                    e_tx_cost += rho_bm[m,b] * P_ub[u][b] * sm[m] / R_ub[u][b]
                    #e_tx_cost += rho_bm[m, u] * P_ub[u][b] * sm[m] / (W_ub[u, b] * math.log2(1 + SNR[u][b]) / 1E9)

                    # bus b가 task m 처리를 위해 걸리는 시간 (t~_bm) 계산 : 식(21)
                    #t_bm_cost += cm[m] * (0.5 * ( power(rho_bm[m, u],2) + power(inv_pos(fbm[m, u]),2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m, u] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m, u]) - inv_pos(f_b_k[m, b])))))
                    t_bm_cost += cm[m] * (0.5 * (power( rho_bm[m,b] + inv_pos(fbm[m,b]), 2) - power(rho_bm_k[m, b],2) - power(inv_pos(f_b_k[m, b]),2) - (rho_bm_k[m, b] * (rho_bm[m,b] - rho_bm_k[m, b])) + (power(inv_pos(f_b_k[m, b]),3) * (inv_pos(fbm[m,b]) - inv_pos(f_b_k[m, b])))))
                    ####### END (for_b) ###########

                # UAV가 task m 처리를 위해 걸리는 시간 (t~_um) 계산 : 식(19)
                t_um_cost+= cm[m] * ( 0.5 * (power(rho_um[m,u] + inv_pos(fum[m,u]),2) - power(rho_um_k[m,u],2) - power(inv_pos(f_u_k[m,u]),2) - ( rho_um_k[m,u] * (rho_um[m, u] - rho_um_k[m,u]) ) + power(inv_pos(f_u_k[m,u]),3) * (inv_pos(fum[m, u]) - inv_pos(f_u_k[m,u]))))

                # UAV가 task m 처리를 위해 필요한 에너지 (e~_um) 계산 : 식(16)
                e_um_cost+= epsilon_u * cm[m] * (( rho_um[m,u] * f_u_k[m,u]**2 + rho_um_k[m,u] * fum[m,u]**2 )) + ( 0.5 * tou_rho_um[m,u] * (rho_um[m,u] - rho_um_k[m,u])**2 ) + ( 0.5 * tou_f_um[m,u] * (fum[m,u] - f_u_k[m,u])**2 )

            #mum[m] = max(t_um_cost, t_bm_cost + t_tx_cost)
            #P2_time_cost += omega1 * (t_tx_cost + t_bm_cost)

            #P2_time_cost += omega1 * mum[m]
            #P2_energy_cost += omega2 * (e_um_cost + e_tx_cost)

        P2 = omega1 * mum + omega2 * (e_um_cost + e_tx_cost)
        obj = cvx.Minimize(P2)
        constraints = \
            [0 <= fum, cvxpy.sum(fum) <=FU] + \
            [0 <= fbm, cvxpy.sum(fbm, axis=0, keepdims=True) <=FB] + \
            [rho_um + cvxpy.sum(rho_bm, axis=1, keepdims=True) == 1] + \
            [0 <= rho_um, rho_um <= 1] + \
            [0 <= rho_bm, rho_bm <= 1] + \
            [mum >= t_um_cost] + \
            [mum >= t_bm_cost + t_tx_cost]

        prob = cvx.Problem(obj, constraints)
        result = prob.solve(solver=SCS)

        if loop % 10 == -1:
        #if 1:
            np.set_printoptions(precision=3)

            print("Iteration : ", loop)
            print("Status : ", prob.status)
            print(rho_um.value)
            print(rho_bm.value)
            print(fum.value)
            print(fbm.value)
            print(mum.value)
            print(result)
            print("")

        rho_um_k = lamda * rho_um.value + (1 - lamda) * rho_um_k
        rho_bm_k = lamda * rho_bm.value + (1 - lamda) * rho_bm_k
        f_u_k = lamda * fum.value + (1 - lamda) * f_u_k
        f_b_k = lamda * fbm.value + (1 - lamda) * f_b_k
        mum_k = lamda * mum.value + (1 - lamda) * mum_k

        loop += 1

    return result, rho_um, rho_bm, fum, fbm, mum