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
from cvxutil import *
import scienceplots
import itertools


# UAV 생성
for i in range(NUM_UAV):
    uavs_original.append(UAV(i, X, Y, Z))

MODE = 2

if MODE == 0: # UAV와 BUS의 ratio 실험

    make_bus(MAX_BUS)
    cal_distance()
    make_task(1, 5, 100, 200)

    result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘
    #result, rho_um, rho_bm, fum, fbm, mum = proposed2(FU)  # equal 알고리즘

    print(rho_um.value, fum.value, rho_bm.value, fbm.value)
    print(mum.value)
    ratio_graph(rho_um, rho_bm, Distance) # UAV와 BUS의 ratio 그래프 그리기


if MODE == 1: # UAV의 CPU를 증가시켜가며 실험

    make_bus()
    cal_distance()
    make_task(10, 20, 100, 200)

    STEP = (FU_MAX + 1)//3
    system_cost = np.zeros((4, STEP))
    k_index = 0

    x = np.arange(3, FU_MAX + 1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(3, FU_MAX+1, 3):

        print("STEP : ", k_index+1, "UAV CPU : ", k)
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(k) # 제안 알고리즘
        result2, fum2, t_um_cost = uav_only_algorithm(k) # uav only 알고리즘
        result3, rho_bm3, fbm3, mum3 = bus_only_algorithm(k)  # bus only 알고리즘
        result4, rho_um4, rho_bm4, fum4, fbm4, mum4 = fixed_algorithm(k)  # fixed 알고리즘

        system_cost[0][k_index] = result1
        system_cost[1][k_index] = result2
        system_cost[2][k_index] = result3
        system_cost[3][k_index] = result4

        print("Proposed : ", system_cost[0][k_index], "UAV Only : ", system_cost[1][k_index], "Bus Only : ", system_cost[2][k_index], "Fixed : ", system_cost[3][k_index])
        k_index += 1

    plt.plot(x, system_cost[0], marker=next(marker), label="Proposed")
    plt.plot(x, system_cost[1], marker=next(marker), label="UAV Only")
    plt.plot(x, system_cost[2], marker=next(marker), label="Bus Only")
    plt.plot(x, system_cost[3], marker=next(marker), label="Fixed UAV-Bus")

    plt.xticks(x)
    plt.xlabel('Computing Resource of UAV (GHz)')
    plt.ylabel('System Cost')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "UAV_CPU_COST")
    plt.clf()

if MODE == 2: # TASK의 data size를 증가시켜가며 테스트

    make_bus()
    cal_distance()

    STEP = (TASK_SIZE_MAX + 1) // 10
    uav_data = np.zeros(STEP)
    bus_data = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(10, TASK_SIZE_MAX + 1, 10)*2
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(10, TASK_SIZE_MAX + 1, 10):

        make_task(k/5, k/5, 200, 200)

        print("STEP : ", k_index+1)
        result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘

        uav_data[k_index] = cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000

        for b in range(NUM_BUS):
            bus_data[k_index] = cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000

        np.set_printoptions(precision=3)

        print(uav_data[k_index])
        print(bus_data[k_index])

        print(cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000)
        print(cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000)

        k_index += 1

    y = bus_data.transpose()

    plt.plot(x, uav_data, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y[b], marker=next(marker), label="BUS"+str(b+1))

    plt.yscale('symlog')
    plt.xticks(x)
    plt.ylim([0.1, 100])
    plt.xlabel('Total Amount of data(Mbit)')
    plt.ylabel('The Amount of data processed(Mbit)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "TASK_SIZE_BIT")
    plt.clf()


if MODE == 3: # 버스를 이동하면서 시뮬레이션

    make_bus()
    cal_distance()
    make_task(1, 5, 100, 200)

    system_cost = [0 for i in range(SIMUL_TIME)]
    k_index = 0

    for k in range(SIMUL_TIME):

        print("STEP : ", k_index + 1)
        result1, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘
        result2, fum2 = uav_only_algorithm(FU)  # uav only 알고리즘
        result3, rho_bm3, fbm3, mum3 = bus_only_algorithm(FU)  # bus only 알고리즘
        result4, rho_um4, rho_bm4, fum4, fbm4, mum4 = fixed_algorithm(FU)  # bus only 알고리즘

        system_cost[k_index] = result1, result2, result3, result4
        print("Proposed : ", system_cost[k_index][0], "UAV Only : ", system_cost[k_index][1], "Bus Only : ",
              system_cost[k_index][2], "Equal Ratio : ", system_cost[k_index][3])
        k_index += 1

        for bus in buses_original:
            bus.move()
        cal_distance()


if MODE == 4: # 버스를 이동하면서 시뮬레이션

    make_bus(MAX_BUS)
    make_task(1, 5, 100, 200)
    system_cost = [0 for i in range(SIMUL_TIME)]
    k_index = 0

    for k in range(SIMUL_TIME):

        print("STEP : ", k_index + 1)
        result, rho_um, rho_bm, fum, fbm, mum = proposed2(MAX_BUS, FU)  # 제안 알고리즘

        system_cost[k_index] = result
        print("Proposed : ", system_cost[k_index])
        k_index += 1

        for bus in buses_original:
            bus.move()

