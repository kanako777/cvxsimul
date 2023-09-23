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

# MODE = 0 : UAV와 BUS의 ratio 실험
# MODE = 1 : UAV의 CPU를 증가시켜가며 실험
# MODE = 2 : TASK의 data size를 증가시켜가며 테스트
# MODE = 3 : BUS 1대의 CPU를 증가시켜가며 실험
# MODE = 4 : omega1를 증가시켜가며 실험
# MODE = 5 : UAV가 버스를 찾는 거리를 증가시키면서 시뮬레이션
# MODE = 6 : draw map

MODE = 0
REAL = 1
BUS_NUM = 5

if MODE == 0: # UAV와 BUS의 ratio 실험

    make_bus(REAL,BUS_NUM)
    make_task(5, 10, 200, 200)

    #result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘
    result, rho_um, rho_bm, fum, fbm, mum, num_bus1 = proposed_algorithm2(FU)  # 제안 알고리즘

    print(rho_um.value, fum.value, rho_bm.value, fbm.value)
    print(mum.value)

    NUM_BUS = num_bus1
    xaxis = np.arange(1, NUM_TASK + 1, 1)

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

    # marker = itertools.cycle(('+', '2', '.', 'x'))
    # plt.style.use(['science', 'ieee', 'no-latex'])

    plt.bar(xaxis, x1[0], label="UAV")
    bottom = x1[0]

    for b in range(NUM_BUS):
        plt.bar(xaxis, y1[b], bottom=bottom, label="BUS" + str(b + 1) + " : " + str(buses_original[b].cpu) + "GHz")
        bottom += y1[b]

    # plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    # plt.legend(loc='upper center', bbox_to_anchor=(0.5, 1.05), ncol=5, fancybox=True, shadow=True)

    plt.xlabel("Task")
    plt.ylabel("Optimal Task Offloading ratio.")
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.xticks(xaxis)
    plt.savefig("./graphs/" + "TASK_BUS_RATIO" + str(REAL))
    plt.clf()

    draw_map(X, Y, buses_original)


if MODE == 1: # UAV의 CPU를 증가시켜가며 실험

    make_bus(REAL,BUS_NUM)
    make_task(10, 20, 100, 200)

    STEP = (FU_MAX + 1)//3
    system_cost = np.zeros((4, STEP))
    k_index = 0

    x = np.arange(3, FU_MAX + 1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(3, FU_MAX+1, 3):

        print("STEP : ", k_index+1, "UAV CPU : ", k)
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm2(k) # 제안 알고리즘
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
    plt.savefig("./graphs/" + "UAV_CPU_COST" + str(REAL))
    plt.clf()

if MODE == 2: # TASK의 data size를 증가시켜가며 테스트

    make_bus(REAL, NUM_BUS)
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

        print("UAV가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000)
        print("버스가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000)

        k_index += 1

    y = bus_data.transpose()

    plt.plot(x, uav_data, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y[b], marker=next(marker), label="BUS"+str(b+1)  + " : " + str(buses_original[b].cpu) +"GHz")

    plt.yscale('symlog')
    plt.xticks(x)
    plt.ylim([0.1, 100])
    plt.xlabel('Total Amount of data(Mbit)')
    plt.ylabel('The Amount of data processed(Mbit)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "TASK_SIZE_BIT")
    plt.clf()

if MODE == 3: # BUS 1대의 CPU를 증가시켜가며 실험

    make_bus(REAL, NUM_BUS)
    cal_distance()
    make_task(10, 20, 100, 200)

    STEP = (FB_MAX + 1)//3 - 1
    system_cost = np.zeros(STEP)
    uav_ratio = np.zeros(STEP)
    bus_ratio = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(6, FB_MAX + 1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(6, FB_MAX+1, 3):

        print("STEP : ", k_index+1, "BUS1 CPU : ", k)
        buses_original[0].cpu = k
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(FU) # 제안 알고리즘

        print(cvxpy.sum(rho_um1).value/10)
        uav_ratio[k_index] = cvxpy.sum(rho_um1).value/10
        for b in range(NUM_BUS):
            bus_ratio[k_index][b] = cvxpy.sum(rho_bm1[:,b:b+1:1]).value/10

        system_cost[k_index] = result1

        print("System Cost : ", system_cost[k_index])
        print("UAV Ratio : ", uav_ratio[k_index])
        print("Bus Ratio : ", bus_ratio[k_index])

        k_index += 1

    y = bus_ratio.transpose()

    plt.plot(x, uav_ratio, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        if b==0:
            plt.plot(x, y[b], marker=next(marker), label="BUS" + str(b + 1))
        else:
            plt.plot(x, y[b], marker=next(marker), label="BUS" + str(b + 1)  + " : " + str(buses_original[b].cpu) +"GHz")

    plt.xticks(x)
    plt.xlabel('Computing Resource of Bus1 (GHz)')
    plt.ylabel('Optimal Task offloading ratio')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "BUS_CPU_RATIO")
    plt.clf()

if MODE == 4: # omega1를 증가시켜가며 실험

    make_bus(REAL, NUM_BUS)
    cal_distance()
    make_task(10, 20, 100, 200)

    STEP = 10
    system_cost = np.zeros(STEP)
    uav_ratio = np.zeros(STEP)
    bus_ratio = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(1, 11, 1)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(1, 11, 1):

        print("STEP : ", k_index+1, "Omega1 : ", k)

        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(FU, k) # 제안 알고리즘

        print(cvxpy.sum(rho_um1).value/10)
        uav_ratio[k_index] = cvxpy.sum(rho_um1).value/10
        for b in range(NUM_BUS):
            bus_ratio[k_index][b] = cvxpy.sum(rho_bm1[:,b:b+1:1]).value/10

        system_cost[k_index] = result1

        print("System Cost : ", system_cost[k_index])
        print("UAV Ratio : ", uav_ratio[k_index])
        print("Bus Ratio : ", bus_ratio[k_index])

        k_index += 1

    y = bus_ratio.transpose()

    plt.plot(x, uav_ratio, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y[b], marker=next(marker), label="BUS" + str(b + 1)  + " : " + str(buses_original[b].cpu) +"GHz")

    plt.xticks(x)
    plt.xlabel('$\\omega_1$')
    plt.ylabel('Optimal Task offloading ratio')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "OMEGA_RATIO")
    plt.clf()

if MODE == 5: # UAv가 버스를 찾는 거리를 증가시키면서 시뮬레이션

    make_bus(REAL, MAX_BUS)
    make_task(10, 20, 100, 200)

    system_cost = np.zeros(SIMUL_TIME)
    bus_ratio = np.zeros(SIMUL_TIME)
    y_bus_num = np.zeros(SIMUL_TIME)
    k_index = 0

    x = np.arange(1, SIMUL_TIME + 1, 1) * 50 + 150
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(SIMUL_TIME):

        print("STEP : ", k_index + 1)
        distance = k * 50 + 150
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm2(FU, distance)  # 제안 알고리즘

        bus_ratio[k_index] = 1 - cvxpy.sum(rho_um1).value / 10
        system_cost[k_index] = result1
        y_bus_num[k_index] = num_bus1

        print("System Cost : ", system_cost[k_index])
        print("Bus Ratio : ", bus_ratio[k_index])

        k_index += 1

    fig, ax1 = plt.subplots()
    color_1 = 'tab:red'
    ax1.set_xlabel('Length of $\L_{CoA}$ (m)')
    ax1.set_ylabel('Offloading ratio to buses', color=color_1)
    ax1.plot(x, bus_ratio, marker=next(marker), color=color_1)
    ax1.tick_params(axis='y', labelcolor=color_1)

    # right side with different scale
    ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
    color_2 = 'tab:blue'
    ax2.set_ylabel('System Cost', color=color_2)
    ax2.plot(x, system_cost, marker=next(marker), color=color_2)
    ax2.tick_params(axis='y', labelcolor=color_2)

    fig.tight_layout()
    plt.savefig("./graphs/" + "DISTANCE_RATIO")
    plt.clf()


if MODE == 10: # 버스를 이동하면서 시뮬레이션

    make_bus()
    cal_distance()
    make_task(10, 20, 100, 200)

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
