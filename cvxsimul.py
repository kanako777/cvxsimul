import sys
import matplotlib.colors as colors
import matplotlib.cm as cm
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

from mpl_toolkits.mplot3d import Axes3D

# UAV 생성
# MODE = 0 : UAV와 BUS의 ratio 실험 (버스대수는 4대, 태스크=10개(5,10,50,100), 딜레이는 1~2로 설정)
# MODE = 1 : UAV의 CPU를 증가 (버스대수는 3대, 태스크=10개(5,10,100,150), 딜레이는 20~30으로 설정)
# MODE = 2 : TASK의 data size를 증가 (버스대수는 4대, 태스크=10개(5,10,100,150), 딜레이는 1~2로 설정)
# MODE = 3 : BUS 1대의 CPU를 증가 (버스대수=3, 태스크=10개(5,10,100,150), 딜레이는 1~2로 설정)
# MODE = 4 : omega1를 증가 (버스대수는 4대, 태스크=1개(5,10,100,150), 딜레이는 1~2로 설정)
# MODE = 5 : UAV가 버스를 찾는 거리를 증가시키면서 시뮬레이션
# MODE = 6 : draw map

MODE = 1
REAL = 1

if MODE == 0: # UAV와 BUS의 ratio 실험

    # 버스대수는 4대, 태스크=(5,10,50,100), 딜레이는 1~2로 설정

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))

    make_bus(REAL,NUM_BUS)
    make_task2(5, 10, 50, 150)

    #result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘
    result, rho_um, rho_bm, fum, fbm, mum, num_bus1 = proposed_algorithm2(FU)  # 제안 알고리즘

    #print(rho_um.value, fum.value, rho_bm.value, fbm.value)
    #print(mum.value)

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

    marker = itertools.cycle(('+', '2', '.', 'x'))
    #plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])

    plt.bar(xaxis, x1[0], label=r'$\rho_{u,m}$')
    bottom = x1[0]

    for b in range(NUM_BUS):
        #plt.bar(xaxis, y1[b], bottom=bottom, label=r'$\rho$' + str(b + 1) + r'$,m$')
        plt.bar(xaxis, y1[b], bottom=bottom, label=r'$\rho_{{%d},m}$' % (b+1))

        bottom += y1[b]

    plt.xlabel("Tasks ($m$)")
    plt.ylabel("Optimal task offloading ratio")
    plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.xticks(xaxis)
    plt.tight_layout()
    plt.savefig("./graphs/" + "TASK_BUS_RATIO" + str(REAL))
    plt.clf()

    f = open('./graphs/TASK_BUS_RATIO.txt', 'w')

    print("Task List : ", file=f)
    for i in range(NUM_TASK):
        print("Task" + str(i+1) + ":", str(sm[i]*1000)+"Mbits", file=f)
    print("Bus CPU : ", file=f)
    for i in range(NUM_BUS):
        print("BUS" + str(i+1) + ":", str(buses_original[i].cpu)+"GHz, 거리:" + str(buses_original[i].distance) +"m", file=f)


    print("Task별 UAV 처리비율 : ", rho_um.value, file=f)
    print("Task별 UAV 처리비율(평균) : ", np.mean(rho_um.value), file=f)
    print("Task별 Bus 처리비율 : ", file=f)
    for i in range(NUM_BUS):
        print("BUS", i+1, np.round(y1[i],2), file=f)
    print("Task별 Bus 처리비율(task별 합계) : ", np.round(np.sum(y1,axis=0),2), file=f)
    print("Task별 Bus 처리비율(버스별 합계) : ", np.round(np.sum(y1,axis=1),2), file=f)

    f.close()

    #draw_map(X, Y, buses_original)

if MODE == 1: # UAV의 CPU를 증가시켜가며 실험

    # 버스대수는 3대, 태스크=(5,10,100,150), 딜레이는 20~30으로 설정
    # 리얼맵 : 태스크=(3,10,50,100)

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL,NUM_BUS)
    make_task(3, 10, 50, 100)

    STEP = (FU_MAX)//3 - 2
    uav_ratio1 = np.zeros(STEP)
    bus_ratio1 = np.zeros((STEP, NUM_BUS))
    uav_ratio2 = np.zeros(STEP)
    bus_ratio2 = np.zeros((STEP, NUM_BUS))
    system_cost = np.zeros((4, STEP))
    delay_cost = np.zeros((4, STEP))
    energy_cost = np.zeros((4, STEP))

    w1=omega1
    w2=omega2
    k_index = 0

    x = np.arange(9, FU_MAX + 1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(9, FU_MAX+1, 3):

        print("STEP : ", k_index+1, "UAV CPU : ", k)

        result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm2(k) # 제안 알고리즘
        system_cost[0][k_index] = round(result1,3)
        delay_cost[0][k_index] = round(cvxpy.sum(mum1).value * w1, 3)
        energy_cost[0][k_index] = round(system_cost[0][k_index] - delay_cost[0][k_index], 3)

        uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)
        for b in range(NUM_BUS):
            bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK, 3)

        print(uav_ratio1[k_index])
        print(bus_ratio1[k_index])

        result2, fum2, t_um_cost = uav_only_algorithm(k) # uav only 알고리즘
        system_cost[1][k_index] = round(result2,3)
        delay_cost[1][k_index] = round(cvxpy.sum(t_um_cost).value * w1, 3)
        energy_cost[1][k_index] = round(system_cost[1][k_index] - delay_cost[1][k_index], 3)

        result3, rho_bm3, fbm3, mum3 = bus_only_algorithm(k)  # bus only 알고리즘
        system_cost[2][k_index] = round(result3,3)
        delay_cost[2][k_index] = round(cvxpy.sum(mum3).value * w1, 3)
        energy_cost[2][k_index] = round(system_cost[2][k_index] - delay_cost[2][k_index], 3)

        result4, rho_um4, rho_bm4, fum4, fbm4, mum4 = fixed_algorithm(k)  # fixed 알고리즘
        system_cost[3][k_index] = round(result4, 3)
        delay_cost[3][k_index] = round(cvxpy.sum(mum4).value * w1, 3)
        energy_cost[3][k_index] = round(system_cost[3][k_index] - delay_cost[3][k_index], 3)

        uav_ratio2[k_index] = round(cvxpy.sum(rho_um4).value / NUM_TASK, 3)
        for b in range(NUM_BUS):
            bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm4[:, b:b + 1:1]).value / NUM_TASK, 3)

        print(uav_ratio2[k_index])
        print(bus_ratio2[k_index])

        print("System Cost")
        print("Proposed : ", system_cost[0][k_index], "UAV Only : ", system_cost[1][k_index], "Bus Only : ", system_cost[2][k_index], "Fixed : ", system_cost[3][k_index])

        print("Delay Cost")
        print("Proposed : ", delay_cost[0][k_index], "UAV Only : ", delay_cost[1][k_index], "Bus Only : ",
              delay_cost[2][k_index], "Fixed : ", delay_cost[3][k_index])

        print("Energy Cost")
        print("Proposed : ", energy_cost[0][k_index], "UAV Only : ", energy_cost[1][k_index], "Bus Only : ",
              energy_cost[2][k_index], "Fixed : ", energy_cost[3][k_index])

        k_index += 1

    plt.plot(x, system_cost[0], marker=next(marker), label="Proposed")
    plt.plot(x, system_cost[1], marker=next(marker), label="LC")
    plt.plot(x, system_cost[2], marker=next(marker), label="FO")
    plt.plot(x, system_cost[3], marker=next(marker), label="FPO")

    plt.xticks(x)
    plt.xlabel('Computing resource of UAV, ' + r'$f_u$(GHz)')
    plt.ylabel('System cost')
    plt.legend(loc='lower left')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "UAV_CPU_COST" + str(REAL))
    plt.clf()

    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['ieee', 'std-colors', 'no-latex'])

    width = 0.5
    plt.bar(x-0.5, delay_cost[0], width, label='Proposed-Delay cost')
    plt.bar(x-0.5, energy_cost[0], width, bottom=delay_cost[0], label='Proposed-Energy cost')
    plt.bar(x, delay_cost[1], width, label='LC-Delay cost')
    plt.bar(x, energy_cost[1], width, bottom=delay_cost[1], label='LC-Energy cost')
    plt.bar(x+0.5, delay_cost[2], width, label='FO-Delay cost')
    plt.bar(x+0.5, energy_cost[2], width, bottom=delay_cost[2], label='FO-Energy cost')
    plt.bar(x+1, delay_cost[3], width, label='FPO-Delay cost')
    plt.bar(x+1, energy_cost[3], width, bottom=delay_cost[3], label='FPO-Energy cost')

    plt.xticks(x)
    plt.xlabel('Computing resource of UAV, ' + r'$f_u$(GHz)')
    plt.ylabel('System Cost')
    plt.legend(bbox_to_anchor=(1.05, 1.0), loc='upper left')
    plt.savefig("./graphs/" + "UAV_CPU_COST_BAR")
    plt.clf()

    f = open('./graphs/UAV_CPU_COST.txt', 'w')

    print("System Cost", file=f)
    print("Proposed : ", system_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(system_cost[0]), file=f)
    print("UAV Only : ", system_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(system_cost[1]), file=f)
    print("Bus Only : ", system_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(system_cost[2]), file=f)
    print("Fixed : ", system_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(system_cost[3]), file=f)
    print("비율(UAV / Proposed) : ", np.mean(system_cost[1]) / np.mean(system_cost[0]), file=f)
    print("비율(BUS / Proposed) : ", np.mean(system_cost[2]) / np.mean(system_cost[0]), file=f)
    print("비율(FIX / Proposed) : ", np.mean(system_cost[3]) / np.mean(system_cost[0]), file=f)

    print("Delay Cost", file=f)
    print("Proposed : ", delay_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(delay_cost[0]), file=f)
    print("UAV Only : ", delay_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(delay_cost[1]), file=f)
    print("Bus Only : ", delay_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(delay_cost[2]), file=f)
    print("Fixed : ", delay_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(delay_cost[3]), file=f)
    print("비율(UAV / Proposed) : ", np.mean(delay_cost[1]) / np.mean(delay_cost[0]), file=f)
    print("비율(BUS / Proposed) : ", np.mean(delay_cost[2]) / np.mean(delay_cost[0]), file=f)
    print("비율(FIX / Proposed) : ", np.mean(delay_cost[3]) / np.mean(delay_cost[0]), file=f)

    print("Energy Cost", file=f)
    print("Proposed : ", energy_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(energy_cost[0]), file=f)
    print("UAV Only : ", energy_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(energy_cost[1]), file=f)
    print("Bus Only : ", energy_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(energy_cost[2]), file=f)
    print("Fixed : ", energy_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(energy_cost[3]), file=f)
    print("비율(UAV / Proposed) : ", np.mean(energy_cost[1]) / np.mean(energy_cost[0]), file=f)
    print("비율(BUS / Proposed) : ", np.mean(energy_cost[2]) / np.mean(energy_cost[0]), file=f)
    print("비율(FIX / Proposed) : ", np.mean(energy_cost[3]) / np.mean(energy_cost[0]), file=f)


    f.close()

if MODE == 2: # TASK의 data size를 증가시켜가며 테스트
    # 버스대수는 4대, 태스크개수 10개, 태스크=(5,10,100,150), 딜레이는 1~2로 설정

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)

    buses_original[0].cpu = 15
    buses_original[1].x = 850
    buses_original[1].y = 150

    buses_original[1].cpu = 13
    buses_original[1].x = 340
    buses_original[1].y = 400

    buses_original[2].cpu = 15
    buses_original[3].cpu = 10

    cal_distance()
    # 딜레이는 10~20으로 설정

    STEP = 10
    uav_data = np.zeros(STEP)
    bus_data = np.zeros((STEP, NUM_BUS))
    uav_cpu = np.zeros(STEP)
    bus_cpu = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(1, STEP+1, 1) * 10
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])
    #plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(1, STEP+1, 1):

        make_task(5, 10, 100, 150)

        print("STEP : ", k_index+1)

        result, rho_um, rho_bm, fum, fbm, mum = proposed_algorithm(FU)  # 제안 알고리즘

        if k_index == 0:
            uav_data[k_index] = cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000
            uav_cpu[k_index] = cvxpy.sum(cvxpy.multiply(cm, rho_um)).value
        else:
            uav_data[k_index] = uav_data[k_index-1] + cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000
            uav_cpu[k_index] = uav_cpu[k_index - 1] + cvxpy.sum(cvxpy.multiply(cm, rho_um)).value

        for b in range(NUM_BUS):
            if k_index == 0:
                bus_data[k_index] = cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000
                bus_cpu[k_index] = cvxpy.sum(cvxpy.multiply(cm, rho_bm), axis=0, keepdims=True).value
            else:
                bus_data[k_index] = bus_data[k_index-1] + cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000
                bus_cpu[k_index] = bus_cpu[k_index - 1] + cvxpy.sum(cvxpy.multiply(cm, rho_bm), axis=0,
                                                                      keepdims=True).value

        np.set_printoptions(precision=3)

        print("UAV가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm, rho_um)).value * 1000)
        print("버스가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm, rho_bm), axis=0, keepdims=True).value * 1000)
        print("Task 데이터량 합계 : ", np.round(np.sum(sm) * 1000), "Mbits")
        print("UAV와 버스처리량 합계 : ", (cvxpy.sum(cvxpy.multiply(sm, rho_um)).value+cvxpy.sum(cvxpy.multiply(sm, rho_bm)).value)*1000)
        print("UAV가 처리한 CPU량 : ", cvxpy.sum(cvxpy.multiply(cm, rho_um)).value)
        print("버스가 처리한 CPU량 : ", cvxpy.sum(cvxpy.multiply(cm, rho_bm), axis=0, keepdims=True).value)

        k_index += 1

    y1 = bus_data.transpose()

    plt.plot(x, uav_data, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        #plt.plot(x, y1[b], marker=next(marker), label="b"+str(b+1)  + " : " + str(buses_original[b].distance) +" m, " + str(buses_original[b].cpu) +" GHz")
        plt.plot(x, y1[b], marker=next(marker),
                 label=r'$b_{%d}$' % (b + 1) + " : " + str(round(buses_original[b].cpu)) + " GHz")

    #plt.yscale('symlog')
    plt.xticks(x)
    #plt.ylim([0.1, 100])
    plt.xlabel('Number of tasks')
    plt.ylabel('The amount of data processed (Mbits)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "TASK_SIZE_BIT1")
    plt.clf()

    y2 = bus_cpu.transpose()

    plt.plot(x, uav_cpu, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y2[b], marker=next(marker),
                 label=r'$b_{%d}$' % (b + 1) + " : " + str(round(buses_original[b].cpu)) + " GHz")

    # plt.yscale('symlog')
    plt.xticks(x)
    # plt.ylim([0.1, 100])
    plt.xlabel('Number of tasks')
    plt.ylabel('The amount of computation (GHz)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "TASK_SIZE_BIT2")
    plt.clf()

    f = open('./graphs/TASK_SIZE_BIT.txt', 'w')

    for b in range(NUM_BUS):
        print("bus", b+1, ": ", buses_original[b].distance, "m,", buses_original[b].cpu," GHz", file=f)

    print("UAV가 처리한 데이터량 : ", uav_data, file=f)
    print("UAV가 처리한 데이터량(평균) : ", np.mean(uav_data), file=f)
    print("버스가 처리한 데이터량 : ", y1, file=f)
    print("버스가 처리한 데이터량(버스평균) : ", np.mean(y1,axis=1), file=f)
    print("버스가 처리한 데이터량(전체평균) : ", np.mean(y1), file=f)
    print("UAV가 처리한 CPU량 : ", uav_cpu, file=f)
    print("UAV가 처리한 CPU(평균) : ", np.mean(uav_cpu), file=f)
    print("버스가 처리한 CPU량 : ", y2, file=f)
    print("버스가 처리한 CPU량(버스평균) : ", np.mean(y2,axis=1), file=f)
    print("버스가 처리한 CPU량(전체평균) : ", np.mean(y2), file=f)
    
    f.close()

if MODE == 222: # TASK의 data size를 증가시켜가며 테스트

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)
    cal_distance()
    # 딜레이는 10~20으로 설정

    STEP = (TASK_SIZE_MAX + 1) // 10
    uav_data = np.zeros(STEP)
    bus_data = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(10, TASK_SIZE_MAX + 1, 10)
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    #plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(10, TASK_SIZE_MAX + 1, 10):

        make_task(k/5, k/5, 100, 100)

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
        plt.plot(x, y[b], marker=next(marker), label="b"+str(b+1)  + " : " + str(buses_original[b].cpu) +"GHz, " + str(buses_original[b].distance) +"m")

    #plt.yscale('symlog')
    plt.xticks(x)
    #plt.ylim([0.1, 100])
    plt.xlabel('Number of tasks')
    plt.ylabel('The amount of computation(Mbits)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "TASK_SIZE_BIT")
    plt.clf()

if MODE == 333: # BUS 1대의 CPU를 증가시켜가며 실험
    # 버스대수=3, 태스크=(5,10,100,150)

    BUS_SAME_CPU = 10
    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS, BUS_SAME_CPU, BUS_SAME_CPU)

    buses_original[0].x = 100
    buses_original[0].y = 100
    buses_original[1].x = 300
    buses_original[1].y = 300
    buses_original[2].x = 150
    buses_original[2].y = 150

    cal_distance()
    make_task(5, 10, 100, 150)

    STEP = (FB_MAX + 1)//3 - 1
    system_cost1 = np.zeros(STEP)
    uav_ratio1 = np.zeros(STEP)
    bus_ratio1 = np.zeros((STEP, NUM_BUS))
    system_cost2 = np.zeros(STEP)
    uav_ratio2 = np.zeros(STEP)
    bus_ratio2 = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(6, FB_MAX + 1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    #plt.style.use(['science', 'ieee', 'no-latex'])
    plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])

    temp = np.zeros(NUM_BUS)
    for k in range(6, FB_MAX+1, 3):

        print("STEP : ", k_index+1, "BUS1 CPU : ", k)
        buses_original[0].cpu = k
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(FU) # 제안 알고리즘

        uav_ratio1[k_index] = cvxpy.sum(rho_um1).value / 10

        for b in range(NUM_BUS):
            bus_ratio1[k_index][b] = cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / 10
        system_cost1[k_index] = result1

        for t in range(NUM_BUS):
            temp[t] = buses_original[t].cpu
            buses_original[t].cpu = k
        result2, rho_um2, rho_bm2, fum2, fbm2, mum2 = proposed_algorithm(FU)  # 제안 알고리즘

        for t in range(NUM_BUS):
            buses_original[t].cpu = temp[t]

        uav_ratio2[k_index] = cvxpy.sum(rho_um2).value / 10
        for b in range(NUM_BUS):
            bus_ratio2[k_index][b] = cvxpy.sum(rho_bm2[:, b:b + 1:1]).value / 10
        system_cost2[k_index] = result2

        print("System Cost : ", system_cost1[k_index])
        print("UAV Ratio : ", uav_ratio1[k_index])
        print("Bus Ratio : ", bus_ratio1[k_index])

        k_index += 1

    y1 = bus_ratio1.transpose()
    plt.plot(x, uav_ratio1, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        if b==0:
            plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1))
        else:
            plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$'% (b + 1)+ " : " + str(round(buses_original[b].cpu)) +" GHz")

    plt.xticks(x)
    plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    #plt.legend(bbox_to_anchor=(0.5, -0.27), loc='lower center')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.ylim([0, 0.7])
    plt.yticks([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7])
    plt.tight_layout()
    plt.savefig("./graphs/" + "BUS_CPU_RATIO1")
    plt.clf()

    plt.plot(x, delay_cost1, marker=next(marker))
    plt.xticks(x)
    #plt.ylim([5, 15])
    #plt.yticks([5, 7, 9, 11, 13, 15])
    plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
    plt.ylabel('System Cost')
    plt.savefig("./graphs/" + "BUS_CPU_COST1")
    plt.clf()

    y2 = bus_ratio2.transpose()
    plt.plot(x, uav_ratio2, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y2[b], marker=next(marker), label=r'$b_{%d}$'% (b + 1))

    plt.xticks(x)
    plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    # plt.legend(bbox_to_anchor=(0.5, -0.27), loc='lower center')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.ylim([0, 0.6])
    plt.yticks([0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])
    plt.savefig("./graphs/" + "BUS_CPU_RATIO2")
    plt.clf()

    plt.plot(x, system_cost2, marker=next(marker))
    plt.xticks(x)
    plt.ylim([5, 15])
    plt.yticks([5, 7, 9, 11, 13, 15])
    plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
    plt.ylabel('System Cost')
    plt.savefig("./graphs/" + "BUS_CPU_COST2")
    plt.clf()

    f = open('./graphs/BUS_CPU_RATIO.txt', 'w')

    for b in range(NUM_BUS):
        print("bus", b + 1, ": ", buses_original[b].distance, "m,", buses_original[b].cpu, " GHz", file=f)

    print("BUS1만 CPU증가하는 경우")
    print("UAV ratio : ", uav_ratio1, file=f)
    print("UAV ratio(평균) : ", np.mean(uav_ratio1), file=f)
    print("Bus ratio : ", y1, file=f)
    print("Bus ratio(버스평균) : ", np.mean(y1, axis=1), file=f)
    print("Bus ratio(전체평균) : ", np.mean(y1), file=f)
    print("System Cost : ", system_cost1, file=f)
    print("System Cost(평균) : ", np.mean(system_cost1), file=f)

    print("모든 BUS의 CPU증가하는 경우")
    print("UAV ratio : ", uav_ratio2, file=f)
    print("UAV ratio(평균) : ", np.mean(uav_ratio2), file=f)
    print("Bus ratio : ", y2, file=f)
    print("Bus ratio(버스평균) : ", np.mean(y2, axis=1), file=f)
    print("Bus ratio(전체평균) : ", np.mean(y2), file=f)
    print("System Cost : ", system_cost2, file=f)
    print("System Cost(평균) : ", np.mean(system_cost2), file=f)

    f.close()

if MODE == 3: # BUS 1대의 CPU를 증가시켜가며 실험
    # 버스대수=3, 태스크=(5,10,100,150)

    BUS_SAME_CPU = 10
    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS, BUS_SAME_CPU, BUS_SAME_CPU)

    cal_distance()
    make_task(5, 15, 50, 150)

    f = open('./graphs/BUS_CPU_RATIO.txt', 'w')

    for w in range(3):
        if w==0:
            w1 = 1
            w2 = 3
        elif w==1:
            w1 = 3
            w2 = 1
        elif w==2:
            w1 = 1
            w2 = 1

        STEP = (FB_MAX + 1)//3 - 1
        system_cost1 = np.zeros(STEP)
        delay_cost1 = np.zeros(STEP)
        energy_cost1 = np.zeros(STEP)
        uav_ratio1 = np.zeros(STEP)
        bus_ratio1 = np.zeros((STEP, NUM_BUS))
        system_cost2 = np.zeros(STEP)
        delay_cost2 = np.zeros(STEP)
        energy_cost2 = np.zeros(STEP)
        uav_ratio2 = np.zeros(STEP)
        bus_ratio2 = np.zeros((STEP, NUM_BUS))
        k_index = 0

        temp = np.zeros(NUM_BUS)
        for k in range(6, FB_MAX+1, 3):

            print("STEP : ", k_index+1, "BUS1 CPU : ", k)
            buses_original[0].cpu = k
            result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(FU, w1, w2)  # 제안 알고리즘

            uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)

            for b in range(NUM_BUS):
                bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK, 3)
            system_cost1[k_index] = round(result1,3)
            delay_cost1[k_index] = round(cvxpy.sum(mum1).value * w1,3)
            energy_cost1[k_index] = round(system_cost1[k_index] - delay_cost1[k_index],3)

            for t in range(NUM_BUS):
                temp[t] = buses_original[t].cpu
                buses_original[t].cpu = k
            result2, rho_um2, rho_bm2, fum2, fbm2, mum2 = proposed_algorithm(FU)  # 제안 알고리즘

            for t in range(NUM_BUS):
                buses_original[t].cpu = temp[t]

            uav_ratio2[k_index] = round(cvxpy.sum(rho_um2).value / NUM_TASK,3)
            for b in range(NUM_BUS):
                bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm2[:, b:b + 1:1]).value / NUM_TASK,3)
            system_cost2[k_index] = round(result2,3)
            delay_cost2[k_index] = round(cvxpy.sum(mum2).value * w1, 3)
            energy_cost2[k_index] = round(system_cost2[k_index] - delay_cost2[k_index], 3)

            print("System Cost : ", system_cost1[k_index], "Delay Cost : ", delay_cost1[k_index], "Energy Cost : ",
                  energy_cost1[k_index])
            print("UAV Ratio : ", uav_ratio1[k_index])
            print("Bus Ratio : ", bus_ratio1[k_index])

            k_index += 1

        x = np.arange(6, FB_MAX + 1, 3)
        marker = itertools.cycle(('+', '2', '.', 'x', '*'))
        plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])
        #plt.style.use(['science', 'std-colors', 'no-latex'])

        y1 = bus_ratio1.transpose()
        plt.plot(x, uav_ratio1, marker=next(marker), label="UAV")
        for b in range(NUM_BUS):
            if b==0:
                plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1))
            else:
                plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$'% (b + 1)+ " : " + str(round(buses_original[b].cpu)) +" GHz")

        plt.xticks(x)
        plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
        plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
        #plt.legend(loc='lower left', bbox_to_anchor=(0, 1.01, 1, 0.1), mode='expand', ncol=3)
        plt.legend(loc='best')
        plt.legend(frameon=True)
        plt.ylim([0, 0.8])
        plt.yticks([0, 0.2, 0.4, 0.6, 0.8])
        plt.tight_layout()
        plt.savefig("./graphs/" + "BUS_CPU_RATIO1_" + "w1_"+ str(w1)+ "w2_"+str(w2))
        plt.clf()

        plt.bar(x, delay_cost1, label='Delay cost')
        plt.bar(x, energy_cost1, bottom=delay_cost1, label='Energy cost')
        plt.xticks(x)
        plt.legend(loc='best')
        plt.legend(frameon=True)
        plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
        plt.ylabel('System Cost')
        plt.savefig("./graphs/" + "BUS_CPU_COST1_" + "w1_"+ str(w1)+ "w2_"+str(w2))
        plt.clf()

        y2 = bus_ratio2.transpose()
        plt.plot(x, uav_ratio2, marker=next(marker), label="UAV")
        for b in range(NUM_BUS):
            plt.plot(x, y2[b], marker=next(marker), label=r'$b_{%d}$'% (b + 1))

        plt.xticks(x)
        plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
        plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
        # plt.legend(bbox_to_anchor=(0.5, -0.27), loc='lower center')
        #plt.legend(loc='lower left', bbox_to_anchor=(0, 1.01, 1, 0.1), mode='expand', ncol=3)
        plt.legend(loc='best')
        plt.legend(frameon=True)
        plt.ylim([0, 0.8])
        plt.yticks([0, 0.2, 0.4, 0.6, 0.8])
        plt.savefig("./graphs/" + "BUS_CPU_RATIO2_" + "w1_"+ str(w1)+ "w2_"+str(w2))
        plt.clf()

        plt.bar(x, delay_cost2, label='Delay cost')
        plt.bar(x, energy_cost2, bottom=delay_cost2, label='Energy cost')
        plt.xticks(x)
        plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
        plt.ylabel('System Cost')
        plt.legend(loc='best')
        plt.legend(frameon=True)
        plt.savefig("./graphs/" + "BUS_CPU_COST2_" + "w1_" + str(w1) + "w2_" + str(w2))
        plt.clf()

        for b in range(NUM_BUS):
            print("bus", b + 1, ": ", buses_original[b].distance, "m,", buses_original[b].cpu, " GHz", file=f)

        print("w1 :", w1, "w2 :", w2, file=f)

        print("BUS1만 CPU증가하는 경우", file=f)
        print("UAV ratio : ", uav_ratio1, file=f)
        print("UAV ratio(평균) : ", np.round(np.mean(uav_ratio1),2), file=f)
        print("Bus ratio : ", y1, file=f)
        print("Bus ratio(버스평균) : ", np.round(np.mean(y1, axis=1),2), file=f)
        print("Bus ratio(전체평균) : ", np.round(np.mean(y1),2), file=f)
        print("System Cost : ", system_cost1, file=f)
        print("System Cost(평균) : ", np.round(np.mean(system_cost1),2), file=f)
        print("Delay Cost : ", delay_cost1, file=f)
        print("Delay Cost(평균) : ", np.round(np.mean(delay_cost1),2), file=f)
        print("Energy Cost : ", energy_cost1, file=f)
        print("Energy Cost(평균) : ", np.round(np.mean(energy_cost1),2), file=f)

        print("모든 BUS의 CPU증가하는 경우", file=f)
        print("UAV ratio : ", uav_ratio2, file=f)
        print("UAV ratio(평균) : ", np.round(np.mean(uav_ratio2), 2), file=f)
        print("Bus ratio : ", y2, file=f)
        print("Bus ratio(버스평균) : ", np.round(np.mean(y2, axis=1), 2), file=f)
        print("Bus ratio(전체평균) : ", np.round(np.mean(y2), 2), file=f)
        print("System Cost : ", system_cost2, file=f)
        print("System Cost(평균) : ", np.round(np.mean(system_cost2), 2), file=f)
        print("Delay Cost : ", delay_cost2,file=f)
        print("Delay Cost(평균) : ", np.round(np.mean(delay_cost2), 2), file=f)
        print("Energy Cost : ", energy_cost2, file=f)
        print("Energy Cost(평균) : ", np.round(np.mean(energy_cost2), 2), file=f)

    f.close()

if MODE == 4: # omega1를 증가시켜가며 실험
    # 버스대수는 4대, 태스크개수는 1개, 태스크=(5,10,100,150), 딜레이는 1~2로 설정

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)

    #for i in range(NUM_BUS):
        #buses_original[i].x = random.randint(500-250, 500+250)
        #buses_original[i].y = random.randint(500 - 250, 500 + 250)

    cal_distance()
    #make_task(10, 20, 100, 200)
    make_task(5, 10, 100, 150)

    STEP = 20
    system_cost = np.zeros(STEP)
    uav_ratio = np.zeros(STEP)
    bus_ratio = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(1, STEP+1, 1)
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(1, STEP+1, 1):

        print("STEP : ", k_index+1, "Omega1 : ", k, " Omega2 : ", STEP/2)
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm(FU, k, STEP/2) # 제안 알고리즘

        #print(rho_um1.value)
        #print(mum1.value)
        uav_ratio[k_index] = cvxpy.sum(rho_um1).value / NUM_TASK
        for b in range(NUM_BUS):
            bus_ratio[k_index][b] = cvxpy.sum(rho_bm1[:,b:b+1:1]).value / NUM_TASK

        system_cost[k_index] = result1

        print("System Cost : ", np.round(system_cost[k_index],2))
        print("UAV Ratio : ", np.round(uav_ratio[k_index],2))
        print("Bus Ratio : ", np.round(bus_ratio[k_index],2))

        k_index += 1

    y = bus_ratio.transpose()

    plt.plot(x, uav_ratio, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        #plt.plot(x, y[b], marker=next(marker), label="BUS" + str(b + 1)  + " : " + str(buses_original[b].cpu) +"GHz, " + str(buses_original[b].distance) +"m")
        plt.plot(x, y[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1) + " : " + str(buses_original[b].distance) + " m, " + str(round(buses_original[b].cpu)) + " GHz")
    plt.xticks(x)
    plt.xlabel(r'$\omega_1$')

    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "OMEGA_RATIO1")
    plt.clf()

if MODE == 5: # UAv가 버스를 찾는 거리를 증가시키면서 시뮬레이션

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, 0, 0, Z))
    SIMUL_TIME = 9
    #make_bus(REAL, NUM_BUS)
    make_task(5, 15, 50, 150)

    system_cost = np.zeros((SIMUL_TIME, NUM_BUS))
    uav_ratio = np.zeros((SIMUL_TIME, NUM_BUS))
    bus_ratio = np.zeros((SIMUL_TIME, NUM_BUS, NUM_BUS))

    for p in range(NUM_BUS):

        make_bus(REAL, 1)

        for k in range(SIMUL_TIME):

            print("버스대수 : ", p + 1, "STEP : ", k + 1)

            numbus = len(buses_original)
            for b in range(numbus):
                buses_original[b].x = k * 50 + 50
                buses_original[b].y = 0

            cal_distance()

            result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm2(FU)  # 제안 알고리즘
            uav_ratio[k][p] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)

            for b in range(NUM_BUS):
                bus_ratio[k][p][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK, 3)
            system_cost[k][p] = round(result1, 3)

            print("System Cost : ", system_cost[k][p])
            print("UAV Ratio : ", uav_ratio[k][p])
            print("Bus Ratio : ", bus_ratio[k][p])

            k += 1

        p += 1

    x = np.arange(1, SIMUL_TIME + 1, 1) * 100
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    b1 = np.sum(bus_ratio.transpose(), axis=0)
    u1 = uav_ratio.transpose()

    plt.xlabel(r'$L_{CoA}$ (m)')
    plt.ylabel('Optimal offloading ratio, ' + r'$\rho_{u,m}$')

    for p in range(NUM_BUS):
        plt.plot(x, u1[p], marker=next(marker), label="B=%d" % (p+1))

    plt.xticks(x)
    plt.yscale('log')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "DISTANCE_RATIO")
    plt.clf()

if MODE == 10: # 버스를 이동하면서 시뮬레이션

    X_STEP = 5
    Y_STEP = 5

    simul_distance = MAP_SIZE / X_STEP / 2

    SIMUL_TIME = 10
    MAP_DIV_COUNT = X_STEP * Y_STEP

    MAP = [[0 for j in range(Y_STEP)] for i in range(X_STEP)]
    for i in range(X_STEP):
        for j in range(Y_STEP):
            MAP[i][j] = [100+MAP_SIZE/X_STEP*j, 100+MAP_SIZE/X_STEP*i, 100]

    make_bus(REAL, BUS_NUM)
    make_task(10, 20, 100, 200)

    system_cost = [[[0 for i in range(Y_STEP)] for j in range(X_STEP)] for k in range(SIMUL_TIME)]
    bus_count = [[[0 for i in range(Y_STEP)] for j in range(X_STEP)] for k in range(SIMUL_TIME)]

    simul_index = 0

    for i in range(SIMUL_TIME):
        count_bus()
        for bus in buses_original:
            bus.move()

    for i in range(SIMUL_TIME):


        k_index = 0

        for j in range(X_STEP):
            for k in range(Y_STEP):

                uavs_original.append(UAV(0, MAP[j][k][0], MAP[j][k][1], MAP[j][k][2]))
                BUS_MAP = count_bus()

                print("STEP : ", k_index + 1, end=" ")

                if BUS_MAP[j][k] > 0 :
                    result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm4(FU, k_index, simul_distance)  # 제안 알고리즘
                    system_cost[simul_index][j][k] = round(result1,2)
                    print("Proposed : ", system_cost[simul_index][j][k])
                    bus_count[i][j][k] = BUS_MAP[j][k]
                else:
                    result2, fum2, t_um_cost = uav_only_algorithm2(FU, k_index, simul_distance)  # uav only 알고리즘
                    system_cost[simul_index][j][k] = round(result2,2)
                    print("UAV only : ", system_cost[simul_index][j][k])

                k_index += 1

        for bus in buses_original:
            bus.move()
        cal_distance()

        simul_index +=1

    fontlabel = {"fontsize": "large", "color": "gray", "fontweight": "bold"}
    system_cost2 = np.mean(system_cost, axis=0)
    system_cost3 = (np.max(system_cost2) - system_cost2) / np.max(system_cost2) * 100
    print(np.max(system_cost2))
    print(system_cost2)
    print(system_cost3)

    bus_count2 = np.mean(bus_count, axis=0)

    #fig = plt.figure()
    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax1 = fig.add_subplot(1,2,1,projection='3d')
    ax2 = fig.add_subplot(1,2,2,projection='3d')

    # x = np.linspace(1, 5, 5)
    # X = np.tile(x, (5, 1))
    # Y = X.transpose()
    #
    # ax.plot_surface(X, Y, system_cost2)
    # ax.plot_surface(X, Y, bus_count2)
    # plt.tight_layout()
    # plt.show()

    X = np.arange(0, X_STEP, 1)
    Y = np.arange(0, Y_STEP, 1)
    Z = np.zeros(X_STEP)
    dx = np.ones(X_STEP) * 0.5
    dy = np.ones(Y_STEP) * 0.5

    for i in range(SIMUL_TIME):
        for x in range(X_STEP):
            for y in range(Y_STEP):

                ax1.bar3d(X[x], Y[y], Z[y], dx[x], dy[y], bus_count2[x][y], alpha=0.05, shade=True)
                ax2.bar3d(X[x], Y[y], Z[y], dx[x], dy[y], system_cost3[x][y], alpha=0.05, shade=True)

                #ax1.plot_surface(X[x], Y[y], bus_count2[x][y], alpha=0.5, shade=True)
                #ax2.plot_surface(X[x], Y[y], system_count2[x][y], alpha=0.5, shade=True)

    ax1.view_init(elev=17., azim=147)
    ax1.invert_yaxis()
    ax1.set_xlabel('$L_y$')
    ax1.set_ylabel('$L_x$')
    ax1.set_zlabel('Number of buses')

    ax2.view_init(elev=17., azim=147)
    ax2.invert_yaxis()
    ax2.set_xlabel('$L_y$')
    ax2.set_ylabel('$L_x$')
    ax2.set_zlabel('Reduction ratio in system cost (%)')

    plt.tight_layout()
    plt.show()
    plt.savefig("./graphs/" + "2D_MAP")
    plt.clf()