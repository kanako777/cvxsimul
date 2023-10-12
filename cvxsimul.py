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

#print("MODE = 0 : UAV와 BUS의 ratio 실험")
#print("MODE = 1 : UAV의 CPU를 증가")
#print("MODE = 11 : TASK의 개수를 증가")
#print("MODE = 2 : TASK의 data size를 증가")
#print("MODE = 3 : BUS 1대의 CPU를 증가")
#print("MODE = 4 : UAV가 버스를 찾는 거리를 증가")
#print("MODE = 90 : omega1를 증가")

MODE = 5
simul_time = 10

if MODE == 0:  # UAV와 BUS의 ratio 실험

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))

    make_bus(REAL, NUM_BUS)
    make_task2(3, 15, 50, 150)

    bus_simul, result, cost1, cost2, rho_um, rho_bm, fum, fbm, mum, num_bus1, t_count = proposed_algorithm2(FU,0, lcoa_mode, 2) # 제안 알고리즘

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
    # plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])

    plt.bar(xaxis, x1[0], label=r'$\rho_{u,m}$')
    bottom = x1[0]

    for b in range(NUM_BUS):
        plt.bar(xaxis, y1[b], bottom=bottom, label=r'$\rho_{{%d},m}$' % (b + 1))

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
        print("Task" + str(i + 1) + ":", str(sm[i] * 1000) + "Mbits", file=f)
    print("Bus CPU : ", file=f)
    for i in range(NUM_BUS):
        print("BUS" + str(i + 1) + ":", str(bus_simul[i].cpu) + "GHz, 거리:" + str(bus_simul[i].distance) + "m",
              file=f)

    print("Task별 UAV 처리비율 : ", rho_um.value, file=f)
    print("Task별 UAV 처리비율(평균) : ", np.mean(rho_um.value), file=f)
    print("Task별 Bus 처리비율 : ", file=f)
    for i in range(NUM_BUS):
        print("BUS", i + 1, np.round(y1[i], 2), file=f)
    print("Task별 Bus 처리비율(task별 합계) : ", np.round(np.sum(y1, axis=0), 2), file=f)
    print("Task별 Bus 처리비율(버스별 합계) : ", np.round(np.sum(y1, axis=1), 2), file=f)

    f.close()

if MODE == 1:  # UAV의 CPU를 증가시켜가며 실험

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)
    make_task(3, 15, 50, 150)

    STEP = FU_MAX // 3
    uav_ratio1 = np.zeros(STEP)
    bus_ratio1 = np.zeros((STEP, NUM_BUS))
    uav_ratio2 = np.zeros(STEP)
    bus_ratio2 = np.zeros((STEP, NUM_BUS))
    system_cost = np.zeros((4, STEP))
    delay_cost = np.zeros((4, STEP))
    energy_cost = np.zeros((4, STEP))
    processed_task = np.zeros((4, STEP))

    w1 = 1
    w2 = 1
    k_index = 0

    x = np.arange(3, FU_MAX+1, 3)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(3, FU_MAX+1, 3):

        print("STEP : ", k_index + 1, "UAV CPU : ", k)

        bus_simul, cost1, cost2, result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1, t_count = proposed_algorithm2(k, 0,lcoa_mode, simul_time)  # 제안 알고리즘

        if rho_um1 is None:
            pass
        else:
            processed_task[0][k_index] = t_count
            if t_count == NUM_TASK:
                system_cost[0][k_index] = round(result1, 3)
                delay_cost[0][k_index] = round(cvxpy.sum(mum1).value * w1, 3)
                energy_cost[0][k_index] = round(system_cost[0][k_index] - delay_cost[0][k_index], 3)
                uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)
                for b in range(NUM_BUS):
                    bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK, 3)
        print("Proposed가 처리한 task 개수 :", t_count)

        result2, cost1, cost2, rho_um2, fum2, mum2, num_bus2, e_um_cost, t_count = uav_only_algorithm(k, 0,lcoa_mode, simul_time)  # uav only 알고리즘

        if rho_um2 is None:
            pass
        else:
            processed_task[1][k_index] = t_count
            if t_count ==NUM_TASK :
                system_cost[1][k_index] = round(result2, 3)
                delay_cost[1][k_index] = round(cvxpy.sum(mum2).value * w1, 3)
                energy_cost[1][k_index] = round(system_cost[1][k_index] - delay_cost[1][k_index], 3)

        print("LC가 처리한 task 개수 :", t_count)

        result3, cost1, cost2, rho_bm3, fbm3, mum3, t_count = bus_only_algorithm(k, 0,lcoa_mode, simul_time)  # bus only 알고리즘
        if rho_bm3 is None:
            pass
        else:
            processed_task[2][k_index] = NUM_TASK
            system_cost[2][k_index] = round(result3, 3)
            delay_cost[2][k_index] = round(cvxpy.sum(mum3).value * w1, 3)
            energy_cost[2][k_index] = round(system_cost[2][k_index] - delay_cost[2][k_index], 3)

        print("FO가 처리한 task 개수 :", t_count)

        result4, cost1, cost2, rho_um4, rho_bm4, fum4, fbm4, mum4, t_count = fixed_algorithm(k, 0,lcoa_mode, simul_time)  # fixed 알고리즘
        if rho_bm4 is None:
            pass
        else:
            processed_task[3][k_index] = t_count
            if t_count == NUM_TASK:
                system_cost[3][k_index] = round(result4, 3)
                delay_cost[3][k_index] = round(cvxpy.sum(mum4).value * w1, 3)
                energy_cost[3][k_index] = round(system_cost[3][k_index] - delay_cost[3][k_index], 3)
                uav_ratio2[k_index] = round(cvxpy.sum(rho_um4).value / NUM_TASK, 3)

                for b in range(NUM_BUS):
                    bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm4[:, b:b + 1:1]).value / NUM_TASK, 3)
        print("FPO가 처리한 task 개수 :", t_count)

        print(uav_ratio1[k_index])
        print(bus_ratio1[k_index])
        print(uav_ratio2[k_index])
        print(bus_ratio2[k_index])

        print("[System Cost]", end=' ')
        print("Proposed : ", system_cost[0][k_index], "UAV Only : ", system_cost[1][k_index], "Bus Only : ",
              system_cost[2][k_index], "Fixed : ", system_cost[3][k_index])

        print("[Delay Cost]", end=' ')
        print("Proposed : ", delay_cost[0][k_index], "UAV Only : ", delay_cost[1][k_index], "Bus Only : ",
              delay_cost[2][k_index], "Fixed : ", delay_cost[3][k_index])

        print("[Energy Cost]", end=' ')
        print("Proposed : ", energy_cost[0][k_index], "UAV Only : ", energy_cost[1][k_index], "Bus Only : ",
              energy_cost[2][k_index], "Fixed : ", energy_cost[3][k_index])

        k_index += 1

    uav_ratio1[uav_ratio1 == 0] = np.nan
    bus_ratio1[bus_ratio1 == 0] = np.nan
    uav_ratio2[uav_ratio2 == 0] = np.nan
    bus_ratio2[bus_ratio2 == 0] = np.nan
    system_cost[system_cost == 0] = np.nan
    delay_cost[delay_cost == 0] = np.nan
    energy_cost[energy_cost == 0] = np.nan

    plt.plot(x, processed_task[0], marker=next(marker), label='Proposed')
    plt.plot(x, processed_task[1], marker=next(marker), label='LC')
    plt.plot(x, processed_task[2], marker=next(marker), label='FO')
    plt.plot(x, processed_task[3], marker=next(marker), label='FPO')

    plt.xticks(x)
    plt.yticks([0,1,2,3,4,5,6,7,8,9,10])
    plt.xlabel('Computing resource of UAV, ' + r'$f_u$(GHz)')
    plt.ylabel('Number of processed tasks')
    plt.legend(loc='lower left')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "UAV_CPU_TASK_NUMBER")
    plt.clf()

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

    plt.style.use(['ieee', 'std-colors', 'no-latex'])

    width = 0.3
    plt.bar(x - 0.9, delay_cost[0], width, label='Proposed-Delay')
    plt.bar(x - 0.9, energy_cost[0], width, bottom=delay_cost[0], label='Proposed-Energy')
    plt.bar(x - 0.3, delay_cost[1], width, label='LC-Delay')
    plt.bar(x - 0.3, energy_cost[1], width, bottom=delay_cost[1], label='LC-Energy')
    plt.bar(x + 0.3, delay_cost[2], width, label='FO-Delay')
    plt.bar(x + 0.3, energy_cost[2], width, bottom=delay_cost[2], label='FO-Energy')
    plt.bar(x + 0.9, delay_cost[3], width, label='FPO-Delay')
    plt.bar(x + 0.9, energy_cost[3], width, bottom=delay_cost[3], label='FPO-Energy')

    plt.xticks(x)
    plt.xlabel('Computing resource of UAV, ' + r'$f_u$(GHz)')
    plt.ylabel('System Cost')
    plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
              fancybox=True, shadow=True, ncol=4)
    plt.savefig("./graphs/" + "UAV_CPU_COST_BAR")
    plt.clf()


    f = open('./graphs/UAV_CPU_COST.txt', 'w')

    print("[Number of processed tasks]", file=f)
    print("Proposed : ", processed_task[0], file=f)
    print("UAV Only : ", processed_task[1], file=f)
    print("Bus Only : ", processed_task[2], file=f)
    print("Fixed : ", processed_task[3], file=f)

    print("[System Cost]", file=f)
    print("Proposed : ", system_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(system_cost[0]), file=f)
    print("UAV Only : ", system_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(system_cost[1]), file=f)
    print("Bus Only : ", system_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(system_cost[2]), file=f)
    print("Fixed : ", system_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(system_cost[3]), file=f)
    print("비율(Proposed / UAV) : ", np.mean(system_cost[0]) / np.mean(system_cost[1]), file=f)
    print("비율(Proposed / BUS) : ", np.mean(system_cost[0]) / np.mean(system_cost[2]), file=f)
    print("비율(Proposed / FIX) : ", np.mean(system_cost[0]) / np.mean(system_cost[3]), file=f)
    print("", file=f)

    print("Delay Cost", file=f)
    print("Proposed : ", delay_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(delay_cost[0]), file=f)
    print("UAV Only : ", delay_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(delay_cost[1]), file=f)
    print("Bus Only : ", delay_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(delay_cost[2]), file=f)
    print("Fixed : ", delay_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(delay_cost[3]), file=f)
    print("비율(Proposed / UAV) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[1]), file=f)
    print("비율(Proposed / BUS) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[2]), file=f)
    print("비율(Proposed / FIX) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[3]), file=f)
    print("", file=f)

    print("Energy Cost", file=f)
    print("Proposed : ", energy_cost[0], file=f)
    print("Proposed(평균) : ", np.mean(energy_cost[0]), file=f)
    print("UAV Only : ", energy_cost[1], file=f)
    print("UAV Only(평균) : ", np.mean(energy_cost[1]), file=f)
    print("Bus Only : ", energy_cost[2], file=f)
    print("Bus Only(평균) : ", np.mean(energy_cost[2]), file=f)
    print("Fixed : ", energy_cost[3], file=f)
    print("Fixed(평균) : ", np.mean(energy_cost[3]), file=f)
    print("비율(Proposed / UAV) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[1]), file=f)
    print("비율(Proposed / BUS) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[2]), file=f)
    print("비율(Proposed / FIX) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[3]), file=f)

    f.close()

if MODE == 11:  # TASK의 개수를 증가시켜가며 실험

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)

    STEP = 3
    STEP_SIZE = MAX_TASK // STEP
    width = STEP_SIZE / 4 * 0.5
    space = STEP_SIZE / 2 / 4

    make_task(3, 15, 50, 150, MAX_TASK)

    x = np.arange(STEP_SIZE, MAX_TASK+1, STEP_SIZE)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    FU_STEP = 3
    plt.tight_layout()
    f = open('./graphs/UAV_CPU_COST.txt', 'w')

    for u in range(FU, FU * FU_STEP + 1, FU*(FU_STEP-1)):

        FU = u

        uav_ratio1 = np.zeros(STEP)
        bus_ratio1 = np.zeros((STEP, NUM_BUS))
        uav_ratio2 = np.zeros(STEP)
        bus_ratio2 = np.zeros((STEP, NUM_BUS))
        system_cost = np.zeros((4, STEP))
        all_system_cost1 = np.zeros((4, STEP))
        all_system_cost2 = np.zeros((4, STEP))
        delay_cost = np.zeros((4, STEP))
        energy_cost = np.zeros((4, STEP))
        processed_task = np.zeros((4, STEP))

        is_end = [0, 0, 0, 0]
        w1 = 1
        w2 = 1
        k_index = 0

        for k in range(STEP_SIZE, MAX_TASK+1, STEP_SIZE):

            print("FU : ", FU, "STEP : ", k_index + 1, "No. of tasks : ", k)

            if is_end[0]==0:
                bus_simul, result1, cost1, cost2, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1, t_count = proposed_algorithm2(FU, 0, lcoa_mode, simul_time, k)  # 제안 알고리즘
                processed_task[0][k_index] = t_count

                all_system_cost1[0][k_index] = round(result1+cost1, 3)
                all_system_cost2[0][k_index] = round(result1+cost2, 3)

                if k==t_count:
                    system_cost[0][k_index] = round(result1 / t_count, 3)
                    delay_cost[0][k_index] = round(cvxpy.sum(mum1).value * w1 / t_count, 3)
                    energy_cost[0][k_index] = round(system_cost[0][k_index] - delay_cost[0][k_index], 3)
                    uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / t_count, 3)
                    for b in range(NUM_BUS):
                        bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / t_count, 3)

                #if (k_index >=1) and (processed_task[0][k_index-1] == processed_task[0][k_index]):
                    #is_end[0]=t_count
                #elif t_count < k:
                    #is_end[0] = t_count

                print("Proposed가 처리한 task 개수 :", t_count)

            else:
                processed_task[0][k_index] = processed_task[0][k_index-1]
                system_cost[0][k_index] = system_cost[0][k_index-1]
                delay_cost[0][k_index] = delay_cost[0][k_index-1]
                energy_cost[0][k_index] = energy_cost[0][k_index-1]
                uav_ratio1[k_index] = uav_ratio1[k_index-1]
                bus_ratio1[k_index] = bus_ratio1[k_index-1]

            if is_end[1]==0:
                result2, cost1, cost2, rho_um2, fum2, mum2, num_bus2, e_um_cost, t_count = uav_only_algorithm(FU, 0,lcoa_mode, simul_time, k)  # uav only 알고리즘

                processed_task[1][k_index] = t_count
                all_system_cost1[1][k_index] = round(result2 + cost1, 3)
                all_system_cost2[1][k_index] = round(result2 + cost2, 3)

                if k == t_count:
                    system_cost[1][k_index] = round(result2 / t_count, 3)
                    delay_cost[1][k_index] = round(cvxpy.sum(mum2).value * w1 / t_count, 3)
                    energy_cost[1][k_index] = round(system_cost[1][k_index] - delay_cost[1][k_index], 3)

                #if (k_index >=1) and (processed_task[1][k_index-1] == processed_task[1][k_index]):
                    #is_end[1]=t_count
                #elif t_count < k:
                    #is_end[1] = t_count

                print("LC가 처리한 task 개수 :", t_count)

            else:
                processed_task[1][k_index] = processed_task[1][k_index-1]
                system_cost[1][k_index] = system_cost[1][k_index-1]
                delay_cost[1][k_index] = delay_cost[1][k_index-1]
                energy_cost[1][k_index] = energy_cost[1][k_index-1]


            if is_end[2]==0:
                result3, cost1, cost2, rho_bm3, fbm3, mum3, t_count = bus_only_algorithm(FU, 0,lcoa_mode, simul_time, k)  # bus only 알고리즘

                processed_task[2][k_index] = t_count
                all_system_cost1[2][k_index] = round(result3 + cost1, 3)
                all_system_cost2[2][k_index] = round(result3 + cost2, 3)

                if k == t_count:
                    system_cost[2][k_index] = round(result3 / t_count, 3)
                    delay_cost[2][k_index] = round(cvxpy.sum(mum3).value * w1 / t_count, 3)
                    energy_cost[2][k_index] = round(system_cost[2][k_index] - delay_cost[2][k_index], 3)

                #if (k_index >=1) and (processed_task[2][k_index-1] == processed_task[2][k_index]):
                    #is_end[2]=t_count
                #elif t_count < k:
                    #is_end[2] = t_count

                print("FO가 처리한 task 개수 :", t_count)

            else:
                processed_task[2][k_index] = processed_task[2][k_index-1]
                system_cost[2][k_index] = system_cost[2][k_index-1]
                delay_cost[2][k_index] = delay_cost[2][k_index-1]
                energy_cost[2][k_index] = energy_cost[2][k_index-1]


            if is_end[3] == 0:

                result4, cost1, cost2, rho_um4, rho_bm4, fum4, fbm4, mum4, t_count = fixed_algorithm(FU, 0,lcoa_mode, simul_time, k)  # fixed 알고리즘

                processed_task[3][k_index] = t_count
                all_system_cost1[3][k_index] = round(result4 + cost1, 3)
                all_system_cost2[3][k_index] = round(result4 + cost2, 3)

                if k == t_count:
                    system_cost[3][k_index] = round(result4 / t_count, 3)
                    delay_cost[3][k_index] = round(cvxpy.sum(mum4).value * w1 / t_count, 3)
                    energy_cost[3][k_index] = round(system_cost[3][k_index] - delay_cost[3][k_index], 3)
                    uav_ratio2[k_index] = round(cvxpy.sum(rho_um4).value / t_count, 3)

                    for b in range(NUM_BUS):
                        bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm4[:, b:b + 1:1]).value / t_count, 3)

                #if (k_index >= 1) and (processed_task[3][k_index - 1] == processed_task[3][k_index]):
                    #is_end[3] = t_count
                #elif t_count < k:
                    #is_end[3] = t_count

                print("FPO가 처리한 task 개수 :", t_count)

            else:
                processed_task[3][k_index] = processed_task[3][k_index-1]
                system_cost[3][k_index] = system_cost[3][k_index-1]
                delay_cost[3][k_index] = delay_cost[3][k_index-1]
                energy_cost[3][k_index] = energy_cost[3][k_index-1]
                uav_ratio2[k_index] = uav_ratio2[k_index-1]
                bus_ratio2[k_index] = bus_ratio2[k_index-1]

            print(uav_ratio1[k_index])
            print(bus_ratio1[k_index])
            print(uav_ratio2[k_index])
            print(bus_ratio2[k_index])

            print("[All System Cost-1]", end=' ')
            print("Proposed : ", all_system_cost1[0][k_index], "UAV Only : ", all_system_cost1[1][k_index], "Bus Only : ", all_system_cost1[2][k_index], "Fixed : ", all_system_cost1[3][k_index])
            print("[All System Cost-2]", end=' ')
            print("Proposed : ", all_system_cost2[0][k_index], "UAV Only : ", all_system_cost2[1][k_index], "Bus Only : ",
            all_system_cost2[2][k_index], "Fixed : ", all_system_cost2[3][k_index])

            print("[System Cost]", end=' ')
            print("Proposed : ", system_cost[0][k_index], "UAV Only : ", system_cost[1][k_index], "Bus Only : ",
                  system_cost[2][k_index], "Fixed : ", system_cost[3][k_index])

            print("[Delay Cost]", end=' ')
            print("Proposed : ", delay_cost[0][k_index], "UAV Only : ", delay_cost[1][k_index], "Bus Only : ",
                  delay_cost[2][k_index], "Fixed : ", delay_cost[3][k_index])

            print("[Energy Cost]", end=' ')
            print("Proposed : ", energy_cost[0][k_index], "UAV Only : ", energy_cost[1][k_index], "Bus Only : ",
                  energy_cost[2][k_index], "Fixed : ", energy_cost[3][k_index])

            k_index += 1

        uav_ratio1[uav_ratio1 == 0] = np.nan
        bus_ratio1[bus_ratio1 == 0] = np.nan
        uav_ratio2[uav_ratio2 == 0] = np.nan
        bus_ratio2[bus_ratio2 == 0] = np.nan
        system_cost[system_cost == 0] = np.nan
        delay_cost[delay_cost == 0] = np.nan
        energy_cost[energy_cost == 0] = np.nan

        plt.bar(x-1*space, processed_task[0], width, label='Proposed')
        plt.bar(x-0*space, processed_task[1], width, label='LC')
        plt.bar(x+1*space, processed_task[2], width, label='FO')
        plt.bar(x+2*space, processed_task[3], width, label='FPO')

        #plt.plot(x, processed_task[0],  marker=next(marker), label='Proposed')
        #plt.plot(x, processed_task[1],  marker=next(marker), label='LC')
        #plt.plot(x, processed_task[2],  marker=next(marker), label='FO')
        #plt.plot(x, processed_task[3],  marker=next(marker), label='FPO')

        plt.xticks(x)
        plt.ylim([0,MAX_TASK])
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('Number of processed tasks')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)
        plt.savefig("./graphs/" + "UAV_CPU_TASK_NUMBER" + str(FU))
        plt.clf()


        plt.bar(x - 1 * space, system_cost[0], width, label='Proposed')
        plt.bar(x - 0 * space, system_cost[1], width, label='LC')
        plt.bar(x + 1 * space, system_cost[2], width, label='FO')
        plt.bar(x + 2 * space, system_cost[3], width, label='FPO')

        plt.xticks(x)
        plt.ylim([0, 1])
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('Avg. system cost per task')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST" + str(FU))
        plt.clf()

        plt.bar(x - 1 * space, all_system_cost1[0], width, label='Proposed')
        plt.bar(x - 0 * space, all_system_cost1[1], width, label='LC')
        plt.bar(x + 1 * space, all_system_cost1[2], width, label='FO')
        plt.bar(x + 2 * space, all_system_cost1[3], width, label='FPO')

        plt.xticks(x)
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('System cost')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST_ALL1" + str(FU))
        plt.clf()


        plt.bar(x - 1 * space, all_system_cost2[0], width, label='Proposed')
        plt.bar(x - 0 * space, all_system_cost2[1], width, label='LC')
        plt.bar(x + 1 * space, all_system_cost2[2], width, label='FO')
        plt.bar(x + 2 * space, all_system_cost2[3], width, label='FPO')

        plt.xticks(x)
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('System cost')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST_ALL2" + str(FU))
        plt.clf()


        print("UAV의 CPU : ", FU, file=f)
        print("[Number of processed tasks]", file=f)
        print("Proposed : ", processed_task[0], file=f)
        print("UAV Only : ", processed_task[1], file=f)
        print("Bus Only : ", processed_task[2], file=f)
        print("Fixed : ", processed_task[3], file=f)

        print("[System Cost]", file=f)
        print("Proposed : ", system_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(system_cost[0]), file=f)
        print("UAV Only : ", system_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(system_cost[1]), file=f)
        print("Bus Only : ", system_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(system_cost[2]), file=f)
        print("Fixed : ", system_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(system_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(system_cost[0]) / np.mean(system_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(system_cost[0]) / np.mean(system_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(system_cost[0]) / np.mean(system_cost[3]), file=f)
        print("", file=f)

        print("Delay Cost", file=f)
        print("Proposed : ", delay_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(delay_cost[0]), file=f)
        print("UAV Only : ", delay_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(delay_cost[1]), file=f)
        print("Bus Only : ", delay_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(delay_cost[2]), file=f)
        print("Fixed : ", delay_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(delay_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[3]), file=f)
        print("", file=f)

        print("Energy Cost", file=f)
        print("Proposed : ", energy_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(energy_cost[0]), file=f)
        print("UAV Only : ", energy_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(energy_cost[1]), file=f)
        print("Bus Only : ", energy_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(energy_cost[2]), file=f)
        print("Fixed : ", energy_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(energy_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[3]), file=f)

    f.close()

if MODE == 15:  # LCOA를 증가시켜가며 실험

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)

    STEP = 3
    STEP_SIZE = MAX_TASK // STEP
    width = STEP_SIZE / 4 * 0.5
    space = STEP_SIZE / 2 / 4

    make_task(3, 15, 50, 150, MAX_TASK)

    x = np.arange(STEP_SIZE, MAX_TASK+1, STEP_SIZE)
    marker = itertools.cycle(('+', '2', '.', 'x'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    FU_STEP = 3
    plt.tight_layout()
    f = open('./graphs/LCOA_DISTANCE.txt', 'w')

    for r in range(200, 400+1, 200):

        FU = u

        uav_ratio1 = np.zeros(STEP)
        bus_ratio1 = np.zeros((STEP, NUM_BUS))
        uav_ratio2 = np.zeros(STEP)
        bus_ratio2 = np.zeros((STEP, NUM_BUS))
        system_cost = np.zeros((4, STEP))
        all_system_cost1 = np.zeros((4, STEP))
        all_system_cost2 = np.zeros((4, STEP))
        delay_cost = np.zeros((4, STEP))
        energy_cost = np.zeros((4, STEP))
        processed_task = np.zeros((4, STEP))

        is_end = [0, 0, 0, 0]
        w1 = 1
        w2 = 1
        k_index = 0

        for k in range(STEP_SIZE, MAX_TASK+1, STEP_SIZE):

            print("FU : ", FU, "STEP : ", k_index + 1, "No. of tasks : ", k)

            if is_end[0]==0:
                bus_simul, result1, cost1, cost2, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1, t_count = proposed_algorithm2(FU, 0, lcoa_mode, simul_time, k)  # 제안 알고리즘
                processed_task[0][k_index] = t_count

                all_system_cost1[0][k_index] = round(result1+cost1, 3)
                all_system_cost2[0][k_index] = round(result1+cost2, 3)

                if k==t_count:
                    system_cost[0][k_index] = round(result1 / t_count, 3)
                    delay_cost[0][k_index] = round(cvxpy.sum(mum1).value * w1 / t_count, 3)
                    energy_cost[0][k_index] = round(system_cost[0][k_index] - delay_cost[0][k_index], 3)
                    uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / t_count, 3)
                    for b in range(NUM_BUS):
                        bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / t_count, 3)

                #if (k_index >=1) and (processed_task[0][k_index-1] == processed_task[0][k_index]):
                    #is_end[0]=t_count
                #elif t_count < k:
                    #is_end[0] = t_count

                print("Proposed가 처리한 task 개수 :", t_count)

            else:
                processed_task[0][k_index] = processed_task[0][k_index-1]
                system_cost[0][k_index] = system_cost[0][k_index-1]
                delay_cost[0][k_index] = delay_cost[0][k_index-1]
                energy_cost[0][k_index] = energy_cost[0][k_index-1]
                uav_ratio1[k_index] = uav_ratio1[k_index-1]
                bus_ratio1[k_index] = bus_ratio1[k_index-1]

            if is_end[1]==0:
                result2, cost1, cost2, rho_um2, fum2, mum2, num_bus2, e_um_cost, t_count = uav_only_algorithm(FU, 0,lcoa_mode, simul_time, k)  # uav only 알고리즘

                processed_task[1][k_index] = t_count
                all_system_cost1[1][k_index] = round(result2 + cost1, 3)
                all_system_cost2[1][k_index] = round(result2 + cost2, 3)

                if k == t_count:
                    system_cost[1][k_index] = round(result2 / t_count, 3)
                    delay_cost[1][k_index] = round(cvxpy.sum(mum2).value * w1 / t_count, 3)
                    energy_cost[1][k_index] = round(system_cost[1][k_index] - delay_cost[1][k_index], 3)

                #if (k_index >=1) and (processed_task[1][k_index-1] == processed_task[1][k_index]):
                    #is_end[1]=t_count
                #elif t_count < k:
                    #is_end[1] = t_count

                print("LC가 처리한 task 개수 :", t_count)

            else:
                processed_task[1][k_index] = processed_task[1][k_index-1]
                system_cost[1][k_index] = system_cost[1][k_index-1]
                delay_cost[1][k_index] = delay_cost[1][k_index-1]
                energy_cost[1][k_index] = energy_cost[1][k_index-1]


            if is_end[2]==0:
                result3, cost1, cost2, rho_bm3, fbm3, mum3, t_count = bus_only_algorithm(FU, 0,lcoa_mode, simul_time, k)  # bus only 알고리즘

                processed_task[2][k_index] = t_count
                all_system_cost1[2][k_index] = round(result3 + cost1, 3)
                all_system_cost2[2][k_index] = round(result3 + cost2, 3)

                if k == t_count:
                    system_cost[2][k_index] = round(result3 / t_count, 3)
                    delay_cost[2][k_index] = round(cvxpy.sum(mum3).value * w1 / t_count, 3)
                    energy_cost[2][k_index] = round(system_cost[2][k_index] - delay_cost[2][k_index], 3)

                #if (k_index >=1) and (processed_task[2][k_index-1] == processed_task[2][k_index]):
                    #is_end[2]=t_count
                #elif t_count < k:
                    #is_end[2] = t_count

                print("FO가 처리한 task 개수 :", t_count)

            else:
                processed_task[2][k_index] = processed_task[2][k_index-1]
                system_cost[2][k_index] = system_cost[2][k_index-1]
                delay_cost[2][k_index] = delay_cost[2][k_index-1]
                energy_cost[2][k_index] = energy_cost[2][k_index-1]


            if is_end[3] == 0:

                result4, cost1, cost2, rho_um4, rho_bm4, fum4, fbm4, mum4, t_count = fixed_algorithm(FU, 0,lcoa_mode, simul_time, k)  # fixed 알고리즘

                processed_task[3][k_index] = t_count
                all_system_cost1[3][k_index] = round(result4 + cost1, 3)
                all_system_cost2[3][k_index] = round(result4 + cost2, 3)

                if k == t_count:
                    system_cost[3][k_index] = round(result4 / t_count, 3)
                    delay_cost[3][k_index] = round(cvxpy.sum(mum4).value * w1 / t_count, 3)
                    energy_cost[3][k_index] = round(system_cost[3][k_index] - delay_cost[3][k_index], 3)
                    uav_ratio2[k_index] = round(cvxpy.sum(rho_um4).value / t_count, 3)

                    for b in range(NUM_BUS):
                        bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm4[:, b:b + 1:1]).value / t_count, 3)

                #if (k_index >= 1) and (processed_task[3][k_index - 1] == processed_task[3][k_index]):
                    #is_end[3] = t_count
                #elif t_count < k:
                    #is_end[3] = t_count

                print("FPO가 처리한 task 개수 :", t_count)

            else:
                processed_task[3][k_index] = processed_task[3][k_index-1]
                system_cost[3][k_index] = system_cost[3][k_index-1]
                delay_cost[3][k_index] = delay_cost[3][k_index-1]
                energy_cost[3][k_index] = energy_cost[3][k_index-1]
                uav_ratio2[k_index] = uav_ratio2[k_index-1]
                bus_ratio2[k_index] = bus_ratio2[k_index-1]

            print(uav_ratio1[k_index])
            print(bus_ratio1[k_index])
            print(uav_ratio2[k_index])
            print(bus_ratio2[k_index])

            print("[All System Cost-1]", end=' ')
            print("Proposed : ", all_system_cost1[0][k_index], "UAV Only : ", all_system_cost1[1][k_index], "Bus Only : ", all_system_cost1[2][k_index], "Fixed : ", all_system_cost1[3][k_index])
            print("[All System Cost-2]", end=' ')
            print("Proposed : ", all_system_cost2[0][k_index], "UAV Only : ", all_system_cost2[1][k_index], "Bus Only : ",
            all_system_cost2[2][k_index], "Fixed : ", all_system_cost2[3][k_index])

            print("[System Cost]", end=' ')
            print("Proposed : ", system_cost[0][k_index], "UAV Only : ", system_cost[1][k_index], "Bus Only : ",
                  system_cost[2][k_index], "Fixed : ", system_cost[3][k_index])

            print("[Delay Cost]", end=' ')
            print("Proposed : ", delay_cost[0][k_index], "UAV Only : ", delay_cost[1][k_index], "Bus Only : ",
                  delay_cost[2][k_index], "Fixed : ", delay_cost[3][k_index])

            print("[Energy Cost]", end=' ')
            print("Proposed : ", energy_cost[0][k_index], "UAV Only : ", energy_cost[1][k_index], "Bus Only : ",
                  energy_cost[2][k_index], "Fixed : ", energy_cost[3][k_index])

            k_index += 1

        uav_ratio1[uav_ratio1 == 0] = np.nan
        bus_ratio1[bus_ratio1 == 0] = np.nan
        uav_ratio2[uav_ratio2 == 0] = np.nan
        bus_ratio2[bus_ratio2 == 0] = np.nan
        system_cost[system_cost == 0] = np.nan
        delay_cost[delay_cost == 0] = np.nan
        energy_cost[energy_cost == 0] = np.nan

        plt.bar(x-1*space, processed_task[0], width, label='Proposed')
        plt.bar(x-0*space, processed_task[1], width, label='LC')
        plt.bar(x+1*space, processed_task[2], width, label='FO')
        plt.bar(x+2*space, processed_task[3], width, label='FPO')

        #plt.plot(x, processed_task[0],  marker=next(marker), label='Proposed')
        #plt.plot(x, processed_task[1],  marker=next(marker), label='LC')
        #plt.plot(x, processed_task[2],  marker=next(marker), label='FO')
        #plt.plot(x, processed_task[3],  marker=next(marker), label='FPO')

        plt.xticks(x)
        plt.ylim([0,MAX_TASK])
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('Number of processed tasks')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)
        plt.savefig("./graphs/" + "UAV_CPU_TASK_NUMBER" + str(FU))
        plt.clf()


        plt.bar(x - 1 * space, system_cost[0], width, label='Proposed')
        plt.bar(x - 0 * space, system_cost[1], width, label='LC')
        plt.bar(x + 1 * space, system_cost[2], width, label='FO')
        plt.bar(x + 2 * space, system_cost[3], width, label='FPO')

        plt.xticks(x)
        plt.ylim([0, 1])
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('Avg. system cost per task')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST" + str(FU))
        plt.clf()

        plt.bar(x - 1 * space, all_system_cost1[0], width, label='Proposed')
        plt.bar(x - 0 * space, all_system_cost1[1], width, label='LC')
        plt.bar(x + 1 * space, all_system_cost1[2], width, label='FO')
        plt.bar(x + 2 * space, all_system_cost1[3], width, label='FPO')

        plt.xticks(x)
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('System cost')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST_ALL1" + str(FU))
        plt.clf()


        plt.bar(x - 1 * space, all_system_cost2[0], width, label='Proposed')
        plt.bar(x - 0 * space, all_system_cost2[1], width, label='LC')
        plt.bar(x + 1 * space, all_system_cost2[2], width, label='FO')
        plt.bar(x + 2 * space, all_system_cost2[3], width, label='FPO')

        plt.xticks(x)
        plt.xlabel('Number of tasks, ' + r'$\mathcal{M}$')
        plt.ylabel('System cost')
        plt.legend(loc='upper center', bbox_to_anchor=(0.5, -0.2),
                   fancybox=True, shadow=True, ncol=4)

        plt.savefig("./graphs/" + "UAV_CPU_SYSTEM_COST_ALL2" + str(FU))
        plt.clf()


        print("UAV의 CPU : ", FU, file=f)
        print("[Number of processed tasks]", file=f)
        print("Proposed : ", processed_task[0], file=f)
        print("UAV Only : ", processed_task[1], file=f)
        print("Bus Only : ", processed_task[2], file=f)
        print("Fixed : ", processed_task[3], file=f)

        print("[System Cost]", file=f)
        print("Proposed : ", system_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(system_cost[0]), file=f)
        print("UAV Only : ", system_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(system_cost[1]), file=f)
        print("Bus Only : ", system_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(system_cost[2]), file=f)
        print("Fixed : ", system_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(system_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(system_cost[0]) / np.mean(system_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(system_cost[0]) / np.mean(system_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(system_cost[0]) / np.mean(system_cost[3]), file=f)
        print("", file=f)

        print("Delay Cost", file=f)
        print("Proposed : ", delay_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(delay_cost[0]), file=f)
        print("UAV Only : ", delay_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(delay_cost[1]), file=f)
        print("Bus Only : ", delay_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(delay_cost[2]), file=f)
        print("Fixed : ", delay_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(delay_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(delay_cost[0]) / np.mean(delay_cost[3]), file=f)
        print("", file=f)

        print("Energy Cost", file=f)
        print("Proposed : ", energy_cost[0], file=f)
        print("Proposed(평균) : ", np.mean(energy_cost[0]), file=f)
        print("UAV Only : ", energy_cost[1], file=f)
        print("UAV Only(평균) : ", np.mean(energy_cost[1]), file=f)
        print("Bus Only : ", energy_cost[2], file=f)
        print("Bus Only(평균) : ", np.mean(energy_cost[2]), file=f)
        print("Fixed : ", energy_cost[3], file=f)
        print("Fixed(평균) : ", np.mean(energy_cost[3]), file=f)
        print("비율(Proposed / UAV) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[1]), file=f)
        print("비율(Proposed / BUS) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[2]), file=f)
        print("비율(Proposed / FIX) : ", np.mean(energy_cost[0]) / np.mean(energy_cost[3]), file=f)

    f.close()


if MODE == 2:  # TASK의 data size를 증가시켜가며 테스트
    # 버스대수는 4대, 태스크개수 10개, 태스크=(5,10,100,150), 딜레이는 1~2로 설정

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))

    make_bus(REAL, NUM_BUS)
    NUM_BUS = count_bus(lcoa_mode, simul_time)

    STEP = 10
    uav_data = np.zeros(STEP)
    bus_data = np.zeros((STEP, NUM_BUS))
    uav_cpu = np.zeros(STEP)
    bus_cpu = np.zeros((STEP, NUM_BUS))
    uav_ratio = np.zeros(STEP)
    bus_ratio = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(1, STEP + 1, 1) * 10
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])
    # plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(1, STEP + 1, 1):

        make_task(5, 10, 50, 150)
        print("STEP : ", k_index + 1)

        bus_simul, result, rho_um, rho_bm, fum, fbm, mum, num_bus, t_count = proposed_algorithm2(FU,0, lcoa_mode, simul_time) # 제안 알고리즘

        if k_index == 0:
            uav_data[k_index] = round(cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_um)).value * 1000,3)
            uav_cpu[k_index] = round(cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_um)).value,3)
            bus_data[k_index] = cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value * 1000
            bus_cpu[k_index] = cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value

        else:
            uav_data[k_index] = uav_data[k_index - 1] + cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_um)).value * 1000
            uav_cpu[k_index] = uav_cpu[k_index - 1] + cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_um)).value
            bus_data[k_index] = bus_data[k_index - 1] + cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value * 1000
            bus_cpu[k_index] = bus_cpu[k_index - 1] + cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value

        uav_ratio[k_index] = uav_data[k_index] / (uav_data[k_index] + np.sum(bus_data[k_index]))
        bus_ratio[k_index] = bus_data[k_index] / (uav_data[k_index] + np.sum(bus_data[k_index]))

        np.set_printoptions(precision=3)

        print("UAV가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_um)).value * 1000)
        print("버스가 처리한 데이터량 : ", cvxpy.sum(cvxpy.multiply(sm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value * 1000)
        print("UAV가 처리한 CPU량(GHz) : ", cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_um)).value)
        print("버스가 처리한 CPU량(GHz) : ", cvxpy.sum(cvxpy.multiply(cm[0:NUM_TASK], rho_bm), axis=0, keepdims=True).value)
        print("UAV offloading ratio : ", cvxpy.sum(rho_um).value / 10)
        print("버스 offloading ratio : ", cvxpy.sum(rho_bm, axis=0, keepdims=True).value / 10)

        k_index += 1

    y1 = bus_data.transpose()
    y2 = bus_cpu.transpose()
    y3 = bus_ratio.transpose()

    plt.plot(x, uav_data, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1) + " : " + str(round(bus_simul[b].cpu)) + " GHz")

    plt.xticks(x)
    plt.xlabel('Number of tasks')
    plt.ylabel(r'The amount of data processed ($S_m \times \rho$, Mbits)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "TASK_SIZE_BIT1")
    plt.clf()

    plt.plot(x, uav_cpu, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y2[b], marker=next(marker),
                 label=r'$b_{%d}$' % (b + 1) + " : " + str(round(bus_simul[b].cpu)) + " GHz")

    plt.xticks(x)
    plt.xlabel('Number of tasks')
    plt.ylabel('The amount of computation (GHz)')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "TASK_SIZE_BIT2")
    plt.clf()

    plt.plot(x, uav_ratio, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y3[b], marker=next(marker),
                 label=r'$b_{%d}$' % (b + 1) + " : " + str(round(bus_simul[b].cpu)) + " GHz")

    plt.xticks(x)
    plt.xlabel('Number of tasks')
    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "TASK_SIZE_BIT3")
    plt.clf()

    f = open('./graphs/TASK_SIZE_BIT.txt', 'w')

    for b in range(NUM_BUS):
        print("bus", b + 1, ": ", bus_simul[b].distance, "m,", bus_simul[b].cpu, " GHz", file=f)

    print("UAV가 처리한 데이터량 : ", uav_data, file=f)
    print("UAV가 처리한 데이터량(평균) : ", np.mean(uav_data), file=f)
    print("버스가 처리한 데이터량 : ", y1, file=f)
    print("버스가 처리한 데이터량(버스평균) : ", np.mean(y1, axis=1), file=f)
    print("버스가 처리한 데이터량(전체평균) : ", np.mean(y1), file=f)

    print("UAV가 처리한 CPU량 : ", uav_cpu, file=f)
    print("UAV가 처리한 CPU(평균) : ", np.mean(uav_cpu), file=f)
    print("버스가 처리한 CPU량 : ", y2, file=f)
    print("버스가 처리한 CPU량(버스평균) : ", np.mean(y2, axis=1), file=f)
    print("버스가 처리한 CPU량(전체평균) : ", np.mean(y2), file=f)

    print("UAV가 처리한 ratio : ", uav_ratio, file=f)
    print("UAV가 처리한 ratio(평균) : ", np.mean(uav_ratio), file=f)
    print("버스가 처리한 ratio : ", y3, file=f)
    print("버스가 처리한 ratio(버스평균) : ", np.mean(y3, axis=1), file=f)
    print("버스가 처리한 ratio(전체평균) : ", np.mean(y3), file=f)

    f.close()

if MODE == 3:  # BUS 1대의 CPU를 증가시켜가며 실험
    # 버스대수=3, 태스크=(5,10,100,150)

    BUS_SAME_CPU = 10
    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS, BUS_SAME_CPU, BUS_SAME_CPU)
    NUM_BUS = count_bus(lcoa_mode, simul_time)

    make_task(5, 10, 50, 100)

    STEP = FB_MAX // 3
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

    x = np.arange(3, FB_MAX + 1, 3)

    for k in range(3, FB_MAX + 1, 3):

        print("STEP : ", k_index + 1, "BUS1 CPU : ", k)

        bus_simul, result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1, t_count = proposed_algorithm2(FU, k, lcoa_mode, simul_time) # 제안 알고리즘
        uav_ratio1[k_index] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)
        for b in range(num_bus1):
            bus_ratio1[k_index][b] = round(cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK, 3)

        system_cost1[k_index] = round(result1, 3)
        delay_cost1[k_index] = round(cvxpy.sum(mum1).value, 3)
        energy_cost1[k_index] = round(system_cost1[k_index] - delay_cost1[k_index], 3)

        bus_simul2, result2, rho_um2, rho_bm2, fum2, fbm2, mum2, num_bus2, t_count = proposed_algorithm2(FU, BUS_SAME_CPU, lcoa_mode, simul_time) # 제안 알고리즘
        uav_ratio2[k_index] = round(cvxpy.sum(rho_um2).value / NUM_TASK, 3)
        for b in range(num_bus2):
            bus_ratio2[k_index][b] = round(cvxpy.sum(rho_bm2[:, b:b + 1:1]).value / NUM_TASK, 3)
        system_cost2[k_index] = round(result2, 3)
        delay_cost2[k_index] = round(cvxpy.sum(mum2).value, 3)
        energy_cost2[k_index] = round(system_cost2[k_index] - delay_cost2[k_index], 3)

        print("System Cost : ", system_cost1[k_index], "Delay Cost : ", delay_cost1[k_index], "Energy Cost : ",
              energy_cost1[k_index])
        print("UAV Ratio : ", uav_ratio1[k_index])
        print("Bus Ratio : ", bus_ratio1[k_index])

        k_index += 1

    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'std-colors', 'no-latex'])
    # plt.style.use(['science', 'std-colors', 'no-latex'])

    y1 = bus_ratio1.transpose()
    plt.plot(x, uav_ratio1, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        if b == 0:
            plt.plot(x, y1[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1))
        else:
            plt.plot(x, y1[b], marker=next(marker),
                     label=r'$b_{%d}$' % (b + 1) + " : " + str(round(bus_simul[b].cpu)) + " GHz")

    plt.xticks(x)
    plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    # plt.legend(loc='lower left', bbox_to_anchor=(0, 1.01, 1, 0.1), mode='expand', ncol=3)
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.ylim([0, 0.8])
    plt.yticks([0, 0.2, 0.4, 0.6, 0.8])
    plt.tight_layout()
    plt.savefig("./graphs/" + "BUS_CPU_RATIO1")
    plt.clf()

    plt.bar(x, delay_cost1, label='Delay cost')
    plt.bar(x, energy_cost1, bottom=delay_cost1, label='Energy cost')
    plt.xticks(x)
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.xlabel('Computing resource of $b_1$, ' + r'$f_1$(GHz)')
    plt.ylabel('System Cost')
    plt.savefig("./graphs/" + "BUS_CPU_COST1")
    plt.clf()

    y2 = bus_ratio2.transpose()
    plt.plot(x, uav_ratio2, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        plt.plot(x, y2[b], marker=next(marker), label=r'$b_{%d}$' % (b + 1))

    plt.xticks(x)
    plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    # plt.legend(bbox_to_anchor=(0.5, -0.27), loc='lower center')
    # plt.legend(loc='lower left', bbox_to_anchor=(0, 1.01, 1, 0.1), mode='expand', ncol=3)
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.ylim([0, 0.8])
    plt.yticks([0, 0.2, 0.4, 0.6, 0.8])
    plt.savefig("./graphs/" + "BUS_CPU_RATIO2")
    plt.clf()

    plt.bar(x, delay_cost2, label='Delay cost')
    plt.bar(x, energy_cost2, bottom=delay_cost2, label='Energy cost')
    plt.xticks(x)
    plt.xlabel('Computing resource of buses, ' + r'$f_b$(GHz)')
    plt.ylabel('System Cost')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "BUS_CPU_COST2")
    plt.clf()

    f = open('./graphs/BUS_CPU_RATIO.txt', 'w')

    for b in range(NUM_BUS):
        print("bus", b + 1, ": ", bus_simul[b].distance, "m,", bus_simul[b].cpu, " GHz", file=f)

    #print("w1 :", w1, "w2 :", w2, file=f)

    print("BUS1만 CPU증가하는 경우", file=f)
    print("UAV ratio : ", uav_ratio1, file=f)
    print("UAV ratio(평균) : ", np.round(np.mean(uav_ratio1), 2), file=f)
    print("Bus ratio : ", y1, file=f)
    print("Bus ratio(버스평균) : ", np.round(np.mean(y1, axis=1), 2), file=f)
    print("Bus ratio(전체평균) : ", np.round(np.mean(y1), 2), file=f)
    print("System Cost : ", system_cost1, file=f)
    print("System Cost(평균) : ", np.round(np.mean(system_cost1), 2), file=f)
    print("Delay Cost : ", delay_cost1, file=f)
    print("Delay Cost(평균) : ", np.round(np.mean(delay_cost1), 2), file=f)
    print("Energy Cost : ", energy_cost1, file=f)
    print("Energy Cost(평균) : ", np.round(np.mean(energy_cost1), 2), file=f)

    print("모든 BUS의 CPU증가하는 경우", file=f)
    print("UAV ratio : ", uav_ratio2, file=f)
    print("UAV ratio(평균) : ", np.round(np.mean(uav_ratio2), 2), file=f)
    print("Bus ratio : ", y2, file=f)
    print("Bus ratio(버스평균) : ", np.round(np.mean(y2, axis=1), 2), file=f)
    print("Bus ratio(전체평균) : ", np.round(np.mean(y2), 2), file=f)
    print("System Cost : ", system_cost2, file=f)
    print("System Cost(평균) : ", np.round(np.mean(system_cost2), 2), file=f)
    print("Delay Cost : ", delay_cost2, file=f)
    print("Delay Cost(평균) : ", np.round(np.mean(delay_cost2), 2), file=f)
    print("Energy Cost : ", energy_cost2, file=f)
    print("Energy Cost(평균) : ", np.round(np.mean(energy_cost2), 2), file=f)

    f.close()

if MODE == 5:  # UAv가 버스를 찾는 거리를 증가시키면서 시뮬레이션

    SIMUL_TIME = 14
    BUS_TIME = 12

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))

    make_bus(REAL, NUM_BUS)
    make_task(3, 10, 50, 100)

    system_cost = np.zeros((SIMUL_TIME, BUS_TIME))
    uav_ratio = np.zeros((SIMUL_TIME, BUS_TIME))
    bus_ratio = np.zeros((SIMUL_TIME, BUS_TIME))
    bus_num = np.zeros((SIMUL_TIME, BUS_TIME))
    task_num = np.zeros((SIMUL_TIME, BUS_TIME))

    for k in range(SIMUL_TIME):

        distance = (k + 1) * 25

        for j in range(BUS_TIME):

            print("STEP : ", k + 1, "TIME :", j + 1)

            simul_time = j

            bus_simul, result1, cost1, cost2, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1, t_count = proposed_algorithm2(FU, 0, lcoa_mode, simul_time, NUM_TASK, distance)  # 제안 알고리즘
            bus_num[k][j]=num_bus1
            task_num[k][j]=t_count

            if rho_um1 is None:
                pass

            else:
                uav_ratio[k][j] = round(cvxpy.sum(rho_um1).value / NUM_TASK, 3)
                bus_ratio[k][j] = round(cvxpy.sum(rho_bm1).value / NUM_TASK, 3)
                system_cost[k][j] = round(result1, 3)

            print("System Cost : ", system_cost[k][j])
            print("UAV Ratio : ", uav_ratio[k][j])
            print("Bus Ratio : ", bus_ratio[k][j])
            print("Bus Count : ", bus_num[k][j])
            print("처리한 Task : ", task_num[k][j])

            j += 1

        k += 1

    uav_ratio[uav_ratio == 0] = np.nan
    bus_ratio[bus_ratio == 0] = np.nan
    system_cost[system_cost == 0] = np.nan

    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])
    plt.xlabel(r'$L_{CoA}$ (m)')

    x = np.arange(50, 701, 50)
    #plt.plot(x, bus_ratio, marker=next(marker), label="Bus ratio")
    plt.plot(x, np.nanmean(system_cost, axis=1), marker=next(marker), label="System cost")
    plt.bar(x, np.mean(bus_num, axis=1), 20, color='#3399e6', label="Number of buses")

    plt.xticks(x)
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.tight_layout()
    plt.savefig("./graphs/" + "DISTANCE_RATIO")
    plt.clf()

    COLOR_TEMPERATURE = "black"
    COLOR_PRICE = "blue"
    fig, ax1 = plt.subplots(figsize=(8, 8))
    ax2 = ax1.twinx()

    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])
    ax1.plot(x, np.nanmean(system_cost, axis=1), marker=next(marker), label="System cost", color=COLOR_TEMPERATURE, lw=3)
    ax2.plot(x, np.mean(bus_num, axis=1), marker=next(marker), label="Number of buses", color=COLOR_PRICE, lw=4)

    ax1.set_xlabel(r'$L_{CoA}$ (m)')
    ax1.set_ylabel('Optimal offloading ratio, ' + r'$\rho_{u,m}$', color=COLOR_TEMPERATURE, fontsize=14)
    ax1.tick_params(axis="y", labelcolor=COLOR_TEMPERATURE)

    ax2.set_ylabel("Number of buses", color=COLOR_PRICE, fontsize=14)
    ax2.tick_params(axis="y", labelcolor=COLOR_PRICE)
    plt.savefig("./graphs/" + "DISTANCE_RATIO2")
    plt.clf()

    f = open('./graphs/LOCA_DISTANCE.txt', 'w')

    print("UAV ratio : ", uav_ratio, file=f)
    print("UAV ratio(평균1) : ", np.round(np.mean(uav_ratio), 2), file=f)
    print("UAV ratio(평균2) : ", np.round(np.mean(uav_ratio, axis=1), 2), file=f)
    print("Bus ratio : ", bus_ratio, file=f)
    print("Bus ratio(평균1) : ", np.round(np.mean(bus_ratio), 2), file=f)
    print("Bus ratio(평균2) : ", np.round(np.mean(bus_ratio, axis=1), 2), file=f)
    print("System Cost : ", system_cost, file=f)
    print("System Cost(평균1) : ", np.round(np.nanmean(system_cost), 2), file=f)
    print("System Cost(평균2) : ", np.round(np.nanmean(system_cost, axis=1), 2), file=f)
    print("Bus count : ", bus_num, file=f)
    print("Bus count(평균1) : ", np.round(np.mean(bus_num), 2), file=f)
    print("Bus count(평균2) : ", np.round(np.mean(bus_num, axis=1), 2), file=f)
    print("처리한 Task : ", task_num, file=f)
    print("처리한 Task(평균1) : ", np.round(np.mean(task_num), 2), file=f)
    print("처리한 Task(평균2) : ", np.round(np.mean(task_num, axis=1), 2), file=f)

if MODE == 90:  # omega1를 증가시켜가며 실험
    # 버스대수는 4대, 태스크개수는 1개, 태스크=(5,10,100,150), 딜레이는 1~2로 설정

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))
    make_bus(REAL, NUM_BUS)
    NUM_BUS = count_bus()
    #cal_distance()

    make_task(5, 10, 100, 150)

    STEP = 20
    system_cost = np.zeros(STEP)
    uav_ratio = np.zeros(STEP)
    bus_ratio = np.zeros((STEP, NUM_BUS))
    k_index = 0

    x = np.arange(1, STEP + 1, 1)
    marker = itertools.cycle(('+', '2', '.', 'x', '*'))
    plt.style.use(['science', 'ieee', 'no-latex'])

    for k in range(1, STEP + 1, 1):

        print("STEP : ", k_index + 1, "Omega1 : ", k, " Omega2 : ", STEP / 2)
        result1, rho_um1, rho_bm1, fum1, fbm1, mum1 = proposed_algorithm2(FU, k, STEP / 2)  # 제안 알고리즘

        uav_ratio[k_index] = cvxpy.sum(rho_um1).value / NUM_TASK
        for b in range(NUM_BUS):
            bus_ratio[k_index][b] = cvxpy.sum(rho_bm1[:, b:b + 1:1]).value / NUM_TASK

        system_cost[k_index] = result1

        print("System Cost : ", np.round(system_cost[k_index], 2))
        print("UAV Ratio : ", np.round(uav_ratio[k_index], 2))
        print("Bus Ratio : ", np.round(bus_ratio[k_index], 2))

        k_index += 1

    y = bus_ratio.transpose()

    plt.plot(x, uav_ratio, marker=next(marker), label="UAV")
    for b in range(NUM_BUS):
        # plt.plot(x, y[b], marker=next(marker), label="BUS" + str(b + 1)  + " : " + str(buses_original[b].cpu) +"GHz, " + str(buses_original[b].distance) +"m")
        plt.plot(x, y[b], marker=next(marker),
                 label=r'$b_{%d}$' % (b + 1) + " : " + str(buses_original[b].distance) + " m, " + str(
                     round(buses_original[b].cpu)) + " GHz")
    plt.xticks(x)
    plt.xlabel(r'$\omega_1$')

    plt.ylabel('Optimal task offloading ratio, ' + r'$\rho$')
    plt.legend(loc='best')
    plt.legend(frameon=True)
    plt.savefig("./graphs/" + "OMEGA_RATIO1")
    plt.clf()

if MODE == 99:  # 버스를 이동하면서 시뮬레이션

    X_STEP = 5
    Y_STEP = 5

    simul_distance = MAP_SIZE / X_STEP / 2

    SIMUL_TIME = 10
    MAP_DIV_COUNT = X_STEP * Y_STEP

    MAP = [[0 for j in range(Y_STEP)] for i in range(X_STEP)]
    for i in range(X_STEP):
        for j in range(Y_STEP):
            MAP[i][j] = [100 + MAP_SIZE / X_STEP * j, 100 + MAP_SIZE / X_STEP * i, 100]

    make_bus(REAL, BUS_NUM)
    make_task(10, 20, 100, 200)

    system_cost = [[[0 for i in range(Y_STEP)] for j in range(X_STEP)] for k in range(SIMUL_TIME)]
    bus_count = [[[0 for i in range(Y_STEP)] for j in range(X_STEP)] for k in range(SIMUL_TIME)]

    simul_index = 0

    for i in range(SIMUL_TIME):
        #count_bus()
        for bus in buses_original:
            bus.move()

    for i in range(SIMUL_TIME):

        k_index = 0

        for j in range(X_STEP):
            for k in range(Y_STEP):

                uavs_original.append(UAV(0, MAP[j][k][0], MAP[j][k][1], MAP[j][k][2]))
                BUS_MAP = busmap()

                print("STEP : ", k_index + 1, end=" ")

                if BUS_MAP[j][k] > 0:
                    result1, rho_um1, rho_bm1, fum1, fbm1, mum1, num_bus1 = proposed_algorithm2(FU, 1,
                                                                                                simul_distance)  # 제안 알고리즘
                    system_cost[simul_index][j][k] = round(result1, 2)
                    print("Proposed : ", system_cost[simul_index][j][k])
                    bus_count[i][j][k] = BUS_MAP[j][k]
                else:
                    result2, rho_um2, rho_bm2, fum2, fbm2, mum2, num_bus2, e_um_cost = uav_only_algorithm(FU, 1, simul_distance)  # uav only 알고리즘
                    system_cost[simul_index][j][k] = round(result2, 2)
                    print("UAV only : ", system_cost[simul_index][j][k])

                k_index += 1

        for bus in buses_original:
            bus.move()
        cal_distance()

        simul_index += 1

    fontlabel = {"fontsize": "large", "color": "gray", "fontweight": "bold"}
    system_cost2 = np.mean(system_cost, axis=0)
    system_cost3 = (np.max(system_cost2) - system_cost2) / np.max(system_cost2) * 100
    print(np.max(system_cost2))
    print(system_cost2)
    print(system_cost3)

    bus_count2 = np.mean(bus_count, axis=0)

    # fig = plt.figure()
    fig = plt.figure(figsize=plt.figaspect(0.5))
    ax1 = fig.add_subplot(1, 2, 1, projection='3d')
    ax2 = fig.add_subplot(1, 2, 2, projection='3d')

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

                # ax1.plot_surface(X[x], Y[y], bus_count2[x][y], alpha=0.5, shade=True)
                # ax2.plot_surface(X[x], Y[y], system_count2[x][y], alpha=0.5, shade=True)

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
if MODE == 100:

    for i in range(NUM_UAV):
        uavs_original.append(UAV(i, X, Y, Z))

    make_bus(REAL, NUM_BUS)
    cal_bus()
