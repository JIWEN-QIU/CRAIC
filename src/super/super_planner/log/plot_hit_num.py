import math

import matplotlib.pyplot as plt
import numpy as np
import numpy as np


#
# # 读取文本文件
# data = np.loadtxt('time_decay_num.txt', delimiter=',')
# data = np.sort(data)
# plt.plot(data)
# plt.show()

def drawDimond(n):

    if(n % 2 == 0):
        n = n + 1
    x = []
    y = []
    r = 1.0
    t = math.pi * 2 / n
    for i in range(n):
        x.append(r * math.cos(i * t))
        y.append(r * math.sin(i * t))

    i = 0
    ordered_x = []
    ordered_y = []
    j = n // 2 + 1
    # while (j != 0):
    #     if (n % j != 0 or j == 1):
    #         i = (i + j) % n
    #         while (i != 0):
    #             ordered_x.append(x[i])
    #             ordered_y.append(y[i])
    #             i = (i + j) % n
    #         ordered_x.append(x[i])
    #         ordered_y.append(y[i])
    #     else:
    #         time = int(n / j)
    #         cnt = 0
    #         while cnt < j:
    #             k = 0
    #             while k < time:
    #                 i = (i + j) % n
    #                 ordered_x.append(x[i])
    #                 ordered_y.append(y[i])
    #                 k = k + 1
    #             i = (i + j - 1) % n
    #             ordered_x.append(x[i])
    #             ordered_y.append(y[i])
    #             cnt = cnt + 1
    #     j = j - 1
    i = 0
    while(i < n):
        ordered_x.append(x[i])
        ordered_y.append(y[i])
        i = (i+j) % n
        if(i==0):
            ordered_x.append(x[i])
            ordered_y.append(y[i])
            break

    ordered_x = np.array(ordered_x)
    ordered_y = np.array(ordered_y)
    ordered_x = ordered_x - ordered_x[0]
    ordered_y = ordered_y - ordered_y[0]
    cmp = plt.get_cmap("jet")
    for i in range(len(ordered_x)-1):
        linex = [ordered_x[i], ordered_x[i+1]]
        liney = [ordered_y[i], ordered_y[i+1]]
        plt.plot(linex, liney,color = cmp(1.0-i/len(ordered_x)), linewidth = (1-i/len(ordered_x))*10+1)
    # plt.plot(ordered_x, ordered_y)
    plt.axis('equal')
    plt.show()


    for i in range(len(ordered_x)):
        print(ordered_x[i], ordered_y[i], 1.2)


drawDimond(7)
