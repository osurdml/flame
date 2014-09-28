import os, sys
import config
#import pylab
import matplotlib.pyplot as plt
import numpy as np

for root,dirs,files in os.walk('/home/ryan/Documents/Programming/flame/output'):
    ll = []
    basic_ave = []
    basic_num = 0
    max_ave = []
    weighted_ave = []
    max_num = 0
    weighted_num = 0
    min_yvalues = True
    for f in files:
        label = f
        ll.append(label.replace(".txt", ""))
        dname = os.path.join(root,f)
        data_list = np.loadtxt(dname)
        yvalues = range(len(data_list))

        if min_yvalues == True:
            min_yvalues = yvalues

        if len(yvalues) < len(min_yvalues):
            min_yvalues = yvalues

        if 'BASIC' in f:
            basic_num = basic_num + 1
            for i in range(len(data_list)):
                if len(basic_ave) < len(data_list):
                    basic_ave.append(data_list[i])
                if basic_num == len(data_list):
                    basic_ave[i] = (data_list[i] + basic_ave[i])
                    for j in range(len(basic_ave)):
                        basic_ave[j] = basic_ave[j] / basic_num
                else:
                    basic_ave[i] = (data_list[i] + basic_ave[i])

        if 'MAX' in f:
            max_num = max_num + 1
            for i in range(len(data_list)):
                if len(max_ave) < len(data_list):
                    max_ave.append(data_list[i])
                if max_num == len(data_list):
                    max_ave[i] = (data_list[i] + max_ave[i])
                    for j in range(len(max_ave)):
                        max_ave[j] = max_ave[j] / max_num
                else:
                    max_ave[i] = (data_list[i] + max_ave[i])

        if 'WEIGHTED' in f:
            weighted_num = weighted_num + 1
            for i in range(len(data_list)):
                if len(weighted_ave) < len(data_list):
                    weighted_ave.append(data_list[i])
                if weighted_num == len(data_list):
                    weighted_ave[i] = (data_list[i] + weighted_ave[i])
                    for j in range(len(weighted_ave)):
                        weighted_ave[j] = weighted_ave[j] / weighted_num
                else:
                    weighted_ave[i] = (data_list[i] + weighted_ave[i])

    print basic_ave[0:len(min_yvalues)]
    min_yvalues = range(550)
    plt.plot(min_yvalues, weighted_ave[0:len(min_yvalues)], label = 'weighted')
    plt.plot(min_yvalues, basic_ave[0:len(min_yvalues)], label = 'basic')
    plt.plot(min_yvalues, max_ave[0:len(min_yvalues)], label = 'max')
    plt.legend(loc= 'upper right')
    plt.show()
