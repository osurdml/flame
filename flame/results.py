import os, sys
import config
from pylab import *
import matplotlib.pyplot as plt
import numpy as np
import math

for root,dirs,files in os.walk('/home/ryan/Documents/Programming/flame/output'):
    ll = []
    max_data = []
    basic_data = []
    weighted_data = []
    for f in files:
        label = f
        ll.append(label.replace(".txt", ""))
        dname = os.path.join(root,f)
        data_list = np.loadtxt(dname)
        if "BASIC" in dname:
            basic_data.append(data_list)
        if "MAX" in dname:
            max_data.append(data_list)
        if "WEIGHTED" in dname:
            weighted_data.append(data_list)

    min_len = True
    for i in range(len(weighted_data)):
        if min_len == True:
            min_len = len(weighted_data[i])
        if len(weighted_data[i]) < min_len:
            min_len = len(weighted_data[i])
    for i in range(len(max_data)):
        if min_len == True:
            min_len = len(max_data[i])
        if len(max_data[i]) < min_len:
            min_len = len(max_data[i])
    for i in range(len(basic_data)):
        if min_len == True:
            min_len = len(basic_data[i])
        if len(basic_data[i]) < min_len:
            min_len = len(basic_data[i])

    for i in range(len(weighted_data)):
        weighted_data[i] = weighted_data[i][0:min_len]
    for i in range(len(max_data)):
        max_data[i] = max_data[i][0:min_len]
    for i in range(len(basic_data)):
        basic_data[i] = basic_data[i][0:min_len]

    weighted_data_ave =  np.mean(weighted_data, axis = 0)
    max_data_ave =  np.mean(max_data, axis = 0)
    basic_data_ave =  np.mean(basic_data, axis = 0)
    weighted_data_sem = np.true_divide(np.std(weighted_data, axis = 0), math.sqrt(len(weighted_data)))
    max_data_sem = np.true_divide(np.std(max_data, axis = 0), math.sqrt(len(max_data)))
    basic_data_sem = np.true_divide(np.std(basic_data, axis = 0), math.sqrt(len(basic_data)))

    fig = plt.figure(figsize=(4,3))
    ax = fig.add_subplot(111)
    plt.fill_between(range(min_len), (weighted_data_ave + weighted_data_sem),  (weighted_data_ave - weighted_data_sem), color='blue', alpha=.3)
    plt.fill_between(range(min_len), (max_data_ave + max_data_sem),  (max_data_ave - max_data_sem), color='red', alpha= .3)
    plt.fill_between(range(min_len), (basic_data_ave + basic_data_sem),  (basic_data_ave - basic_data_sem), color='green', alpha=.3)
    plt.plot(range(min_len), weighted_data_ave, label = 'weighted', color= 'blue', linewidth = 6)
    plt.plot(range(min_len), max_data_ave, label = 'max', color = 'red', linewidth = 3, linestyle = "dashed")
    plt.plot(range(min_len), basic_data_ave, label = 'basic', color = 'green', linewidth = 6)
    ax.set_xlim(0, min_len)
    ax.set_ylim(0)
    ax.tick_params(axis='x', labelsize=50)
    ax.tick_params(axis='y', labelsize=50)
    ax.set_xlabel('Time', fontsize= 100)
    ax.set_ylabel('Comparison Metric', fontsize = 100)
    ax.xaxis.labelpad = 20 
    ax.yaxis.labelpad = 20 
    leg = plt.legend(loc= 'upper left', prop={'size':90})
    for legobj in leg.legendHandles:
            legobj.set_linewidth(8.0)
    plt.show()
