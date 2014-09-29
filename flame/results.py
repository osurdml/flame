import os, sys
import config
#import pylab
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

    plt.errorbar(range(min_len), weighted_data_ave, yerr = weighted_data_sem, label = 'weighted')
    plt.errorbar(range(min_len), max_data_ave, yerr = max_data_sem, label = 'max')
    plt.errorbar(range(min_len), basic_data_ave, yerr = basic_data_sem, label = 'basic')
    plt.legend(loc= 'upper right')
    plt.show()
