# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 13:13:01 2020

@author: FujiiChang
"""

import sys
sys.path.append('../')

import math
from numpy.core.shape_base import _accumulate
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import colors, patches
from statistics import mean
from const import parameter

diff_distance = []
earth_R = 6378137 # radius of the earth

parameter = parameter()
workspace = parameter.workspace

def calc_diff_longitude(target_lon, latitude, longitude):
    theta = mean(latitude)
    ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # keido, longitude 1[deg] = ex[m]
    start_longitude = target_lon
    diff_longitude = []
    for i in range(len(longitude)):
        diff_deg = longitude[i] - start_longitude
        diff_m = diff_deg / ex
        diff_longitude.append(diff_m)
    return diff_longitude

def calc_diff_latitude(target_lon, latitude):
    ey = 360/(2*math.pi*earth_R) # ido, latitude 1[deg] = ey[m]
    start_latitude = target_lon
    diff_latitude = []
    for i in range(len(latitude)):
        diff_deg = latitude[i] - start_latitude
        diff_m = diff_deg / ey
        diff_latitude.append(diff_m)
    return diff_latitude
    
def pos_plotter(str_date, title, diff_longitude, diff_latitude):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    plt.scatter(diff_longitude, diff_latitude, s=3)
    plt.scatter(0, 0, s=30, c="magenta")
    
    lim  = 10.0 # the max & min distance from the start position
    resolution = 1.0 # the width of the line in the graph
    max_x = lim
    max_y = lim
    min_x = -1 * lim
    min_y = -1 * lim
    plt.xlim(min_x, max_x)
    plt.ylim(min_y, max_y)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    x_ticks = [min_x]
    y_ticks = [min_y]
    for i in range(int(lim/resolution)*2+1):
        x_ticks.append(min_x+resolution*i)
        y_ticks.append(min_y+resolution*i)
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    plt.title(title)
    plt.grid()
    fig.savefig(workspace + str_date + "/pos_" + title + ".jpg")
    plt.clf()
    plt.close()
    # plt.show()

def tiemseries_pos_plotter(str_date, title, diff_longitude, diff_latitude, count):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.set_aspect('equal')
    plt.grid()
    plt.scatter(diff_longitude, diff_latitude, s=3)
    plt.scatter(0, 0, s=30, c="magenta")
    
    patch = patches.Circle(xy=(0, 0), radius=3, fill=False)
    ax.add_patch(patch)
    lim  = 10.0 # the max & min distance from the start position
    resolution = 1.0 # the width of the line in the graph
    max_x = lim
    max_y = lim
    min_x = -1 * lim
    min_y = -1 * lim
    plt.xlim(min_x, max_x)
    plt.ylim(min_y, max_y)
    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    x_ticks = [min_x]
    y_ticks = [min_y]
    for i in range(int(lim/resolution)*2+1):
        x_ticks.append(min_x+resolution*i)
        y_ticks.append(min_y+resolution*i)
    plt.xticks(x_ticks)
    plt.yticks(y_ticks)
    plt.title(title)
    plt.savefig(workspace + str_date + "/"  + title + "/" + str(count) + ".jpg")
    plt.cla()
    plt.clf()
    plt.close()

def power_plotter(str_date, title, p_data, max_y):
    fig = plt.figure()
    max_x = len(p_data) + 1
    plt.xlim(0, max_x)
    plt.ylim(0, max_y)
    plt.xlabel("t [ms]")
    plt.ylabel("P [J]")
    plt.title(title)
    plt.plot(p_data)
    plt.grid()
    plt.savefig(workspace + str_date + "/P_" + title + ".jpg")
    plt.clf()
    plt.close()

def calc_mean_diff_distance(diff_longitude, diff_latitude):
    for i in range(len(diff_latitude)):
        diff = math.sqrt(diff_longitude[i]**2+diff_latitude[i]**2)
        diff_distance.append(diff)
    avg = mean(diff_distance)
    return avg

def pos_data_extraction(data_file):
    data = pd.read_csv(data_file, index_col=0)
    a_longitude = data['a_longitude'].values.tolist()
    a_latitude = data['a_latitude'].values.tolist()   
    e_longitude = data['e_longitude'].values.tolist()
    e_latitude = data['e_latitude'].values.tolist()
    return a_longitude, a_latitude, e_longitude, e_latitude

def p_data_extraction(data_file):
    data = pd.read_csv(data_file, index_col=0)
    P = data['P'].values.tolist()
    return P

def acum_power_plot(length, data_list, label_list, color_list, disturbance):
    i = 0
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    maxP_list = []
    for file in data_list:
        maxP_list.append(file[-1])
    max_P = max(maxP_list) + 100

    for data in data_list:
        s = "solid"
        c = color_list[i%4]
        if i < 4:
            pass
        else:
            s = "dashed"
        ax.plot(length, data, label=label_list[i], linestyle=s, color=c)
        i = i + 1
    xmax = length[-1]+1
    ymax = max_P
    ymax = 250
    for d in disturbance:
        ax.vlines(d, 0, ymax, colors='pink', linestyle='dashed', linewidth=1)
    plt.xlim(0, xmax)
    plt.ylim(0, ymax)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("P [J]")
    ax.set_title("The comparison of electric energy between each strategy")
    ax.legend(loc='upper left')
    plt.show()

def make_power_consumption_graph(data_list, str_date, s='Simple'):
    df = pd.DataFrame()
    for file in data_list:
        print(file)
        if 'Simple' in file:
            s = 'Simple'
        elif 'Strict' in file:
            s = 'Strict'
        elif 'Flexible' in file:
            s = 'Flexible'
        elif 'Diagonal' in file:
            s = 'Diagonal'
        data = pd.read_csv(file, index_col=0)
        P = data['P'].values.tolist()
        df[s] = P
    path = workspace + str_date + '/power_consumption.csv'
    df.to_csv(path)

    data = pd.read_csv(path, index_col=0, skipinitialspace=True)
    simple = np.array(data['Simple'].values.tolist())
    strict = np.array(data['Strict'].values.tolist())
    flexible = np.array(data['Flexible'].values.tolist())
    diagonal = np.array(data['Diagonal'].values.tolist())
    data_length = len(simple)
    timestep = parameter.TIME_STEP/1000
    count = np.arange(0, data_length*timestep, timestep)
    data_list = [simple, strict, flexible, diagonal]
    label_list = ["Simple", "Strict", "Flexible", "Diagonal"]
    color_list = ["green", "red", "blue", "orange"]
    disturbance_timing = np.arange(0, data_length, data_length/5)
    acum_power_plot(count, data_list, label_list, color_list, disturbance_timing)

if __name__ == '__main__':
    path = 'C:/Users/is0232xf/OneDrive - 学校法人立命館/デスクトップ/exp/'
    data = pd.read_csv(path + "power_consumption_cp.csv", index_col=0, skipinitialspace=True)
    simple = np.array(data['Simple'].values.tolist())
    strict = np.array(data['Strict'].values.tolist())
    flexible = np.array(data['Flexible'].values.tolist())
    diagonal = np.array(data['Diagonal'].values.tolist())
    e_simple = np.array(data['simple'].values.tolist())
    e_strict = np.array(data['strict'].values.tolist())
    e_flexible = np.array(data['flexible'].values.tolist())
    e_diagonal = np.array(data['diagonal'].values.tolist())
    count = np.arange(0, len(simple)*0.016, 0.016)
    data_list = [simple, strict, flexible, diagonal, e_simple, e_strict, e_flexible, e_diagonal]
    label_list = ["Simple without GPS error", "Strict without GPS error", "Flexible without GPS error", "Diagonal without GPS error",
                "Simple with GPS error", "Strict with GPS error", "Flexible with GPS error", "Diagonal with GPS error"]
    color_list = ["green", "red", "blue", "orange"]
    disturbance_timing = np.arange(0, 1800, 360)
    acum_power_plot(count, data_list, label_list, color_list, disturbance_timing)