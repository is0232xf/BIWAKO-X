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
    print(title)
    fig.savefig(workspace + str_date + "/pos_" + title + ".jpg")
    plt.clf()
    plt.close()
    # plt.show()

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
        elif 'Oct-directional' in file:
            s = 'Oct-directional'
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
    oct_directional = np.array(data['Oct-directional'].values.tolist())
    timestep = (parameter.TIME_STEP/1000)/60 # 秒単位の時TIME_STEP/1000, 分単位の時さらに/60
    count_max_value = len(simple)*timestep
    count = np.arange(0, int(count_max_value), timestep)
    data_list = [simple, strict, flexible, diagonal, oct_directional]
    label_list = ["Simple", "Strict", "Flexible", "Diagonal", "Oct-directional"]
    color_list = ["green", "red", "blue", "orange", "black"]
    disturbance_timing = np.arange(0, count_max_value, count_max_value/5)
    acum_power_plot(count, data_list, label_list, color_list, disturbance_timing)

def acum_power_plot(count, data_list, label_list, color_list, disturbance):
    i = 0
    fig = plt.figure()
    ax = fig.add_subplot(1,1,1)
    max_P =  800

    for data in data_list:
        s = "solid"
        c = color_list[i%len(data_list)]
        if i < len(data_list):
            pass
        else:
            s = "dashed"
        ax.plot(count, data, label=label_list[i], linestyle=s, color=c)
        i = i + 1
    xmax = count[-1]
    ymax = max_P
    
 
    for d in disturbance:
        ax.vlines(d, 0, ymax, colors='pink', linestyle='dashed', linewidth=1)    
    
    x_ticks = np.arange(0, xmax+(xmax/6)-1, xmax/6)
    plt.xticks(x_ticks)
    plt.xlim(0, xmax)
    plt.ylim(0, ymax)
    ax.set_xlabel("t [min]")
    ax.set_ylabel("P [kJ]")
    ax.set_title("The comparison of electric energy between each strategy")
    ax.legend(loc='upper left')
    plt.show()

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

def make_any_timeseries_graph(csv_file):
    df = pd.read_csv(csv_file, index_col=0)
    d1 = np.array(df[df.columns.values[0]].values.tolist())
    d2 = np.array(df[df.columns.values[1]].values.tolist())
    d3 = np.array(df[df.columns.values[2]].values.tolist())
    d4 = np.array(df[df.columns.values[3]].values.tolist())
    timestep = (100/1000)/60 # 秒単位の時TIME_STEP/1000, 分単位の時さらに/60
    count_max_value = len(d1)*timestep
    count = np.arange(0, int(count_max_value), timestep)
    data_list = [d1, d2, d3, d4]
    label_list = df.columns.values
    color_list = ["green", "red", "blue", "orange", "black"]
    disturbance_timing = np.arange(0, count_max_value, count_max_value/5)
    acum_power_plot(count, data_list, label_list, color_list, disturbance_timing)

if __name__ == '__main__':
    path = 'power_consumption.csv'
    make_any_timeseries_graph(path)