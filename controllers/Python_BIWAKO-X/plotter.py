# -*- coding: utf-8 -*-
"""
Created on Wed Dec  9 13:13:01 2020

@author: FujiiChang
"""

import math
import pandas as pd
import matplotlib.pyplot as plt
from statistics import mean

diff_distance = []

def calc_diff_longitude(target_lon, longitude, ex):
    start_longitude = target_lon
    diff_longitude = []
    for i in range(len(longitude)):
        diff_deg = longitude[i] - start_longitude
        diff_m = diff_deg / ex
        diff_longitude.append(diff_m)
    return diff_longitude

def calc_diff_latitude(target_lon, latitude, ey):
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
    fig.savefig("./result/" + str_date + "/pos_" + title + ".jpg")
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
    plt.show()
    fig.savefig("./result/" + str_date + "/P_" + title + ".jpg")
    plt.clf()
    plt.close()

def calc_mean_diff_distance(diff_longitude, diff_latitude):
    for i in range(len(diff_latitude)):
        diff = math.sqrt(diff_longitude[i]**2+diff_latitude[i]**2)
        diff_distance.append(diff)
    avg = mean(diff_distance)
    return avg

def data_extraction(data_file):
    data = pd.read_csv(data_file, index_col=0)
    longitude = data['longitude'].values.tolist()
    latitude = data['latitude'].values.tolist()   
    P = data['P'].values.tolist()
    return longitude, latitude, P


if __name__ == '__main__':
    data = pd.read_csv("./3hour/202107011952strict.csv", index_col=0)
    title = 'Strict strategy'
    longitude = data['longitude'].values.tolist()
    latitude = data['latitude'].values.tolist()   
    P = data['P'].values.tolist()
    
    earth_R = 6378137 # radius of the earth
    ey = 360/(2*math.pi*earth_R) # ido, latitude 1[deg] = ey[m]
    theta = mean(latitude)
    ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # keido, longitude 1[deg] = ex[m]
    
    target = [135.9239421, 35.04932481]
    
    diff_longitude = calc_diff_longitude(target[0], longitude, ex)
    diff_latitude = calc_diff_latitude(target[1], latitude, ey)
    avg = calc_mean_diff_distance(diff_longitude, diff_latitude)
    """
    for count in range(len(diff_longitude)):
      pos_plotter(title, target, diff_longitude[count], diff_latitude[count], count)
    """
    power_plotter(title, P)
