# -*- coding: utf-8 -*-
"""
Created on Thu Sep 13 15:31:53 2018

@author: FujiiChang
"""

import math
import numpy as np

e_radius = 6378137.0
# use Hubeny theory
# e_radius is equatorial radius
# p_radius is polar radius
def huveny_distance(target_point, current_point):
    e_radius = 6378137.0
    p_radius = 6356752.314140
    dy = target_point[0] - current_point[0]
    dx = target_point[1] - current_point[1]
    my = (target_point[0] + current_point[0]) / 2
    e = math.sqrt((e_radius**2 - p_radius**2) / e_radius**2)
    W = math.sqrt(1 - (e * math.sin(my))**2 )
    M = e_radius * (1 - e**2) / W**3
    N = e_radius / W
    d = math.sqrt((dy * M)**2 + (dx * N * math.cos(my))**2)
    # translate m to km
    d = d / 1000
    return d

# translate sexagesimall to decimal
def translate_sexagesimal_to_decimal(degval):
    decimal, integer = math.modf(degval/ 100)
    decimal_val = integer + decimal / 60.0 * 100.0
    return decimal_val

def translate_GPGGA_to_decimal(GPGGAval):
    decimal_longtitude = translate_sexagesimal_to_decimal(GPGGAval[0])
    decimal_latitude = translate_sexagesimal_to_decimal(GPGGAval[1])
    decimal_val = [decimal_longtitude, decimal_latitude]
    return decimal_val

# translate degree to radian
def translate_deg_to_rad(deg_val):
    rad = deg_val * math.pi / 180
    return rad

def translate_decimal_to_rad(decimal_val):
    rad_longtitude = translate_deg_to_rad(decimal_val[0])
    rad_latitude = translate_deg_to_rad(decimal_val[1])
    rad_val = [rad_longtitude, rad_latitude]
    return rad_val

# Using Haversine fomula
def calculate_bearing(target_point, current_point):
    lat1 = math.radians(target_point[0])
    lat2 = math.radians(current_point[0])
    lon1 = math.radians(target_point[1])
    lon2 = math.radians(current_point[1])
    dlon = lon2 - lon1
    bearing = 90 - math.degrees(math.atan2(math.cos(lat1)*math.sin(lat2)
                 -math.sin(lat1)*math.cos(lat2)*math.cos(dlon), math.sin(dlon)*math.cos(lat2)))
    return bearing

def limit_angle(angle_in):
    angle_out = np.absolute(angle_in)
    while angle_out > np.pi:
        angle_out -= np.pi * 2

    if angle_in < 0:
        angle_out *= -1

    return angle_out

def get_bearing_in_degree(compass_value):
    # convert rad to deg range[-180, 180]
    north = compass_value[0]
    east = compass_value[2]
    rad = math.atan2(north, east)
    bearing = rad/math.pi*180
    return bearing

def calc_amp(thrust, thruster_direction):
    cmd = calc_control_mode(thruster_direction)
    thrust = abs(thrust)
    power = thrust/20
    A = 0.0
    if cmd == 0:
        A = 0.085
    # control mode is 4 thruster
    elif cmd == 1:
        if 0.0 <= power <= 0.3:
            A = 16.03*power**2 - 1.41*power + 0.10
        elif power > 0.3:
            A = 1.37
    # control mode is diadgonal
    elif cmd == 2:
        if 0.0 <= power <= 0.3:
            A = 7.96*power**2 - 0.76*power + 0.10
        elif power > 0.3:
            A = 0.74
    # control mode is push or pull
    elif cmd == 3 or 4:
        if 0.0 <= power <= 0.3:
            A = 8.20*power**2 - 0.58*power + 0.09
        elif power > 3.0:
            A = 0.78
    return A

def calc_peak_amp(thrust, thruster_direction):
    cmd = calc_control_mode(thruster_direction)
    thrust = abs(thrust)
    power = thrust/20
    A = 0.0
    if cmd == 0:
        A = 0.085
    # control mode is 4 thruster
    elif cmd == 1:
        if 0.0 <= power <= 0.1:
            A = 12.65*power + 0.085
        elif 0.1 < power <= 0.2:
            A = 2.0*power + 1.15
        elif 0.2 < power <= 0.35:
            A = 36.8*power - 5.81
        elif power > 0.35:
            A = 5.10
    # control mode is diagonal
    elif cmd == 2:
        if 0.0 <= power <= 0.1:
            A = 7.25*power + 0.085
        elif 0.1 < power <= 0.2:
            A = 0.7*power + 0.74
        elif 0.2 < power <= 0.35:
            A = 19.3*power - 2.98
        elif power > 0.35:
            A = 2.83
    # control mode is push or pull
    elif cmd == 3 or 4:
        if 0.0 <= power <= 0.1:
            A = 7.35*power + 0.085
        elif 0.1 < power <= 0.2:
            A = 2.0*power + 0.62
        elif 0.2 < power <= 0.35:
            A = 17.1*power - 2.4
        elif power > 0.35:
            A = 2.81
    return A

def calc_control_mode(thruster_direction):
    cmd = 0
    zero_count = thruster_direction.count(0)
    thruster_count = 4 - zero_count
    # cmd 0: stop, 1: 4thruster, 2: diagonal, 3: push, 4: pull
    if thruster_count == 4:
        cmd = 1
    elif thruster_count == 2:
        list_sum = sum(thruster_direction)
        if list_sum == 0:
            cmd = 2
        elif list_sum == 2:
            cmd = 3
        elif list_sum == -2:
            cmd = 4
    elif thruster_count == 0:
        cmd = 0
    return cmd

def calc_temp_goal(k, current_point, target_point):
    current_point = np.array([current_point])
    target_point = np.array([target_point])
    temp_target = target_point-(current_point-target_point)*(k-1)
    temp_target = [temp_target[0][0], temp_target[0][1]]
    return temp_target

def calc_flexible_temp_goal(current_point, target_point, prev_target, r):
    lat_t = target_point[0]
    lon_t = target_point[1]
    earth_R = 6378137
    theta = lat_t
    ex = 360/(2*math.pi*earth_R*math.cos(theta*math.pi/180)) # keido, longitude 1[deg] = ex[m]
    ey = 360/(2*math.pi*earth_R) # ido, latitude 1[deg] = ey[m]
    bearing = calculate_bearing(current_point, prev_target)
    lat_temp_next = lat_t+r*ey*math.sin(math.radians(90-bearing)) # 90-bearing means "coordinate transformation"
    lon_temp_next = lon_t+r*ex*math.cos(math.radians(90-bearing)) # 90-bearing means "coordinate transformation"
    target_point = np.array([lat_temp_next, lon_temp_next])
    return target_point

def calc_Watt(V, A, timestep):
    dt = timestep/1000 # 1[msec] = 0.001[sec]
    W = V * A * dt# unit Watt sec, 4 means use four thrusters
    return W