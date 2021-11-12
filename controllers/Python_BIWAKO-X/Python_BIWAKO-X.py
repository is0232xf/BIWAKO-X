"""Python_BIWAKO-X controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import sys
sys.path.append('../')

import os
import re
import random
import csv
import math
import time
import datetime
import mpu
import glob
import plotter
import robot_controller as controller
import numpy as np
import pandas as pd
import calculate_degree as calculator
from controller import Supervisor
from const import parameter
from statistics import mean


path_list = ["Simple", "Diagonal", "Strict", "Oct-directional", "Flexible"]
now = datetime.datetime.now()
str_date = now.strftime("%Y%m%d%H%M%S")

# create the Robot instance.
robot = Supervisor()
robot_node = robot.getFromDef("BIWAKO-X")
pos = robot_node.getField("translation") #+ type Field
fluid_node = robot.getFromDef("STILL_WATER") # type Node
stream_vel = fluid_node.getField("streamVelocity") # type Field
parameter = parameter()

workspace = parameter.workspace
# TIME_STEP = int(robot.getBasicTimeStep())
TIME_STEP = parameter.TIME_STEP
gps_sampling_rate = 1000 # [msec]
compass_sampling_rate = 100 # [msec]

thruster1 = robot.getDevice("thruster1")
thruster2 = robot.getDevice("thruster2")
thruster3 = robot.getDevice("thruster3")
thruster4 = robot.getDevice("thruster4")

# set thrusters to infinity rotation mode
thruster1.setPosition(float('+inf'))
thruster2.setPosition(float('+inf'))
thruster3.setPosition(float('+inf'))
thruster4.setPosition(float('+inf'))

way_point_file = parameter.way_point_file
target_point = np.genfromtxt(way_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')

def set_thruster_vel(thruster_direction, thrust):
    thruster1.setVelocity(thrust*thruster_direction[0])
    thruster2.setVelocity(thrust*thruster_direction[1])
    thruster3.setVelocity(thrust*thruster_direction[2])
    thruster4.setVelocity(thrust*thruster_direction[3])

def set_disturbance(x, z):
    st_vel = [x, 0.0, z]
    stream_vel.setSFVec3f(st_vel)

def initialize():
    initial_point = [0.0, 0.58, 0.0]
    pos.setSFVec3f(initial_point)

def log_param_data(filename):
    f = open(filename, 'a')
    control_mode = parameter.control_mode
    policy = parameter.policy
    main_target_distance_torelance = parameter.main_target_distance_torelance
    temp_target_distance_torelance = parameter.temp_target_distance_torelance
    head_torelance = parameter.head_torelance
    duration = parameter.duration
    f.write("CONTROL MODE: " + str(control_mode) + '\n')
    f.write("POLICY: " + str(policy) + '\n')
    f.write("MAIN TORELANCE: " + str(main_target_distance_torelance) + '\n')
    f.write("TEMP TORELANCE: " + str(temp_target_distance_torelance) + '\n')
    f.write("HEAD TORELANCE: " + str(head_torelance) + '\n')
    f.write("DURATION: " + str(duration) + '\n')
    f.close()

def make_distance_log_file(path, m_a_diff_distance, m_t_diff_distance, m_diff_distance, P):
    f = open(path, 'a')
    f.write("mean error distance(ground truth): " + str(m_a_diff_distance) + '\n')
    f.write("mean error distance(flexible): " + str(m_t_diff_distance) + '\n')
    f.write("mean error distance: " + str(m_diff_distance) + '\n')
    f.write("P: " + str(P))
    f.close()

def make_dirs(str_date):
    os.mkdir(workspace + str_date)
    """
    path_list = ["Simple", "Diagonal", "Strict", "Oct-directional", "Flexible"]
    for path in path_list:
        os.mkdir(workspace + str_date + "/" + path)
        """

debug_mode = parameter.debug_mode

# Main loop:
def main(strategy, disturbance_mode, gps_error_mode, filename):

    a_gps = robot.getDevice("a_gps")
    a_gps.enable(gps_sampling_rate)

    e_gps = robot.getDevice("e_gps")
    e_gps.enable(gps_sampling_rate)

    compass = robot.getDevice("compass")
    compass.enable(compass_sampling_rate)
    # simulation mode parameters
    control_mode = strategy[0]
    policy = strategy[1]
    distance_torelance = strategy[2][0]
    temp_distance_torelance = strategy[2][1]
    total_step = parameter.total_step
    display_mode = parameter.state_display_mode

    # log parameter lists
    diff_distance = [0.0]
    t_diff_distance = [0.0]
    a_diff_distance = [0.0]
    diff_deg = [0.0]

    # electlicity parameters
    V = parameter.V
    P = 0.0

    # localizaation parameters
    next_goal = target_point[0]
    current_point = [35.0494, 135.924]

    # disturbance parameters
    d_count = 0
    x_list = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3]
    z_list = [0.05, 0.1, 0.15, 0.2, 0.25, 0.3]

    # control flags
    count = 0.0
    temp_flag = 0
    is_First = 0

    if parameter.data_log_mode == True:
        param_file = workspace + str_date + "/" + filename + ".txt"
        csv_filename = workspace + str_date + "/" + str_date + "_" + filename + ".csv"
        f = open(csv_filename, 'a', newline='')
        log_param_data(param_file)
        csvWriter = csv.writer(f)
        csvWriter.writerow(['count', 'a_latitude', 'a_longitude', 'e_latitude', 'e_longitude', 'W', 'P'])

    while robot.step(TIME_STEP) != -1:
        a_gps_value = a_gps.getValues()
        e_gps_value = e_gps.getValues()
        compass_value = compass.getValues()
        bearing = math.radians(calculator.get_bearing_in_degree(compass_value))

        a_latitude = a_gps_value[1]
        a_longitude = a_gps_value[0]
        e_latitude = e_gps_value[1]
        e_longitude = e_gps_value[0]
        a_current_point = [a_latitude, a_longitude]
        e_current_point = [e_latitude, e_longitude]

        if gps_error_mode == True:
            current_point = e_current_point
        else:
            current_point = a_current_point
        target_direction = math.radians(calculator.calculate_bearing(current_point, next_goal))
        deg =  math.degrees(calculator.limit_angle(target_direction - bearing))
        diff_deg.append(deg)

        distance = round(mpu.haversine_distance(current_point, next_goal), 5)*1000
        t_distance = round(mpu.haversine_distance(current_point, target_point[0]), 5)*1000
        a_distance = round(mpu.haversine_distance(a_current_point, target_point[0]), 5)*1000
        diff_distance.append(distance)
        t_diff_distance.append(t_distance)
        a_diff_distance.append(a_distance)
        if diff_distance[-1] <= distance_torelance:
            if policy == 1:
                if is_First == 0:
                    is_First = 1
                temp_flag = 0
                next_goal = target_point[0]
                distance_torelance = 3.0
            if debug_mode == True:
                print("STAY")

            thruster_direction = [0, 0, 0, 0]
            thrust = 0.0

        if diff_distance[-1] > distance_torelance:
            if policy == 0:
                pass
            elif policy == 1 and temp_flag == 0 and is_First == 1:
                temp_goal = calculator.calc_temp_goal(current_point, next_goal)
                next_goal = temp_goal
                distance_torelance = temp_distance_torelance
                distance = round(mpu.haversine_distance(current_point, next_goal), 5)*1000
                temp_flag = 1

            if control_mode == 0:
                thruster_direction, thrust = controller.omni_control_action(diff_distance, diff_deg)
            elif control_mode == 1:
                thruster_direction, thrust = controller.diagonal_control_action(diff_distance, diff_deg)
            elif control_mode == 2:
                thruster_direction, thrust = controller.fixed_head_action(diff_distance, diff_deg)
            elif control_mode == 3:
                thruster_direction, thrust = controller.oct_directional_action(diff_distance, diff_deg)
        set_thruster_vel(thruster_direction, thrust)

        # calculate E-energy
        A = calculator.calc_amp(thrust)
        W = calculator.calc_Watt(V, A, thruster_direction, TIME_STEP)
        P = P + W/1000 # 消費電力グラフの単位を[kJ]としているため1000で割る

        if count % (total_step/5) == 0:
            if disturbance_mode == 0:
                x = 0.05
                z = 0
            elif disturbance_mode == 1:
                x = round(random.uniform(-0.3, 0.3), 2)
                z = round(random.uniform(-0.3, 0.3), 2)
            elif disturbance_mode == 2:
                x = x_list[d_count]
                z = 0.0
                d_count = d_count + 1
            set_disturbance(x, z)
        count = count + 1

        if count == total_step + 1:
            if parameter.data_log_mode == True:
                m_a_diff_distance = mean(a_diff_distance)
                m_t_diff_distance = mean(t_diff_distance)
                m_diff_distance = mean(diff_distance)
                path = workspace + str_date + "/" + filename + "_distance.txt"
                make_distance_log_file(path, m_a_diff_distance, m_t_diff_distance, m_diff_distance, P)
                print("File close")
                f.close()
                x = 0.0
                z = 0.0
                count = 0
            break
        
        if parameter.data_log_mode == True:
            csvWriter.writerow([count, a_latitude, a_longitude, e_latitude, e_longitude, W, P])

        if display_mode:
            power_label = "Comsumed energy: " + str('{:.2f}'.format(P)) + " [Ws]"
            robot.setLabel(4, power_label, 0.5, 0.4, 0.1, 0x00FF00, 0, "Arial")

if parameter.data_log_mode == True:
    make_dirs(str_date)

disturbance_mode = parameter.disturbance_mode
gps_mode = parameter.gps_error_mode


# main() for Unit test
title = "Flexible"
control_mode = 0
policy = 1
torelance = [3.0, 1.5]
strategy = [control_mode, policy, torelance]
main(strategy, disturbance_mode, gps_mode, title)
initialize()
robot.simulationSetMode(2) # First mode
robot.simulationResetPhysics()
robot.simulationSetMode(-1)

"""
# main() loop
for title in path_list:
    d_count = 0
    if title == "Simple":
        control_mode = 0
        policy = 0
        torelance = [3.0, 0.0]
    elif title == "Diagonal":
        control_mode = 1
        policy = 0
        torelance = [3.0, 0.0]
    elif title == "Oct-directional":
        control_mode = 3
        policy = 0
        torelance = [3.0, 0.0]
    elif title == "Strict":
        control_mode = 0
        policy = 0
        torelance = [1.5, 0.0]
    elif title == "Flexible":
        print("FLEX")
        control_mode = 0
        policy = 1
        torelance = [3.0, 1.5]
    strategy = [control_mode, policy, torelance]
    main(strategy, disturbance_mode, gps_mode, title)
    initialize()
    robot.simulationSetMode(2) # First mode
    robot.simulationResetPhysics()
robot.simulationSetMode(-1)
"""

if parameter.data_log_mode == True:
    print("Finish the all simulation")
    time.sleep(3)
    csv_file_list = glob.glob(workspace + str_date + "/" + str_date + "*.csv")
    P_list = []    
    target = target_point[0]
    # make a power consumption file
    plotter.make_power_consumption_graph(csv_file_list, str_date)

    for file in csv_file_list:
        a_longitude, a_latitude, e_longitude, e_latitude = plotter.pos_data_extraction(file)
        a_diff_longitude = plotter.calc_diff_longitude(target[1], a_latitude, a_longitude)
        a_diff_latitude = plotter.calc_diff_latitude(target[0], a_latitude)
        e_diff_longitude = plotter.calc_diff_longitude(target[1], e_latitude, e_longitude)
        e_diff_latitude = plotter.calc_diff_latitude(target[0], e_latitude)
        p = r"\_(.*)\."
        title = re.findall(p, file)[0]
        """
            # 上から誤差なしのGPSで取得した値によるグラフ，誤差ありのGPSで取得した値によるグラフ，消費電力のグラフ
            plotter.pos_plotter(str_date, "a_" + title, a_diff_longitude, a_diff_latitude)
            plotter.pos_plotter(str_date, "e_" + title, e_diff_longitude, e_diff_latitude)
            for count in range(len(a_diff_latitude)-time_width):
                plotter.tiemseries_pos_plotter(str_date, title, a_diff_longitude[count:count+time_width], a_diff_latitude[count:count+time_width], count)
            
        """