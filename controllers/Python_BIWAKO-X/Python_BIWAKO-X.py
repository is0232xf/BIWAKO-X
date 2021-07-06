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
import datetime
import mpu
import glob
import plotter
import robot_controller as controller
import numpy as np
import calculate_degree as calculator
from controller import Supervisor
from const import parameter
from statistics import mean


now = datetime.datetime.now()
str_date = now.strftime("%Y%m%d%H%M")

# create the Robot instance.
robot = Supervisor()
robot_node = robot.getFromDef("BIWAKO-X")
pos = robot_node.getField("translation") #+ type Field
fluid_node = robot.getFromDef("STILL_WATER") # type Node
stream_vel = fluid_node.getField("streamVelocity") # type Field
parameter = parameter()

# TIME_STEP = int(robot.getBasicTimeStep())
TIME_STEP = parameter.TIME_STEP

thruster1 = robot.getDevice("thruster1")
thruster2 = robot.getDevice("thruster2")
thruster3 = robot.getDevice("thruster3")
thruster4 = robot.getDevice("thruster4")

# set thrusters to infinity rotation mode
thruster1.setPosition(float('+inf'))
thruster2.setPosition(float('+inf'))
thruster3.setPosition(float('+inf'))
thruster4.setPosition(float('+inf'))

a_gps = robot.getDevice("a_gps")
a_gps.enable(TIME_STEP)

e_gps = robot.getDevice("e_gps")
e_gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
# get the time step of the current world.

way_point_file = parameter.way_point_file
target_point = np.genfromtxt(way_point_file,
                          delimiter=',',
                          dtype='float',
                          encoding='utf-8')

def get_bearing_in_degree(compass_value):
    # convert rad to deg range[-180, 180]
    north = compass_value[0]
    east = compass_value[2]
    rad = math.atan2(north, east)
    bearing = rad/math.pi*180
    return bearing

def set_thruster_vel(thruster_direction, thrust):
    thruster1.setVelocity(thrust*thruster_direction[0])
    thruster2.setVelocity(thrust*thruster_direction[1])
    thruster3.setVelocity(thrust*thruster_direction[2])
    thruster4.setVelocity(thrust*thruster_direction[3])

def calc_amp(thrust):
    thrust = abs(thrust)
    power = (thrust / 20) * 100
    A = 0.0005*power**2-0.0149*power+0.1012
    # A = round(A, 2)
    if power < 20:
        A = 0.0
    return A

def calc_temp_goal(current_point, target_point):
    current_point = np.array([current_point])
    target_point = np.array([target_point])
    temp_target = target_point-(current_point-target_point)/1.5
    temp_target = [temp_target[0][0], temp_target[0][1]]
    return temp_target

def set_disturbance():
    x = round(random.uniform(-0.5, 0.5), 2)
    z = round(random.uniform(-0.5, 0.5), 2)
    st_vel = [x, 0.0, z]
    stream_vel.setSFVec3f(st_vel)

def initialize():
    initial_point = [0.0, 0.58, 0.0]
    pos.setSFVec3f(initial_point)

def log_param_data(filename):
    f = open(filename, 'a')
    control_mode = parameter.control_mode
    strategy = parameter.strategy
    main_target_distance_torelance = parameter.main_target_distance_torelance
    temp_target_distance_torelance = parameter.temp_target_distance_torelance
    head_torelance = parameter.head_torelance
    duration = parameter.duration
    f.write("CONTROL MODE: " + str(control_mode) + '\n')
    f.write("STRATEGY: " + str(strategy) + '\n')
    f.write("MAIN TORELANCE: " + str(main_target_distance_torelance) + '\n')
    f.write("TEMP TORELANCE: " + str(temp_target_distance_torelance) + '\n')
    f.write("HEAD TORELANCE: " + str(head_torelance) + '\n')
    f.write("DURATION: " + str(duration) + '\n')
    f.close()
control_mode = parameter.control_mode
debug_mode = parameter.debug_mode
is_mkdir = True
# Main loop:
def main(control_mode, filename, is_mkdir):
    def calc_Watt(V, A, thruster_direction):
        T_count = 4 - thruster_direction.count(0)
        dt = 0.001 # 1[msec] = 0.001[sec]
        c_W = 3.0 # c_W means constant consumed energy by main computer
        W = T_count * (V * A * dt) + c_W * dt# unit Watt sec, 4 means use four thrusters
        return W

    distance_torelance = parameter.main_target_distance_torelance
    next_goal = target_point[0]
    diff_distance = [0.0]
    diff_deg = [0.0]

    total_step = parameter.total_step
    display_mode = parameter.state_display_mode
    strategy = parameter.strategy

    V = parameter.V
    P = 0.0

    if parameter.data_log_mode == True:
        if is_mkdir == True:
            os.mkdir('./result/' + str_date)
        param_file = "./result/" + str_date + "/" + filename + ".txt"
        csv_filename = "./result/" + str_date + "/" + str_date + "-" + filename + ".csv"
        f = open(csv_filename, 'a', newline='')
        log_param_data(param_file)
        csvWriter = csv.writer(f)
        csvWriter.writerow(['count', 'a_latitude', 'a_longitude', 'e_latitude', 'e_longitude', 'W', 'P'])

    count = 0.0
    temp_flag = 0
    is_First = 0

    while robot.step(TIME_STEP) != -1:
        a_gps_value = a_gps.getValues()
        e_gps_value = e_gps.getValues()
        compass_value = compass.getValues()
        bearing = math.radians(get_bearing_in_degree(compass_value))

        a_latitude = a_gps_value[1]
        a_longitude = a_gps_value[0]
        e_latitude = e_gps_value[1]
        e_longitude = e_gps_value[0]
        e_current_point = [e_latitude, e_longitude]

        target_direction = math.radians(calculator.calculate_bearing(e_current_point, next_goal))
        deg =  math.degrees(calculator.limit_angle(target_direction - bearing))
        diff_deg.append(deg)
        
        distance = round(mpu.haversine_distance(e_current_point, next_goal), 5)*1000
        diff_distance.append(distance)
        if diff_distance[-1] <= distance_torelance:
            if strategy == 1:
                if is_First == 0:
                    is_First = 1
                temp_flag = 0
                next_goal = target_point[0]
                distance_torelance = parameter.main_target_distance_torelance
            if debug_mode == True:
                print("STAY")

            thruster_direction = [0, 0, 0, 0]
            thrust = 0.0

        if diff_distance[-1] > distance_torelance:
            if strategy == 0:
                pass
            elif strategy == 1 and temp_flag == 0 and is_First == 1:
                temp_goal = calc_temp_goal(e_current_point, next_goal)
                next_goal = temp_goal
                distance_torelance = parameter.temp_target_distance_torelance
                distance = round(mpu.haversine_distance(e_current_point, next_goal), 5)*1000
                temp_flag = 1

            if control_mode == 0:
                thruster_direction, thrust = controller.omni_control_action(diff_distance, diff_deg)
            elif control_mode == 1:
                thruster_direction, thrust = controller.fixed_head_action(diff_distance, diff_deg)
            elif control_mode == 2:
                thruster_direction, thrust = controller.diagonal_control_action(diff_distance, diff_deg)
            elif control_mode == 3:
                thruster_direction, thrust = controller.oct_directional_action(diff_distance, diff_deg)
        set_thruster_vel(thruster_direction, thrust)

        # calculate E-energy
        A = calc_amp(thrust)
        W = calc_Watt(V, A, thruster_direction)
        P = P + W

        count = count + 1
        if count % (total_step/10) == 0:
            set_disturbance()
        
        if count == total_step + 1:
            if parameter.data_log_mode == True:
                print("File close")
                f.close()
                robot.simulationSetMode(-1)
                count = 0
            break
        
        if parameter.data_log_mode == True:
            csvWriter.writerow([count, a_latitude, a_longitude, e_latitude, e_longitude, W, P])

        if display_mode:
            power_label = "Comsumed energy: " + str('{:.2f}'.format(P)) + " [Ws]"
            robot.setLabel(4, power_label, 0.5, 0.4, 0.1, 0x00FF00, 0, "Arial")
        
# main loop
control_mode = 0
main(control_mode, "flexible", is_mkdir)
is_mkdir = False
initialize()
robot.simulationSetMode(2) # First mode
robot.simulationResetPhysics()
parameter.main_target_distance_torelance = 1.5
parameter.strategy = 0
main(control_mode, "strict", is_mkdir)
initialize()
robot.simulationSetMode(2) # First mode
robot.simulationResetPhysics()
parameter.main_target_distance_torelance = 3.0
for control_mode in range(4):
    parameter.control_mode = control_mode
    if control_mode == 0:
        filename = "vertical"
    elif control_mode == 1:
        filename = "fixed_head"
    elif control_mode == 2:
        filename = "diagonal"
    elif control_mode == 3:
        filename = "oct_directional"
    main(control_mode, filename, is_mkdir)
    initialize()
    robot.simulationSetMode(2) # First mode
    robot.simulationResetPhysics()
robot.simulationSetMode(-1)

if parameter.data_log_mode == True:
    csv_file_list = glob.glob("./result/" + str_date + "/*.csv")
    P_list = []    
    target = target_point[0]

    for file in csv_file_list:
        P = plotter.p_data_extraction(file)
        P_list.append(P[-1])
    max_P = max(P_list) + 100
    
    for file in csv_file_list:
        a_longitude, a_latitude, e_longitude, e_latitude = plotter.pos_data_extraction(file)
        P = plotter.p_data_extraction(file)
        
        a_diff_longitude = plotter.calc_diff_longitude(target[1], a_latitude, a_longitude)
        a_diff_latitude = plotter.calc_diff_latitude(target[0], a_latitude)
        e_diff_longitude = plotter.calc_diff_longitude(target[1], e_latitude, e_longitude)
        e_diff_latitude = plotter.calc_diff_latitude(target[0], e_latitude)
        p = r"\-(.*)\."
        title = re.findall(p, file)[0]

        plotter.pos_plotter(str_date, "a_" + title, a_diff_longitude, a_diff_latitude)
        plotter.pos_plotter(str_date, "e_" + title, e_diff_longitude, e_diff_latitude)
        plotter.power_plotter(str_date, title, P, max_P)
