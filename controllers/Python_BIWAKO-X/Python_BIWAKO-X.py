"""Python_BIWAKO-X controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
import time
import mpu
from controller import Robot
import calculate_degree as calculator
import robot_controller

MAX_VELOCITY = 10.0
# create the Robot instance.
robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

thruster1 = robot.getDevice("thruster1")
thruster2 = robot.getDevice("thruster2")
thruster3 = robot.getDevice("thruster3")
thruster4 = robot.getDevice("thruster4")

# set thrusters to infinity rotation mode
thruster1.setPosition(float('+inf'))
thruster2.setPosition(float('+inf'))
thruster3.setPosition(float('+inf'))
thruster4.setPosition(float('+inf'))

gps = robot.getDevice("gps")
gps.enable(TIME_STEP)

compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
# get the time step of the current world.

def get_bearing_in_degree(compass_value):
    # convert rad to deg range[-180, 180]
    north = compass_value[0]
    east = compass_value[2]
    rad = math.atan2(north, east)
    bearing = rad/math.pi*180
    return bearing

def set_thruster_vel(vel_array):
    coeff = 1.1
    thruster1.setVelocity(coeff*math.pi*vel_array[0])
    thruster2.setVelocity(coeff*math.pi*vel_array[1])
    thruster3.setVelocity(coeff*math.pi*vel_array[2])
    thruster4.setVelocity(coeff*math.pi*vel_array[3])

def thruster_control(cmd):
    # 1:forward, 2:backward, 3:left, 4:right, 5:CW, 6:CCW
    if cmd == 1:
        vel_array = [1, 1, -1, -1]
        print("FORWARD")
    elif cmd == 2:
        vel_array = [-1, -1, 1, 1]
        print("BACKWARD")
    elif cmd == 3:
        vel_array = [-1, 1, -1, 1]
        print("RIGHT")
    elif cmd == 4:
        vel_array = [1, -1, 1, -1]
        print("LEFT")
    elif cmd == 5:
        vel_array = [-1, 1, 1, -1]
        print("CW")
    elif cmd == 6:
        vel_array = [1, -1, -1, 1]
        print("CCW")
    else:
        vel_array = [0, 0, 0, 0]
        print("STAY")
    set_thruster_vel(vel_array)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

target_point = [35.049377, 135.923899]
distance_torelance = 3.0
cmd = 0

while robot.step(TIME_STEP) != -1:
    gps_value = gps.getValues()
    compass_value = compass.getValues()
    bearing = get_bearing_in_degree(compass_value)
    current_point = [gps_value[1], gps_value[0]]
    print(current_point)

    diff_distance = round(mpu.haversine_distance(current_point, target_point), 5)*1000

    if diff_distance < distance_torelance:
        cmd = 0
        print("diff distance: ", diff_distance)
        print("change target position")
        target_point = [35.049333,135.924028]
    elif diff_distance >= distance_torelance:
        target_direction = math.radians(calculator.calculate_bearing(current_point, target_point))
        diff_deg =  math.degrees(calculator.limit_angle(target_direction - bearing))
        print("diff deg: ", diff_deg)
        print("diff distance: ", diff_distance)
        cmd = robot_controller.fixed_head_action(diff_deg, diff_distance)
        print("cmd: ", cmd)
    thruster_control(cmd)
