"""Python_BIWAKO-X controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import math
from controller import Robot

MAX_VELOCITY = 10.0
# create the Robot instance.
robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())

thruster1 = robot.getDevice("thruster1")
thruster2 = robot.getDevice("thruster2")
thruster3 = robot.getDevice("thruster3")
thruster4 = robot.getDevice("thruster4")

motor1 = robot.getDevice("")

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
    bearing = (rad-1.5708)/math.pi*180
    if bearing < 0:
        bearing = bearing + 360
    return bearing

def set_thruster_vel(vel_array):
    coeff = 1.15
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
        print("THRUSTER CONTROL ERROR!!")
    set_thruster_vel(vel_array)

# Main loop:
# - perform simulation steps until Webots is stopping the controller
i = 0
while robot.step(TIME_STEP) != -1:
    cmd = 1
    thruster_control(cmd)
    gps_value = gps.getValues()
    speed = gps.getSpeed()

    compass_value = compass.getValues()
    bearing = get_bearing_in_degree(compass_value)
    print(bearing)

    longitude = gps_value[0]
    latitude = gps_value[1]
    print("longitude: ", longitude)
    print("latitude: ", latitude)
    print("speed: ", speed)
    print("compass: ", compass_value)
    i = i + 1
