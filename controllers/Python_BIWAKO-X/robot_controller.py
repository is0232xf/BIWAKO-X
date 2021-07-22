import sys
sys.path.append('../')

from const import parameter

parameter = parameter()

MAX_THRUST = parameter.MAX_THRUST
TIME_STEP = parameter.TIME_STEP
distance_torelance = parameter.main_target_distance_torelance
debug_mode = parameter.debug_mode

def omni_control_action(diff_distance, diff_deg):
    def determine_cmd(diff_deg):
        cmd = 0
        if -45.0 <= diff_deg < 45:
            cmd = 1

        elif -180.0 <= diff_deg < -135.0 or 135.0 <= diff_deg < 180.0:
            cmd = 2

        elif 45.0 <= diff_deg < 135.0:
            cmd = 3

        elif -135.0 <= diff_deg < -45.0:
            cmd = 4
        return cmd
    distance = diff_distance[-1]
    deg = diff_deg[-1]
    cmd = determine_cmd(deg)
    thruster_direction = determine_thrust_direction(cmd)
    e_distance = diff_distance[-2] - diff_distance[-1]
    thrust = PD_distance_control(distance, e_distance)
    return thruster_direction, thrust

def diagonal_control_action(diff_distance, diff_deg):
    def determine_cmd(diff_distance, diff_deg):
        cmd = 0
        if 0.0 <= diff_deg <90:
            cmd = 7

        elif -90 <= diff_deg < 0.0:
            cmd = 8

        elif -180 <= diff_deg < -90:
            cmd = 9

        elif 90 <= diff_deg < 180:
            cmd = 10
        return cmd
    distance = diff_distance[-1]
    deg = diff_deg[-1]
    cmd = determine_cmd(distance, deg)
    thruster_direction = determine_thrust_direction(cmd)
    e_distance = diff_distance[-2] - diff_distance[-1]
    thrust = PD_distance_control(distance, e_distance)
    return thruster_direction, thrust

def fixed_head_action(diff_distance, diff_deg):
    def determine_cmd(diff_distance, diff_deg):
        head_torelance = parameter.head_torelance
        cmd = 0
        if diff_distance < distance_torelance:
            cmd = 0
        else:
            if abs(diff_deg) < head_torelance:
                cmd = 1
            elif diff_deg >= head_torelance:
                # CCW
                cmd = 5
            elif diff_deg < -1.0 * head_torelance:
                # CW
                cmd = 6
        return cmd

    distance = diff_distance[-1]
    deg = diff_deg[-1]

    cmd = determine_cmd(distance, deg)
    thruster_direction = determine_thrust_direction(cmd)
    if cmd == 1:
        e_distance = diff_distance[-2] - diff_distance[-1]
        thrust = PD_distance_control(distance, e_distance)
    elif cmd == 5 or cmd == 6:
        e_deg = diff_deg[-1] - diff_deg[-2]
        thrust = PD_head_control(deg, e_deg)
    elif cmd == 0:
        thrust = 0
    return thruster_direction, thrust
    
def oct_directional_action(diff_distance, diff_deg):
    def determine_cmd(diff_deg):
        cmd = 0
        if -22.0 <= diff_deg <22.0:
            cmd = 1

        elif -180.0 <= diff_deg < -157.0 or 157.0 <= diff_deg <= 180.0:
            cmd = 2

        elif -112.0 <= diff_deg < -67.0:
            cmd = 4

        elif 67.0 <= diff_deg < 112.0:
            cmd = 3

        elif 22.0 <= diff_deg < 67.0:
            cmd = 7

        elif -67.0 <= diff_deg < -22.0:
            cmd = 8

        elif -157.0 <= diff_deg < -112.0:
            cmd = 9

        elif 112.0 <= diff_deg < 157.0:
            cmd = 10
        return cmd
    distance = diff_distance[-1]
    deg = diff_deg[-1]
    cmd = determine_cmd(deg)
    thruster_direction = determine_thrust_direction(cmd)
    e_distance = diff_distance[-2] - diff_distance[-1]
    thrust = PD_distance_control(distance, e_distance)
    return thruster_direction, thrust

def determine_thrust_direction(cmd):
    # 1:forward, 2:backward, 3:left, 4:right, 5:CW, 6:CCW
    # 7:first quadrant, 8:second quadrant, 9:third quadrant, 10: fourth quadrant
    if cmd == 0:
        thruster_direction = [0, 0, 0, 0]
    elif cmd == 1:
        thruster_direction = [1, 1, -1, -1]
    elif cmd == 2:
        thruster_direction = [-1, -1, 1, 1]
    elif cmd == 3:
        thruster_direction = [-1, 1, -1, 1]
    elif cmd == 4:
        thruster_direction = [1, -1, 1, -1]
    elif cmd == 5:
        thruster_direction = [1, -1, -1, 1]
    elif cmd == 6:
        thruster_direction = [-1, 1, 1, -1]
    elif cmd == 7:
        thruster_direction = [0, 1, -1, 0]
    elif cmd == 8:
        thruster_direction = [1, 0, 0, -1]
    elif cmd == 9:
        thruster_direction = [0,-1, 1, 0]
    elif cmd == 10:
        thruster_direction = [-1, 0, 0, 1]

    if debug_mode == True:
        if cmd == 0:
            print("STAY")
        elif cmd == 1:
            print("FORWARD")
        elif cmd == 2:
            print("BACKWARD")
        elif cmd == 3:
            print("RIGHT")
        elif cmd == 4:
            print("LEFT")
        elif cmd == 5:
            print("CW")
        elif cmd == 6:
            print("CCW")
        elif cmd == 7:
            print("FIRST QUAD")
        elif cmd == 8:
            print("SECOND QUAD")
        elif cmd == 9:
            print("THIRD QUAD")
        elif cmd == 10:
            print("FOURCE QUAD")

    return thruster_direction

def PD_distance_control(distance, e_distance):
    Kp = parameter.distance_Kp
    Kd = parameter.distance_Kd

    A = Kp * distance
    B = Kd * (e_distance/1000)
    thrust = A + B

    if abs(thrust) > MAX_THRUST:
        thrust = MAX_THRUST

    return thrust

def PD_head_control(deg, e_deg):
    Kp = parameter.degree_Kp
    Kd = parameter.degree_Kd

    thrust = Kp * deg + Kd * e_deg * (1/TIME_STEP)

    if abs(thrust) > MAX_THRUST:
        thrust = MAX_THRUST

    if deg > 0:
        thrust = -thrust
    return thrust
