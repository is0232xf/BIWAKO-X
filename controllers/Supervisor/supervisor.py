import sys
sys.path.append('../')

import random
from controller import Supervisor
from const import parameter

def calc_elaspsed_time(step, timestep):
    elapsed_time = ((step/1000)*timestep)/60
    return elapsed_time

supervisor = Supervisor()
parameter = parameter()

log_mode = parameter.data_log_mode

fluid_node = supervisor.getFromDef("STILL_WATER") # type Node
stream_vel = fluid_node.getField("streamVelocity") # type Field

step = 0
min_duration = parameter.duration # duration time[minutes]
total_step = parameter.total_step
timestep = parameter.TIME_STEP
display_mode = parameter.state_display_mode

x_strength = [0.1, 0.15, 0.2, 0.25, 0.3]
z_strength = [0.1, 0.15, 0.2, 0.25, 0.3]

count = 0
play_count = 0

while supervisor.step(timestep) != 1:
    if display_mode:
        step_label = "STEP: " + str(step+1) + "/" + str(total_step)
        elapsed_time = round(calc_elaspsed_time(step, timestep), 2)
        time_label = "Elapsed time: " + str('{:.2f}'.format(elapsed_time)) + "/" + str(min_duration) + "[min]"
        supervisor.setLabel(1, step_label, 0.5, 0.1, 0.1, 0x00FF00, 0, "Arial")
        supervisor.setLabel(2, time_label, 0.5, 0.2, 0.1, 0x00FF00, 0, "Arial")
    
    step += 1
