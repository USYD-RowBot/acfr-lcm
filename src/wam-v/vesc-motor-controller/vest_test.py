import lcm
# name = left_motor
from acfrlcm import asv_torqeedo_motor_command_t
# type = acfrlcm_asv_torqeedo_motor_command_t

import time

while(True):

    time.sleep(1)
    msg = asv_torqeedo_motor_command_t()
    
