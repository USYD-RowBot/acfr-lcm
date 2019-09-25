#!/usr/bin/python3

# adapted from https://github.com/recantha/EduKit3-RC-Keyboard/blob/master/rc_keyboard.py

import sys, termios, tty, os, time, lcm, math, thread, textwrap

sys.path.append('/usr/local/lib/python2.7/dist-packages/perls/lcmtypes')

from acfrlcm.auv_spektrum_control_command_t import auv_spektrum_control_command_t 



global lc 
lc = lcm.LCM();

global msg 
msg = auv_spektrum_control_command_t()
msg.channels = 6
global mid_value
mid_value = 1024
msg.values = [mid_value]*6
#msg.values[auv_spektrum_control_command_t.RC_AUX1] = 800
global max_value 
max_value = 1624
global min_value 
min_value = 424
global button_delay 
button_delay = 0.2
global vehicle_name

if (len(sys.argv) > 1):
    vehicle_name = sys.argv[1]
else:
    print 'Wrong number of args'

def print_help():
    print textwrap.dedent("""\
===================================            
            Controls
            W - Forwards
            S - Back/Slow
            A - Turn Left
            D - Turn Right
            Q - Rudder Mode
            E - Tunnel Turn Mode
            Z - Strafe Left
            C - Strafe Right
            1 - Auto Mode
            2 - Dead Mode
            3 - Manual Mode
            h - print Help Menu
            p - Kill Process then press ctrl + c
===================================
            """)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def publish_spek():
    while 1:
        lc.publish(vehicle_name+'.SPEKTRUM_CONTROL', msg.encode())
        time.sleep(button_delay/2)

def listen_to_keyboard():
    while 1:
        char = getch()

        if (char == "p"):
            print("Stop!")
            msg.values = [1024]*6
            exit(0)

        if (char == "z"):
            msg.values[auv_spektrum_control_command_t.RC_RUDDER] = msg.values[auv_spektrum_control_command_t.RC_RUDDER] + 100
            if (msg.values[auv_spektrum_control_command_t.RC_RUDDER] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_RUDDER] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_RUDDER] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_RUDDER] = min_value
            print "z pressed - strafe left - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_RUDDER] - mid_value)


        if (char == "c"):
            msg.values[auv_spektrum_control_command_t.RC_RUDDER] = msg.values[auv_spektrum_control_command_t.RC_RUDDER] - 100
            if (msg.values[auv_spektrum_control_command_t.RC_RUDDER] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_RUDDER] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_RUDDER] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_RUDDER] = min_value
            print "c pressed - strafe right - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_RUDDER] - mid_value)



        if (char == "a"):
            msg.values[auv_spektrum_control_command_t.RC_AILERON] = msg.values[auv_spektrum_control_command_t.RC_AILERON] + 100
            if (msg.values[auv_spektrum_control_command_t.RC_AILERON] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_AILERON] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_AILERON] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_AILERON] = min_value
            print "a pressed - turn left - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_AILERON] - mid_value)

        elif (char == "d"):
            msg.values[auv_spektrum_control_command_t.RC_AILERON] = msg.values[auv_spektrum_control_command_t.RC_AILERON] - 100
            if (msg.values[auv_spektrum_control_command_t.RC_AILERON] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_AILERON] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_AILERON] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_AILERON] = min_value
            print "d pressed - turn right - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_AILERON] - mid_value)

        elif (char == "w"):
            msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = msg.values[auv_spektrum_control_command_t.RC_THROTTLE] + 100
            if (msg.values[auv_spektrum_control_command_t.RC_THROTTLE] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_THROTTLE] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = min_value
            print "w pressed - thruster foward - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_THROTTLE] - mid_value)

        elif (char == "s"):
            msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = msg.values[auv_spektrum_control_command_t.RC_THROTTLE] - 100
            if (msg.values[auv_spektrum_control_command_t.RC_THROTTLE] > max_value):
                msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = max_value
            elif (msg.values[auv_spektrum_control_command_t.RC_THROTTLE] < min_value):
                msg.values[auv_spektrum_control_command_t.RC_THROTTLE] = min_value
            print "s pressed - thruster back - current val {}".format(msg.values[auv_spektrum_control_command_t.RC_THROTTLE] - mid_value)

        elif (char == "q"):
            print("Number q pressed - RUDDER Mode")
            msg.values[auv_spektrum_control_command_t.RC_GEAR] = 200
            msg.values[auv_spektrum_control_command_t.RC_AILERON] = mid_value


        elif (char == "e"):
            print("Number e pressed - TUNNEL TURN Mode")
            msg.values[auv_spektrum_control_command_t.RC_GEAR] = 1500
            msg.values[auv_spektrum_control_command_t.RC_AILERON] = mid_value


        elif (char == "1"):
            print("Number 1 pressed - AUTO Mode")
            msg.values = [mid_value]*6
            msg.values[auv_spektrum_control_command_t.RC_AUX1] = 1500


        elif (char == "2"):
            print("Number 2 pressed - DEAD Mode")
            msg.values = [mid_value]*6
            msg.values[auv_spektrum_control_command_t.RC_AUX1] = 800


        elif (char == "3"):
            print("Number 3 pressed - MANUAL Mode")
            msg.values = [mid_value]*6
            msg.values[auv_spektrum_control_command_t.RC_AUX1] = 200

        elif (char == "h"):
            print_help()

        lc.publish(vehicle_name+'.SPEKTRUM_CONTROL', msg.encode())
        time.sleep(button_delay/2)

print_help()

try:
    thread.start_new_thread( listen_to_keyboard , ())
    thread.start_new_thread( publish_spek , ())
except:
    print("Thread Init Error")

while 1:
    pass


