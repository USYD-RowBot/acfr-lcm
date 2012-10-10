import math
import sys
import datetime
import time

def deg2rad(deg):
    return deg/180.*math.pi

def rad2deg(rad):
    return rad/math.pi*180.

def timeNow():
    t = datetime.datetime.now()
    time_received = long(time.mktime(t.timetuple())) * 1000000L + long(t.microsecond)
    return time_received

# avoid time jumps during runtime
base_utc = datetime.datetime.utcnow()
first = 1
start_time = 0
one_hour = 60*60*long(1000000)
def hauv2long_helper(str, base_utc):
    year = base_utc.year
    month = base_utc.month
    day = base_utc.day
    hh = int(str[0:2])
    mm = int(str[2:4])
    ss = int(str[4:6])
    us = int(float('0' + str[6:]) * 1000000.0)
    # what a nightmare in python...
    dt = datetime.datetime(year, month, day, hh, mm, ss)
    t = time.mktime(dt.timetuple())
    return long(t) * 1000000L + long(us)


def time_hauv2long(str):
    global first
    global start_time
    if first==1:
        first = 0
        start_time = hauv2long_helper(str, base_utc)
    time = hauv2long_helper(str, base_utc)
    # if time significantly before start time, add 24 hours...
    if time+2*one_hour < start_time:
        time = time + 24*one_hour
    return time

def time_long2hauv(l):
#    return "123456.78"
    l = timeNow(); # todo: ignoring l
    seconds = long(l / 1000000L)
    usecs = l - seconds*1000000L
    t = datetime.datetime.fromtimestamp(seconds)
    # todo: 12 hour (%h) or 24 hour (%H)?
    res = t.strftime("%H%M%S.")
    return ("%s%02i" % (res,(usecs/10000)))

class _Getch:
    """Gets a single character from standard input.  Does not echo to screen"""
    def __init__(self):
        try:
            self.impl = _GetchWindows()
        except ImportError:
            self.impl = _GetchUnix()
    def __call__(self): return self.impl()

class _GetchUnix:
    def __init__(self):
        import tty, sys
    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:            termios.tcsetattr(fd,termios.TCSADRAIN, old_settings)
        return ch

class _GetchWindows:
    def __init__(self):
        import msvcrt
    def __call__(self):
        import msvcrt
        return msvcrt.getch()
getch = _Getch()
