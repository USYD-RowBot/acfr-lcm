#!/usr/bin/env python
#from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
#import pyqtgraph as pg
import math

# fill in this bit
x_len = 150
y_len = 150
leader = 10
vx = 0.5
centre_x = 0
centre_y = 0
rotation = 0
origin_lat = -33.839238
origin_lon = 151.25389963



def compass_box(xlen, ylen, leader):
    box = np.array([[leader, 0], [xlen + leader, 0],
                   [xlen + 2 * leader, leader], [xlen + 2 * leader, ylen + leader],
                   [xlen + leader, ylen + 2 * leader], [leader, ylen + 2 * leader],
                   [0, ylen + leader], [0, leader]])

    return box


def shift_mission(mis, x, y):
    shift = np.array([[x,y]])
    return np.add(mis, shift)

def rotate_mission(mis, r):
    rot = np.array([[math.cos(r), -math.sin(r)], [math.sin(r), math.cos(r)]])
    return np.dot(mis, rot)


b1 = compass_box(x_len, y_len, leader)
b1 = shift_mission(b1, -(x_len/2+leader), -(y_len/2+leader))

b2 = b1
b2=rotate_mission(b2, math.radians(45))

# two laps
b=np.concatenate([b1, b1, b2, b2])

#final shift and rotate
b = rotate_mission(b, math.radians(rotation))
b = shift_mission(b, centre_y, centre_x) # x and y are flipped as the missions are x north

#app = QtGui.QApplication([])
#win = pg.GraphicsWindow(title="Mission")
#p1 = win.addPlot()

#p1.plot(b, pen=(200,200,200), symbolBrush=(255,0,0), symbolPen='w')
#p1.setXRange(-500,500)
#p1.setYRange(-500, 500)

# output the mission
fp = open('mission.xml', 'w')

fp.write('<?xml version="1.0" standalone="no" ?>\n')
fp.write('<mission>\n')
fp.write('\t<desc>\n')
fp.write('\t\tIver compass calibration mission\n')
fp.write('\t</desc>\n')
fp.write('\t<global>\n')
fp.write('\t\t<location lat="{}" lon="{}" />\n'.format(origin_lat, origin_lon))
fp.write('\t\t<turn_radius m="5" />\n')
fp.write('\t\t<drop_distance m="4" />\n')
fp.write('\t\t<drop_angle deg="40" />\n')
fp.write('\t\t<mission_timeout t = "1"/>\n')
fp.write('\t\t<!-- Mission timeout not implemented. To have the same drop distance in the corners use drop_angle [rad] = drop_distance / turn_radius -->\n')
fp.write('\t</global>\n')



for i  in range(0, len(b)):
    fp.write('\t<primitive>\n')
    fp.write('\t\t<goto>\n')
    fp.write('\t\t\t<position x="{}" y="{}" z="0.0"/>\n'.format(b[i][0], b[i][1]))
    try:
        fp.write('\t\t\t<heading deg="{}" />\n'.format(math.degrees(math.atan2((b[i+1][1] - b[i][1]), (b[i+1][0] - b[i][0])))))
    except:
        fp.write('\t\t\t<heading deg="{}" />\n'.format(math.degrees(math.atan2((b[i][1] - b[i-1][1]), (b[i][0] - b[i-1][0])))))
    fp.write('\t\t\t<timeout t="3000" />\n')
    fp.write('\t\t\t<velocity x="{}" />\n'.format(vx))
    fp.write('\t\t\t<depth mode = "depth" />\n')
    fp.write('\t\t</goto>\n')
    fp.write('\t</primitive>\n')


fp.write('</mission>\n')

fp.close()

#if __name__ == '__main__':
#    import sys
#    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
#        QtGui.QApplication.instance().exec_()



