#!/usr/bin/env python

'''
rplidarplot.py : A little Python class to display scans from the RP Lidar
             
2016 Karel De Coster

Based on XVlidar by Simon D. Levy (http://github.com/simondlevy/xvlidar).

This code is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as 
published by the Free Software Foundation, either version 3 of the 
License, or (at your option) any later version.

This code is distributed in the hope that it will be useful,     
but WITHOUT ANY WARRANTY without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU Lesser General Public License 
along with this code.  If not, see <http://www.gnu.org/licenses/>.
'''

COM_PORT                    = '/dev/ttyO1'    # Linux

# XVLIDAR-04LX specs
RPLIDAR_MAX_SCAN_DIST_MM    = 6000
RPLIDAR_DETECTION_DEG       = 360
RPLIDAR_SCAN_SIZE           = 360

from rplidar import RPLidar

from math import sin, cos, radians
import time
from sys import exit, version
import signal
import sys

#GLOBALS
done = 0

def signal_handler(signal, frame):
        global done 
        done = 1


if version[0] == '3':
    import _thread as thread
else:
    import thread

if __name__ == '__main__':
    signal.signal(signal.SIGINT, signal_handler)
    lidar = RPLidar(COM_PORT)
    while done == 0 :
        time.sleep(1)
        distances = [pair[0] for pair in lidar.getScan()]
        
        print " New Scan :    angle |   distance"
        i = 0
        while i < 360:
            print "           {angle} | {distance}".format(angle = i, distance = distances[i])
            i = i+1

    lidar.set_exitflag()
    print""
    print "DONE"
        
