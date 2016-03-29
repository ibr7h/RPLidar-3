#!/usr/bin/env python

'''
    xvlidar.py - Python class for reading from GetSurreal's XV Lidar Controller.  

    Adapted from lidar.py downloaded from 

      http://www.getsurreal.com/products/xv-lidar-controller/xv-lidar-controller-visual-test

    Copyright (C) 2016 Simon D. Levy

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as 
    published by the Free Software Foundation, either version 3 of the 
    License, or (at your option) any later version.
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
'''

import threading, time, serial, traceback

class RPLidar(object):

    def __init__(self, com_port):
        '''
        Opens a serial connection to the XV Lidar on the specifiec com port (e.g.,
        'COM5', '/dev/ttyACM0').  Connection will run on its own thread.
        '''
        self.ser = serial.Serial(com_port, 115200)
        self.thread = threading.Thread(target=self._read_lidar, args=())
        self.thread.daemon = True
        self.state = 0
        self.index = 0
        self.lidar_data = [()]*360 # 360 elements (distance,quality), indexed by angle
        self.speed_rpm = 0
        self.thread.start()

    def getScan(self):
        '''
        Returns 360 (distance, quality) tuples.
        '''
        return [pair if len(pair) == 2 else (0,0) for pair in self.lidar_data]

    def getRPM(self):
        '''
        Returns speed in RPM.
        '''
        return self.speed_rpm

    def _read_bytes(self, n):

        return self.ser.read(n).decode('ISO-8859-1')
    
    def _check_header(self, header):
    	
    	if(0xA5!=ord(header[0]) or 0x5A!=ord(header[1])) :
            return 1
    	if(((ord(header[5])&0xC0)>>6) != 0x01) :
            return 1
    	if((ord(header[2])&0xCFFF) != 5) :
            return 1
    	if(ord(header[6]) != 0x81) :
            return 1
    	return 0
   
    def	_stop_scan(self):
        i = self.ser.write("\xA5\x25")
        if i != 2 :
            return 1
        else :
            return 0
    
    def _start_scan(self):
        i = self.ser.write("\xA5\x20")
        if i != 2 :
            return 1
        header = self.ser.read(7)
        return self._check_header(header)
     
    def _read_lidar(self):

        nb_errors = 0
        #time.sleep(120.)	# Need to wait 2 minutes after starting to get 'good' data
        while True:
            try:

                time.sleep(0.0001) # do not hog the processor power

                if self.state == 0 :
                    print " STATE 0"
                    self._stop_scan()
                    time.sleep(0.1)
                    # start byte
                    if(self._start_scan() == 0) :
                        self.state = 1
                    else:
		             	self.state = 0
                elif self.state == 1 :
                    print "STATE 1"
                    angle = 0
                    while(angle < 360.):
                        data = [ord(b) for b in self._read_bytes(5)]

                        # quality
                        quality = data[0] & 0xFC

                        # start flag	-- 1 for new rotation of data
                        start_flag = data[0] & 0x01
                        start_flag_inv = data[0] & 0x02

                        # angle
                        angle = (data[2]*128 + (data[1]>>1)) / 64.

                        # distance
                        distance = (data[4]<<8 | data[3]) / 4

                        # check flag
                        check_flag = data[1] & 0x01
                        #print "Angle = {angle}    |    distance = {distance}    | quality = {quality}".format(angle=angle, distance=distance, quality = quality)
                        #if(start_flag == start_flag_inv):
                        #    self.state = 0

                        #if(check_bit != 1):
                         #   self.state = 0

                        if(self.state == 1 and quality > 0):
                            self.lidar_data[int(round(angle))] = distance,quality

                else: # default, should never happen...
                    self.state = 0

            except:
                traceback.print_exc()
                exit(0)
                
