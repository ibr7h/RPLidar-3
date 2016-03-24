'''
Python distutils setup file for installing RPlidar package

Copyright (C) 2016 Karel De Coster

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
along with this code.  If not, see <http:#www.gnu.org/licenses/>.
'''

from distutils.core import setup

setup (name = 'rplidar',
       packages = ['rplidar'],
       version = '0.1',
       description = 'A Python class for the Robot Peak Lidar',
       author='Karel De Coster',
       author_email='k.decoster94@gmail.com',
       url='https://github.com/kareldecoster/rplidar',
       license='LGPL',
       platforms='Linux')
