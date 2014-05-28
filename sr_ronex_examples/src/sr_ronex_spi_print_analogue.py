#!/usr/bin/env python

# ####################################################################
# Copyright (c) 2013, Shadow Robot Company, All rights reserved.
#
# This library is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation; either
# version 3.0 of the License, or (at your option) any later version.
#
# This library is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
# Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public
# License along with this library.
# ####################################################################

import rospy
from time import sleep
from sr_ronex_msgs.srv import SPI

rospy.init_node('sr_ronex_spi')
waitress = rospy.ServiceProxy("/ronex/spi/1385468312/command/passthrough/0", SPI)

while not rospy.is_shutdown():
    resp = waitress([6,0,0]).data
    left = '{:08b}'.format(int(resp[1]))[4:]
    righ = '{:08b}'.format(int(resp[2]))
    print(int(left + righ, 2))
    sleep(0.25)
