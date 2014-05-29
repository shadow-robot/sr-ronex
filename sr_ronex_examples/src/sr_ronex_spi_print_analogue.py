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
from std_msgs.msg import Float64
from math import radians

class SpiRobotConnector():
    """
    retrieves values from sliders connected to spi
    and publishes as command to the hand
    """
    def __init__(self):
        self.waitress = rospy.ServiceProxy("/ronex/spi/1385468312/command/passthrough/0", SPI)
        self.pub = rospy.Publisher("sh_ffj0_position_controller/command", Float64, latch=True)

    def get_spi_value(self):
        resp = self.waitress([6,0,0]).data
        left = '{:08b}'.format(int(resp[1]))[4:]
        righ = '{:08b}'.format(int(resp[2]))
        return int(left + righ, 2)

    def execute(self):
        while not rospy.is_shutdown():
            value = self.get_spi_value()
            print("analogue value = {}".format(value))
            print("radians for hand = {}".format(float(value)))
            self.pub.publish(radians(float(value)))
            sleep(0.05)

if __name__ == '__main__':
    rospy.init_node('sr_ronex_spi')
    connector = SpiRobotConnector()
    connector.execute()
