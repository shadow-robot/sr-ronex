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

class SpiRobotConnector():
    """
    retrieves values from sliders connected to spi
    and publishes as command to the hand
    """
    def __init__(self):
        self.waitress = rospy.ServiceProxy("/ronex/spi/1385468312/command/passthrough/0", SPI)

        self.pubs = [
                rospy.Publisher("/sh_lfj3_position_controller/command", Float64, latch=True),
                rospy.Publisher("/sh_rfj3_position_controller/command", Float64, latch=True),
                rospy.Publisher("/sh_mfj3_position_controller/command", Float64, latch=True),
                rospy.Publisher("/sh_ffj3_position_controller/command", Float64, latch=True)]

    def get_spi_values(self):
        responses = [
                self.waitress([6,128,0]).data,  # 18
                self.waitress([6,192,0]).data,  # 26
                self.waitress([6,0,0]).data,    # 2
                self.waitress([6,64,0]).data]   # 10

        results = ['{0:08b}{1:08b}'.format(int(res[1]), int(res[2])) for res in responses]

        return [int(res[4:], 2) for res in results]

    def execute(self):
        while not rospy.is_shutdown():
            values = self.get_spi_values()
            print("analogs = [ {0:6} {1:6} {2:6} {3:6} ]".
                    format(values[0], values[1], values[2], values[3]))

            for val, pub in zip(values, self.pubs):
                # scale   3500 -> 500
                # to      pi/2 ->   0
                # (pi/2)/(3500-500) = 
                command = 0.000523599*(float(val) - 500.0)
                pub.publish(command)
            #print("publish 0.5")
            #self.pubs[1].publish(0.5)    

            sleep(0.2)

if __name__ == '__main__':
    rospy.init_node('sr_ronex_spi')
    connector = SpiRobotConnector()
    connector.execute()

