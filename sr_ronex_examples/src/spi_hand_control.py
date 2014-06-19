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

from rospy import ServiceProxy, Publisher, init_node, is_shutdown, sleep
from sr_ronex_msgs.srv import SPI
from std_msgs.msg import Float64

class SpiRobotConnector():
    """
    retrieves values from sliders connected to spi
    and publishes as command to the hand
    """
    def __init__(self):
        self.waitress = [ServiceProxy("/ronex/spi/1402573926/command/passthrough/0", SPI),
                         ServiceProxy("/ronex/spi/1402573926/command/passthrough/1", SPI),
                         ServiceProxy("/ronex/spi/1402574043/command/passthrough/0", SPI),
                         ServiceProxy("/ronex/spi/1402574043/command/passthrough/1", SPI),
                         ServiceProxy("/ronex/spi/1402574043/command/passthrough/2", SPI),
                         ServiceProxy("/ronex/spi/1402574043/command/passthrough/3", SPI)]

        self.joints = ("lfj0", "lfj3", "lfj4", "lfj5",
                       "rfj0", "rfj3", "rfj4",
                       "mfj0", "mfj3", "mfj4",
                       "ffj0", "ffj3", "ffj4",
                       "thj1", "thj2", "thj3", "thj4", "thj5")

        self.joint_coefficients = [(lim/1.57) for lim in
        #     joint max degs        0 90  3 45  4 -20  5 20   finger
                                   (1.57, 0.79, -0.30, 0.30,  # lf
                                    1.57, 0.79, -0.30,        # rf
                                    1.57, 0.79, -0.30,        # mf
                                    1.57, 0.79, -0.30,        # ff
        #            thumb          1 45  2 20  3  5  4 70  5 30
                                    0.79, 0.30, 0.09, 1.22, 0.52)]

        self.publishers = [Publisher("/sh_{}_position_controller/command".format(j), Float64, latch=True)
                           for j in self.joints]

    def get_spi_values(self):
        responses = ((waitress([6, 128, 0]).data,  # 18
                      waitress([6, 192, 0]).data,  # 26
                      waitress([6, 0, 0]).data,    # 2
                      waitress([6, 64, 0]).data)   # 10
                      for waitress in self.waitress)

        bin_results = ('{0:08b}{1:08b}'.format(int(ch[1]), int(ch[2]))
                       for res in responses for ch in res)

        # this scales analog values that go   500 -> 3500
        # to radian values in the range       0   -> pi/2
        # with low analog values clamped at 500

        # (pi/2)/(3500-500) = 0.000523599

        # return [0,1,2,3]
        # pots   [L,R,M,F]
        results = [int(res[4:], 2)
                   for res in bin_results]
        print results
        return [0.000523599*(min(max(res, 600), 3300) - 600)
                for res in results]

    def execute(self):
        while not is_shutdown():
            values = self.get_spi_values()
            # R 1 -> lf rf  close j0, j3, j5
            # M 2 -> mf ff  close j0, j3
            # F 3 -> th     close j1
            # L 0 -> joints 4

            #           j0         j3         j4         j5
            commands = (values[1], values[1], values[0], 0,                      # lf
                        values[1], values[1], values[0],                         # rf
                        values[2], values[2], values[0],                         # mf
                        values[2], values[2], values[0],                         # ff
            #           j1         j2         j3         j4         j5
                        values[3], values[3], values[3], values[3], values[3])   # th

            #for com, coef, pub in zip(commands, self.joint_coefficients, self.publishers):
            #    pub.publish(coef*com)

            sleep(0.01)

if __name__ == '__main__':
    init_node('sr_ronex_spi')
    connector = SpiRobotConnector()
    connector.execute()
