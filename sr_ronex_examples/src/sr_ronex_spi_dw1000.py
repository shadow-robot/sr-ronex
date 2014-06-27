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

import roslib; roslib.load_manifest('sr_ronex_examples')
import rospy
from time import sleep
from sr_ronex_msgs.srv import SPI

class DW1000SpiInterface(object):
    """
    """

    def __init__(self, ):
        """
        """
        self.spi_srvs_ = []

        for spi_out_index in range(0, 4):
            self.spi_srvs_.append(rospy.ServiceProxy("/ronex/spi/35/command/passthrough/"+str(spi_out_index), SPI))

    def read_register(self, spi_out_index, address, data_packet):
        """
        Reads from the DW1000 register.

        @spi_out_index: which spi are you using (0 to 3)
        @address: trying to read from which address
        @data_packet: the packet that will be sent

        @return values read from SPI, None if failed
        """
        return_data_packet = []

        if address < 64:
            data_packet.insert(0, address)
        else:
            rospy.logerr("Address provided ("+str(address)+") is > 64")
            return None

        try:
            resp = self.spi_srvs_[spi_out_index](data_packet)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return None

        #removing the first item from the received packet of data
        return_data_packet = resp.data[1:]

        print "SPI[",spi_out_index,"]: reading ",self.hexify_list(return_data_packet), " from address: ",address

        return return_data_packet

    def write_register(self, spi_out_index, address, data_packet):
        """
        Writes to the DW1000 register.

        @spi_out_index: which spi are you using (0 to 3)
        @address: trying to write to which address
        @data_packet: the packet of data you want to write

        @return True if success, False otherwise
        """
        print "SPI[",spi_out_index,"]: writing ", self.hexify_list(data_packet),"] to address: ",address
        if address < 64:
            data_packet.insert(0, address + 0x80)
        else:
            rospy.logerr("Address provided ("+str(address)+") is > 64")
            return False

        try:
            resp = self.spi_srvs_[spi_out_index](data_packet)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s"%e)
            return False

        print " ... OK data written"
        return True

    def hexify_list(self, l):
        hex_l = []
        for item in l:
            hex_l.append( str(hex(int(item))) )
        return hex_l


if __name__ == "__main__":
    rospy.init_node('sr_ronex_spi_dw1000')

    dw1000 = DW1000SpiInterface()

    print "--- new write"
    success = dw1000.write_register(0, 0x03, [1,1,1,1])
    packet = dw1000.read_register(0, 0x03, [0,1,2,3])

    print "--- new write"
    success = dw1000.write_register(0, 0x03, [0x01,0x20,0x30, 0x40])
    packet = dw1000.read_register(0, 0x03, [1,1,1,1])




