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
from sr_ronex_msgs.msg import GeneralIOState

#--------------------------------------------------------------------------------

"""
The callback function given to the subscribe() call.
"""
def generalIOState_callback(data):
    # Note that the type of data.analogue is tuple.
    analogue = data.analogue 
    rospy.loginfo( "analogue = %s", analogue )

#--------------------------------------------------------------------------------

"""
This class demonstate how to read the analog data for a given ronex.
"""
class SrRonexExample(object):

    def __init__(self, ronex_id):
        self.ronex_id = ronex_id
        
    def get_ronex_path(self):
        """
        Find the ronexes present on the system.
        """
        # Wait until there's one ronex.
        while True:
            try:
                rospy.get_param("/ronex/devices/0/ronex_id")
                break
            except:
                rospy.loginfo("Waiting for the ronex to be loaded properly.")
                sleep(0.1)

        # Retreive all the ronex ids from the parameter server.
        ronex_param = rospy.get_param("/ronex/devices")
        for key in ronex_param:
            if self.ronex_id == ronex_param[key]["ronex_id"]:
                path = ronex_param[key]["path"]
                return path

#--------------------------------------------------------------------------------

"""
This class demonstate how to read the analog data for a given ronex.
"""
if __name__ == "__main__":
    rospy.init_node("sr_ronex_read_analog_data")

    ronex_id = "1"
    example = SrRonexExample( ronex_id )
    path = example.get_ronex_path()
    
    if path != None:
        # For example "/ronex/general_io/1" + "/state"
        topic = path + "/state"
        rospy.Subscriber( topic, GeneralIOState, generalIOState_callback )
        rospy.spin()
    else:
        rospy.logerr( "Failed to find the ronex with the given ronex_id %s.", ronex_id )

#--------------------------------------------------------------------------------
