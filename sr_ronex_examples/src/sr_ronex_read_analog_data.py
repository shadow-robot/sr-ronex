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
This class demonstrates how to find the General I/O module with the given ronex_id.
"""
class SrRonexFindGeneralIOModule(object):

    def __init__(self, ronex_id):
        self.ronex_id = ronex_id
        
    """
    Get the path of the General I/O module with the given ronex_id.
    Note that None is returned if the module is not found.
    """
    def get_path(self):
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

        """
        Retrieve all the ronex parameter ids from the parameter server.
        If there are three General I/O modules, then ronex_param_ids is [0,1,2].
        Note that the id starts from zero. And the size of the returned variable
        is equal to the number of General I/O modules.
        """
        ronex_param_ids = rospy.get_param("/ronex/devices")
        for key in ronex_param_ids:
            if self.ronex_id == ronex_param_ids[key]["ronex_id"]:
                path = ronex_param_ids[key]["path"]
                return path

#--------------------------------------------------------------------------------

"""
Assume that your RoNeX consists of a Bridge (IN) module, and one 
or multiple General I/O module(s). This example demonstrates how 
to read analogue data with RoNeX.
"""
if __name__ == "__main__":
    rospy.init_node("sr_ronex_read_analog_data")

    # Note that you may have to set the value of ronex_id,
    # depending on which General I/O board the input device is connected to.
    ronex_id = raw_input( "Please enter the ronex id: " )
    findModule = SrRonexFindGeneralIOModule( str(ronex_id) )
    path = findModule.get_path()

    if path != None:
        # For example "/ronex/general_io/1" + "/state".
        topic = path + "/state"
        rospy.Subscriber( topic, GeneralIOState, generalIOState_callback )
        rospy.spin()
    else:
        rospy.loginfo("Failed to find the General I/O module with the given ronex_id %s.", ronex_id)

#--------------------------------------------------------------------------------
