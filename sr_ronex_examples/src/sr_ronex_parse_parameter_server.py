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

#--------------------------------------------------------------------------------

"""
Assume that your RoNeX consists of a Bridge (IN) module, and one or multiple
General I/O module(s). This class demonstrates how to access the General I/O module(s)
listed in the parameter server. For each General I/O module, the parameter server 
stores parameters such as its product_id, product_name, ronex_id, path, and serial.
Note that the Python version is simpler than the C++ version, because parameters
are stored as a dictionary in Python.
"""
class SrRonexParseParamExample(object):

    def __init__(self):
        self.find_general_io_modules()
        
    def find_general_io_modules(self):
        """
        Find the General I/O modules present on the system.
        """
        # Wait until there's at least one General I/O module.
        while True:
            try:
                rospy.get_param("/ronex/devices/0/ronex_id")
                break
            except:
                rospy.loginfo("Waiting for the General I/O module to be loaded properly.")
                sleep(0.1)

        # Retrieve all General I/O modules (stored in a dict) from the parameter server.
        # Note that the dict's keyword is ronex_param_id, and it starts from zero.
        devices = rospy.get_param("/ronex/devices")
        for ronex_param_id in devices:
            # Retrieve the values of all parameters related to the current General I/O module.
            # Path looks like "/ronex/general_io/2", where 2 is a ronex_id.
            rospy.loginfo( "*** General I/O Module %s ***",  ronex_param_id );
            rospy.loginfo( "product_id   = %s", devices[ronex_param_id]["product_id"] );
            rospy.loginfo( "product_name = %s", devices[ronex_param_id]["product_name"] );
            rospy.loginfo( "ronex_id     = %s", devices[ronex_param_id]["ronex_id"] );
            rospy.loginfo( "path         = %s", devices[ronex_param_id]["path"]);
            rospy.loginfo( "serial       = %s", devices[ronex_param_id]["serial"] );

#--------------------------------------------------------------------------------

if __name__ == "__main__":

    rospy.init_node("sr_ronex_parse_parameter_server") 

    # This class demonstrates how to access the General I/O module(s) 
    # listed in the parameter server. 
    SrRonexParseParamExample()

#--------------------------------------------------------------------------------
