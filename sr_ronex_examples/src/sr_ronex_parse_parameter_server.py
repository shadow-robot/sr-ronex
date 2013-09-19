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
This class demonstate how to use the ronexes listed in the parameter server.
For each ronex, the parameter server stores parameters such as
its product_id, product_name, ronex_id, path, and serial.
Note that the Python version is simpler than the C++ version, because params
are stored as a dictionary in Python.
"""
class SrRonexExample(object):

    def __init__(self):
        self.find_ronexes()
        
    def find_ronexes(self):
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
            # Retrieve the values of all parameters related to the current ronex.
            rospy.loginfo( "*** Ronex %s ***",  key );
            rospy.loginfo( "product_id   = %s", ronex_param[key]["product_id"] );
            rospy.loginfo( "product_name = %s", ronex_param[key]["product_name"] );
            rospy.loginfo( "ronex_id     = %s", ronex_param[key]["ronex_id"] );
            rospy.loginfo( "path         = %s", ronex_param[key]["path"]);
            rospy.loginfo( "serial       = %s", ronex_param[key]["serial"] );

#--------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("sr_ronex_parse_parameter_server")
    SrRonexExample()

#--------------------------------------------------------------------------------
