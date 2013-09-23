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
import dynamic_reconfigure.client

from time import sleep

#--------------------------------------------------------------------------------

"""
This class demonstrate how to change the configuration parameters of a running ronex module.
"""
class ChangeRonexConfigurationExample(object):

    def __init__(self):
        #Define the ronex id of the module to be configured
        ronex_id = "1"
        ronex_path = "/ronex/general_io/" + ronex_id + "/"
        self.configure_module(ronex_path)
        
    def configure_module(self, path):
        """
        Use the /ronex/general_io/1/set_parameters service
        """
        client = dynamic_reconfigure.client.Client(path)

#--------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("change_ronex_configuration")
    ChangeRonexConfigurationExample()

#--------------------------------------------------------------------------------
