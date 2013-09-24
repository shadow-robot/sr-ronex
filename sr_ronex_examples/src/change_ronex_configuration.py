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
        ronex_id = "test_ronex"
        ronex_path = "/ronex/general_io/" + ronex_id + "/"
        self.configure_ronex(ronex_path)
        
    def configure_ronex(self, path):
        """
        In this example we are using the dynamic_reconfigure.client.
        It could also be done by calling the /ronex/general_io/X/set_parameters service directly (as in the c++ example)
        """
        client = dynamic_reconfigure.client.Client(path)
        
        # calling update_configuration with a dictionary of changes to make
        params = { 'input_mode_0' : False, 'input_mode_1' : False, 'pwm_period_0' : 200 , 'pwm_clock_divider' : 3000}
        config = client.update_configuration(params)
        
        # config now contains the full configuration of the node after the parameter update


#--------------------------------------------------------------------------------

if __name__ == "__main__":
    rospy.init_node("change_ronex_configuration_py")
    ChangeRonexConfigurationExample()

#--------------------------------------------------------------------------------
