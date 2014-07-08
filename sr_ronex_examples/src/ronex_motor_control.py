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

from rospy import get_param, loginfo, Subscriber, Publisher, init_node, is_shutdown, sleep
from sr_ronex_msgs.msg import PWM, GeneralIOState

class SrRonexFindGeneralIOModule(object):
    """
    This class demonstrates how to find the General I/O module with the given ronex_id.
    """

    def __init__(self, ronex_id):
        self.ronex_id = ronex_id

    def get_path(self):
        """
        Get the path of the General I/O module with the given ronex_id.
        Note that None is returned if the module is not found.
        """

        # Wait until there's one ronex.
        while True:
            try:
                get_param("/ronex/devices/0/ronex_id")
                break
            except:
                loginfo("Waiting for the ronex to be loaded properly.")
                sleep(0.05)

        """
        Retrieve all the ronex parameter ids from the parameter server.
        If there are three General I/O modules, then ronex_param_ids is [0,1,2].
        Note that the id starts from zero. And the size of the returned variable
        is equal to the number of General I/O modules.
        """
        for device in get_param("/ronex/devices").itervalues():
            if self.ronex_id == device["ronex_id"]:
                return device["path"]

def general_io_state_cb(data, controller):
    controller.analog = data.analogue[0]

class SpeedController():
    def __init__(self):
        path = SrRonexFindGeneralIOModule('10').get_path()
        topic = path + "/state"
        self.analog = 0.0
        self.subscriber = Subscriber(topic, GeneralIOState, general_io_state_cb, self)
        topic = path + "/command/pwm/0"
        self.publisher = Publisher(topic, PWM, latch=True)

    def execute(self):
        while not is_shutdown():
            self.publisher.publish(PWM(pwm_period=6400, pwm_on_time_1=0, pwm_on_time_0=self.analog))
            sleep(0.01)

init_node('sr_speed_controller')
controller = SpeedController()
controller.execute()
