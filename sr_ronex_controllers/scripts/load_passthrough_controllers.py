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

from controller_manager_msgs.srv import LoadController, ListControllers, SwitchController, SwitchControllerRequest

class LoadPassthroughControllers(object):
    """
    Load the passthrough controllers for all the RoNeXes present on the bus.
    """

    def __init__(self):
        """
        """
        ronex_ids = self.find_ronexes()

        if len(ronex_ids) > 0:
            self.set_param(ronex_ids)
            self.load_and_start_ctrl(ronex_ids)

    def find_ronexes(self):
        """
        Find the ronexes present on the system

        @return a list of ronex_ids
        """
        ronex_ids = []

        # retrieve all the ronex ids from the parameter server
        # wait until there's one ronex
        attempts = 50
        rospy.loginfo("Waiting for the ronex to be loaded properly.")
        while attempts and not rospy.has_param("/ronex/devices/0/ronex_id"):
            sleep(0.1)
            attempts -= 1

        if attempts > 0:
            ronex_param = rospy.get_param("/ronex/devices")
            for key in ronex_param:
                ronex_ids.append(ronex_param[key]["ronex_id"])
        else:
            rospy.loginfo("Failed to find ronex devices in parameter server")

        return ronex_ids

    def set_param(self, ronex_ids):
        """
        Load the parameters for the present ronexes into the parameter server

        @param ronex_ids the ids of the ronexes
        """
        for ronex_id in ronex_ids:
            rospy.set_param("/ronex_" + ronex_id + "_passthrough/type", "sr_ronex_controllers/GeneralIOPassthroughController")
            rospy.set_param("/ronex_" + ronex_id + "_passthrough/ronex_id", ronex_id)

    def load_and_start_ctrl(self, ronex_ids):
        """
        Load and start the passthrough controllers for the list of given ronexes.

        @param ronex_ids the ids of the ronexes
        """
        # building a list of controller names
        controllers_list = []
        for ronex_id in ronex_ids:
            controllers_list.append("ronex_" + ronex_id + "_passthrough")

        # calling the services to load and switch the controllers on
        rospy.loginfo("Waiting for controller_manager services: list/load/switch")
        rospy.wait_for_service('controller_manager/list_controllers')
        rospy.wait_for_service('controller_manager/load_controller')
        rospy.wait_for_service('controller_manager/switch_controller')

        list_controllers = rospy.ServiceProxy('controller_manager/list_controllers', ListControllers)
        load_controller = rospy.ServiceProxy('controller_manager/load_controller', LoadController)
        switch_controller = rospy.ServiceProxy('controller_manager/switch_controller', SwitchController)

        rospy.loginfo("Starting controllers: " + str(controllers_list))
        
        # first list the available controllers
        available_controllers = None
        try:
            available_controllers = list_controllers()
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)

        # load the ones that don't exist
        for ctrl in controllers_list:
            if ctrl not in available_controllers.controller:
                try:
                    resp1 = load_controller(ctrl)
                except rospy.ServiceException, e:
                    print "Service did not process request: %s" % str(e)

        # start the controllers
        try:
            resp1 = switch_controller(controllers_list, [], SwitchControllerRequest.BEST_EFFORT)
        except rospy.ServiceException, e:
            print "Service did not process request: %s" % str(e)


if __name__ == "__main__":
    rospy.sleep(2.5)
    rospy.init_node("load_passthrough_controllers")
    load_passthrough = LoadPassthroughControllers()
