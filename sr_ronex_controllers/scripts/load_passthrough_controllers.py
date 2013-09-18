#!/usr/bin/env python
"""
 Copyright 2013 Shadow Robot Company Ltd.

 This program is Proprietary software: you cannot redistribute it or modify it

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 FITNESS FOR A PARTICULAR PURPOSE.
"""

import roslib; roslib.load_manifest('sr_ronex_controllers')
import rospy

from pr2_mechanism_msgs.srv import LoadController, ListControllers, SwitchController, SwitchControllerRequest

class LoadPassthroughControllers(object):
    """
    Load the passthrough controllers for all the RoNeXes present on the bus.
    """

    def __init__(self, ):
        """
        """
        ronex_ids = self.find_ronexes()
        self.set_param(ronex_ids)
        self.load_and_start_ctrl(ronex_ids)

    def find_ronexes(self):
        """
        Find the ronexes present on the system

        @return a list of ronex_ids
        """
        ronex_ids = []

        #retreive all the ronex ids from the parameter server
        ronex_param = rospy.get_param("/ronex")
        for key in ronex_param:
            ronex_ids.append(ronex_param[key]["ronex_id"])

        return ronex_ids

    def set_param(self, ronex_ids):
        """
        Load the parameters for the present ronexes into the parameter server

        @param ronex_ids the ids of the ronexes
        """
        for ronex_id in ronex_ids:
            rospy.set_param("/ronex_"+ronex_id+"_passthrough/type", "sr_ronex_controllers/GeneralIOPassthroughController")
            rospy.set_param("/ronex_"+ronex_id+"_passthrough/ronex_id", ronex_id)

    def load_and_start_ctrl(self, ronex_ids):
        """
        Load and start the passthrough controllers for the list of given ronexes.

        @param ronex_ids the ids of the ronexes
        """
        #building a list of controller names
        controllers_list = []
        for ronex_id in ronex_ids:
            controllers_list.append("/ronex_"+ronex_id+"_passthrough")

        #calling the services to load and switch the controllers on
        rospy.wait_for_service('/pr2_controller_manager/list_controllers')
        rospy.wait_for_service('/pr2_controller_manager/load_controller')
        rospy.wait_for_service('/pr2_controller_manager/switch_controller')

        list_controllers = rospy.ServiceProxy('/pr2_controller_manager/list_controllers', ListControllers)
        load_controller = rospy.ServiceProxy('/pr2_controller_manager/load_controller', LoadController)
        switch_controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)

        #first list the available controllers
        available_controllers = None
        try:
            available_controllers = list_controllers()
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)

        #load the ones that don't exist
        for ctrl in controllers_list:
            if ctrl not in available_controllers.controllers:
                try:
                    resp1 = load_controller(ctrl)
                except rospy.ServiceException, e:
                    print "Service did not process request: %s"%str(e)

        #start the controllers
        switch_controller = rospy.ServiceProxy('/pr2_controller_manager/switch_controller', SwitchController)
        try:
            resp1 = switch_controller(controllers_list, [], SwitchControllerRequest.BEST_EFFORT)
        except rospy.ServiceException, e:
            print "Service did not process request: %s"%str(e)


if __name__ == "__main__":
    rospy.init_node("load_passthrough_controllers")
    load_passthrough = LoadPassthroughControllers()
