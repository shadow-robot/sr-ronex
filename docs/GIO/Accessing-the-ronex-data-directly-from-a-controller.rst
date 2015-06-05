Accessing the RoNeX data directly from a controller
===================================================

This tutorial shows how you can access RoNex data directly from a
controller at 1kHz.

As input, we use here a potentiometer (a three-terminal resistor with a
sliding contact that forms an adjustable voltage divider). And it is
connected to the first analogue sampling channel (i.e., index 0) on the
General I/O module with :doc:`the alias </General/Using-aliases-with-your-RoNeX>`
"test\_ronex".

Files belonging to this tutorial can be found in the
**sr\_ronex\_examples** package.

-  *model/simple\_robot.urdf*: a description of the robot.
-  *launch/sr\_ronex\_simple\_controller.launch*: a launch file to start
   the drivers, the controllers, and a few utils.
-  *config/data\_access\_controller.yaml*: the parameter settings.
-  *controller\_plugins.xml*: the plugin settings.

For a detailed desription of how these files are used, please read this
tutorial: :doc:`Working with a joint based robot </GIO/Control-a-Joint-Based-Robot-with-RoNeX>`

The code
--------

Change directories to your **sr\_ronex\_examples** package.

.. code-block:: bash

    roscd sr_ronex_examples/
    cd src
    pwd

C++ header file **sr\_ronex\_simple\_controller.hpp** is located inside
the **include/sr\_ronex\_examples** directory.

.. code-block:: c++

    #pragma once

    #include <ros/node_handle.h>

    #include <controller_interface/controller.h>
    #include <ros_ethercat_model/robot_state.hpp>
    #include <sr_ronex_hardware_interface/mk2_gio_hardware_interface.hpp>
    #include <realtime_tools/realtime_publisher.h>
    #include <sr_ronex_utilities/sr_ronex_utilities.hpp>

    #include <std_msgs/Bool.h>
    #include <sr_ronex_msgs/PWM.h>

    namespace ronex
    {
      class SrRoNeXSimpleController
	: public controller_interface::Controller<ros_ethercat_model::RobotState>
      {
    public:
	SrRoNeXSimpleController();
	virtual ~SrRoNeXSimpleController();

	virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

	virtual void starting(const ros::Time&);

	virtual void update(const ros::Time&, const ros::Duration&);

	virtual void stopping(const ros::Time&);

      private:
	int loop_count_;

	ronex::GeneralIO* general_io_;
      };
    }

C++ source file **sr_ronex_simple_controller.cpp** is located inside the **src** directory.

.. code-block:: c++

    #include "sr_ronex_examples/sr_ronex_simple_controller.hpp"
    #include "pluginlib/class_list_macros.h"

    PLUGINLIB_EXPORT_CLASS( ronex::SrRoNeXSimpleController, controller_interface::ControllerBase)

    namespace ronex
    {
    SrRoNeXSimpleController::SrRoNeXSimpleController()
      : loop_count_(0)
    {
    }

    SrRoNeXSimpleController::~SrRoNeXSimpleController()
    {
    }

    bool SrRoNeXSimpleController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
    {
      assert (robot);

      std::string path("/ronex/general_io/test_ronex");
      general_io_ = static_cast<ronex::GeneralIO*>( robot->getCustomHW(path) );
      if( general_io_ == NULL)
      {
	ROS_ERROR_STREAM("Could not find RoNeX module (i.e., test_ronex). The controller is not loaded.");
	return false;
      }

      return true;
    }

    void SrRoNeXSimpleController::starting(const ros::Time&)
    {
      // Do nothing.
    }

    void SrRoNeXSimpleController::update(const ros::Time&, const ros::Duration&)
    {
      double position = general_io_->state_.analogue_[0];
      if (loop_count_++ % 100 == 0)
      {
	ROS_INFO_STREAM( "Position = " << position );
	loop_count_ = 0;
      }
    }

    void SrRoNeXSimpleController::stopping(const ros::Time&)
    {
      // Do nothing.
    }

    }


The Code Explained
------------------

First, we look for General I/O module "test\_ronex".

.. code-block:: c++

    bool SrRoNeXSimpleController::init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n)
    {
      assert (robot);

      std::string path("/ronex/general_io/test_ronex");
      general_io_ = static_cast<ronex::GeneralIO*>( robot->getCustomHW(path) );
      if( general_io_ == NULL)
      {
	ROS_ERROR_STREAM("Could not find RoNeX module (i.e., test_ronex). The controller is not loaded.");
	return false;
      }

      return true;
    }

If the module is found, we repeatedly read the data from the first
analogue sampling channel and output it to the console.

.. code-block:: c++

    void SrRoNeXSimpleController::update(const ros::Time&, const ros::Duration&)
    {
      double position = general_io_->state_.analogue_[0];
      if (loop_count_++ % 100 == 0)
      {
	ROS_INFO_STREAM( "Position = " << position );
	loop_count_ = 0;
      }
    }

Note that class SrRoNeXSimpleController inherits from the template class
controller\_interface::Controller

.. code-block:: c++

    namespace ronex
    {
      class SrRoNeXSimpleController
	: public controller_interface::Controller<ros_ethercat_model::RobotState>
      {
      public:
	SrRoNeXSimpleController();
	virtual ~SrRoNeXSimpleController();

Running the code
----------------

First, if you are using **eth0** instead of **eth1**, please update the
following line in the launch file.

.. code-block:: xml

     <arg name="ethercat_port" default="$(optenv ETHERCAT_PORT eth1)" />

Now, run terminal in **sudo** mode.

.. code-block:: bash

    sudo -s

Launch the controller (it was compiled to a library:
**libsr\_ronex\_simple\_controller.so**).


.. code-block:: bash

    roslaunch sr_ronex_examples sr_ronex_simple_controller.launch

Now you should to able to see the analogue data on the console.

::

    [ INFO] [1380207765.910351751]: Position = 4095
    [ INFO] [1380207765.911347629]: Position = 4095
    [ INFO] [1380207765.912353727]: Position = 4095
    [ INFO] [1380207765.913331810]: Position = 4095
    [ INFO] [1380207765.914350895]: Position = 4095
    [ INFO] [1380207765.915329345]: Position = 4095
