This tutorial shows how you can access RoNex data directly from a
controller.

As input, we use here a potentiometer (a three-terminal resistor with a
sliding contact that forms an adjustable voltage divider). And it is
connected to the first analogue sampling channel (i.e., index 0) on the
General I/O module with [[alias\|Using-aliases-with-your-RoNeX]]
"test\_ronex".

Files belonging to this tutorial can be found in the
**sr\_ronex\_examples** package.

-  *model/simple\_robot.urdf*: a description of the robot.
-  *launch/sr\_ronex\_simple\_controller.launch*: a launch file to start
   the drivers, the controllers, and a few utils.
-  *config/data\_access\_controller.yaml*: the parameter settings.
-  *controller\_plugins.xml*: the plugin settings.

For a detailed desription of how these files are used, please read this
tutorial: [[Control a Joint Based Robot with
RoNeX\|Control-a-Joint-Based-Robot-with-RoNeX]].

The code
========

Change directories to your **sr\_ronex\_examples** package.

::

    $ roscd sr_ronex_examples/
    $ cd src
    $ pwd

C++ header file **sr\_ronex\_simple\_controller.hpp** is located inside
the **include/sr\_ronex\_examples** directory.

\`\`\`c++

ifndef *SR*\ RONEX\_SIMPLE\_CONTROLLER\_HPP\_
=============================================

define *SR*\ RONEX\_SIMPLE\_CONTROLLER\_HPP\_
=============================================

include 
========

include 
========

include 
========

include 
========

include 
========

include 
========

include 
========

include 
========

namespace ronex { class SrRoNeXSimpleController : public
controller\_interface::Controller { public: SrRoNeXSimpleController();
virtual ~SrRoNeXSimpleController();

::

    virtual bool init(ros_ethercat_model::RobotState* robot, ros::NodeHandle &n);

    virtual void starting(const ros::Time&);

    virtual void update(const ros::Time&, const ros::Duration&);

    virtual void stopping(const ros::Time&);

private: int loop\_count\_;

::

    ronex::GeneralIO* general_io_;

}; }

endif /\* *SR*\ RONEX\_SIMPLE\_CONTROLLER\_HPP\_ \*/ \`\`\`
===========================================================

C++ source file **sr\_ronex\_simple\_controller.cpp** is located inside
the **src** directory.

\`\`\`c++ #include
"sr\_ronex\_examples/sr\_ronex\_simple\_controller.hpp" #include
"pluginlib/class\_list\_macros.h"

PLUGINLIB\_EXPORT\_CLASS( ronex::SrRoNeXSimpleController,
controller\_interface::ControllerBase)

namespace ronex { SrRoNeXSimpleController::SrRoNeXSimpleController() :
loop\_count\_(0) { }

SrRoNeXSimpleController::~SrRoNeXSimpleController() { }

bool SrRoNeXSimpleController::init(ros\_ethercat\_model::RobotState\*
robot, ros::NodeHandle &n) { assert (robot);

std::string path("/ronex/general\_io/test\_ronex"); general\_io\_ =
static\_cast( robot->getCustomHW(path) ); if( general\_io\_ == NULL) {
ROS\_ERROR\_STREAM("Could not find RoNeX module (i.e., test\_ronex). The
controller is not loaded."); return false; }

return true; }

void SrRoNeXSimpleController::starting(const ros::Time&) { // Do
nothing. }

void SrRoNeXSimpleController::update(const ros::Time&, const
ros::Duration&) { double position =
general\_io\_->state\_.analogue\_[0]; if (loop\_count\_++ % 100 == 0) {
ROS\_INFO\_STREAM( "Position = " << position ); loop\_count\_ = 0; } }

void SrRoNeXSimpleController::stopping(const ros::Time&) { // Do
nothing. }

} \`\`\`

The Code Explained
==================

First, we look for General I/O module "test\_ronex".

\`\`\`c++ bool
SrRoNeXSimpleController::init(ros\_ethercat\_model::RobotState\* robot,
ros::NodeHandle &n) { assert (robot);

std::string path("/ronex/general\_io/test\_ronex"); general\_io\_ =
static\_cast( robot->getCustomHW(path) ); if( general\_io\_ == NULL) {
ROS\_ERROR\_STREAM("Could not find RoNeX module (i.e., test\_ronex). The
controller is not loaded."); return false; }

return true; } \`\`\`

If the module is found, we repeatedly read the data from the first
analogue sampling channel and output it to the console.

``c++ void SrRoNeXSimpleController::update(const ros::Time&, const ros::Duration&) {   double position = general_io_->state_.analogue_[0];   if (loop_count_++ % 100 == 0)   {     ROS_INFO_STREAM( "Position = " << position );     loop_count_ = 0;   } }``

Note that class SrRoNeXSimpleController inherits from the template class
controller\_interface::Controller

``c++ namespace ronex {   class SrRoNeXSimpleController      : public controller_interface::Controller<ros_ethercat_model::RobotState>   {   public:     SrRoNeXSimpleController();     virtual ~SrRoNeXSimpleController();``

Running the code
================

First, if you are using **eth0** instead of **eth1**, please update the
following line in the launch file.

::

     <arg name="ethercat_port" default="$(optenv ETHERCAT_PORT eth1)" />

Now, run terminal in **sudo** mode.

::

    $ sudo -s

Launch the controller (it was compiled to a library:
**libsr\_ronex\_simple\_controller.so**).

::

    # roslaunch sr_ronex_examples sr_ronex_simple_controller.launch

Now you should to able to see the analogue data on the console.

::

    [ INFO] [1380207765.910351751]: Position = 4095
    [ INFO] [1380207765.911347629]: Position = 4095
    [ INFO] [1380207765.912353727]: Position = 4095
    [ INFO] [1380207765.913331810]: Position = 4095
    [ INFO] [1380207765.914350895]: Position = 4095
    [ INFO] [1380207765.915329345]: Position = 4095

