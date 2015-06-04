In this tutorial, we will create a package called **my\_first\_package**
(inside workspace **my\_first\_ws**) to interact with RoNeX. Make sure
that you read the following tutorials first.

1. [[Creating a workspace for
   catkin\|http://wiki.ros.org/catkin/Tutorials/create\_a\_workspace]]
2. [[Creating a catkin
   package\|http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage]]

This tutorial assumes that you have installed catkin and sourced your
environment.

::

    $ source /opt/ros/hydro/setup.bash 

Creating a catkin workspace
===========================

Let's create a catkin workspace:

::

    $ mkdir -p ~/my_first_ws/src
    $ cd ~/my_first_ws/src
    $ catkin_init_workspace
    Creating symlink "~/my_first_ws/src/CMakeLists.txt" pointing to "/opt/ros/hydro/share/catkin/cmake/toplevel.cmake"

Even though the workspace is empty (there are no packages in the **src**
folder, just a single CMakeLists.txt link) you can still "build" the
workspace:

::

    $ cd ~/my_first_ws/
    $ catkin_make

Before continuing source your new setup.\*sh file:

::

    $ source devel/setup.bash

Creating a catkin Package
=========================

Change directories to the **src** folder (i.e., THE SOURCE SPACE) in
your workspace:

::

    $ cd ~/my_first_ws/src

Now use the **catkin\_create\_pkg** script to create a new package
called **my\_first\_package** which depends on std\_msgs,
sr\_ronex\_msgs, sr\_ronex\_utilities, dynamic\_reconfigure, roscpp, and
rospy.

::

    $ catkin_create_pkg my_first_package std_msgs sr_ronex_msgs sr_ronex_utilities dynamic_reconfigure roscpp rospy

This will create a **my\_first\_package** folder which contains a
[[package.xml\|http://wiki.ros.org/catkin/package.xml]] and a
[[CMakeLists.txt\|http://wiki.ros.org/catkin/CMakeLists.txt]], which
have been partially filled out with the information you gave
catkin\_create\_pkg.

Writing the Source Code
=======================

For this tutorial, we simply copy
**sr\_ronex\_flash\_LED\_with\_PWM.cpp** from the
**sr\_ronex\_examples** package to the **src** folder, and rename it to
**sr\_ronex\_flash\_LED\_with\_PWM\_2.cpp**. You can find the source
code in [[the PWM tutorial\|Flash-an-LED-light-using-PWM-(CPP)]].

::

    $ ls ~/my_first_ws/src/my_first_package/src
    sr_ronex_flash_LED_with_PWM_2.cpp

Edit the file (i.e., **sr\_ronex\_flash\_LED\_with\_PWM\_2.cpp**) and
change the node name we give to ``ros::init``.

\`\`\`c++ int main(int argc, char \*\*argv) { // Initialize ROS with a
unique node name. ros::init(argc, argv,
"sr\_ronex\_flash\_LED\_with\_PWM\_2");

// Create a handle to this process' node. ros::NodeHandle n; \`\`\`

In **sr\_ronex\_flash\_LED\_with\_PWM\_2.cpp**, we use the advertise()
function to tell ROS that we want to publish on a given topic name. Note
that this version of advertise is a templated convenience function.

\`\`\`c++ void flash\_LED( ros::NodeHandle& n, const std::string& topic
) { ros::Publisher pub = n.advertise( topic, 1000 );

// ......... } \`\`\`

Of course, we have to tell the compiler where it can find the definition
of ``sr_ronex_msgs::PWM``.

``c++ #include "sr_ronex_msgs/PWM.h"``

If you want to know more about ``sr_ronex_msgs::PWM``, you can use
**rosmsg show**.

::

    $ rosmsg show sr_ronex_msgs/PWM
    uint16 pwm_period
    uint16 pwm_on_time_0
    uint16 pwm_on_time_1

Customizing Your Package
========================

::

    $ ls ~/my_first_ws/src/my_first_package
    CMakeLists.txt  include  package.xml  src

Customizing the package.xml
---------------------------

Follow [[this
instruction\|http://wiki.ros.org/ROS/Tutorials/catkin/CreatingPackage#Customizing\_the\_package.xml]]
to customize the package.xml.

Customizing the CMakeLists.txt
------------------------------

Declare things to be passed to dependent projects. You must invoke
catkin\_package() before adding any targets (libraries and executables).
The reason is because catkin\_package() will change the location where
the targets are built.

::

    catkin_package(
      CATKIN_DEPENDS dynamic_reconfigure roscpp rospy sr_ronex_msgs sr_ronex_utilities std_msgs
    )

Add the following lines to the end of **CMakeLists.txt**.

::

    add_executable(sr_ronex_flash_LED_with_PWM_2 ${PROJECT_SOURCE_DIR}/src/sr_ronex_flash_LED_with_PWM_2.cpp)
    add_dependencies(sr_ronex_flash_LED_with_PWM_2 sr_ronex_msgs_gencpp ${PROJECT_NAME}_gencfg)
    target_link_libraries(sr_ronex_flash_LED_with_PWM_2 ${catkin_LIBRARIES})

Running the code
================

Change directories to your RoNeX workspace, and compile the code.

::

    $ cd ~/my_first_ws
    $ catkin_make

Now use **rosrun** to run the code (after starting **roscore** in
another terminal).

::

    $ source ~/my_first_ws/devel/setup.bash
    $ rosrun my_first_package sr_ronex_flash_LED_with_PWM_2

Check [[the PWM tutorial\|Flash-an-LED-light-using-PWM-(CPP)]] for more
information about how to set up the experiment etc.

If you want to use **roslaunch** instead of **rosrun**, create a launch
file.

::

    $ mkdir ~/my_first_ws/src/my_first_package/launch/

Create launch file **sr\_ronex\_flash\_LED\_with\_PWM\_2.launch**, and
place it inside the **launch** folder.

::

    <launch>
      <node name="sr_ronex_flash_LED_with_PWM_2" pkg="my_first_package" type="sr_ronex_flash_LED_with_PWM_2" output="screen"/>
    </launch>

Now you can use **roslaunch** to run the code.

::

    $ roslaunch my_first_package sr_ronex_flash_LED_with_PWM_2.launch

