sr-ronex
========

Contains ROS packages for RoNeX (Robot Nervous System). [RoNeX](http://www.shadowrobot.com/products/ronex/) is a high speed low latency modular system to link your actuators, sensors, etc. to ROS. More documentation can be found [in the wiki](https://github.com/shadow-robot/sr-ronex/wiki).

This meta-package contain the following:
 - **sr_ronex** convenient meta package for making it possible to install all necessary RoNeX packages.
 - **sr_ronex_controllers** controllers specific to RoNeX - to be able to send commands to the RoNeX.
 - **sr_ronex_drivers** EtherCAT drivers for the RoNeX based on the ros_ethercat drivers.
 - **sr_ronex_examples** a set of examples associated to their [Tutorials](https://github.com/shadow-robot/sr-ronex/wiki/Tutorials).
 - **sr_ronex_external_protocol** header files for the protocol used between the hardware and the software.
 - **sr_ronex_hardware_interface** classes for storing the RoNeX data in the CustomHW map of the hardware interface.
 - **sr_ronex_launch** main launch files.
 - **sr_ronex_msgs** messages used for sending and receiving the RoNeX data in ROS.
 - **sr_ronex_transmissions** transmissions to map the data store in the hardware interface to a joint state - very handy for creating a joint based robot.
 - **sr_ronex_utilities** light weight utility library for accessing the RoNeX.
