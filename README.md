|     Service       |  Status  |
| ----------------- | -------- |
| Documentation     | [![Documentation Status](https://readthedocs.org/projects/sr-ronex/badge)](http://sr-ronex.readthedocs.org/) |
| Code style checks | [![Circle CI](https://circleci.com/gh/shadow-robot/sr-ronex.svg?style=shield)](https://circleci.com/gh/shadow-robot/sr-ronex) |
| Unit tests        | [![Build Status](https://img.shields.io/shippable/554b2991edd7f2c052e402cd.svg)](https://app.shippable.com/projects/554b2991edd7f2c052e402cd) |
| Install tests     | [![Build Status](https://semaphoreci.com/api/v1/projects/b84707ef-b651-423c-bf5a-e21efe8bb8a0/530156/shields_badge.svg)](https://semaphoreci.com/shadow-robot/sr-ronex) |
| Code Coverage     | [![codecov.io](http://codecov.io/github/shadow-robot/sr-ronex/coverage.svg?branch=indigo-devel)](http://codecov.io/github/shadow-robot/sr-ronex?branch=indigo-devel) |

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
