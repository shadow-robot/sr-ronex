General System Overview
=======================

This page describes some of the key parameters, services and topics in
the RoNeX system. If you're unfamiliar with these concepts, you can read
more about the ROS system architecture
`here <http://wiki.ros.org/ROS/Concepts>`__. If you're new to ROS you
may also find it helpful to run through some of the `ROS Wiki
Tutorials <http://wiki.ros.org/ROS/Tutorials>`__.

This page mostly describes the GIO module which is a very general
module. More specialised modules, like the SPI module, will present
additional interfaces or different types of interfaces to the user, so
you should review their specific documentation. The serial number of the
sample RoNeX GIO module used here was 12. On other systems the number 12
will be replaced with the serial number corresponding to the connected
GIO module.

Topics
------

ROS Topics provide a fast, flexible way to transfer data between nodes.

Topics can be published or subscribed to from the command line or within
a program. For more information see the
:doc:`General Examples and Tutorials </General/General-RoNeX-Tutorials>`.

- ``/diagnostics`` : RoNeX publishes diagnostics messages containing information on connected modules and communication status to this topic.

Parameters
----------

The parameter server acts as a database of information to be shared
between various nodes running within your ROS system.

Any of these parameters can be read using the `rosparam
get <http://wiki.ros.org/rosparam>`__ command at any time.

- ``/robot_description``: This parameter will contain a copy of your robot's URDF file, to be used by amongst other things, joint controllers and visualisation tools.
- ``/ronex/devices/0/path``: The path to the first detected RoNeX device, all parameters, topics and services related to this device will start with this path.
- ``/ronex/devices/0/product_id``: This parameter tells you the product_id number corresponding to the type of module detected.
- ``/ronex/devices/0/product_name``: This parameter tells you the name corresponding to the type of module detected.
- ``/ronex/devices/0/ronex_id``: This parameter will contain the id of the RoNeX module, which will be the module alias if you have set one (as described :doc:`here </General/Using-aliases-with-your-RoNeX>`, otherwise it will match the serial number.)
- ``/ronex/devices/0/serial``: The pre-programmed serial number of the RoNeX module, which makes it possible to distinguish between different modules of the same type on the bus.
