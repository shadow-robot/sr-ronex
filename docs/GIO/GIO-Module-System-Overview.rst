GIO Module - System Overview
============================

This page describes some of the key parameters, services and topics
you'll need to interact with when using the RoNeX GIO module. The
information here builds on that related to the general RoNeX system
which can be found :doc:`here </General/RoNeX-General-System-Overview>`.

The serial number of the sample RoNeX GIO module used here was 12. On
other systems the number 12 will be replaced with the serial number
corresponding to the connected GIO module.

Topics
------

ROS Topics provide a fast, flexible way to transfer data between nodes.

Topics can be published or subscribed to from the command line or within
a program. For more information see the
:doc:`tutorials </GIO/GIO-Module-Tutorials>`.

- ``/ronex/general_io/12/command/digital/0`` *(0-11) [std_msgs/Bool]*

By publishing a Boolean value to one of these topics, you can turn the
corresponding digital output on or off (if it has been configured as an
ouput using the input_mode parameter).

- ``/ronex/general_io/12/command/pwm/0`` *(0-5) [sr_ronex_msgs/PWM]*

Publishing to one of these topics configures the corresponding PWM
module. There are 6 modules in total, each controlling 2 channels. You
can configure the period for each module, and the on time for each
channel individually.

- ``/ronex/general_io/12/parameter_updates``

RoNeX publishes information on the current configuration of the module
i/o channels to this topic, including input_mode, clock divider and PWM
period. As the name suggests, parameter updates will be reflected in the
data published on this topic.

- ``/ronex/general_io/12/parameter_descriptions``

RoNeX publishes a short description of the function of each of the
module parameters to this topic.

- ``/ronex/general_io/12/state``

This topic contains the current state of all of the channels on the
RoNeX module, including digital channel values and input_mode
configuration, PWM clock divider and analogue input values. If you're
looking to monitor the activities of your RoNeX module, this is the
place to get the information.

Parameters
----------

The parameter server acts as a database of information to be shared
between various nodes running within your ROS system.

Any of these parameters can be read using the `rosparam
get <http://wiki.ros.org/rosparam>`__ command. Module parameters such as
input modes or click dividers are dynamic parameters, and as such you
need to **use the dynamic reconfigure** interfaces listed in the
changing the configuration section of the
:doc:`tutorials </GIO/GIO-Module-Tutorials>` in order for them to update
correctly.

- ``/ronex/general_io/12/input_mode_0`` *(0-11)*

These parameters (one for each digital io channel) contain the current
mode of each digital channel, True for input mode, and False for output
mode.

- ``/ronex/general_io/12/pwm_clock_divider``

The master clock divider, used in configuring PWM output parameters. For
more information on using PWM functionality see the
:doc:`tutorials </GIO/GIO-Module-Tutorials>`.

- ``/ronex/general_io/12/pwm_period_0`` *(0-5)*

The cycle period for each PWM unit. For more information on using PWM
functionality see the :doc:`tutorials </GIO/GIO-Module-Tutorials>`.

Services
--------

ROS Services provide a reliable communication method for operations that
require a response or confirmation, such as the updating of system
configuration.

Any of these services can be called directly from the command line using
`rosservice call <http://wiki.ros.org/rosservice>`__, or from a program
as described :doc:`here </GIO/GIO-Module-Config-(CPP)>`.

- ``/ronex/general_io/12/set_parameters``

This service allows for dynamic reconfiguration of the module parameters
listed above. Usage can be found in the :doc:`tutorials </GIO/GIO-Module-Tutorials>`.
