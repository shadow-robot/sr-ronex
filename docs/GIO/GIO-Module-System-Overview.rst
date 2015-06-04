This page describes some of the key parameters, services and topics
you'll need to interact with when using the RoNeX GIO module. The
information here builds on that related to the general RoNeX system
which can be found [[here\|RoNeX-General-System-Overview]]

The serial number of the sample RoNeX GIO module used here was 12. On
other systems the number 12 will be replaced with the serial number
corresponding to the connected GIO module. ## Topics

ROS Topics provide a fast, flexible way to transfer data between nodes.

Topics can be published or subscribed to from the command line or within
a program. For more information see the
`tutorials <GIO-Module-Tutorials>`__.

/ronex/general\_io/12/command/digital/0 (0-11) [std\_msgs/Bool]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

By publishing a Boolean value to one of these topics, you can turn the
corresponding digital output on or off (if it has been configured as an
ouput using the input\_mode parameter).

/ronex/general\_io/12/command/pwm/0 (0-5) [sr\_ronex\_msgs/PWM]
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Publishing to one of these topics configures the corresponding PWM
module. There are 6 modules in total, each controlling 2 channels. You
can configure the period for each module, and the on time for each
channel individually.

/ronex/general\_io/12/parameter\_updates
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

RoNeX publishes information on the current configuration of the module
i/o channels to this topic, including input\_mode, clock divider and PWM
period. As the name suggests, parameter updates will be reflected in the
data published on this topic.

/ronex/general\_io/12/parameter\_descriptions
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

RoNeX publishes a short description of the function of each of the
module parameters to this topic.

/ronex/general\_io/12/state
^^^^^^^^^^^^^^^^^^^^^^^^^^^

This topic contains the current state of all of the channels on the
RoNeX module, including digital channel values and input\_mode
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
`tutorials <GIO-Module-Tutorials>`__ in order for them to update
correctly.

/ronex/general\_io/12/input\_mode\_0 (0-11)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These parameters (one for each digital io channel) contain the current
mode of each digital channel, True for input mode, and False for output
mode.

/ronex/general\_io/12/pwm\_clock\_divider
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The master clock divider, used in configuring PWM output parameters. For
more information on using PWM functionality see the
`tutorials <GIO-Module-Tutorials>`__.

/ronex/general\_io/12/pwm\_period\_0 (0-5)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The cycle period for each PWM unit. For more information on using PWM
functionality see the `tutorials <GIO-Module-Tutorials>`__.

Services
--------

ROS Services provide a reliable communication method for operations that
require a response or confirmation, such as the updating of system
configuration.

Any of these services can be called directly from the command line using
`rosservice call <http://wiki.ros.org/rosservice>`__, or from a program
as described `here <GIO-Module-Config-(CPP)>`__

/ronex/general\_io/12/set\_parameters
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This service allows for dynamic reconfiguration of the module parameters
listed above. Usage can be found in the
`tutorials <GIO-Module-Tutorials>`__.
