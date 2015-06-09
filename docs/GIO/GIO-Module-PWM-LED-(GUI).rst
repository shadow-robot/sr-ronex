Flashing a LED (GUI)
====================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:

The ROS GUI contains a number of useful graphical tools which can be
used to interact with your GIO module. In this example we will look at
how to configure a PWM module to control an LED on digital channel 0.

The first thing we need to do is set channel 0 as an output, which can
be done by unchecking the appropriate box in the Dynamic Reconfigure
GUI.

We can then open the publisher plugin, select the topic and message
type, then configure the message. Not that the box on the left hand side
must be ticked in order to publish the message.
