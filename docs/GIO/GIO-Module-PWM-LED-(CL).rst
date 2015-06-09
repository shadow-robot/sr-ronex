Flashing a LED (command line)
=============================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:

ROS Terminal commands are a quick and easy way to see what's going on
with your RoNeX module. If you need to run something over and over again
you'll want to write a script or use the GUI, but for one off tests, or
when you only have command line access, these commands can be very
handy.

For the following commands you can use the tab key to auto complete the
name of the topic/service and message type, and once you've entered the
message type you can actually double tap tab again to produce an empty
message in the correct format, handy for when you're not familiar with
the message types.

So if we're interfacing with an LED we need to set the corresponding
digital channel as an output, this can be done from the command line by
calling the set\_parameters service. In the example below we have also
set the pwm\_clock\_divider parameter in the same message for
convenience.

.. code-block:: bash

    rosservice call /ronex/general_io/12/set_parameters "config:
      bools:
      - {name: 'input_mode_0', value: false}
      ints:
      - {name: 'pwm_clock_divider', value: 20}"

Once the channel is configured correctly we can publish a message to the
command topic for the corresponding PWM module as shown below. Changing
the pwm\_on\_time\_0 value will adjust the brightness of the LED.

.. code-block:: bash

    rostopic pub /ronex/general_io/12/command/pwm/0 sr_ronex_msgs/PWM "pwm_period: 64000
        pwm_on_time_0: 1000
        pwm_on_time_1: 0"
