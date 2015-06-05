This example demonstrates how to flash an LED using `PWM (Pulse Width Modulation) <http://en.wikipedia.org/wiki/Pulse-width_modulation>`__ by varying duty cycle (i.e., the amount of time in the period that the pulse is active or high).

Note that General I/O (GIO) module consists of one GIO Node board and one GIO Peripheral board, so there are 12 digital I/O channels (a GIO Node board has 6 digital I/O channels, a GIO Peripheral board has also 6 digital I/O channels). PWM is available on all 12 digital channels.

For this example, we assume that the LED is connected to digital channel 0.

The code
========

First change directories to your **sr\_ronex\_examples** package.

.. code-block:: bash

    $ roscd sr_ronex_examples/

C++ file **sr\_ronex\_flash\_LED\_with\_PWM.cpp** is located inside the
**src** directory.

.. code-block:: c++

     #include <ros/ros.h>
     #include "sr_ronex_msgs/PWM.h"

     void flash_LED( ros::NodeHandle& n, const std::string& topic )
     {
       ros::Publisher pub = n.advertise<sr_ronex_msgs::PWM>( topic, 1000 );
       short unsigned int pwm_period = 320;
       short unsigned int pwm_on_time_0 = pwm_period;
       short unsigned int pwm_on_time_1 = 0;

       ros::Rate loop_rate(100);
       while ( ros::ok() )
       {
	 pwm_on_time_0 -= 10;
	 if (pwm_on_time_0 < 0)
	   pwm_on_time_0 = pwm_period;
	 sr_ronex_msgs::PWM msg;
	 msg.pwm_period    = pwm_period;
	 msg.pwm_on_time_0 = pwm_on_time_0;
	 msg.pwm_on_time_1 = pwm_on_time_1;
	 pub.publish(msg);
	 ros::spinOnce();
	 loop_rate.sleep();
       }
     }

     int main(int argc, char **argv)
     {
       ros::init(argc, argv, "sr_ronex_flash_LED_with_PWM");
       ros::NodeHandle n;
       std::string topic = "/ronex/general_io/12/command/pwm/0";
       flash_LED( n, topic );
       return 0;
     }


The Code Explained
------------------

Now lets have a look at the code:

.. code-block:: c++

     #include <ros/ros.h>
     #include "sr_ronex_msgs/PWM.h"

First we load import appropriate modules and messages, including the standard ros.h header to create a ros node and get access to the topics, sleep function etc and the PWM message format we're going to publish.

.. code-block:: c++

     int main(int argc, char **argv)
     {
       ros::init(argc, argv, "sr_ronex_flash_LED_with_PWM");
       ros::NodeHandle n;
       std::string topic = "/ronex/general_io/12/command/pwm/0";
       flash_LED( n, topic );
       return 0;
     }

In the main function (at the bottom of the file) we initialise the ROS
node that will publish the messages, and add a handle to this node. We
then set up the topic to be published to (we assume the LED is connected
to digital channel 0), then call the function to flash the LED.

For simplicity, in this example we hard code the ronex id (12), and you
will need to change it to correspond to that of your RoNeX GIO module.
To make this example more robust, the path to the first connected device
could be retrieved from the parameter server. To learn how to do this
you can follow the [[Parse Parameter Server (CPP) Tutorial\|Parse
Parameter Server (CPP)]].

.. code-block:: c++

     void flash_LED( ros::NodeHandle& n, const std::string& topic )
     {
       ros::Publisher pub = n.advertise<sr_ronex_msgs::PWM>( topic, 1000 );
       short unsigned int pwm_period = 320;
       short unsigned int pwm_on_time_0 = pwm_period;
       short unsigned int pwm_on_time_1 = 0;


The handle to the node we created in the main function and the name of
the topic to be published to is passed to the flash\_LED function from
the main function. The publisher is then initialised, with message
format PWM, the topic name passed from main, and a queue size of 1000.
We set the PWM period to 320, and then the on time for channel 0 to the
same value. This means corresponds to a 100% duty cycle, meaning the LED
will receive full power. Channel 1 is not used, so we set the on time to 0.

.. code-block:: c++

       ros::Rate loop_rate(100);
       while ( ros::ok() )
       {
	 pwm_on_time_0 -= 10;
	 if (pwm_on_time_0 < 0)
	   pwm_on_time_0 = pwm_period;

Next we create a rate variable which will be used in combination with a
sleep to maintain a 100Hz rate for the following loop. We then have a
while loop that runs continuously until ROS is shutdown (or the program
is interrupted). For every increment of the loop we subtract 10 from the
channel 0 on time, making the LED gradually dimmer. If the on time has
reached 0 (i.e. the LED is completely off), we set it equal to PWM
period again (full power).

.. code-block:: c++

	 sr_ronex_msgs::PWM msg;
	 msg.pwm_period    = pwm_period;
	 msg.pwm_on_time_0 = pwm_on_time_0;
	 msg.pwm_on_time_1 = pwm_on_time_1;
	 pub.publish(msg);
	 ros::spinOnce();
	 loop_rate.sleep();
       }
     }

We then create a PWM message and populate it with the values we have
just assigned, publish it, call spinOnce() to send out the command, and
sleep for the required time to maintain a 100Hz rate before returning to
the start of the while loop.

Running the code
================

If you're running this code from your own workspace, you'll first need
to build it using Catkin, if you're not sure how to do this you can
follow the instructions
`here <Create-a-package-to-interact-with-RoNeX#running-the-code>`__.

Next sure that a roscore and the RoNeX driver are running (see `Launch
driver <Home#launching-the-ronex-driver>`__ ).

Digital i/o channel 0 needs to be configured as an output in order to
flash the LED (all digital channels are set to input by default). The
easiest way to do this is to use the [[GUI\|GIO Module Config (GUI)]]
and set ``input_mode_0`` to ``false``.

Once this is done we can run our C++ program:

.. code-block:: bash

    rosrun sr_ronex_examples sr_ronex_flash_LED_with_PWM

You should now see your LED flashing. You can try adjusting the
pwm\_on\_time\_0 increments and sleep time to achieve different light
patterns.
