This example demonstrates how to flash an LED using `PWM (Pulse Width
Modulation) <http://en.wikipedia.org/wiki/Pulse-width_modulation>`__ by
varying duty cycle (i.e., the amount of time in the period that the
pulse is active or high).

Note that General I/O (GIO) module consists of one GIO Node board and
one GIO Peripheral board, so there are 12 digital I/O channels (a GIO
Node board has 6 digital I/O channels, a GIO Peripheral board has also 6
digital I/O channels). PWM is available on all 12 digital channels.

For this example, we assume that the LED is connected to digital channel
0.

The code
========

First change directories to your **sr\_ronex\_examples** package.

::

    $ roscd sr_ronex_examples/

Python file **sr\_ronex\_flash\_LED\_with\_PWM.py** is located inside
the **src** directory.

.. code:: python

    #!/usr/bin/env python

    import rospy
    from sr_ronex_msgs.msg import PWM

    #--------------------------------------------------------------------------------

    def flashLED(topic):
        pwm_period = 320
        pwm_on_time_0 = pwm_period
        pwm_on_time_1 = 0

        pub = rospy.Publisher( topic, PWM )
        while not rospy.is_shutdown():
            pwm_on_time_0 -= 10
            if pwm_on_time_0 < 0:
                pwm_on_time_0 = pwm_period
            pwm = PWM()
            pwm.pwm_period    = pwm_period
            pwm.pwm_on_time_0 = pwm_on_time_0
            pwm.pwm_on_time_1 = pwm_on_time_1

            pub.publish( pwm )
            rospy.sleep( 0.01 )

    #--------------------------------------------------------------------------------

    if __name__ == "__main__":
        rospy.init_node('sr_ronex_flash_LED_with_PWM')

        topic = "/ronex/general_io/12/command/pwm/0"
        try:
            flashLED(topic)
        except rospy.ROSInterruptException:
            pass

The Code Explained
------------------

Now lets have a look at the code:

.. code:: python

    import rospy
    from sr_ronex_msgs.msg import PWM

First we load import appropriate modules and messages, including rospy
to make us a ros node and get access to the topics, sleep function etc
and the PWM message format we're going to publish.

.. code:: python

    if __name__ == "__main__":
        rospy.init_node('sr_ronex_flash_LED_with_PWM')

        topic = "/ronex/general_io/12/command/pwm/0"
        try:
            flashLED(topic)
        except rospy.ROSInterruptException:
            pass

In the main function (at the bottom of the file) we initialise the ROS
node that will publish the messages, set up the topic to be published to
(we assume the LED is connected to digital channel 0), then call the
function to flash the LED.

For simplicity, in this example we hard code the ronex id (12), and you
will need to change it to correspond to that of your RoNeX GIO module.
To make this example more robust, the path to the first connected device
could be retrieved from the parameter server. To learn how to do this
you can follow the [[Parse Parameter Server (Python) Tutorial\|Parse
Parameter Server (Python)]].

.. code:: python

    def flashLED(topic):
        pwm_period = 320
        pwm_on_time_0 = pwm_period
        pwm_on_time_1 = 0
        pub = rospy.Publisher( topic, PWM )

The name of the topic to be published to is passed to the flashLED
function from the main function. We set the PWM period to 320, and then
the on time for channel 0 to the same value. This means corresponds to a
100% duty cycle, meaning the LED will receive full power. Channel 1 is
not used, so we set the on time to 0. The publisher is then initialised,
with the topic name passed from main, and message format PWM.

.. code:: python

        while not rospy.is_shutdown():
            pwm_on_time_0 -= 10
            if pwm_on_time_0 < 0:
                pwm_on_time_0 = pwm_period

Next we have a while loop that runs continuously until ROS is shutdown
(or the program is interrupted). For every increment of the loop we
subtract 10 from the channel 0 on time, making the LED gradually dimmer.
If the on time has reached 0 (i.e. the LED is completely off), we set it
equal to PWM period again (full power).

.. code:: python

            pwm = PWM()
            pwm.pwm_period    = pwm_period
            pwm.pwm_on_time_0 = pwm_on_time_0
            pwm.pwm_on_time_1 = pwm_on_time_1

            pub.publish( pwm )
            rospy.sleep( 0.01 )

We then create a PWM message and populate it with the values we have
just assigned, publish it (send the command) and sleep for 10ms before
returning to the start of the while loop.

Running the code
================

First make sure that the RoNeX driver is running (see `Launch
driver <Home#launching-the-ronex-driver>`__ ).

Digital i/o channel 0 needs to be configured as an output in order to
flash the LED (all digital channels are set to input by default). The
easiest way to do this is to use the [[GUI\|GIO Module Config (GUI)]]
and set ``input_mode_0`` to ``false``.

Once this is done we can run our Python script:

::

    $ rosrun sr_ronex_examples sr_ronex_flash_LED_with_PWM.py

You should now see your LED flashing. You can try adjusting the
pwm\_on\_time\_0 increments and sleep time to achieve different light
patterns.
