This Tutorial is to accompany the RoNeX Interfacing video on Youtube.
Click on the video screenshot below to watch:

Startup
=======

In the video we first start Roscore:

::

    > roscore

Then in a new terminal window elevate permissions to superuser ( you can
skip this step if you have followed the instructions in `Ronex driver
without sudo <Ronex-driver-without-sudo>`__ )

::

    > sudo -s

Then start the ROS driver:

::

    > roslaunch sr_ronex_launch sr_ronex.launch

Next in another new terminal we start the ROS GUI by running:

::

    > rosrun rqt_gui rqt_gui

Once in the GUI you can find the relevant plugins in the menu at the
top. The topic we subscribe to in order to plot the analogue input data
is:

::

    /ronex/general_io/12/state/analogue[0]

The number 12 in the above command should be replaced by the serial
number of your RoNeX GIO Module.

In the Dynamic Reconfigure plugin, untick the boxes relating to
whichever input channels you plan to use as outputs (channel 0 in our
case). For the PWM servo motor we used, the PWM clock divider was set to
20.

The topic we publish to in order to control the channel 0 PWM output is:

::

    /ronex/general_io/12/command/pwm/0

Set pwm\_period to 64000, then pwm\_on\_time\_0 can be set from 1600 -
8000 to correspond to Servo angles of 0 - 180°.

The Code
========

The Python script to control the Servo with the Potentiometer shown in
the video is as follows:

.. code:: python

    #!/usr/bin/env python

    import roslib; roslib.load_manifest('sr_ronex_examples')
    import rospy
    from sr_ronex_msgs.msg import PWM, GeneralIOState

    def callback(data):

        analogue = data.analogue 
        pwmTopic = '/ronex/general_io/12/command/pwm/0'
        pub = rospy.Publisher(pwmTopic, PWM)
        pwm = PWM()
        pwm.pwm_period = 64000
        pwm.pwm_on_time_0 = int(2.6042*analogue[0] + 1600)
        pwm.pwm_on_time_1 = 0
        pub.publish(pwm)   

    if __name__ == "__main__":

        rospy.init_node("ronex_pot_servo_demo")
        rospy.Subscriber('/ronex/general_io/12/state', GeneralIOState, callback)
        rospy.loginfo('RoNeX Pot Servo Demo Started!')
        rospy.spin()
        

The Code Explained
==================

.. code:: python

    if __name__ == "__main__":

        rospy.init_node("ronex_pot_servo_demo")
        rospy.Subscriber('/ronex/general_io/12/state', GeneralIOState, callback)
        rospy.loginfo('RoNeX Pot Servo Demo Started!')
        rospy.spin()

In the main function we initialise our ROS node, then setup a subscriber
to receive the position of the potentiometer on the state topic. Then an
info message is printed to let the user know the program is running,
then spin() is called to get the node up and running.

.. code:: python

    def callback(data):

        analogue = data.analogue 
        pwmTopic = '/ronex/general_io/12/command/pwm/0'
        pub = rospy.Publisher(pwmTopic, PWM)

In the callback function the analogue data is retrieved from the
message, and a publisher is created to send the required PWM command to
the appropriate topic.

.. code:: python

        pwm = PWM()
        pwm.pwm_period = 64000
        pwm.pwm_on_time_0 = int(2.6042*analogue[0] + 1600)
        pwm.pwm_on_time_1 = 0
        pub.publish(pwm)   

Here a message of format PWM is created, and filled with the required
fields.

The calculations for converting the analogue value from the
potentiometer into a corresponding PWM value for the servo can be found
here: `Analogue to PWM Configuration <Analogue-to-PWM-Configuration>`__.

Note: The video shows a value of 1.5625 instead of 2.6042, this was
mapping the 300° range of the potentiometer to the 180° range of the
servo. In this case we directly map the bottom 180° of the potentiometer
to the servo.

Running the Code
================

You can copy this straight into a new ROS package, be sure to make it
executable using chmod +x if you want to run it using rosrun.

When you run the script, be sure that input\_mode is set to false for
the PWM signal channel, and that the publisher plugin in the GUI is not
publishing.
