GIO - Read analogue inputs (python)
====================================

A General I/O Module consists of one GIO Node board and one GIO
Peripheral board. Each module has 12 Analogue sampling channels (12bits,
1kHz) (a GIO Node board has 6 analogue input channels, a GIO Peripheral
board has also 6).

This example demonstrates how to read analogue data with RoNeX. As
input, you can use for example a potentiometer (a three-terminal
resistor with a sliding contact that forms an adjustable voltage
divider).

The code
========

First change directories to your **sr\_ronex\_examples** package.

::

    $ roscd sr_ronex_examples/

Python file **sr\_ronex\_read\_analog\_data.py** is located inside the
**src** directory.

.. code:: python

    #!/usr/bin/env python

    import rospy
    from sr_ronex_msgs.msg import GeneralIOState

    def generalIOState_callback(data):
        analogue = data.analogue
        rospy.loginfo( "analogue = %s", analogue )

    if __name__ == "__main__":
        rospy.init_node("sr_ronex_read_analog_data")
        topic = "/ronex/general_io/12/state"
        rospy.Subscriber( topic, GeneralIOState, generalIOState_callback )
        rospy.spin()

The Code Explained
------------------

.. code:: python

    import rospy
    from sr_ronex_msgs.msg import GeneralIOState

First we load import appropriate modules and messages, including rospy
to make us a ros node and get access to the topics, and the
GeneralIOState message format we're going to receive from the state
topic.

.. code:: python

    if __name__ == "__main__":
        rospy.init_node("sr_ronex_read_analog_data")
        topic = "/ronex/general_io/12/state"
        rospy.Subscriber( topic, GeneralIOState, generalIOState_callback )
        rospy.spin()

In the main function (at the bottom of the file) we initialise the ROS
node that will publish the messages, and set up the topic to be
subscribed to. We then create the subscriber specifying topic, message
format and callback function, then finally the spin() function is called
to start listening for messages.

For simplicity, in this example we hard code the ronex id (12), and you
will need to change it to correspond to that of your RoNeX GIO module.
To make this example more robust, the path to the first connected device
could be retrieved from the parameter server. To learn how to do this
you can follow the [[Parse Parameter Server (Python) Tutorial\|Parse
Parameter Server (Python)]].

.. code:: python

    def generalIOState_callback(data):
        analogue = data.analogue
        rospy.loginfo( "analogue = %s", analogue )

This callback function is called every time a message is received on the
state topic. We extract the analogue data from the state message and
then output it to screen. The analogue data is in the format tuple.

Running the code
================

First make sure that the RoNeX driver is running (see `Launch
driver <Home#launching-the-ronex-driver>`__ ).

Once this is done we can run our Python script:

::

    $ rosrun sr_ronex_examples sr_ronex_read_analog_data.py

Now you should to able to see the analogue data on the console.
