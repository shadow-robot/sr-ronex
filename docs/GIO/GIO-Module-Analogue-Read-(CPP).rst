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

Change directories to your sr\_ronex\_examples package.

::

    $ roscd sr_ronex_examples/

C++ file **sr\_ronex\_read\_analog\_data.cpp** is located inside the
**src** directory.

\`\`\`c++ #include #include "sr\_ronex\_msgs/GeneralIOState.h"

void generalIOState\_callback(const
sr\_ronex\_msgs::GeneralIOState::ConstPtr& msg) { const std::vector
&analogue = msg->analogue; const size\_t len = analogue.size(); for
(size\_t k = 0; k < len; k++) ROS\_INFO\_STREAM( "analogue[" << k << "]
= " << analogue[k] << "" ); }

int main(int argc, char \*\*argv) { ros::init(argc, argv,
"sr\_ronex\_read\_analog\_data"); ros::NodeHandle n; ros::Subscriber sub
= n.subscribe( "/ronex/general\_io/12/state", 1000,
generalIOState\_callback); ros::spin(); } \`\`\`

The Code Explained
------------------

``C++ #include <string> #include <ros/ros.h> #include "sr_ronex_msgs/GeneralIOState.h"``
First we load import appropriate modules and messages, including the
standard ros.h header to create a ros node and get access to the topics,
and the GeneralIOState message format we're going to receive from the
state topic.

``c++ int main(int argc, char **argv) {   ros::init(argc, argv, "sr_ronex_read_analog_data");   ros::NodeHandle n;   ros::Subscriber sub = n.subscribe( "/ronex/general_io/12/state", 1000, generalIOState_callback);   ros::spin(); }``
In the main function (at the bottom of the file) we initialise the ROS
node that will publish the messages, and add a handle to this node. We
then create the subscriber specifying topic, message queue size and
callback function, then finally the spin() function is called to start
listening for messages.

For simplicity, in this example we hard code the ronex id (12), and you
will need to change it to correspond to that of your RoNeX GIO module.
To make this example more robust, the path to the first connected device
could be retrieved from the parameter server. To learn how to do this
you can follow the [[Parse Parameter Server (CPP) Tutorial\|Parse
Parameter Server (CPP)]].

``c++ void generalIOState_callback(const sr_ronex_msgs::GeneralIOState::ConstPtr& msg) {   const std::vector<short unsigned int> &analogue = msg->analogue;   const size_t len = analogue.size();   for (size_t k = 0; k < len; k++)     ROS_INFO_STREAM( "analogue[" << k << "] = " << analogue[k] << "\n" ); }``
This callback function is called every time a message is received on the
state topic. We first create a vector of type short unsigned int then
populate it with the analogue values from the current message, and
assign the length of this vector to another variable. We then cycle
through the vector outputting the values to the screen and ROS log
files.

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
easiest way to do this is to use the [[GUI\|Dynamic Reconfigure GUI]]
and set ``input_mode_0`` to ``false``.

Once this is done we can run our C++ program:

::

    $ rosrun sr_ronex_examples sr_ronex_read_analog_data

Now you should to able to see the analogue data on the console.
