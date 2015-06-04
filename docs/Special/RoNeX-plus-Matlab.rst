MATLAB now supports direct integration with
`ROS <http://wiki.ros.org/hydro>`__! Of course one of the first things
we did was test this with RoNeX, and happily, it worked great. In this
section, we detail how to use RoNeX with MATLAB (**R2013a** or
**R2013b**).

If you don’t already have it, ROS support for MATLAB can be downloaded
`here <http://www.mathworks.co.uk/hardware-support/robot-operating-system.html>`__:

::

    http://www.mathworks.co.uk/hardware-support/robot-operating-system.html

Requirements
~~~~~~~~~~~~

In this example, we are running Matlab and all ROS nodes (including
RoNeX nodes) on a single machine. Since RoNeX is not supported on
Windows, the example was executed on a **64-bit** Linux (we used
`**Ubuntu** <http://www.ubuntu.com>`__) machine. However, it is possible
to run Matlab on a Windows machine, whereas RoNeX stays on a Linux
machine, and let them communicate with each other over network.

Getting Started
~~~~~~~~~~~~~~~

Inside the package, there is one pdf file
(*ros-io-package-getting-started-guide.pdf*). Simply follow the
instructions in it to install the MATLAB ROS I/O package.

In the pdf file, there is one example that explains the basic
instructions for creating publishers and subscribers inside MATLAB.
Here, we explain how to publish data outside MATLAB and receive it
inside MATLAB.

Assuming that you just want to run MATLAB and other ROS nodes on your
local machine, set these environment variables (for example in the
.bashrc file):

::

    $ export ROS_HOSTNAME=localhost
    $ export ROS_MASTER_URI=http://localhost:11311

ROS\_HOSTNAME affects the network address of a node, and it is the key
environment variable for this example. The ROS\_MASTER\_URI is an
important environment variable that tells a node where the master is.
However, great care should be taken when using localhost in
ROS\_MASTER\_URI, as that can lead to unintended behaviors with remotely
launched nodes.

Example Setup
~~~~~~~~~~~~~

Make sure that you have installed MATLAB, ROS Hydro, RoNeX software on
the Linux computer, and the RoNeX hardware is connected to the computer.

In the following MATLAB code example, we create a ROS node inside MATLAB
and then connect it to an existing ROS master (outside MATLAB). The ROS
node inside MATLAB will act as the subscriber, whereas the RoNeX
software will act as the publisher. Note that the topic name is
*my\_data* and the message type is
`*std\_msgs/Int16MultiArray* <http://docs.ros.org/api/std_msgs/html/msg/Int16MultiArray.html>`__
in this example.

::

    %% Create a Subscriber.
    % Create a new node and connect it to the master.
    % Type help rosmatlab.node in MATLAB to get more info about it.
    node = rosmatlab.node('my_data_listener_node','http://localhost:11311');

    % Add a subscriber to a topic named my_data to the node to receive message of type std_msgs/Int16MultiArray.
    subscriber = rosmatlab.subscriber('/my_data','std_msgs/Int16MultiArray',100,node);

    % Set my_function to execute when a valid message is published.
    subscriber.setOnNewMessageListeners({@my_function});

    %% Keep the node alive for a while.
    for i = 1:10000
        pause(0.1)
    end

    %% Remove the subscriber from the node.
    node.removeSubscriber(subscriber);

    %% Clean up workspace.
    clear;

Once an message is sent by the RoNeX software and then received by the
ROS node inside MATLAB, we extract the integers stored inside the array
*data*, and output it to the drive. Note that the *layout* info inside
message is not processed here.

::

    function my_function(message)
    mydata = message.getData()
    savefile = 'mydata.mat';
    save(savefile, 'mydata');
    end

