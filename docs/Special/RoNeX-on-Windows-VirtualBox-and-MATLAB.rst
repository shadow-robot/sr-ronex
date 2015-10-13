RoNeX on Windows with Matlab
============================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


This tutorial shows how to interact with RoNeX from MATLAB with minimal
prior knowledge of Linux and ROS. RoNeX drivers run on Ubuntu within a
VirtualBox virtual machine (A pre-configured VM image is available for
download) so a native Linux install is not required. We use the MATLAB
ROS support package under MATLAB R2013a or R2013b to send/receive data
from RoNeX and process it in MATLAB.

For this tutorial we used Windows 8 64 bit install as the host system,
although it should work on other versions of Windows without issues. It
is running on a Lenovo Notebook computer, with a RoNeX stack connected
to the Ethernet port, and a Wifi card connected to a network using DHCP.

If you experience problems during this tutorial, `please let us
know <https://github.com/shadow-robot/sr-ronex/issues?state=open>`__!

Installing VirtualBox + RoNeX Image
-----------------------------------

First download + install VirtualBox. The latest install file can be
downloaded from: https://www.virtualbox.org/wiki/Downloads

Next install a Virtual Machine image with Ubuntu, ROS and the RoNeX drivers.

Configure VirtualBox
--------------------

Once installed, open VirtualBox, go to File > Import Appliance, then
find the Ubuntu ROS\_RoNeX.ova system image you've just downloaded. You
can adjust the amount of resources available to the VM here if you like,
otherwise click import.

The host machine has both Wired and Wireless network cards configured to
obtain an IP address automatically using DHCP. Once you have imported
the virtual machine, you will need to open Settings > Network for that
machine, and update the settings for adapter 1 (wired) and adapter 2
(wireless). By default the network adapter names correspond to those on
our test machine, so you will need to select the correct names for your
machine from the drop-down list. Ensure both adapters are attached to:
"Bridged Adapter". You shouldn't need to adjust the advanced settings.

Ensure the machine is connected to your wifi network and has obtained an
IP address. If you don't have a wireless or second wired card for a
network connection, it is possible to set up static IPs for
communication between MATLAB and VirtualBox, but that is beyond the
scope of this tutorial.

Install MATLAB + ROS toolbox
----------------------------

To interact with ROS you will require MATLAB R2013a or R2013b. If you
don't have MATLAB already, you can head over to the Mathworks website to
find out how to get it:

http://www.mathworks.com/

Once MATLAB is set up, you will need to set up ROS support by
downloading it from here:

http://www.mathworks.com/hardware-support/robot-operating-system.html

Configure ROS inside Virtual Machine
------------------------------------

With your network adapters correctly configured in VirtualBox, once
Ubuntu has booted, it should have obtained a unique IP on your network.
You can check this IP by opening a terminal window (ctrl+alt+t) and
typing:

::

    ifconfig

Under adapter eth1 - inet addr: you should see the IP assigned to the
virtual machine, make a note of it. Now in the same terminal we're going
to set this IP in our ROS configuration. Open up the .bashrc config file
by typing:

::

    gedit ~/.bashrc

Scroll to the bottom of the file, and you should see the following
lines：

::

    export ROS_HOSTNAME=192.168.0.14
    export ROS_MASTER_URI=http://192.168.0.14:11311

Change the IP address there to the one obtained from ifconfig. Be sure
to leave the "http://" and ":11311" on the ROS\_MASTER\_URI.

Hardware Configuration
----------------------

As previously mentioned, RoNeX is connected to the wired Ethernet port
on the Laptop, which should now be set as eth0 in the virtual machine.

Connect one of your pre-made RoNeX GIO cables, to the analogue port on
the GIO node, then connect 5V, GND and Channel 0 to your potentiometer,
preferably on a solderless breadboard.

MATLAB Code
-----------

The MATLAB code below starts a RoNeX node and subscriber to receive
RoNeX data.

The first section should be copied to a new MATLAB script in your
workspace, named rostest.m

.. code-block:: matlab

    rosinit('http://192.168.0.14:11311')
    sub = rossubscriber('/ronex/general_io/12/state', @my_function, 'Buffersize', 10))

    i = 'a';
    while i ~= 'q'
        i = input('q to quit', 's')
    end

    rosshutdown
    clear;

This code should be copied into a file named my\_function.m, in the same
directory as rostest.m

.. code-block:: matlab

    function my_function(~, message)
        mydata = message.Analogue;
        disp (mydata(1));
    end

Modifying the MATLAB Code
-------------------------

You will need to change the first line of the MATLAB script, to ensure
the IP address listed for the ROS master, matches that we obtained from
Ubuntu in the ROS configuration step.

.. code-block:: matlab

    rosinit('http://192.168.0.14:11311')

The Code Explained
------------------

The MATLAB code we are running is fairly simple, you can find a PDF
manual with more information on the various functions in the MATLAB ROS
Support package you downloaded. We'll start by looking at rostest.m:

.. code-block:: matlab

    rosinit('http://192.168.0.14:11311')
    sub = rossubscriber('/ronex/general_io/12/state', @my_function, 'Buffersize', 10)

All of the ROS setup is done in these three lines. In the first a ROS
node is defined (the user could choose an appropriate name),
with the address of the ROS master.

Secondly a subscriber is defined to receive messages on the specified
topic. The RoNeX message formats are included in the MATLAB ROS package
by default, so we don't have to worry about custom messages.
We set the callback function for the subscriber, i.e. whenever a
valid message is received on the topic, this function is executed.

Finally we set a queue size of 10, this may need to be
increased on slower machines so they don't miss messages.

.. code-block:: matlab

    i = 'a';
    while i ~= 'q'
        i = input('q to quit', 's')
    end

This code is to allow us to quit the program cleanly. It loops while
waiting for keyboard input, if the received input character is a 'q', it
will move on to shutdown the node.

.. code-block:: matlab

    rosshutdown
    clear;

Once the quit message has been received, the node is shutdown and
then everything is removed from the variable workspace using
the clear command.

Next we will look at the my\_function.m file:

.. code-block:: matlab

    function my_function(~, message)
        mydata = message.Analogue;
        disp (mydata(1));
    end

This function is called every time a message is received on the
corresponding topic, and is passed the message as an input argument.
MATLAB allows to access the fields of the message but renames them to a 
camel case format instead of the lower case with underscores that is usual in ROS.
The RoNeX state message has an analogue field,
so we can access the Analogue field in the message (we could
similarly access Digital).

Once we have the data, as the potentiometer is connected to channel 0,
we are interested in the first value in the array, so we display this
value then wait for the next message.

Execute Programs
----------------

First make sure ROS and the RoNeX driver are running in the virtual
machine, as described in (see :doc:`Launch driver </General/Launching-the-RoNeX-driver>` ).

Next run the MATLAB script (Either by right clicking on it in the
current folder window and clicking run, typing the name of the file in
the command window, or pressing f5 in the rostest.m editor window)

You should see the data from the potentiometer displayed in the command
window, as you turn the pot, the values will reflect the angle of the
shaft. Press q then enter when you are ready to quit the program.
