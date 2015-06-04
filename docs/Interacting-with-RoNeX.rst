There are four main methods to interact with RoNeX, this section will
give a brief introduction to these methods.

Command Line
^^^^^^^^^^^^

We can access these interfaces using ROS's command line tools, which are
convenient for getting a quick overview of what is going on in the
system.

Python and C++
^^^^^^^^^^^^^^

Once you're familiar with the basic operation of the RoNeX from the
command line, we can take a step to automate the interactions by writing
them into a program. This page features examples using both Python and
C++. While coding in Python is sometimes simpler, it is mainly down to
user preference when selecting which language to use.

ROS GUI
^^^^^^^

There are a number of tools included in the rqt\_gui packages, that
allow us to interact with RoNeX through the standard ROS GUI, without
having to go through typing out the terminal commands repeatedly. These
can be handy for plotting received data, or repeatedly sending commands.
To start the gui, run:

::

    rosrun rqt_gui rqt_gui

