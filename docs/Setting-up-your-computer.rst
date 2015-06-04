The current RoNeX driver is based on `ROS <http://www.ros.org>`__. We
recommend that you use a computer with Ubuntu LTS 12.04 ("Precise" or
"Pangolin") installed on it. You will need less than 1GB of additional
disk space for all of the ROS components. Follow the instructions here:
`"Installing ROS
Hydro" <http://wiki.ros.org/hydro/Installation/Ubuntu>`__ to install ROS
Hydro on Ubuntu - we recommend the "ros-hydro-desktop-full" package.

Installing the drivers for ROS Hydro
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This is the **easiest and recommended way** to install the RoNeX
drivers. To install the drivers and their dependencies, simply run:

::

     $ sudo apt-get install ros-hydro-sr-ronex

Using ROS is made much easier by setting up the terminal window and its
environment. If hydro is the only ROS version on the computer then you
should do:

::

     $ sudo echo "source /opt/ros/hydro/setup.bash" >> ~/.bashrc

Now each new terminal window will have the ROS Hydro environment setup
automatically.

If you need to have multiple versions of ROS installed, then using this
method would force everyone to use Hydro! If you don't want to do that,
then in each new terminal window you should type:

::

     $ source /opt/ros/hydro/setup.bash

Installing the drivers for ROS Groovy
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Due to some changes to the pr2 code base that were integrated in ROS
Hydro only, if you want to use RoNeX with ROS Groovy, you'll have to
install it from sources. This is a more time consuming approach. If you
need to go down this route, please follow [[these instructions\|Groovy
Install]].

Setting up a Catkin workspace
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you're planning on developing code with RoNeX, chances are you'll be
using Catkin to build your programs. Therefore you're going to need a
Catkin workspace, which can be set up following [[these
instructions\|http://wiki.ros.org/catkin/Tutorials/create\_a\_workspace]]

Don't forget to source the workspace devel/setup.bash file in your
.bashrc file to add the packages in the workspace to the ROS
environment.
