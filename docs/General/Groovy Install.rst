Groovy Install
==============

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


We assume you've installed ROS groovy `following those
instructions <http://wiki.ros.org/groovy/Installation/Ubuntu>`__.

-  You'll need to install the following packages to be able to build
   from source.

::

    sudo apt-get install python-rosdep python-wstool build-essential
    sudo rosdep init
    rosdep update

-  You can now download the `rosinstall
   file <https://gist.github.com/ugocupcic/6636982/download>`__ - this
   file contains all the information needed to download the different
   source of the packages that need to be built. We'll assume the file
   has been downloaded to ``~/Downloads/sr_ronex.rosinstall``.

-  Let's create a catkin workspace to download and compile the different
   packages. We'll use ``~/catkin_ws`` for this step by step (but it
   could be anywhere).

::

     mkdir -p ~/catkin_ws/src
     cd ~/catkin_ws/src
     wstool init
     wstool merge ~/Downloads/ronex-groovy.rosinstall
     wstool update
     rosdep install --from-paths src --ignore-src --rosdistro groovy -y

-  To load the workspace, you need to source the setup.bash:

::

     source ~/catkin_ws/devel/setup.bash

-  If you want the workspace to be sourced each time you open a
   terminal, then you can source it in your ``~/.bashrc``. To do this
   run:

::

     echo "source ~/catkin_ws/devel/setup.bash" >~/.bashrc

-  It is now time to build all those packages (this can take a long
   time)

::

     cd ~/catkin_ws
     catkin_make_isolated

Once this command finishes successfuly, the RoNeX drivers are installed.
You can continue reading the wiki to :doc:`get started with your RoNeX </README>`.
