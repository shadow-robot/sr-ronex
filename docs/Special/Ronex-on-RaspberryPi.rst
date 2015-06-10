RoNeX on Raspberry Pi
=====================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


*This is highly experimental!*

Using pre-built image
---------------------

You can download a pre-built and ready to use image, that can be copied
to a 16GB (or bigger) SD card.

The following commands show a way to do it. sdx should be replaced with
the device name that your system assigns to the SD card when you plug it
in (e.g. using a card reader).

Bear in mind that this process will destroy any data on that SD card.

.. code-block:: bash

    cd ~/Downloads
    wget -O rPi_ronex.gz "https://doc-0c-2c-docs.googleusercontent.com/docs/securesc/ha0ro937gcuc7l7deffksulhg5h7mbp1/33asrpi69rh5a2hgqp8j7qdqjc4tv1ne/1389708000000/00764167951976606724/*/0B7wJhvk4Ba2NSWdTWkVfY05sQjg?h=16653014193614665626&e=download"
    gunzip rPi_ronex.gz
    sudo dd if=~/Downloads/rPi_ronex of=/dev/sdx bs=1M

If the previous download link doesn't work, please try to download the
image from the following link: `rPi\_ronex
image <https://drive.google.com/file/d/0B7wJhvk4Ba2NSWdTWkVfY05sQjg/edit?usp=sharing>`__.

User and password
~~~~~~~~~~~~~~~~~

This image has the default user and password for NOOBS install
procedure:

::

    user: pi
    pwd: raspberry

Building step by step
---------------------

If you don't want to use the pre-built image, you can install raspbian
on your SD card, and install the necessary software for Ronex by
following this step by step procedure.

References
~~~~~~~~~~

The main install procedure has been taken from:

`Setting up Hydro on
RaspberryPi <http://wiki.ros.org/ROSberryPi/Setting%20up%20Hydro%20on%20RaspberryPi>`__

It is reproduced here with some slight changes to have full step by step
procedure.

Raspberry Pi hardware version
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This procedure has been tested on a Raspberry Pi with 256 MB RAM.

Install Raspbian
~~~~~~~~~~~~~~~~

-  Start with NOOBS. There are several ways to do this, and plenty of
   tutorials on the internet.
-  Install Raspbian from NOOBS.

Tweaks to the OS
~~~~~~~~~~~~~~~~

On a 256 MB RAM board, we will need to maximise the available RAM and
increase the swap size to succeed in the compilation of the necessary
packages.

-  Reduce GPU memory size to allow more ram:

::

    sudo nano /boot/config.txt

then add or edit:

::

    gpu_mem=16

-  Change the size of the swap space in /etc/dphys-swapfile to 500MB
   (for compilation at least. It could be set back to 100 MB afterwards)

::

    sudo nano /etc/dphys-swapfile

-  It is also possible to overclock the arm processor:

::

    sudo nano /boot/config.txt

then edit

::

    arm_freq=800

All these changes require a reboot to take effect.

Install ROS and the necessary packages for Ronex
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

-  Install repositories:

::

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu raring main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get upgrade

-  Install the dependencies:

::

    sudo apt-get install python-pip
    sudo pip install rosdistro
    sudo pip install wstool

-  Install setup tools:

::

    cd ~/Downloads
    wget https://pypi.python.org/packages/source/s/setuptools/setuptools-1.1.6.tar.gz
    tar xvf setuptools-1.1.6.tar.gz
    cd setuptools-1.1.6
    sudo python setup.py install

-  Install stdeb:

::

    sudo apt-get install python-stdeb

-  Install dependencies

::

    sudo pip install rosdep
    sudo pip install rosinstall-generator
    sudo pip install wstool

    pypi-install rospkg
    sudo apt-get install python-rosdep python-rosinstall-generator build-essential

-  Now continue from the Hydro install from source page. This downloads
   all the packages, and takes a couple hours.

::

    mkdir ~/ros_catkin_ws
    cd ~/ros_catkin_ws
    rosinstall_generator ros_comm sr_ronex --rosdistro hydro --deps --wet-only > hydro-sr_ronex-wet.rosinstall
    wstool init -j8 src hydro-sr_ronex-wet.rosinstall
    sudo rosdep init
    rosdep update
    rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy

Now the rosdep fails :

::

    Package sbcl is not available, but is referred to by another package.
    This may mean that the package is missing, has been obsoleted, or
    is only available from another source
    However the following packages replace it:
      sbcl-source sbcl-doc

    E: Package 'sbcl' has no installation candidate
    ERROR: the following rosdeps failed to install
      apt: command [sudo apt-get install -y sbcl] failed

Apparently the roslisp package uses sbcl, which is not available for the
pi, so we have to remove that.

::

    cd src
    wstool rm roslisp
    rm -rf roslisp
    cd ..
    $
    rosdep install  --from-paths src --ignore-src --rosdistro hydro -y --os=debian:wheezy

That worked! Now check that the ethercat\_hardware package is at least
in the version 1.8.6:

::

    cd ~/ros_catkin_ws/src
    wstool info pr2_ethercat_drivers/ethercat_hardware

    If it's not (i.e it is in 1.8.5-0) then do:
    wstool set pr2_ethercat_drivers/ethercat_hardware -v release/hydro/ethercat_hardware/1.8.6-0
    wstool up pr2_ethercat_drivers/ethercat_hardware

-  For a Raspberry Pi it is recommended to reduce the realtime loop
   frequency from the default 1 KHz to 250 Hz. An easy way to do it is
   to download and apply this patch:

::

    cd ~/Downloads
    wget -O reduced_loop_freq.patch "https://doc-0o-2c-docs.googleusercontent.com/docs/securesc/ha0ro937gcuc7l7deffksulhg5h7mbp1/9lnv6nmdd4evkvbbkf1ltgt35dhackgh/1389636000000/00764167951976606724/*/0B7wJhvk4Ba2NLVhJSGw3ZUF5M0E?h=16653014193614665626&e=download"
    cd ~/ros_catkin_ws/src/pr2_ethercat
    patch -p1 < ~/Downloads/reduced_loop_freq.patch

-  Now it's time to try building it:

::

    cd ~/ros_catkin_ws
    ./src/catkin/bin/catkin_make_isolated --install

Success!!!!

Make sure you reference the newly created install:

::

    cd ~
    echo "source ~/ros_catkin_ws/install_isolated/setup.bash" >> .bashrc
    source .bashrc
