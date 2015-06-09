Launching the RoNeX driver
==========================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


To start the driver, you'll need elevated permissions. Run:

::

     sudo -s
     roslaunch sr_ronex_launch sr_ronex.launch

By default the driver is looking for the RoNeX on **eth0**. If you want
to start it another interface, you can add the correct ethercat port
argument to the launch command:

::

     roslaunch sr_ronex_launch sr_ronex.launch ethercat_port:=eth1
