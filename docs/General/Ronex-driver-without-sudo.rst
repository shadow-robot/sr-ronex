Running RoNeX without superuser
===============================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


To be able to run the RoNeX driver as a normal Linux user (no superuser
privileges), we provide you with a handy script.

::

    rosrun ros_ethercat_loop ethercat_grant

Note: Executing the above commands in a terminal window once will set
file capabilities until you next update the ros\_ethercat package, after
which you will need to set them again.
