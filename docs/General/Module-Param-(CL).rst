Accessing Module parameters [Command Line]
==========================================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


The ROS parameter server stores details of each RoNeX module such as
serial number, module type, module alias etc. These can be read using
the rosparam commands as follows.

To list all of the entries currently in the parameter server:

.. code-block:: bash

    rosparam list

You should see a number of entries under /ronex/devices for each module
you have connected. You can view all of the details for one module (the
first on the bus in this case) like so:

.. code-block:: bash

    rosparam get /ronex/devices/0

Which will return something along the lines of:

.. code-block:: yaml

    {path: /ronex/general_io/12, product_id: '33554433', product_name: general_io, ronex_id: '12',
      serial: '12'}

You can also read the input mode for each of the channels along with
some other info for each module, but beware that as these are dynamic
parameters, changing them with the rosparam set command will not work.
You should instead use the dynparam interfaces as explained in the
Changing GIO Configuration tutorials.
