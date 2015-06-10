Configure GIO Module (command line)
====================================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:

To update the configuration of your GIO module, for example setting each
channel as an input or output mode, you'll need to use the Dynamic
Reconfigure interfaces. To do this from the command line we can call a
service and tell the RoNeX driver which configurations we want to
change.

For the following command you can use the tab key to auto complete the
name of the service and message type, and once you've entered the
message type you can actually double tap tab again to produce an empty
message in the correct format, handy for when you're not familiar with
the message types.

In the example below we will set 4 of the digital channels as an
outputs:


.. code-block:: bash

    rosservice call /ronex/general_io/12/set_parameters "config:
      bools:
      - {name: 'input_mode_0', value: false, name: 'input_mode_1', value: false, name: 'input_mode_2', value: false, name: 'input_mode_3', value: false}"

The service call will return the current state of the GIO parameters,
alternatively you can view this information at any time by echoing the
relevant topic:

.. code-block:: bash

    rostopic list /ronex/general_io/12/parameter_updates
