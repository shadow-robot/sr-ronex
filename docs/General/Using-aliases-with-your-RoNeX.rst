Using aliases with your RoNeX
=============================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


You can give aliases (i.e., pseudonyms) to your RoNeX modules (such as
General I/O modules). An alias will be used as a unique identifier for
one specific RoNeX module instead of its serial number.

Set an alias for a RoNeX module
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Start a roscore in a terminal:

.. code-block:: bash

    roscore

Then in another terminal, set the RoNeX module to use the alias
``test_ronex``. In the following command, we use ``1234`` as the serial
number - replace it with your own.


.. code-block:: bash

    rosparam set /ronex/mapping/1234 "test_ronex"

Now you can start the RoNeX driver, and the ronex\_id will reflect the
alias above. If the driver is already running, you will need to restart
it in order for the changes to take effect. Additionally, when you stop
the roscore, the parameters will be erased and you will need to set the
parameter again before running the driver next time. If you are planning
on using the same alias constantly, you can set this parameter at the
start of the sr\_ronex.launch file by adding the following line after
the first tag:

.. code-block:: xml

    <param name="/ronex/mapping/1234" value="test_ronex" />
