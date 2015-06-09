Configure GIO Module (python)
=============================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:

This tutorial will walk you through the basics of using the dynamic
reconfigure client in a python script to adjust your RoNeX
configuration. If you are often executing different RoNeX scripts that
require different module configuration, it may be beneficial adjust the
configuration at the start of each script.

This example can be found under
sr\_ronex\_examples/src/change\_ronex\_configuration.py # The Code

.. code:: python

    #!/usr/bin/env python

    import rospy
    import dynamic_reconfigure.client

    class ChangeRonexConfigurationExample(object):

        def __init__(self):
            ronex_id = "test_ronex"
            ronex_path = "/ronex/general_io/" + ronex_id + "/"
            self.configure_ronex(ronex_path)

        def configure_ronex(self, path):
            client = dynamic_reconfigure.client.Client(path)
            params = { 'input_mode_0' : False, 'input_mode_1' : False, 'pwm_period_0' : 200 , 'pwm_clock_divider' : 3000}
            config = client.update_configuration(params)

    if __name__ == "__main__":
        rospy.init_node("change_ronex_configuration_py")
        ChangeRonexConfigurationExample()

The Code Explained
------------------

Now lets break down the code to make it easier to reuse in other
programs:

.. code:: python

    #!/usr/bin/env python

    import rospy
    import dynamic_reconfigure.client

We need to import the dynamic reconfigure client module at the start of
the script.

.. code:: python

    if __name__ == "__main__":
        rospy.init_node("change_ronex_configuration_py")
        ChangeRonexConfigurationExample()

At the bottom of the script we have the main function where we
initialise a node to carry out the config changes. If you've already
created a node in your script for other purposes, you can just use that
one. We then call the ChangeRonexConfigurationExample class which will
execute the config changes.

.. code:: python

        def __init__(self):
            ronex_id = "test_ronex"
            ronex_path = "/ronex/general_io/" + ronex_id + "/"
            self.configure_ronex(ronex_path)

The class' init function gets called automatically. Here we set the path
to the RoNeX module for which we want to change the configuration. This
example assumes the alias "test\_ronex" has already been set for your
module, as described in :doc:`this tutorial </General/Using-aliases-with-your-RoNeX>`. Alternatively you can just
use the serial number of your RoNeX module here instead. This path is
then passed to the configure\_ronex function.

.. code:: python

        def configure_ronex(self, path):
            client = dynamic_reconfigure.client.Client(path)
            params = { 'input_mode_0' : False, 'input_mode_1' : False, 'pwm_period_0' : 200 , 'pwm_clock_divider' : 3000}
            config = client.update_configuration(params)

Here we create a client instance and tell it the path to the RoNeX
module to configure. Next we list the parameters than we want to change,
here we are setting the first digital I/O channel to output mode,the
second channel to input mode, adjusting pwm\_period\_0 and the PWM clock
divider. We then call update\_configuration to pass these changes to the
dynamic reconfigure server.

The contents of this configure\_ronex function (plus the dynamic
reconfigure client module import from the top of the script) are all you
really need if you want change your configuration from an existing
script.

Running the code
----------------

Make sure that a roscore is up and running:

::

    roscore

Then run the driver (see :doc:`Launch driver </General/Launching-the-RoNeX-driver>` ).

Now we can execute the example script:

::

    rosrun sr_ronex_examples change_ronex_configuration.py

Now if you echo the contents of the parameter\_descriptions topic for
this module, you should see that the configuration has been updated
accordingly.
