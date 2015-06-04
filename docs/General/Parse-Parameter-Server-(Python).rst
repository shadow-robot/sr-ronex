The example demonstrates how to access RoNeX modules listed in the
parameter server. For each module, the parameter server stores
parameters such as its product\_id, product\_name, ronex\_id, path, and
serial. Note that we assume here that RoNeX consists of a Bridge (IN)
module, and one or multiple General I/O module(s), although all modules
will display similar attributes.

The code
========

First change directories to your **sr\_ronex\_examples** package.

::

    $ roscd sr_ronex_examples/

Python file **sr\_ronex\_parse\_parameter\_server.py** is located inside
the **src** directory.

.. code:: python

    #!/usr/bin/env python

    import rospy

    #--------------------------------------------------------------------------------

    class SrRonexParseParamExample(object):

        def __init__(self):
            self.find_general_io_modules()
            
        def find_general_io_modules(self):
            """
            Find the General I/O modules present on the system.
            """
            while True:
                try:
                    rospy.get_param("/ronex/devices/0/ronex_id")
                    break
                except:
                    rospy.loginfo("Waiting for the General I/O module to be loaded properly.")
                    sleep(0.1)

            devices = rospy.get_param("/ronex/devices")
            for ronex_param_id in devices:
                rospy.loginfo( "*** General I/O Module %s ***",  ronex_param_id );
                rospy.loginfo( "product_id   = %s", devices[ronex_param_id]["product_id"] );
                rospy.loginfo( "product_name = %s", devices[ronex_param_id]["product_name"] );
                rospy.loginfo( "ronex_id     = %s", devices[ronex_param_id]["ronex_id"] );
                rospy.loginfo( "path         = %s", devices[ronex_param_id]["path"]);
                rospy.loginfo( "serial       = %s", devices[ronex_param_id]["serial"] );

    #--------------------------------------------------------------------------------

    if __name__ == "__main__":

        rospy.init_node("sr_ronex_parse_parameter_server") 
        SrRonexParseParamExample()

    #--------------------------------------------------------------------------------

The Code Explained
------------------

.. code:: python

    if __name__ == "__main__":

        rospy.init_node("sr_ronex_parse_parameter_server") 
     
        SrRonexParseParamExample()

``rospy.init_node(NAME)`` is very important as it tells rospy the name
of your node -- until rospy has this information, it cannot start
communicating with the ROS Master. In this case, your node will take on
the name **sr\_ronex\_parse\_parameter\_server**.

The next line ``SrRonexParseParamExample()`` launches the demo by
creating an object of class **SrRonexParseParamExample**.

.. code:: python

      while True:
                try:
                    rospy.get_param("/ronex/devices/0/ronex_id")
                    break
                except:
                    rospy.loginfo("Waiting for the General I/O module to be loaded properly.")
                    sleep(0.1)

Loop until at least one General I/O module has been properly loaded.

.. code:: python

           devices = rospy.get_param("/ronex/devices")
            for ronex_param_id in devices:
                rospy.loginfo( "*** General I/O Module %s ***",  ronex_param_id );
                rospy.loginfo( "product_id   = %s", devices[ronex_param_id]["product_id"] );
                rospy.loginfo( "product_name = %s", devices[ronex_param_id]["product_name"] );
                rospy.loginfo( "ronex_id     = %s", devices[ronex_param_id]["ronex_id"] );
                rospy.loginfo( "path         = %s", devices[ronex_param_id]["path"]);
                rospy.loginfo( "serial       = %s", devices[ronex_param_id]["serial"] ); 

Retrieve information about all loaded General I/O modules stored in a
dictionary (with ``ronex_param_id`` as its keyword). By iterating
through all values of ``ronex_param_id``, we can retrieve the
information about each General I/O module's product\_id, product\_name,
ronex\_id, path, and serial. Note that if ``ronex_id`` (its type is
string) has not been set to an alias name, its value is equal to the
value of ``serial``.

Running the code
================

First make sure that the RoNeX driver is running (see `Launch
driver <Home#launching-the-ronex-driver>`__ ).

Once this is done we can run our Python script:

::

    $ rosrun sr_ronex_examples sr_ronex_parse_parameter_server.py

You will see something similar to:

::

    [INFO] [WallTime: 1380010917.786130] *** General I/O Module 0 ***
    [INFO] [WallTime: 1380010917.786482] product_id = 33554433
    [INFO] [WallTime: 1380010917.786716] product_name = general_io
    [INFO] [WallTime: 1380010917.786938] ronex_id = 2
    [INFO] [WallTime: 1380010917.787193] path = /ronex/general_io/2
    [INFO] [WallTime: 1380010917.787444] serial = 12

