Configuration parameters of a ronex module can also be set from a
user-defined launchfile, by using the dynparam set option.

This is probably the most common way to set up the right parameters for
a user environment.

The following example shows how to set the mode to "output" for the I/O
pin number 2.

.. code-block:: xml

    <launch>

    ...

    <node pkg="dynamic_reconfigure" type="dynparam" name="dynparam_i2"
           args="set /ronex/general_io/0 input_mode_2 false" />

    ...

    </launch>
