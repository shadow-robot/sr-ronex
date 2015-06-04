The easiest way to configure a RoNeX is from a launch file. This will
allow you to set io channels as inputs/outputs as well as set any other
relevant parameters.

.. code:: xml

    <launch> 
      <param name="/ronex/mapping/12345" value="my_ronex" />
      <param name="/ronex/general_io/my_ronex/pwm_clock_divider" value="1" />
      <param name="/ronex/general_io/my_ronex/input_mode_0"  value="false" />
    </launch>

