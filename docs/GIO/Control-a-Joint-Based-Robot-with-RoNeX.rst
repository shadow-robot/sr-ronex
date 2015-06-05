If you're working with a joint based robot, thanks to RoNeX, you can
control your whole robot with just a few lines of xml (simply adding
custom transmissions to the URDF robot model).

You can find this example in the **sr\_ronex\_examples** package. The
files used are: - *model/ronex.urdf*: a description of the robot -
*launch/sr\_ronex\_urdf.launch*: a launch file to start the drivers, the
controllers, and a few utils. -
*config/joint\_position\_controller.yaml*: the parameter settings.

Set an alias for your RoNeX module
----------------------------------

To be able to run this example, you'll need to setup an alias for your
RoNeX module. To do this, simply follow [[the
instruction\|Using-aliases-with-your-RoNeX]].

Creating your URDF
------------------

There are `extensive Tutorials <http://wiki.ros.org/urdf/Tutorials>`__
on the ROS wiki, covering how to write a URDF. We'll assume you're
familiar with URDF for this tutorial.

Let's use this very simple robot model as a base, with one continuous
joint.

.. code:: xml

    <?xml version="1.0"?>
    <robot name="flexible">
      <!-- standard urdf -->
      <link name="base_link">
        <visual>
          <geometry>
            <cylinder length="0.6" radius="0.2"/>
          </geometry>
          <material name="blue">
            <color rgba="0 0 .8 1"/>
          </material>
        </visual>
      </link>

      <link name="head">
        <visual>
          <geometry>
            <sphere radius="0.2"/>
          </geometry>
          <material name="white"/>
        </visual>
      </link>

      <joint name="head_swivel" type="continuous">
        <parent link="base_link"/>
        <child link="head"/>
        <axis xyz="0 0 1"/>
        <origin xyz="0 0 0.3"/>
      </joint>
    </robot>

Let's say that a **position sensor** is plugged into the **analogue pin
6** of your General I/O board. We want to map this position sensor to
our robot's joint position. To do this, we use a **RonexTransmission**.
Add the following after the ``</joint>`` in the previous URDF:

.. code:: xml

      <transmission type="sr_ronex_transmissions/RonexTransmission" name="my_ronex_transmission">
        <joint name="head_swivel"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="position" analogue_pin="6" scale="0.001" offset="0.0"/>
      </transmission>

The first line loads the transmission and gives it the name
*my\_ronex\_transmission*. As you can see, this transmission is acting
on the joint *head\_swivel* (which has been defined previously in the
URDF). The **mapping** line is then mapping one given RoNeX module
(using it's RoNeX id), to the *position* property of our joint. We're
specifying which analogue pin to use (6 in this example), and also
scaling the input to a better range by using a scale and offset.

Let's now assume that you've also got a **servo**. It is plugged into
the **PWM module 1 - pin 0** using **digital pin 0** as the direction
pin. We now want to control the joint in position. We can refine the
previous transmission to add this servo.

.. code:: xml

      <transmission type="sr_ronex_transmissions/RonexTransmission" name="my_ronex_transmission">
        <joint name="head_swivel"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="position" analogue_pin="6" scale="0.001" offset="0.0"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="command" pwm_module="1" pwm_pin="0" direction_pin="0"/>
      </transmission>

As you can see we're adding a *command* mapping line to map the
*command\_* of the joint to the servo plugged into our PWM module.

If you have an effort sensor (on analogue pin 7 for example), you could
also map it in the transmission:

.. code:: xml

      <transmission type="sr_ronex_transmissions/RonexTransmission" name="my_ronex_transmission">
        <joint name="head_swivel"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="position" analogue_pin="6" scale="0.001" offset="0.0"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="effort"   analogue_pin="7" scale="1.0" offset="0.0"/>
        <mapping ronex="/ronex/general_io/test_ronex" property="command" pwm_module="1" pwm_pin="0" direction_pin="0"/>
      </transmission>

As you can see each mapping line is optional so you can adjust the
transmission to your available hardware.

Writing the launch file
-----------------------

Now that you have a model of your robot which maps the different sensors
and motors you have to your RoNeX's inputs, we want to load the robot
model, start the driver, and publish the current joint states (position,
effort, velocity) of our bot. To do this you can use this simple launch
file.

.. code:: xml

    <launch>
      <!-- Load the robot description -->
      <param name="robot_description" command="$(find xacro)/xacro.py '$(find sr_ronex_examples)/model/ronex.urdf'" />

      <!-- Allows to specify the ethernet interface to be used. It defaults to the value of the env var ETHERCAT_PORT -->
      <arg name="ethercat_port" default="$(optenv ETHERCAT_PORT eth0)" />

      <!-- Start the ronex driver -->
      <node name="ronex" pkg="pr2_ethercat" type="pr2_ethercat" args="-i $(arg ethercat_port) -r /robot_description" output="screen"  launch-prefix="nice -n -20"/>

      <!-- publishes the joint states -->
      <include file="$(find ros_ethercat_model)/launch/joint_state_publisher.launch"/>
    </launch>

You can view your robot using `rviz <http://wiki.ros.org/rviz>`__. If
you add a *Robot Model* plugin and use */base\_link* as the fixed frame
you should see a D2R2 like robot. You can swivel the head around using
your analogue "position" sensor.

Adding Controllers
------------------

Now that we can both read the position and control a servo, let's start
some joint position controllers. To do this, we first need to make the
joint *head\_swivel* controllable. We'll use the RoNeX fake calibration
controllers for that.

Setting up the different controller settings
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If you create a *joint\_position\_controller.yaml* file you can define
these simple parameters.

.. code:: yaml

    head_swivel_fake_calib:
      type: sr_ronex_controllers/FakeCalibrationController
      joint: head_swivel

Now that it is possible to use our joints in the standard ROS
controller, we can setup a PID joint position controller on our head
swivel topic. Let's add the parameters to our yaml controller parameter
file. We can use the ROS standard
`robot\_mechanism\_controllers/JointPositionController <http://wiki.ros.org/robot_mechanism_controllers>`__.

.. code:: yaml

    head_swivel_controller:
      type: robot_mechanism_controllers/JointPositionController
      joint: head_swivel
      pid: &head_swivel_gains
        p: 1000.0
        d: 0.0
        i: 0.0
        i_clamp: 0.0

Loading the settings and spawning the controllers
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The different settings for our controllers are ready to be loaded. We
can go back to editing the launch file to load them and then we'll spawn
the fake calibration controllers (making the joints controllable) and
the position controller.

.. code:: xml

    <!-- Loads the controller parameter -->
      <rosparam command="load" file="$(find sr_ronex_examples)/config/joint_position_controller.yaml" />

      <!-- spawn fake calibration controller: the pr2 controllers need the joints
           to be set to calibrated = true to work -->
      <node name="fake_calib_controllers_spawner"
            pkg="pr2_controller_manager" type="spawner" output="screen"
            args="head_swivel_fake_calib" />

      <!-- spawning traditional joint controllers -->
      <node name="joint_controllers_spawner"
            pkg="pr2_controller_manager" type="spawner" output="screen"
            args="--wait-for=calibrated head_swivel_controller" />

      <!-- publishes tf from joint states to be able to view in rviz -->
      <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>

Making sure our pins are set to output mode
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The last thing we need to do is to change the mode of the pins we use
for the servo to **output**. By default all the digital pins are set to
input. To do this we access the `dynamic reconfigure
interface <http://wiki.ros.org/dynamic_reconfigure>`__ from our launch
file.

.. code:: xml

      <!-- setting the corresponding pins to output mode on the RoNeX -->
      <node pkg="dynamic_reconfigure" type="dynparam" name="dynparam_i2"
        args="set /ronex/general_io/test_ronex input_mode_2 false" />
      <node pkg="dynamic_reconfigure" type="dynparam" name="dynparam_i5"
        args="set /ronex/general_io/test_ronex input_mode_5 false" />

