Configure GIO Module (c++)
==========================

This tutorial will walk you through the basics of calling the RoNeX
/set\_parameters service in a C++ script to adjust your RoNeX
configuration. If you are often executing different RoNeX scripts that
require different module configuration, it may be beneficial adjust the
configuration at the start of each script.

This example can be found under
sr\_ronex\_examples/src/change\_ronex\_configuration.cpp # The Code

\`\`\`c++

include
========

include
========

include
========

include
========

include
========

include
========

include
========

class ChangeRonexConfigurationExample { public:

ChangeRonexConfigurationExample() { }

~ChangeRonexConfigurationExample() { }

void configureRonex(std::string path) {
dynamic\_reconfigure::ReconfigureRequest srv\_req;
dynamic\_reconfigure::ReconfigureResponse srv\_resp;
dynamic\_reconfigure::BoolParameter bool\_param;
dynamic\_reconfigure::IntParameter int\_param;
dynamic\_reconfigure::Config conf;

::

    bool_param.name = "input_mode_0";
    bool_param.value = false;
    conf.bools.push_back(bool_param);

    bool_param.name = "input_mode_1";
    bool_param.value = false;
    conf.bools.push_back(bool_param);

    int_param.name = "pwm_period_0";
    int_param.value = 200;
    conf.ints.push_back(int_param);

    int_param.name = "pwm_clock_divider";
    int_param.value = 3000;
    conf.ints.push_back(int_param);

    srv_req.config = conf;

    ros::service::call(path + "/set_parameters", srv_req, srv_resp);

}

};

int main(int argc, char \*\*argv) { ros::init(argc, argv,
"change\_ronex\_configuration\_cpp"); ChangeRonexConfigurationExample
example; std::string ronex\_id = "test\_ronex"; std::string ronex\_path
= "/ronex/general\_io/" + ronex\_id + "/";
example.configureRonex(ronex\_path); return 0; } \`\`\`

The Code Explained
==================

Now lets break down the code into manageable bite sized portions and
explain each piece:

\`\`\`c++ #include #include #include

include
========

include
========

include
========

include
========

\`\`\` First we import the standard ROS headers, plus those for the
dynamic reconfigure formats we will be using.

``c++ int main(int argc, char **argv) {   ros::init(argc, argv, "change_ronex_configuration_cpp");   ChangeRonexConfigurationExample example;   std::string ronex_id = "test_ronex";   std::string ronex_path = "/ronex/general_io/" + ronex_id + "/";   example.configureRonex(ronex_path);   return 0; }``
At the bottom of the script we have the main function where we first
initialise a node to carry out the config changes, then create an
instance of the ChangeRonexConfigurationExample class. The path to the
RoNeX module of which we want to configure is then defined, using the
"test\_ronex" alias, which has been defined using the method described
in [[this tutorial\|Using-aliases-with-your-RoNeX]]. Alternatively you
can just use the serial number of your RoNeX module here instead.

We then call the configureRonex function from the instance of the
ChangeRonexConfigurationExample class.

``c++   void configureRonex(std::string path)   {     dynamic_reconfigure::ReconfigureRequest srv_req;     dynamic_reconfigure::ReconfigureResponse srv_resp;     dynamic_reconfigure::BoolParameter bool_param;     dynamic_reconfigure::IntParameter int_param;     dynamic_reconfigure::Config conf;``
At the start of the configureRonex function we define the variables that
we will be using to construct the service call message. The request will
contain the config which in turn will contain the bool and int
parameters. The response will contain the data returned by the service.

\`\`\`c++ bool\_param.name = "input\_mode\_1"; bool\_param.value =
false; conf.bools.push\_back(bool\_param);

::

    int_param.name = "pwm_period_0";
    int_param.value = 200;
    conf.ints.push_back(int_param);

\`\`\` Next we populate the config message with the various parameters.
The section of code above shows how we set the name and value for bool
and int parameters and push them into the config variable. Here we set
configure digital I/O channel 1 as an output, and the period for PWM
module 0 to 200.

``c++     srv_req.config = conf;     ros::service::call(path + "/set_parameters", srv_req, srv_resp);``
Finally we populate the request variable with the config, and call the
set\_parameters service with the request and response variables as
arguments.

Running the code
================

Make sure that a roscore is up and running:

::

    $ roscore

Then run the driver (see `Launch
driver <Home#launching-the-ronex-driver>`__ ).

Now we can execute the example script:

::

    $ rosrun sr_ronex_examples change_ronex_configuration

Now if you echo the contents of the parameter\_descriptions topic for
this module, you should see that the configuration has been updated
accordingly.
