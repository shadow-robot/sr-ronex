GIO - Read analogue inputs (command line)
=========================================

Viewing GIO analogue input data on the command line is incredibly
simple. The rostopic echo command subscribes to the topic in question
then displays an data received in the terminal window:

::

    $ rostopic echo /ronex/general_io/12/state

This will display the status of the whole GIO module, if you'd prefer to
display a single analogue channel's data instead (channel 0 in the
example below) you can use the following command:

::

    $ rostopic echo /ronex/general_io/12/state/analogue[0]
