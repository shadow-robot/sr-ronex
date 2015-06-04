This tutorial explains the calculations for converting an analogue value
from a potentiometer into a PWM value corresponding to the same angle
for a servo motor, as described in this tutorial and video: `Youtube
Interfacing Code <Youtube-Interfacing-Code>`__

From the range of rotation of our potentiometer and resolution of the
RoNeX ADC we know:

::

    Potentiometer rotational range = 0 - 300°
    12 bit Analogue input channel range = 0 - 4095
    Analogue value at 180° = 4096 / 300 * 180 = 2457.6

From the Servo Datasheet we know:

::

    Servo works on a 20ms control period.
    On time of 0.5ms corresponds to an angle of 0°.
    0.5ms on time in 20ms period corresponds to 2.5% duty cycle.
    On time of 2.5ms corresponds to an of 180°
    2.5ms on time in 20ms period corresponds to 12.5% duty cycle.

From the RoNeX Manual we know:

::

    RoNeX base clock frequency  = 64MHz.
    Master clock divider = 20.
    Resultant master clock frequency = 64MHz/20 = 3.2 MHz.
    PWM Period = 64000
    Output frequency = 3.2MHz/64000 = 50 Hz
    50Hz corresponds to period of 20ms.

From this we can deduce that:

::

    In a 64000 period, 2.5% duty cycle = 64000 * 0.025 = 1600.
    In a 64000 period, 12.5% duty cycle = 64000 * 0.125 = 8000.

For ease of calculation we offset the pwm\_on\_time by -1600 to give a
range of 0-6400. This gives rise to the following conversion constant:

::

    PWM range / Analogue range  = 6400 / 2457.6 = 2.6042

Which is then apparent in the Python code:

::

    pwm.pwm_on_time_0 = int(2.6042*analogue[0] + 1600)

