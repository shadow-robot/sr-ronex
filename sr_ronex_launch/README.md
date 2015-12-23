sr_ronex_launch
========

This package contains the main launch files for the sr_ronex.

**sr_ronex_spi_sensor.launch** launch file for the rotary sensor connected to the spi module.
The 1 kHz controller reads the sensor through SPI ronex module and publishes  */ronex/spi/ronex_id/sensor_message* which is a *std_msgs/Float64* message.
