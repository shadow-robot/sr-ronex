sr_ronex_launch
========

This package contains the main launch files for the sr_ronex.

**sr_ronex_spi_sensor.launch** launch file for the rotary sensor connected to the spi module.
The 1 kHz controller reads the sensor through SPI ronex module and publishes  */ronex_[ronex_id]_sensor_controller/sensor_message* which is a *std_msgs/Float64* message.
The SPI channel can be set with setting */ronex_[ronex_id]_sensor_controller/SPI_sensor_channel*. Default value is 1.
