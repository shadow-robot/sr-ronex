sr_ronex_launch
========

This package contains the main launch files for the sr_ronex.

**sr_ronex_spi_sensor.launch** launch file for the rotary sensor connected to the spi module.
SPI parameters should be set in **spi_sensor_read_controller.yaml** file. 
The parameters are controller name, type, SPI channel and ronex_id.
The 1 kHz controller reads the sensor through SPI ronex module and publishes  */[controller_name]/sensor_message_[ronex_id]_[spi_channel]* which is a *std_msgs/Float64* message.
