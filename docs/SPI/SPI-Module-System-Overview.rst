SPI Module Overview
===================

.. toctree::
   :maxdepth: 2
   :glob:
   :hidden:


This page describes some of the key parameters, services and topics
you'll need to interact with when using the RoNeX SPI module. The
information here builds on that related to the general RoNeX system
which can be found :doc:`here </General/RoNeX-General-System-Overview>`.

Data Flow
---------

A priority when developing the RoNeX drivers is to ensure the ease of use of our product. For the SPI module, we developed a very generic driver that packs and unpacks the data into (and from) the proper etherCAT format so that it's easily accessible in the rest of the code.

By default a passthrough controller is loaded. This makes it possible to easily send / receive data to / from the SPI using a ROS service. Since it's a ROS service, you can access it from any of the usual languages in ROS: c++, python, command line.

Using that service is very convenient for a variety of use cases, but it will not be enough if you need to control exactly what's being sent to the SPI at each tick. For this you'll need to develop your own controller as explained in the :doc:`Tutorials <SPI-Tutorials>`

Services
--------

Any of these services can be called directly from the command line using `rosservice call <http://wiki.ros.org/rosservice>`__, or from a program as described in the :doc:`Tutorials <SPI-Tutorials>`.

- ``/ronex/spi/12/command/passthrough/[0..4]``

This service can be used to send and receive data from the SPI module.

Dynamic Reconfigure
-------------------

Different configurations are available through the `dynamic reconfigure <http://wiki.ros.org/dynamic_reconfigure>`__ interface for the SPI module.

Generic module configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``command_type``: Type of the command to be sent (normal / config). This is set for the spi module as a whole.
- ``pin_output_state_pre_DIO_0``: The pre state pin for DIO 0
- ``pin_output_state_post_DIO_0``: The post state pin for DIO 0

- ``pin_output_state_pre_DIO_1``: The pre state pin for DIO 1
- ``pin_output_state_post_DIO_1``: The post state pin for DIO 1

- ``pin_output_state_pre_DIO_2``: The pre state pin for DIO 2
- ``pin_output_state_post_DIO_2``: The post state pin for DIO 2

- ``pin_output_state_pre_DIO_3``: The pre state pin for DIO 3
- ``pin_output_state_post_DIO_3``: The post state pin for DIO 3

- ``pin_output_state_pre_DIO_4``: The pre state pin for DIO 4
- ``pin_output_state_post_DIO_4``: The post state pin for DIO 4

- ``pin_output_state_pre_DIO_5``: The pre state pin for DIO 5
- ``pin_output_state_post_DIO_5``: The post state pin for DIO 5

- ``pin_output_state_pre_CS_0``: The pre state pin for spi CS 0
- ``pin_output_state_post_CS_0``: The post state pin for spi CS 0

- ``pin_output_state_pre_CS_1``: The pre state pin for spi CS 1
- ``pin_output_state_post_CS_1``: The post state pin for spi CS 1

- ``pin_output_state_pre_CS_2``: The pre state pin for spi CS 2
- ``pin_output_state_post_CS_2``: The post state pin for spi CS 2

- ``pin_output_state_pre_CS_3``: The pre state pin for spi CS 3
- ``pin_output_state_post_CS_3``: The post state pin for spi CS 3


Per SPI line configuration
~~~~~~~~~~~~~~~~~~~~~~~~~~

The ``0`` in the following configurations can be replaced by 1,2 or 3 since there are 4 SPI lines per module.

- ``spi_mode_0``: Select SPI Mode for SPI 0 - specify any mode of the spi_mode_enum.
- ``spi_0_input_trigger``: The input trigger config - SPI[0]
- ``spi_0_mosi_somi``: The MOSI SOMI pin config - SPI[0]
- ``spi_0_inter_byte_gap``: The Inter Byte Gap - SPI[0]
- ``spi_0_clock_divider``: The Clock Divider - SPI[0]


Values used for some of the parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- ``spi_mode_enum``: SPI modes.

+-------+---------------------------------------------+
| Mode  | doc                                         |
+=======+=============================================+
| NL_RE | Clock normally low, sample on rising edge   |
+-------+---------------------------------------------+
| NL_FE | Clock normally low, sample on falling edge  |
+-------+---------------------------------------------+
| NH_FE | Clock normally high, sample on falling edge |
+-------+---------------------------------------------+
| NH_RE | Clock normally high, sample on rising edge  |
+-------+---------------------------------------------+

- ``spi_input_trigger_enum``: Input trigger configurations.

+-------------------------------+------------------------+
|        Mode                   | doc                    |
+===============================+========================+
| SPI_CONFIG_INPUT_TRIGGER_NONE | SPI input trigger NONE |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D0   | SPI input trigger D0   |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D1   | SPI input trigger D1   |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D2   | SPI input trigger D2   |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D3   | SPI input trigger D3   |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D4   | SPI input trigger D4   |
+-------------------------------+------------------------+
| SPI_CONFIG_INPUT_TRIGGER_D5   | SPI input trigger D5   |
+-------------------------------+------------------------+

- ``spi_mosi_somi_enum``: MOSI SOMI pin state.

+------------------------------------+-----------------------------+
|             Mode                   | doc                         |
+====================================+=============================+
| SPI_CONFIG_MOSI_SOMI_DIFFERENT_PIN | SPI MOSI SOMI different pin |
+------------------------------------+-----------------------------+
| SPI_CONFIG_MOSI_SOMI_SAME_PIN      | SPI MOSI SOMI same pin      |
+------------------------------------+-----------------------------+
