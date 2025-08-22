SCB EZI2C: Hardware mode 
========================
Description
^^^^^^^^^^^
- This example demonstrates SCB-EZI2C in HW Slave Mode, with PC/Aardvark in master Mode.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- TVII Base Board (BB)

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core
- Slave address of the MCU is 0x08
- Aardvark SCL/0  - P1_0_SCB0_I2C_SCL (JP8/4 on BB)
- Aardvark SDA/2 - P1_1_SCB0_I2C_SDA (JP8/3 on BB)
- Aardvark GND/1 to any GND on the CPU/BB

Expectation
^^^^^^^^^^^
- MCU is set in Slave Mode always in EZI2C, Aardvark shall be set in Master mode and click start.
- PC/Aardvark sends data to MCU. Aardvark will show the sent data and received data.
- Only SCB0 is supported and no interrupt handling.