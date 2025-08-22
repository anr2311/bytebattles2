SCB I2C: Master Mode 
====================
Description
^^^^^^^^^^^
- This example demonstrates SCB-I2C in Master Mode, MCU is set as master and PC/Aardvark in slave Mode.

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
- Aardvark to be configured for a slave address “0x08”
- Aardvark SCL/0  - P1_0_SCB0_I2C_SCL (JP8/4 on BB)
- Aardvark SDA/2 - P1_1_SCB0_I2C_SDA (JP8/3 on BB)
- Aardvark GND/1 to any GND on the CPU/BB

Expectation
^^^^^^^^^^^
- MCU is set in Master Mode, Aardvark shall be set in slave mode and click start.
- MCU prepares data and sends it to the PC. The screen of Aardvark will show the data from MCU.