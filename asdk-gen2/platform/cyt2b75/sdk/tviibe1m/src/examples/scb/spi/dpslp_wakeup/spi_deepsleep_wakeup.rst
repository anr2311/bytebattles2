SCB SPI: DeepSleep Wakeup 
=========================
Description
^^^^^^^^^^^
- This example demonstrates usage of SCB-SPI to wake up the MCU.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- TVII Base Board (BB)

Dependency
^^^^^^^^^^
- Both the examples to be copied to the main “src”
- Aardvark to be configured as a Master
- Aardvark MOSI/8 - JP6/14 on BB
- Aardvark MISO/5 - JP6/15 on BB
- Aardvark SCLK/7 - JP6/9 on BB
- Aardvark SS/9 - JP6/8 on BB
- Aardvark GND/1 to any GND on the CPU/BB board

Expectation
^^^^^^^^^^^
- MCU is set in Slave Mode, Aardvark shall be set in Master mode and click start.
- Both cores will be in DeepSleep mode, send a message from PC/Aardvark to wakeup the MCU.