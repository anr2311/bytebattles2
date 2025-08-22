SCB SPI: Low Level Driver Master 
================================
Description
^^^^^^^^^^^
- This example demonstrates SCB-SPI in Master Mode (Low level), MCU is set as master and PC/Aardvark set in slave Mode.

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
- Aardvark to be configured as a slave
- Aardvark MOSI/8 - JP6/14 on BB
- Aardvark MISO/5 - JP6/15 on BB
- Aardvark SCLK/7 - JP6/9 on BB
- Aardvark SS/9 - JP6/8 on BB
- Aardvark GND/1 to any GND on the CPU/BB board

Expectation
^^^^^^^^^^^
- MCU is set in Master Mode, Aardvark shall be set in slave mode and click start.
- MCU prepares data and sends it to the PC. The screen of Aardvark will show the data from MCU.