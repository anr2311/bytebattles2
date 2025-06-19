SCB SPI: High Level Driver Slave 
================================
Description
^^^^^^^^^^^
- This example demonstrates SCB-SPI in Slave Mode (High level), MCU is set as slave and PC/Aardvark set in master Mode.

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
- Aardvark to be configured as a master
- Aardvark MOSI/8 - JP6/14 on BB
- Aardvark MISO/5 - JP6/15 on BB
- Aardvark SCLK/7 - JP6/9 on BB
- Aardvark SS/9 - JP6/8 on BB
- Aardvark GND/1 to any GND on the CPU/BB board

Expectation
^^^^^^^^^^^
- MCU is set in Slave Mode, Aardvark shall be set in master mode and click start.
- PC/Aardvark sends data to MCU and receive data back from the MCU. Aardvark will show the sent and the received data.