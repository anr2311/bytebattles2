SCB SPI: Onboard SPI EEPROM 
===========================
Description
^^^^^^^^^^^
- This example demonstrates SCB-SPI in Master Mode to read and write on board SPI EEPROM.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- TVII Base Board (BB)

Dependency
^^^^^^^^^^
- Baseboard connected to the CPU board
- Hook jumpers J47/48/49/50

Expectation
^^^^^^^^^^^
- First part of the example writes byte by byte (255 bytes) to the SPI EEP with values 0 to 255, reads back. Observe the read buffer after the completion “readData”.
- Second part write in pages of size 32 bytes. Observe the read buffer after the completion “readData2”.