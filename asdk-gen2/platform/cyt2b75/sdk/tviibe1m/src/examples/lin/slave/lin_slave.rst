LIN Slave 
=========
Description
^^^^^^^^^^^
- This example demonstrates LIN slave configuration that will communicate with the LIN master device.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Jumper J80 to be short on position 1-2 on Base Board.
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.
- Jumper J59, J60, J63 and J73 should be closed on base board.
- PLIN pro or CANoe should be installed on the system.

Expectation
^^^^^^^^^^^
- Connect LIN0 connector (P5 upper one) on the base board and CANoe or other analyzer if available.
- Import “LinConfig_ForSlaveTest.cfg” and monitor using trace, statistics window. 
- Slave will respond to the “RX_HEADER”, “TX_RESPONSE” and “RX_RESPONSE” from the master device.