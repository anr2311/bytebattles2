CANFD - TopFifo 
===============
Description
^^^^^^^^^^^
- This example demonstrates transmission and reception on CANFD using top FIFO.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.
- Jumper J70 (RX line), J72 (TX line), J71 (if wakeup needed) and J69 (terminal resistor) should be closed on base board.
- CANoe should be installed on the system.

Expectation
^^^^^^^^^^^
- Connect CAN0 connector (P6 upper one) on the base board and CANoe or other analyzer if available.
- If you have CANoe, launch CANFD_CFG.cfg in UseAsCANFDMode folder. It is configured simple test for this example.
   - Make sure the Type in CAN IG tab is CAN FD (for standard ID) and Extended CAN FD (for extended ID)
   - Just run and monitor using trace, statistics window.
- CANFD mode (500kHz for nominal bitrate, 1MHz for fast bitrate. Sampling point of both are 75%)
   - 4 message is sent from periodically. (100ms)
   - 2 standard IDs and 2 extended IDs. 
      - All ID filters are configured as the clasical filter only following IDs are accepted by the MCU:
        STD : 0x010, 0x020
        EXT : 0x10010, 0x10020
   - At the receive call back CAN_RxFifoWithTopCallback in the main.c, received message is ID 300.