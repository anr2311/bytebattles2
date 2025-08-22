TCPWM: PWM 
==========
Description
^^^^^^^^^^^
- This example is used to demonstrate PWM output.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Check each port on Base Board.
   Port 6.1: BB_LIN3_TX
   Port 6.3: BB_CAN8_RXD
   Port 6.5: BB_LIN4_SLP
   Port 2.1: BB_LIN3_TX
   Port 5.0: BB_CAN7_WAKE
   Port 5.1: BB_CAN8_WAKE
   Port 7.1: BB_CAN_SPI1_MISO
   Port 0.1 :BB_LIN1_TX
   Port 0.0: BB_LIN1_RX
   Port 8.0: BB_CAN0_TXD

Expectation
^^^^^^^^^^^
- Check each PWM port, it will output 3.814 Hz pulse with duty 50%.