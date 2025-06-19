TCPWM: SR 
=========
Description
^^^^^^^^^^^
- This example is used to demonstrate SR Test for TCPWM.
   1. Short TCPWM_IN and GPIO_OUT on the base board and see the signal of GPIO_OUT and TCPWM_OUT.
   2. Signal of TCPWM_OUT delays about 8 [us] from GPIO_OUTs.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Short TCPWM_IN and GPIO_OUT on the base board
   TCPWM_OUT - Port 6.1: BB_LIN3_TX
   TCPWM_IN - Port 6.2: BB_CAN8_TXD
   GPIO_OUT - Port 6.3: BB_CAN8_RXD

Expectation
^^^^^^^^^^^
- Signal of TCPWM_OUT delays about 8 [us] from GPIO_OUTs.