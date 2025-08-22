TCPWM: Capture 
==============
Description
^^^^^^^^^^^
- This example is used to demonstrate Capture test of PWM
   1. If short Capture port and PWM line the interrupt will be occur.
   2. Get the Compare value and calculate the value difference.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Short Port 6.1 and Port 6.3.
   Port 6.1 :BB_LIN3_TX
   Port 6.3: BB_CAN8_RXD

Expectation
^^^^^^^^^^^
- Short Port6.1 and Port6.3, then Interrupt will be occur. Then get the compare value and calculate the value difference.