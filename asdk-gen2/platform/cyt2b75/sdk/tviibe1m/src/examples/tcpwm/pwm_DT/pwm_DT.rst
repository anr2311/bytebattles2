TCPWM: PWM_DT 
=============
Description
^^^^^^^^^^^
- This example is used to demonstrate PWM dead Time Test. PWM Port outputs the 200Hz with right side dead time 250 [us] and left side dead time 500 [us].

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Check the following port output:
   Port 6.0: BB_LIN3_RX
   Port 6.1: BB_LIN3_TX

Expectation
^^^^^^^^^^^
- Check the port 6.0 and Port 6.1: It will outputs 200Hz with right side dead time 250 [us] and left side dead time 500 [us].