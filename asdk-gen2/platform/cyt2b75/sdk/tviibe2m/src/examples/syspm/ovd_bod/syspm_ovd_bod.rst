SYSPM – OVD/BOD 
===============
Description
^^^^^^^^^^^
- This example demonstrates the setup of Over-Voltage Detection (OVD) and Brown-Out Detection (BOD) for VDDD, VDDA and VCCD.

.. warning::
   Only manipulate the voltages if you know exactly what you are doing and after thoroughly checking the schematics to prevent any damage to the board or MCU.


Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Disconnect the debugger after programming
- Serial terminal (115200, 8N1)
- Open J9 (onboard reset IC output)

Expectation
^^^^^^^^^^^
- After reset the OVD/BOD related reset cause will be printed through UART
- LED is toggling to indicate that the MCU is still running
- TP7 gives access to VCCD and J8, J10 can be opened to be able to supply VDDA and VDDD for the MCU directly/independently.
- MCU should enter reset state when OVD/BOD event is detected and restarts again when the OVD/BOD condition no longer applies