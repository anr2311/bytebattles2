PROT: SMPU_PC 
=============
Description
^^^^^^^^^^^
- This example uses SMPUs to verify the functionality of the PC (Protection context).
  The CM0+ tries to access the prohibited memory area, and the CM4 catches the 
  details of violation by fault reporting system.
  	1. Setting MSx_CTL (in SMPU) to allow the PC bits of the PROT_MPU register to be changed to "6"
	2. Setting the PC bits of the PROT_MPU register of the CM0+ to "6"
	3. Setting SMPU STRUCTURE2 so that only masters who have "6" as PC value can access the area (1).
	4. Setting SMPU STRUCTURE3 so that only masters who have "5" as PC value can access the area (2).
	5. Accessing the area (1) to confirm the CM0+ can access the area.
	6. Accessing the area (2) to confirm the CM0+ can't access the area and causes HardFault.
	7. The CM4 was notified of the violation details by the fault reporting system.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- None.

Expectation
^^^^^^^^^^^
- CM0+ core gets into a hard fault handler.
- CM4 blinks an LED continuously in its fault report handler.