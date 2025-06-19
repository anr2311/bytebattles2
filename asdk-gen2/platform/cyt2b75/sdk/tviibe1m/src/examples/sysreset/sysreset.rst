SYSRESET 
========
Description
^^^^^^^^^^^
- This example demonstrates various reset types:
   1. Watchdog (WDT) reset
   2. Fault Structure reset
   3. Reset triggered by software
   4. Multi-Counter Watchdog (MCWDT) reset

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- All J102 jumpers (base board) are closed to ensure that buttons are connected
- Disconnect debugger after programming and power-cycle the board

Expectation
^^^^^^^^^^^
- Last reset cause is indicated through LEDs on the base board
- Various reset types can be provoked by pressing buttons on the base board