SYSWDT: Reset 
=============
Description
^^^^^^^^^^^
- This example demonstrates the WDT reset trigger functionality. This example works only when debugger is NOT connected to the MCU.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core.

Expectation
^^^^^^^^^^^
- At first this program turns a LED off.
- After 2 seconds, WDT reset will occur. At the top of main code, if WDT reset is detected, CPU is going to blink an LED.
- Duration of LED off should be 2 seconds.