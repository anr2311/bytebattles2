SYSPM – DeepSleep – Wake-up from RTC 
====================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter DeepSleep mode and exit it through an RTC alarm interrupt

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- main_cm0plus.c and main_cm4.c need to be used because system DeepSleep mode is only entered when both cores enter sleep mode with SleepDeep bit set
- Disconnect the debugger after programming otherwise system will not enter DeepSleep mode

Expectation
^^^^^^^^^^^
- Two LEDs on the base board are toggled for some seconds (each by one of the cores)
- Then toggling stops before both cores enter DeepSleep mode
- MCU will wake-up by RTC alarm after one minute and the example starts over
