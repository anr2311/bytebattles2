SYSPM – DeepSleep – Wake-up from GPIO 
=====================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter DeepSleep mode and exit it through an GPIO interrupt

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
- CM0+ turns on the LED on the CPU board and CM4 turns on one LED on the base board, then they wait for the button on the CPU board to be pressed
- Once the button has been pressed, both LEDs will be turned off and both cores enter DeepSleep mode
- Press the button again to wake-up the MCU and the example starts over
