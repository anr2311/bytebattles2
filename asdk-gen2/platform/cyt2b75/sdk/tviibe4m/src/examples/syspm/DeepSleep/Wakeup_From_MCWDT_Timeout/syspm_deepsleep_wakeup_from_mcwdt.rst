SYSPM – DeepSleep – Wake-up from MCWDT 
======================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter DeepSleep mode and exit it through an MCWDT interrupt

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
- CM4 sets up all three sub counters of an MCWDT to generate an IRQ
   - subcounter #0: every 1s
   - subcounter #1: every 2s
   - subcounter #2: every 1s (when counter bit15 toggles
- 215 / 32 kHz)
- Both cores will enter DeepSleep mode
- On wake-up CM4 outputs information through 3 LEDs on the base board
- Each LED is „assigned“ to one of the MCWDT subcounters
- When the corresponding subcounter raised the interrupt the LED will be toggled
- Then DeepSleep mode is entered again