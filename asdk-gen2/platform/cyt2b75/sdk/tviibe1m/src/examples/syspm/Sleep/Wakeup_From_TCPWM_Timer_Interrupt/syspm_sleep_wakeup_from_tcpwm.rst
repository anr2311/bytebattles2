SYSPM – Sleep – Wake-up from TCPWM 
==================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter Sleep mode and exit it through a TCPWM interrupt

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- None

Expectation
^^^^^^^^^^^
- CM4 configures the TCPWM and then enters Sleep mode
- Every 1 second the CPU will wake up
- It toggles one LED on the base board in the TCPWM IRQ handler
- It toggles another LED on the base board for half a second
- Then sleep mode is entered again