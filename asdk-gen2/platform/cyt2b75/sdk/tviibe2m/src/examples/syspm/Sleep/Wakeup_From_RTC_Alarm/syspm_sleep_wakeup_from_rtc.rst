SYSPM – Sleep – Wake-up from RTC 
================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter Sleep mode and exit it through an RTC alarm interrupt

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
- CM4 configures the RTC and an alarm and then enters Sleep mode
- Every 2 seconds the CPU will wake up
- It toggles one LED on the base board in the RTC alarm handler
- It toggles another LED on the base board for half a second
- Then sleep mode is entered again