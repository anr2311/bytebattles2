SYSPM – Sleep – Wake-up from UART 
=================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter Sleep mode and exit it through an UART interrupt

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Serial terminal (115200, 8N1)

Expectation
^^^^^^^^^^^
- CM4 configures the UART and enters Sleep mode
- User will then need to send characters via the serial terminal, the CPU will wake-up from UART RX interrupt and echo the received character through UART and go back to sleep
- Additionally, one LED on the base board will be toggled on every wake-up