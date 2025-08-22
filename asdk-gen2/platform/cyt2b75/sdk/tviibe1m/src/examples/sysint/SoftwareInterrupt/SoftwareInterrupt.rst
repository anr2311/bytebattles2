SYSINT: Software Interrupt 
==========================
Description
^^^^^^^^^^^
- This example demonstrates software interrupt
   1. Use software interrupt 0 handler
   2. Use software interrupt 7 handler

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
Set break points inside the ISRs
- If software interrupt 0 handler occurred, then SoftwareInterupt0Handler will be hit.
- If software interrupt 7 handler occurred, then SoftwareInterupt7Handler will be hit.
