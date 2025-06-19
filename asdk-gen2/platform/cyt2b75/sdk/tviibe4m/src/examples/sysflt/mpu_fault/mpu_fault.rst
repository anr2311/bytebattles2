SYSFLT : MPU fault 
==================
Description
^^^^^^^^^^^
Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Both the examples need to be copied to the “src”.

Expectation
^^^^^^^^^^^
- cm0plus example generates MPU fault and cm4 captures the fault
- Put a breakpoint inside the fault ISR of cm4 example
- Run both the cores
- Observe for the fault data to understand which SRAM address caused fault to trigger