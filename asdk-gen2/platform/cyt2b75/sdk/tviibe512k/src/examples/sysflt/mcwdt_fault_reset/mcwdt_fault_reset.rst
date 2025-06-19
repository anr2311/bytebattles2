SYSFLT: MCWDT fault and reset generation 
========================================
Description
^^^^^^^^^^^
- This example demonstrates the generation of faults using MCWDT and subsequent reset.

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
- Run the program
- Put a break point in the fault ISR and observe for the status and see whether which sub-counter and lower/upper limit caused the fault
- By default check for the reset too, if not interested in checking resets enable the last part of the code inside the infinite while loop