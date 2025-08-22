EVTGEN: CM4 DeepSleep Wakeup 
=============================
Description
^^^^^^^^^^^
- This example demonstrates the wakeup of cm4 from DeepSleep by Event Generator.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core

Expectation
^^^^^^^^^^^
- Run cm4, cm4 goes to DeepSleep, then woken up by EVTGEN
   - CPU board LED blinks in interval of 2 s
      - Blinks: wakeup/ Active
      - Turns off: transitioning to DeepSleep
- Run cm0plus, cm0plus keeps being in DeepSleep
