EVTGEN: CM4 DeepSleep Wakeup and Trigger ADC
=============================================
Description
^^^^^^^^^^^
- This example demonstrates the wakeup of cm4 from DeepSleep by AD conversion which is triggered by Event Generator.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Both example could be copied to the main “src”.

Expectation
^^^^^^^^^^^
- Run cm0plus, cm0plus keeps being in DeepSleep
- Run cm4, cm4 goes to DeepSleep, then woken up by ADC triggered by EVTGEN
   - CPU board LED blinks each cycle