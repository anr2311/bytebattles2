SYSCLK: Set and measure 
=======================
Description
^^^^^^^^^^^
- In this example, program sets all internal clock.
- Using clock measurement counters, it measures some of internal clocks. 
  The source clock of measuring counter is ECO. (It outputs ECO clock from
  EXT_CLK port via HF1. Therefore, if the source clock of HF1 is ECO, user 
  can check the ECO frequency to verify the result. EXT_CLK port is used also 
  as TDI, this means it is assumed the debugger is using SWD).
- It reads register value and recalculates the frequency.
- Measured and calculated frequencies are checked as they are close to expected value.

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
- Program counter should reach to infinite loop at the bottom of the main function.
- Output frequency from “SWEOE_TDI” is 16MHz (same as ECO).