SYSCLK: Different CPU clock 
===========================
Description
^^^^^^^^^^^
- In this example, user can try two combination of core frequencies – 
  (CM0+, CM4) = (80MHz, 160MHz) or (100MHz, 100MHz).
- Also the clock source is selectable from PLL or FLL.
- These option can be selected by modifying value of “TEST_CASE_CPU_FREQ” and “USE_PLL_OR_FLL”.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev. A Board

Dependency
^^^^^^^^^^
- Jumper J80 to be short on position 1-2 on Base Board.
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.

Expectation
^^^^^^^^^^^
- If TEST_CASE_CPU_FREQ == CM4_100MHz_CM0_100MHz,
    CM0+ blinks LED0 at the interval of 100KHz.
    CM4 blinks LED8 at the interval of 100KHz.
- If TEST_CASE_CPU_FREQ == CM4_160MHz_CM0_80MHz,
    CM0+ blinks LED0 at the interval of 80KHz.
    CM4 blinks LED8 at the interval of 160KHz.