MCWDT DeepSleep Wakeup 
======================
Description
^^^^^^^^^^^
- This example demonstrates the deep sleep wakeup feature based on the MCWDT (Multi Counter Watchdog Timer) counters interrupt.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Jumper J80 to be short on position 1-2 on Base Board.
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.
- Both the cores (CM0+ and CM4) should be in DeepSleep to validate this feature.
- Debugger should not be connected while testing DeepSleep mode feature.

Expectation
^^^^^^^^^^^
- Initially both the cores are in DeepSleep mode and MCWDT counters are configured for CM4 core only. Whenever MCWDT counter reaches the configured limit, it will generate interrupt and wake up the CM4 core. After wakeup it will indicate counter status on LED and again go to DeepSleep mode.
	Counter 0 : Warn Interrupt (1 sec)	-> LED7 turn on
	Counter 1 : Warn Interrupt (2 sec) 	-> LED8 turn on
	Counter 2 : Bit15 toggle Interrupt 	-> LED9 turn on