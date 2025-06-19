MCWDT Reset 
===========
Description
^^^^^^^^^^^
- This example demonstrates the CPU reset feature of the sub-counters and 32-bit counter of MCWDT (Multi Counter Watchdog Timer) on each core.

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
- Debugger should not be connected while testing this feature.
- Comment the infinite loop section to generate reset from MCWDT.

Expectation
^^^^^^^^^^^
- On CM0+ core counter1 of MCWDT0 will reset the MCU after 2 seconds. If MCWDT0 reset is detected, CPU is will to blink the LED5 on CPU board in next boot.
- On CM4 core counter0 of MCWDT1 will reset the MCU after 1 seconds. If MCWDT1 reset is detected, CPU is will to blink the LED8 on Base board in next boot.
	MCWDT0_Counter 1 : Warn Limit Interrupt (2 sec) 	-> LED5 blink
	MCWDT1_Counter 0 : Warn Limit Interrupt (1 sec) 	-> LED8 blink