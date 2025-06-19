MCWDT Interrupt 
===============
Description
^^^^^^^^^^^
- This example demonstrates the interrupt feature of the sub-counters and 32-bit counter of MCWDT (Multi Counter Watchdog Timer) on each core.

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

Expectation
^^^^^^^^^^^
- Both the cores (CM0+ and CM4) have separate MCWDT instance. MCWDT0 counters are configured in CM0+ and MCWDT1 counters are configured in CM4 core. Whenever MCWDT’s sub-counter0 or sub-counter1 reaches the configured limit, it will generate an interrupt.
	MCWDT0_Counter 0 : Warn Limit Interrupt (1 sec) 	-> LED0 toggle
	MCWDT0_Counter 1 : Warn Limit Interrupt (2 sec) 	-> LED1 toggle
	MCWDT0_Counter 2 : Bit15 toggle Interrupt (1 sec) 	-> LED2 toggle
	MCWDT1_Counter 0 : Warn Limit Interrupt (1 sec) 	-> LED7 toggle
	MCWDT1_Counter 1 : Warn Limit Interrupt (2 sec) 	-> LED8 toggle
	MCWDT1_Counter 2 : Bit15 toggle Interrupt (1 sec)	-> LED9 toggle