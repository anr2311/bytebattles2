MCWDT Fault 
===========
Description
^^^^^^^^^^^
- This example demonstrates the fault generation feature on occurrence of the MCWDT (Multi Counter Watchdog Timer) counters interrupt on each core.

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
- Both the cores (CM0+ and CM4) have separate MCWDT instance. MCWDT0 counters are configured for CM0+ and MCWDT1 counters are configured for CM4 core. Whenever MCWDT’s sub-counter0 or sub-counter1 reaches the configured upper limit, it will generate fault interrupt.
	MCWDT0_Counter 0 : Upper Limit Interrupt (1 sec) -> LED0 toggle
	MCWDT0_Counter 1 : Upper Limit Interrupt (2 sec) -> LED1 toggle
	MCWDT1_Counter 0 : Upper Limit Interrupt (1 sec) -> LED8 toggle
	MCWDT1_Counter 1 : Upper Limit Interrupt (2 sec) -> LED9 toggle