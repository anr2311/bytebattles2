IPC Using Low Level API 
=======================
Description
^^^^^^^^^^^
- This example demonstrates
1. IPC low level api communication between the two cores (CM0+ and CM4).
2. Transfer of a word message from IPC Client (CM4 core) to IPC Server (CM0+ core).

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Jumper J80 to be short on position 1-2 on Base Board
- CYTVII-B-E-1M-176-CPU board should be connected on CYTVII-B-E-BB board.

Expectation
^^^^^^^^^^^
- CM0+ core toggle LED0 (message id 0) or LED1 (message id 1) or LED2 (message id 3) on the Base Board as per the received message from CM4 core.
	On CM4 core
		Wait until CM0+ core release lock
		Send LED id to CM0+ core using IPC channel
	On CM0+ core
		Get notify interrupt and toggle LED based on the message
		Release the IPC lock for CM4 core