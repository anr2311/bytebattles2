IPC Using Pipe Interface 
========================
Description
^^^^^^^^^^^
- This example demonstrates
1. IPC using pipe interface between the two cores (CM0+ and CM4).
2. Transfer of a word message from IPC Server (CM0+ core) to IPC Client (CM4 core).

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
- CM4 core toggle LED6 (message id 1) or LED7 (message id 2) or LED8 (message id 3) or LED9 (message id 4) on the Base Board as per the received message from CM0+ core.
	On CM4 core
		Get received message interrupt and toggle LED based on the message
		Release the IPC lock and notify Server (CM0+ core)
	On CM0+ core
		Wait until CM4 core release lock (received data)
		Send LED id to CM4 core using IPC channel