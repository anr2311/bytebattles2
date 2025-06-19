PROT: PPC_PC 
============
Description
^^^^^^^^^^^
- Example demonstrates the use of fixed Peripheral Protection Unit (PPU)
	In this example, CM4 core sets fixed PPUs for two protection contexts
	(PC), one of the PC enables access a GPIO register and the other 
	blocks access to the GPIO register.

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
- CM4 will read the GPIO register with the permitted PC, and with the prohibited PC.
- CM4 core gets into hard fault handler.
- CM0+ will detect bus error caused by the prohibited access of the CM0+.
- CM0+ blinks an LED continuously in its fault report handler.