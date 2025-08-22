GPIO LEDs 
=========
Description
^^^^^^^^^^^
- This example demonstrates blinking of multiple LEDs available on the TVII Base Board.

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
- All the Base Board LEDs will toggle at the interval of 50ms.
	LED0 - LED9 on
	Delay 50ms
	LED0 - LED9 off
	Delay 50ms