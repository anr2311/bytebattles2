GPIO Interrupts 
===============
Description
^^^^^^^^^^^
- This example demonstrates GPIO interrupt configuration and events of User Button (SW3) on CPU board.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Jumper J14 to be short
- Jumper J15 to be short

Expectation
^^^^^^^^^^^
- On falling edge interrupt event from the User Button, User LED will toggle.
	Press User Button (SW3) -> Falling edge interrupt  User LED (LED5) on
	Press User Button (SW3) -> Falling edge interrupt  User LED (LED5) off