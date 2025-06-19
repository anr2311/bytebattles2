SYSINT: GPIO Edge Detection
===========================
Description
^^^^^^^^^^^
- This example demonstrates GPIO edge detection interrupt.
- Use 2 LED ports and 2 button ports on the base board.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- None

Expectation
^^^^^^^^^^^
- BUTTON1 : Detects falling edge. When falling edge is detected, toggle LED0.
- BUTTON2 : Detects rising edge. When rising edge is detected, toggle LED1.