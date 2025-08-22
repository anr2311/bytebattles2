LVD Fault 
=========
Description
^^^^^^^^^^^
- This example demonstrates configuration of Low Voltage Detect(LVD) and capture the fault, if voltage drops below the threshold on the VDDD rails of CPU.

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
- Remove jumper J5 from CPU board and provide the external variable voltage supply on J5_1 pin (initially should be set to 5V).

Expectation
^^^^^^^^^^^
- Initially LED0 on the Base board will blink at slower rate. Regulate the externally supplied voltage down and observe LED0. When VDDD rail voltage will fall below 4.5V, LVD fault will occur and LED0 will start blinking at faster rate.