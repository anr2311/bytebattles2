SYSWDT: Warning Interrupt 
=========================
Description
^^^^^^^^^^^
- This example demonstrates the WDT warning interrupt functionality. As soon as the timer value overshoots the warn limit, interrupt is generated.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core.

Expectation
^^^^^^^^^^^
- Put a break point in the ISR “Wdt_Warn_IntrISR”.
- Run the code and observe for the warning interrupt occurrence.
- LED gets inverted after the occurrence of the interrupt.