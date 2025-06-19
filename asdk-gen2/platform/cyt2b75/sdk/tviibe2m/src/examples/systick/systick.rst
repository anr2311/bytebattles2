SYSTICK 
=======
Description
^^^^^^^^^^^
- This example demonstrates the setup of the SYSTICK and corresponding callback function for LED toggling

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Scope might be needed to see/confirm the fast LED toggling

Expectation
^^^^^^^^^^^
- LED on the base board is toggled every 10ms (via the SYSTICK callback function)