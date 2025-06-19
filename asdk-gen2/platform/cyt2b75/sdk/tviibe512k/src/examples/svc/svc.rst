SVC: Supervision call 
=====================
Description
^^^^^^^^^^^
- This example explains the usage of SVC (Supervisor call). The CPU core tries 
  to access restricted registers that only cores in privileged mode can access. 
    1. Configures the MPU so that only privileged cores can access the registers 
    that control the GPIO port connected to the LED on the CPU board.
    2. Enters user mode
    3. Blinks the LED by SVC
    4. Enters privileged mode by SVC
    5. Blinks the LED by directly accessing the registers
    6. Enters user mode again
    7. Tries to blink the LED by directly accessing the registers, resulting in a hard-fault.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- None.

Expectation
^^^^^^^^^^^
- The CPU will blink the LED on the board twice. And finally, it enters the hard-fault 
  handler by accessing restricted register in user mode. It is recommended to use a debugger 
  to check the function step-by-step.