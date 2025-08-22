Flash: Work Flash – CM0+, CM4 Blocking
======================================
Description
^^^^^^^^^^^
- This example demonstrates the test on flash address by erasing/writing/reading/verifying a whole sector for blocking mode.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Copy and paste flash_ex_rand_data.h, both main_cm0plus.c and main_cm4.c into main “src”  folder
- Use only sram for debug. DO NOT WRITE ON FLASH.

Expectation
^^^^^^^^^^^
- Run, and wait 3[s] to avoid breaking debugger connection.
- The value written in Flash is checked and the program will stop if there is a mismatch. Operation is okay when you reach the last while(1) of the main function.