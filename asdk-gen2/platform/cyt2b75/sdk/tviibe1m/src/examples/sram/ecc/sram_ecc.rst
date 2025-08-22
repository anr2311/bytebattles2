SRAM: SRAM_ECC 
==============
Description
^^^^^^^^^^^
- This example checks SRAM ECC error injection functionality using hard coded SRAM 
  address of “0x0800_0500”. 
- Write value of “0x5A5A_5A5A” to the address at first.
- Inject ECC value of 0x5D which is supposed to cause one bit error.
- Read the address
- Check expected fault address and fault cause was reported from fault reporting system.
- Inject ECC value of 0x14 which is supposed to cause two bit error (non-collectable).
- Read the address
- Check expected fault address and fault cause was reported from fault reporting system.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Jumper J15 to be short.

Expectation
^^^^^^^^^^^
- LED on the CPU board blinks after the checking.
- Global variable “fail” should contain “0” if everything worked as expected.