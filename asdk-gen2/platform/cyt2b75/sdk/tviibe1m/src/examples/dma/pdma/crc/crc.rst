P-DMA CRC 
=========
Description
^^^^^^^^^^^
- This example demonstrates the CRC check on PDMA.

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
- After running the example, check the CRC result ‘u32CrcResult’ in the watch window of the IDE. CRC result should match the EXPECTED_RESULT_VALUE (0x3C4687AFUL). LED blinks after the transfer completion.