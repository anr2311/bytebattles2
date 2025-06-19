Trigger MUX: Mem to Mem SW Trigger 
==================================
Description
^^^^^^^^^^^
- This example demonstrates transfer of data from Memory to Memory using P-DMA

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
- After running the example user should be able to see the ‘au8SrcBuffer’ contents in the destination buffers ‘au8DestBuffer0’ and ‘au8DestBuffer1’ in the watch window of the IDE.
- LED blinks after the transfer completion