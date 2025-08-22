PDMA Interrupt 
===============
Description
^^^^^^^^^^^
- This example demonstrates PDMA interrupt test. 
- Destination buffer address is changed alternately via receiving the complete interrupt which indicates Mem to Mem transfer is executed.

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
- After running the example, user should be able to see the ‘au8SrcBuffer’ contents in the destination buffers ‘au8DestBuffer0’ and ‘au8DestBuffer1’ in the watch window of the IDE.