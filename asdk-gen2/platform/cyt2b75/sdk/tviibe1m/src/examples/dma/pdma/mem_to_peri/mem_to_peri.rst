PDMA: Mem to Peripheral 
=======================
Description
^^^^^^^^^^^
- This example demonstrates the memory transfer to peripheral (UART) via PDMA.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core.
- Connect a micro-USB cable between the board J12 and the PC
- Open a serial terminal (hyper terminal or a putty etc.) and configure the USB-UART COM port with the settings – Baud rate: 115200,  Data: 8 bit, Parity: None, Stop bit: 1 bit, Flow control: None.

Expectation
^^^^^^^^^^^
- Run the program
- After 2 sec, DMA transfer (36 byte) starts. User should be able to see the transfer in the serial terminal screen.