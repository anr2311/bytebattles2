SCB UART: Read/Rx and Write/Tx 
==============================
Description
^^^^^^^^^^^
- This example demonstrates SCB-UART functionality for read and write in a way as if it looks like a loopback.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core
- Ensure J11 and J13 and hooked
- Connect a micro-USB cable between the board J12 and the PC
- Open a serial terminal (hyper terminal or a putty etc.) and configure the USB-UART COM port with the settings – Baud rate: 115200,  Data: 8 bit, Parity: None, Stop bit: 1 bit, Flow control: None.

Expectation
^^^^^^^^^^^
- Send characters via the configured Hyper Terminal or Putty, user will see the typed character back on the window, depicting a loopback scenario.