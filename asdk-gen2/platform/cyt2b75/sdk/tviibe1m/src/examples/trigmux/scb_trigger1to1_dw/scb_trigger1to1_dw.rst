Trigger MUX: Peripheral to Mem Trigger 
======================================
Description
^^^^^^^^^^^
- This example demonstrates transfer of data from Peripheral to Memory using P-DMA and SCB/UART.

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
- Run the program
- Input a couple of characters on the serial terminal
- Observe received characters (hex equivalent of ASCII) inside the buffer “au8DestBuffer” using the watch window.