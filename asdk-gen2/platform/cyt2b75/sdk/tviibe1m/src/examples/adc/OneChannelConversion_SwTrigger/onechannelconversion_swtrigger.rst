ADC: One Channel Conversion 
===========================
Description
^^^^^^^^^^^
- This example demonstrates the ADC single channel conversion via SW trigger.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- Any example could be copied to the main “src” based on the interested core
- Also copy example_config.h into main “src”  folder
- In the copied example_config.h, select which core shall run the example
- Ensure J11 and J13 and hooked
- Connect a micro-USB cable between the board J12 and the PC
- Open a serial terminal (hyper terminal or a putty etc.) and configure the USB-UART COM port with the settings – Baud rate: 115200,  Data: 8 bit, Parity: None, Stop bit: 1 bit, Flow control: None.

Expectation
^^^^^^^^^^^
- Run program.
- One channel conversion interrupt done. User should be able to see the result in buffer ‘resultBuff’ in the watch window of the IDE.
- The result is displayed via UART terminal on PC.
- The potentiometer (POTI) on the Traveo II base board is used as analog source.