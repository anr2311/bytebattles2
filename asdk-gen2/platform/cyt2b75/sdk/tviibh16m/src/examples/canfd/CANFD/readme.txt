Please refer to base board schematics and put in the jumper settings.

PSVP: 
HW connection:
CAN0 on BB (Upper DB9 on P6), Connected to CANFD0_1 on the device.
Jumpers: 	Short - J70/72 for CAN TXD/RXD lines
			Short - J71 if wakeup is needed (currently in the sample not used)
			Short - J69 (terminal resistor)

* Copy main_cm0plus.c into your workspace and re-build.
  
* Usage:
  [1] Copy one of the main file to the src folder.
  [2] Rebuild the project.
  [3] Start debug and run.
  [4] Start PEAK PCAN-view with the configuration.
      That is configured:
        - CANFD mode (500kHz for nominal bitrate, 1MHz for fast bitrate. Samping point of both are 75%)
        - At the start message will be sent to the PC with ID 0x525
          2 standard IDs and 2 extended IDs can be sent from the PC side.
        - All ID filters are configured as the classical filter,
          only following IDs are accepted by the MCU:
                STD : 0x010, 0x020
                EXT : 0x10010, 0x10020
          At the receive call back CanfdRxCallback in the main.c,
          received message is sent back with received ID +1.
          You can check the behavior on the trace window.

