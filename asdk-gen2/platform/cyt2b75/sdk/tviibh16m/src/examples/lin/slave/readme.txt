* Board setting
  - Short following pins on the base board:
    J59, J60, J63
  - Short J58 on the base board if using T2G as a master node
  - Short following pins on the adapter board. (Power suply to base board)
    J150, J151, J152 
  - Connect LIN0 connector (P5 upper one) on the base board
    and CANoe or other analyzer if available.
	For PEAK PLIN interface,
	PLIN/4 to P5/7
	PLIN/5 to P5/3
	PLIN/9 to P5/9

* Copy main_cm0plus.c into your workspace and re-build. Device works as a slave, and the PLIN as a master

* Use the PLIN_Master.linproj project and send data as below.
  From master, send say 0x10: response will be 0x00
  Send 0x11 with correct data
  Send 0x10 again, you will be able to view the data last time sent with 0x11
  Same is true with 0x01 and 0x02, for 8 bytes of data
