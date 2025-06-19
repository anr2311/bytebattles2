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

* Copy main_cm0plus.c into your workspace and re-build. Device works as a master, and the PLIN as a slave

* Use the PLIN_Slave.linproj project and send data as below.
  Master sends data 0x00 initially.
  Send 1 byte from slave on 0x10, master sends the data received over 0x10 in 0x11
  Send 8 bytes from slave on 0x01, master sends the data received over 0x01 in 0x02

