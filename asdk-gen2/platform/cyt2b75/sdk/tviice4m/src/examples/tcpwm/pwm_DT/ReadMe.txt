* CYTVII-C-E-4M-176-CPU Board setting
  - Check the following pins on the CPU Board:
	TCPWM0_LINE256 		--> R1011_2
	TCPWM0_LINE256_COMP	--> P17_8

	"Please use GPR1 or GRP2 channels for DT mode."
  
* Copy main_cm0plus.c or main_cm4.c into your workspace and re-build.
  Both file behave same. Only one of those can be used at once.

* Please see pin outputs the 200Hz with right side dead time 250 [us] 
  and left side dead time 500 [us].

