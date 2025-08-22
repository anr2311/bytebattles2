/******* Connection Steps for the CXPI Polling method NRZ_Master mode **********/

1. Use a TVII-Baseboard with a 1M CPU board, power it up
2. Follow the connections mentioned in the below table.

| Sl.No | 176-TQFP CPU Board-Pins | TVII-BB-Pins |
|-------|-------------------------|--------------|
|	1   | P16_9	              	  | J12.2(CLK)   |
|	2   | P17_13	              | J11.1(TxD)   |
|	3   | P17_15	              | J14.1(RxD)   |
|   4   | P17_17	              | J13.2(NSLP)  |
|   5   | P17_11                  | J9.2(SELMS)  |
--------|-------------------------|--------------|

3. Connect the ground of the 176-TQFP CPU board's to the TVII-BB (BaseBoard) ground.
4. Power the 176-TQFP CPU board and flash the CXPI NRZ_Master code.
5. Connect the Pin #7 of P1 DB connector in the TVII-BB to the BUS of external slave/analyzer.
6. Example is based on the polling method it continuously pType is sent in the bus and the slave responds with a CXPI frame if it has anything to send.
7. Also the example send the PID and the salve responds with a response to the PID and also it send the PID with response to the slave.

* only verified by 20kbps clock rate and master tx commands.
