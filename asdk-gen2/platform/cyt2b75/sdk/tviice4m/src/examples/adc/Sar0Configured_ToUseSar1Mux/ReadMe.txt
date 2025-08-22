
This example gets analog value input from ADC[1]_0 using ADC0.

Hardware Configuration on TVII-C-E-4M CPU Board:
R1011 should be closed near VR1.

Software Configuration:
1. Configure SAR0 as 
    CTL.SARMUX_EN=1, 
    CTL.ADC_EN=1, 
    CTL.ENABLE=1
2. Configure SAR1 as 
    CTL.SARMUX_EN=1, 
    CTL.ADC_EN=0, 
    CTL.ENABLE=1
3. Configure analog input ADC[1]_0 as 
    SAMPLE_CTL.PIN_ADDR=0,
    SAMPLE_CTL.PORT_ADDR=SARMUX1,
    SAMPLE_CTL.SAMPLE_TIME=1000,
4. Initialize logical channel with PASS0_SAR0->CH[0]  
5. Enable and trigger logical channel PASS0_SAR0->CH[0]
