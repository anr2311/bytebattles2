SYSCLK: CSV for HF Clock 
===========================
Description
^^^^^^^^^^^
- In this example, can monitor the HFCLK_1 using CSV unit, with reference to – 
  ECO -- External ECO with 16MHz clock
  IMO -- Internal IMO with 8MHz clock
- The fault trigger can be selected from lower limit. upeer limit and period settings.
- Use "CSV_REF_CLK_SELECT” to select between IMO/ECO and "generateCSVFault" for violating 
  CSV unit and generate fault.
- Check "AN32474" to get the lower limit, upper limit, period and start delay values
  based on the Monitor clock (HFCLK_1 -- 80Mhz) and Reference clock (IMO or ECO).

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B9x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-2M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- None

Expectation
^^^^^^^^^^^
- The CSV unit will contineously monitor selected HF clock
    LED5 will be ON to indicate the correct operation.
- If generateCSVFault != CSV_TRIG_ERROR_NONE then
    The fault will trigger and LED5 will start blinking on successfult fault capture.
- Do reprogramming for generating fault from different trigger.