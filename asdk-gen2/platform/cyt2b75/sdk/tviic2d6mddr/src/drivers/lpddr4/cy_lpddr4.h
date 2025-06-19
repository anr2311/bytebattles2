/***************************************************************************//**
* \file cy_lpddr4.h
*   $Date: 2023-01-24 14:43:16 +0530 (Tue, 24 Jan 2023) $
*   $Revision: 320853 $
*
* \brief Provides an API implementation of the lpddr4 controller driver.
*
********************************************************************************
* \copyright
* Copyright 2020-2021, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/**
 * \defgroup group_lpddr Low Power Double Data Rate (LPDDR)
 * \{
 * Driver API for LPDDR4 Memory Controller in TVIIC2D6MDDR <br><br>
 * \ref group_lpddr_user_api <br><br>
 * \ref group_lpddr_general_information <br>
 * \ref group_lpddr_pll <br>
 * <BLOCKQUOTE>
 *      \ref group_lpddr_pll_overview <br>
 *      \ref group_lpddr_pll_setup <br>
 * </BLOCKQUOTE>
 * \ref group_lpddr_controller <br>
 * <BLOCKQUOTE>
 *      \ref group_lpddr_cntlr_overview <br>
 *      <BLOCKQUOTE>
 *          \ref group_lpddr_data_mod <br>
 *          \ref group_lpddr_cntrl_mod <br>
 *      </BLOCKQUOTE>
 *      \ref group_lpddr_reg_prog <br>
 *      \ref group_lpddr_timing_prog <br>
 *      \ref group_lpddr_dram_prog <br>
 *      \ref group_lpddr_address_mapping <br>
 *      \ref group_lpddr_init_process <br>
 *      <BLOCKQUOTE>
 *          \ref group_lpddr_phy_init <br>
 *          \ref group_lpddr_phy_set_reload <br>
 *          \ref group_lpddr_cmd_dly_load <br>
 *          \ref group_lpddr_dll_init <br>
 *          \ref group_lpddr_dram_init <br>
 *          \ref group_lpddr_training <br>
 *              <BLOCKQUOTE>
 *                  \ref group_lpddr_trainings_VREFCAEN <br>
 *                  \ref group_lpddr_trainings_WRLVLEN <br>
 *                  \ref group_lpddr_trainingsRDLVLEN <br>
 *                  \ref group_lpddr_trainingsGTEN <br>
 *                  \ref group_lpddr_trainingsDQSDQEN <br>
 *</BLOCKQUOTE>
 *</BLOCKQUOTE>
 *</BLOCKQUOTE>
 *\ref
 *
 * ----
 * 
 * \section group_lpddr_user_api How to setup the LPDDR4 Controller.
 * Before the external memory can be used the Memory Controller needs to be 
 * initialized and trained. To setup the controller user must provide the  
 * configuration parameters in \ref cy_stc_lpddr4_config_t lpddr_config structure
 * and an initialization type in \ref cy_en_lpddr4_inittype_t. In 
 * cy_dram_config.h the user must set the  CHANNEL_DENSITY and DATA_RATE of his
 * particular hardware. These parameters are needed inside the driver as some 
 * timing parameters depend on the selected Memory Density and/or Data Rate. 
 * To avoid user changes in the main driver file these parameters are placed 
 * in a separate file.
 *  
 * \ref cy_stc_lpddr4_config_t lpddr_config  structure contains all the 
 * neccessary informations needed to properly setup the controller. With the 
 * initialization type \ref cy_en_lpddr4_inittype_t user can select if the 
 * controller is initialized with a complete retraining or if initilaizing is 
 * done by reloading known training data from the PTSR0..25 to the PHY.<br>
 * To initialize the driver, call the \ref Cy_Lpddr_ControllerInit function 
 * providing the base address, the filled  \ref cy_stc_lpddr4_config_t 
 * lpddr_config structure and initialzation type \ref  cy_en_lpddr4_inittype_t. 
 * The \ref Cy_Lpddr_ControllerInit() function is performing the following steps to 
 * setup the LPDDR4 controller:
 *<ul>
 *  <li>Init driver Context</li>
 *  <li>Setup QoS Registers ( Quality of Service )</li>
 *  <li>sets the Main Enabler of the IP</li>
 *  <li>configures the PLL800 which is used to the clock the IP with the values
 *      provided in the config structure and activates the PLL output for FSP[0] 
 *      when PLL has locked FSP[0]</li>
 *  <li>Setup Arbiter Registers RTCFG0_RT0..3 /RTCFG1_RT0..3</li>
 *  <li>Setup DRAM Registers ( LPMR1,LPMR2,LPMR3,LPMR11,LPMR12,LPMR13,
 *      LPMR14,LPMR22..</li>
 *  <li>Setup PHY DDR Mode (VTGC,PHY,DQSDQCR)</li>     
 *  <li>Setup Bist Configuration Register  (BISTCFG_CH0/unBISTCFG_CH1)</li>
 *  <li>Setup Phy Timing Registers (TREG0..15 )
 *      These registers contain the timing parameters as specified in 
 *      Jedec209-4B.<br>For additional information refer Register TRM and 
 *      JEDEC209-4B</li>
 *  <li>Setup DLL Registers DLLCTL_CA_CH0/CH1, DLLCTL_DQ_S0..3</li>
 *  <li>Setup Address Mapping</li>
 *  <li>Phy Initialization (DLL Init,DRAM,Init, PHY Training for FSP[0]</li>
 *  <li>Phy Reload(reload trained values from a former training run which 
 *      were stored in the PTSRx registers.) can be used for quickstart 
 *      without complete training to get the controller up and running 
 *      after a power down event.<br>
 *      <b>or</b><br>
 *      Perform high speed operation training (CBT,WRLEVL,RDLEVL,Gate 
 *      Training,...including switching the PLL fout several times)<br>
 *      and switch to high speed operation FSP[1]</li>
 *  <li>Check if Refresh adaption/timing derating is necessary (warm start)</li>
 *  <li>Program MR23 DQS Oscillator Interval </li>
 *  <li>Start DQS Oscillator and Read DQS Oscillator value as
 *      reference for retraining recognition</li>
 *  <li>Start the Comntroller with Run Command </li>
 * </ul>
 * 
 * After successfully returning from Cy_Lpddr_ControllerInit() the memory is accessible.
 * As voltage and temperature have impact on timing and may invalidate already 
 * trained interface values. The system needs to monitor the LPDDR4 continuously
 * to avoid data corruption.This can be done by periodically calling the 
 * \ref Cy_Lpddr_MemoryMonitoring() function. The cycle time in which this 
 * function needs to be callled can be calculate using \ref 
 * Cy_Lpddr_CalcMR4ReadInterval() with the parameters sysResponseDelay [ms] 
 * and sysTemperatureGradient[°C/s] of users target system.The function returns 
 * a interval time calculated as defined in JEDEC209-4B. With the interval time
 * calculated a timer can be started to call \ref Cy_Lpddr_MemoryMonitoring.<br>
 * This function perform the following steps:<br> 
 * <ul>
 * <li>read Memory Temperature MR4[2:0]</li>
 * <li>based on MR4 temperature adjust refresh rate if needed</li>
 * <li>based on MR4 temperature perform timing derating if needed</li>
 * <li>Memory over-/undertemp supervision</li>
 * <li>Start the  DQS Osc. / Read DQS Osc.<br>
 *     Check if retraining is necessary and set retraining request in 
 *        driver context \ref cy_stc_lpddr4_context_t</li>
 *</ul>
 *
 * If due to voltage / temperature drift a re-training might be required the 
 * flags retrainRequestCHA/CHB in the driver context are set to true.
 * The application software must monitore these flags by periodically calling 
 * \ref Cy_Lpddr_CheckForRetraining() and trigger a retraining if needed with 
 * \ref Cy_Lpddr_RequestDQS2DQRetrain. Once the training has finished the 
 * retrainRequestCHA/CHB flags are cleared. Before re-training is triggered 
 * the timer which is calling the \ref Cy_Lpddr_MemoryMonitoring should be 
 * stopped and restarted directly after retraining has finished.
 * <br><br>
 * 
 * 
 *   
 * 
 *
 * \section group_lpddr_general_information General LPDDR4 Information
 * The mxlpddr4 IP provides a LPDDR4 SDRAM interface for MXS40 Traveo II 
 * microcontrollers. The configuration is optimized for usage in 
 * TVII-C-2D-6M-DDR. The IP provides the following functionality:
 *
 * <ul>
 * <li>JESD209-4B compliant controller and PHY to connect to external 16bit or 
 *     32bit LPDDR4 SDRAM memories of maximum 1GByte</li>
 * <li>Target speed grade LPDDR4 1600 @800MHz </li>
 * <li>Two LPDDR4 Command/Address channels</li>
 * <li>AHB-Lite Programming Interface</li>
 * <li>4 AXI slave ports </li>
 * <li>Programmable AXI to LPDDR4 address mapping/li>
 * <li>Programmable access re-ordering for bandwidth optimization</li>
 * <li>Priority and latency-based port arbitration</li>
 * <li>Memory Protection Unit for LPDDR4 SDRAM</li>
 * <li>Performance counters to measure throughput and latencies between AXI 
 *     ports and SDRAM </li>
 * <li>Quality of Service (QoS) emulation by AXI master ID</li>
 * <li>Inline ECC for the external SDRAM </li>
 * <li>Integrated PLL for generating the PHY clock from the external crystal 
 * oscillator clock with Spread Spectrum Clock Generation (SSCG)<br> ability 
 * (0.25% … 1.5% down modulation, programmable modulation frequency)</li>
 * <li>Programmable Timing Registers</li>
 * <li>Independent training mode for gate, data eye, write leveling, VREF and CA training</li>
 * <li>Testability provided through BIST, loopback, scan and JTAG (Mentor) </li>
 * <li>Junction temperature range TJ = -40˚C … +150˚C</li>
 * </ul>
 * 
 * 
 * 
 * 
 * 
 * <img src="lpddr_blockdiagram.png" >
 * <br>LPDDR4 Block Diagram<br>
 * \section group_lpddr_pll  PLL800
 * \subsection group_lpddr_pll_overview PLL Overview
 * <br><br>
 * \subsection group_lpddr_pll_setup Setting up the PLL
 * <br><br>
 * 
 * \section group_lpddr_controller LPDDR4 Memory Controller
 * 
 * \subsection group_lpddr_cntlr_overview Controller Overview
 * <br>
 * \subsubsection group_lpddr_data_mod Data Modul
 * <br>
 * 
 * \subsubsection group_lpddr_cntrl_mod Control Modul
 * <br>
 * \subsection group_lpddr_
 * \subsection group_lpddr_reg_prog Register Programming through APB Interface
 * 
 * Dynamo provides users with a range of registers for setting up DRAM’s 
 * operation mode, timing parameters,configuring the controller and receiving 
 * status. The internal registers are classified as follows:
 * <ul>
 *  <li>User Command Register: Stores commands from users</li>
 *  <li>Control Registers: Stores configurations for Memory Controller and DRAM’s 
 *      operation modes. </li>
 *  <li>Timing Registers: Stores timing parameters</li>
 *  <li>Status Registers: Stores status of the controller</li>
 * </ul>
 * 
 * To program internal registers, the LPDDR4 memory controller has one APB bus 
 *  which supports basic read and write bursts. All internal registers can be 
 * written or read with this interface. Timing registers, control registers and 
 * user command register are writable ones and can only be accessed with write 
 * burst of APB interface. Status registers, error registers and interrupt 
 * registers  are read only ones. 
 * 
 * \subsection group_lpddr_timing_prog Timing Register Programming 
 * To prevent the LPDDR4 Memory Controller from malfunction, all timing 
 * registers must be programmed to right value,unless some SDRAM timing 
 * constraints are violated. The values of timing registers are set as the 
 * number of DRAM clock cycles. Regarding timing paramaters refer also to
 * Controller Register TRM and JEDEC209-B specification.
 * 
 * 
 * 
 * \subsection group_lpddr_dram_prog DRAM Operation Mode Programming 
 * The LPDDR4 Controller provides users with command interface to set all DRAM 
 * mode registers.There are two ways to set DRAM mode registers by automatic 
 * initialization or manual setting.
 * 
 * Automatic Initialization Setting: LPDDR4 Controller starts DRAM automatic 
 * initialization process before going into normal or training operations. 
 * Therefore, before initialization, DRAM Mode Registers in Register Block are 
 * required to be of valid values which, during initialization, will be used to 
 * update all DRAM mode registers by LPDDR4 Controller. 
 
 * Manually Setting: The APB master can stop LPDDR4 Controller and re-set DRAM 
 * mode registers by issuing USER_CMD_MRW command.
 * 
 * During the initialization and training processes, some parameters will be 
 * changed and different from the set users programming to mode registers 
 * (lpmr*). Users can check the current values of DRAM’s mode registers stored 
 * in shadow registers (shad_lpmr*) of register block.
 * 
 * \subsection group_lpddr_address_mapping Address Mapping Scheme 
 * To gain higher DRAM bus efficiency, Dynamo memory controller supports a 
 * flexible method to configure DRAM channel, rank, bank, row and column 
 * address. AXI4 address can be separated into 4 segments:<br>
 * <ul>
 * <li>DRAM Channel</li>
 * <li>DRAM Bank</li>
 * <li>DRAM Row</li>
 * <li>DRAm Coloumn</li>
 * </ul>
 * For better flexibility, each bit in AXI4 Address can be assigned to one of 
 * these segments. Table 13 shows the address mapping registers. 
  * <table class="doxtable">
 * <tr>
 * <th>Register</th>
 * <th>Register Field</th>
 * <th>Description</th>
 * <th>Width</th>
 * <th>Reset Value</th>
 * </tr>
 * <tr><td>addr0</td>
 * <td>col_addr_map_b0 </td>
 * <td> Address mapping for column bit 0</td> 
 * <td>5</td> <td> 0</td></tr>
 * <tr><td>addr0</td><td>col_addr_map_b1 </td>
 * <td> Address mapping for column bit 1</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr0</td><td>col_addr_map_b2 </td>
 * <td> Address mapping for column bit 2</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr0</td><td>col_addr_map_b3 </td>
 * <td> Address mapping for column bit 3</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr0</td><td>col_addr_map_b4 </td>
 * <td> Address mapping for column bit 4</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr0</td><td>col_addr_map_b5 </td>
 * <td> Address mapping for column bit 5</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr1</td><td>col_addr_map_b6 </td>
 * <td> Address mapping for column bit 6</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr1</td><td>col_addr_map_b7 </td>
 * <td> Address mapping for column bit 7</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr1</td><td>col_addr_map_b8 </td>
 * <td> Address mapping for column bit 8</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr1</td><td>col_addr_map_b9 </td>
 * <td> Address mapping for column bit 9</td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr2</td><td>row_addr_map_b0 </td>
 * <td> Address mapping for row bit 0</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr2</td><td>row_addr_map_b1 </td>
 * <td> Address mapping for row bit 1</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr2</td><td>row_addr_map_b2 </td>
 * <td> Address mapping for row bit 2</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr2</td><td>row_addr_map_b3 </td>
 * <td> Address mapping for row bit 3</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr2</td><td>row_addr_map_b4 </td>
 * <td> Address mapping for row bit 4</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr2</td><td>row_addr_map_b5 </td>
 * <td> Address mapping for row bit 5</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr3</td><td>row_addr_map_b6 </td>
 * <td> Address mapping for row bit 6</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr3</td><td>row_addr_map_b7 </td>
 * <td> Address mapping for row bit 7</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr3</td><td>row_addr_map_b8 </td>
 * <td> Address mapping for row bit 8</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr3</td><td>row_addr_map_b9 </td>
 * <td> Address mapping for row bit 9</td> <td>5</td><td>  0</td></tr>
 * <tr><td>addr3</td><td>row_addr_map_b10</td>
 * <td>  Address mapping for row bit 10</td> </td> <td>5</td> <td> 0</td></tr> 
 * <tr><td>addr3</td><td>row_addr_map_b11</td>
 * <td>  Address mapping for row bit 11</td> <td> 5</td> <td> 0</td> </tr>
 * <tr><td>addr4</td><td>row_addr_map_b12</td>
 * <td>  Address mapping for row bit 12</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr4</td><td>row_addr_map_b13</td>
 * <td>  Address mapping for row bit 13</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr4</td><td>row_addr_map_b14</td>
 * <td>  Address mapping for row bit 14</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr5</td><td>bank_addr_map_b0</td>
 * <td>  Address mapping for bank bit 0</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr5</td><td>bank_addr_map_b1</td>
 * <td>  Address mapping for bank bit 1</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr5</td><td>bank_addr_map_b2</td>
 * <td>  Address mapping for bank bit 2</td> <td> 5</td> <td> 0</td></tr> 
 * <tr><td>addr5</td><td>chan_addr_map_b0</td>
 * <td>  Address mapping for channel bit 0</td>  <td>5</td> <td> 0</td></tr> 
 * </table>
 * Some lowest bits in AXI4 Address are unused, they must be assigned to 0. 
 * The number of unused bits depend  on the DRAM data width (Total DQ bits): 
 * <ul>
 * <li>x8 (8-bit DQ): No bit is unused </li>
 * <li>x16 (16-bit DQ): 1 bit is unused (axi4 address[0])</li>
 * <li>x32 (32-bit DQ): 2 bits are unused (axi4 address[1:0])</li>
 * <li>x64 (64-bit DQ): 3 bits are unused (axi4 address[2:0])</li>
 * 

 * 
 * \subsection group_lpddr_init_process Initialization Process
 * <br><br>
 * <br><br>
 * \subsection group_lpddr_phy_init  PHY Inititialization
 * <br><br>
 * \subsection group_lpddr_phy_set_reload PHY Setting Reload 
 * <br><br>
 * \subsection group_lpddr_cmd_dly_load Command Delay Load 
 * <br><br>
 * \subsection group_lpddr_dll_init DLL Initialization
 * <br><br> 
 * \subsection group_lpddr_dram_init  DRAM Inititialization
 * <br><br>
 * \subsection group_lpddr_training Interface Trainings
 * <br><br>
 * 
 * \subsubsection group_lpddr_trainings_VREFCAEN Command Bus Training (VREFCAEN)
 * 
 * The purpose of the Command Bus Training is to adjust:<br>
 * <ul>
 *  <li>the command address voltage reference VREF(CA)</li>
 *  <li>center align the the delay between CA[5:0] and the rising edge of CK to 
 *      meet setup and hold times</li>
 *  <li>center  align the delay between CS and rising edge of CK to meet setup 
 *      and hold times</li>
 * </ul>
 * 
 * \image html cbt_overview.png "Command Bus Training Overview"
 * 
 * <center>
 * Pictures below show (left) before CBT was executed (right) after CBT was 
 * executed 
 * <table class="doxtable">
 *   <tr>
 *     <td> \image html before_cbt.png "before command bus training"</td>
 *     <td> \image html after_cbt.png "after command bus training finished"</td>
 *   </tr>
 * </table>
 * </center>
 * After the PHY and SDRAM has been successfully initialized, LPDDR4 DRAM is 
 * operating in FSP[0] frequency set point with the low clock frequency.
 * To switch to higher clock frequency, the command bus training for 
 * LPDDR4 must be performed in order to satisfy setup and hold time for control 
 * signals (CS and CA) to DRAM. Users must configure POM (PHY Operational Mode) 
 * register to run Command Bus Training: 
 * <ul>
 *      <li><b>fs:</b>The Frequency Set Point (FSP) to be trained, must be 
 *                  different from operating set point. </li>
 *      <li><b>vrefcaen:</b>DRAM VREF-CA training enable </li>
 * </ul>
 *
 * The  completion of Command Bus Training is indicated through bit 
 * vrefcac in the POS register. The field vrefcaerr in the PTS0 shows VREF-CA 
 * training error for each channel. Before starting command bus training, timing
 * registers must be programmed to correct settings to prevent timing violation 
 * while switching between frequency set point [0] and frequency set point [1]. 
 * The timing registers are set during controller init 
 * \ref Cy_Lpddr_ControllerInit(). The user needs to set the registers 
 * appropriated to the selected FSP[0]/FSP[1] frequencies in the lpddr config 
 * structure \ref cy_stc_lpddr4_config_t before calling the 
 * Cy_Lpddr_ControllerInit(). The values passed with the lpddr_config will be 
 * programmed to the controller during init.
 * 
 * The registers used during the command bus training ( CBT )  are listed below. 
 * 
 * <table class="doxtable">
 *
 *   <tr>
 *     <th>Timing </th>
 *     <th>Programming Value</th>
 *     <th>Description</th>
 *   </tr>
 *   <tr>
 *     <td>t_mrd </td>
 *     <td>>= RU(tMRD/tfs_period) </td>
 *     <td>Mode Register Set command cycle time </td>
 *   </tr>
 *   <tr>
 *     <td>t_dqscke</td>
 *     <td>>= RU(tDQSCKE/tfs_period) </td>
 *     <td>Valid Strobe Requirement before CKE Low </td>
 *   </tr>
 *   <tr>
 *     <td>t_caent </td>
 *     <td>>= RU(tCAENT/ofs_period)</td>
 *     <td>First CA Bus Training Command Following CKE Low </td>
 *   </tr>
 *    <tr>
 *     <td>t_dtrain </td>
 *     <td>>= RU(tDTRAIN/tfs_period) </td>
 *     <td>tDStrain and tDHtrain</td>
 *   </tr>
 *    <tr>
 *     <td>t_ckehdqs</td>
 *     <td>>= RU(tCKEHDQS/ofs_period)+8 </td>
 *     <td>CKE high to DQS</td>
 *   </tr>
 *   <tr>
 *     <td>t_ckelck </td>
 *     <td>>= 15 </td>
 *     <td>CKE low to CK</td>
 *   </tr>
*   <tr>
 *     <td>t_ckckeh</td>
 *     <td>>= RU(tCKCKEH/ofs_period) </td>
 *     <td>CK to CKE high </td>
 *   </tr>
 *   <tr>
 *     <td>t_fc </td>
 *     <td>>= RU(tFC/tfs_period) </td>
 *     <td>Frequency Set Point Switching Time </td>
 *   </tr>
 *   <tr>
 *     <td>t_vreftimelong </td>
 *     <td>>= RU(tVREFca_Long/tfs_period) </td>
 *     <td>VREF changing time for multiple steps </td>
 *   </tr>
 *   <tr>
 *     <td>t_vreftimeshort</td>
 *     <td>>= RU(tVREFca_Short/tfs_period)</td>
 *     <td>VREF changing time for single steps</td>
 *   </tr>
 *   <tr>
 *     <td>t_lvlload </td>
 *     <td>>= 4</td>
 *     <td>Delay change to load pulse </td>
 *   </tr>
 *   <tr>
 *     <td>t_lvlresp_nr </td>
 *     <td>>= 8 </td>
 *     <td>Leveling response </td>
 *   </tr>
 *   <tr>
 *     <td>t_lvlaa</td>
 *     <td>>= 8 </td>
 *     <td>STRB/READ/MRR to STRB/READ/MRR </td>
 *   </tr>
 *   <tr>
 *     <td>t_lvlexit </td>
 *     <td>>= 10 </td>
 *     <td>Leveling disable to MRS leveling disable </td>
 *   </tr>
 *   <tr>
 *     <td>t_lvldis </td>
 *     <td>>= 10 </td>
 *     <td>Respond to leveling disable </td>
 *   </tr>
 * </table>
 * 
 * 


 * 
 * \subsubsection group_lpddr_trainings_WRLVLEN Write Leveling (WRLVLEN)
 * The write leveling  must be performed in order to ensure that the DQS is 
 * aligned correctly with the memory clock at the DRAM for write access. The 
 * Controller is sweeping the DQS signal for each byte lane until it finds a 
 * rising edge of CLK. The controller has 2 channels with 2 byte lanes/channel.
 * The clock is sampled with DQS and the sampled value is return through DQ 
 * line to the controller until a  0->1 edge is detected. At this point the 
 * delay line is locked and the DQS is now aligned to CLK. The training values 
 * for write leveling are stored in the PTSR4 ( Phy Training Status Register 4)
 * 
 *
 * <br>Write leveling aligns DQS to the rising edge of CK<br>
 * <img src="writeleveling.png" >
 * 
 * 
 * \subsubsection group_lpddr_trainingsRDLVLEN Read Leveling (RDLVLEN)
 * The purpose of the Read Leveling is to center the VREF(DQ) of the DQx/DMx 
 * signals and center align of the DQx/DM Bits to the DQS. To achieve this the 
 * Memory sends known bit pattern stored in the Mode Registers MR15,MR20,Mr32,
 * MR40 to the LPDDR4 controller. The LPDDR4 controller compares what was 
 * recieved with the known pattern. The controller varies the delays in both 
 * directions until the the pattern is no longer correctly received. This will 
 * give a upper and lower threshold. The final delay setting will be center
 * aligned between the upper and lower threshold.
 * 
 * 
 * <br>Read Leveling aligns DQx/DM signals to the middle of DQS from LPDDR4
 * Memory. <br>
 * <img src="readleveling.png" >
 * 
 * The values trained are stored in the following registers:
 * <ul>
 *      <li>PTSR14,PTSR15,PTSR16,PTSR17,PTSR18,PTSR19,PTSR20,PTSR21
 *          for DQ delays</li>
 *      <li>PTSR22 for DM ( Data Mask ) delays</li>
 *      <li>PTSR23 VREF(DQ) settings</li>
 * </ul>
 * 
 * 
 * 
 * \subsubsection group_lpddr_trainingsGTEN Gate Training (GTEN)
 * The controller support 2 ways for gate training.
 *	<ul>
 *  <li>automatic gate training rtgc0_gt_dis =0</li>
 *	<li>firmware triggered gate training pom_gten=1 </li>
 *  </ul>
 * 
 * The purpose of gate training is to locate the delay at which the initial read
 * DQS rising edge aligns with the rising edge of the read DQS gate. Once this 
 * point is identified, the read DQS gate can be adjusted prior to the DQS, to 
 * the approximate midpoint of the read DQS preamble. The gate training 
 * operation requires that the read DQS gate be placed within the bounds of the 
 * beginning of the read DQS preamble and the falling edge of the first read DQS
 * for the response to properly indicate the alignment of gate to the first 
 * read DQS.
 * 
 * <img src="gatetraining.png" >
 * 
 * \subsubsection group_lpddr_trainingsDQSDQEN DQS2DQ Training(DQSDQEN)
 * 
 * 
 * 
 * \section group_lpddr_configuration Configuration Considerations
 *
 *
 * 
 * 
 * 
 * 
 *
 * Refer to the technical reference manual (TRM) and the device datasheet.
 *
 * \section group_lpddr_changelog Changelog
 * <table class="doxtable">
 *   <tr><th>Version</th><th>Changes</th><th>Reason for Change</th></tr>
 *   <tr>
 *     <td>1.0</td>
 *     <td>Initial version</td>
 *     <td></td>
 *   </tr>
 *   <tr>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *   </tr>
 *   <tr>
 *     <td></td>
 *     <td></td>
 *     <td></td>
 *   </tr>
 * </table>
 *
 * \defgroup group_lpddr_macro               Macro
 * \defgroup group_lpddr_functions           Functions
 * \defgroup group_lpddr_data_structures     Data structures
 * \defgroup group_lpddr_enums               Enumerated Types
 * \defgroup group_lpddr_globals             Global variables
 */





#if !defined(__CY_LPDDR4_H__)
#define __CY_LPDDR4_H__



#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include "cy_dramconfsel.h"
#include "cy_en_lpddr4.h"
#include "cy_device_headers.h"

#define SVN_REV_CY_LPDDR4_H 		"$Revision: 320853 $"
#define SVN_DATE_CY_LPDDR4_H		"$Date: 2023-01-24 14:43:16 +0530 (Tue, 24 Jan 2023) $"



#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */



/*******************************************************************************
*       Macro definitions
*******************************************************************************/

/**
* \addtogroup group_lpddr_macro
* \{
*/

/** Driver major version */
#define  CY_LPDDR4_DRV_VERSION_MAJOR   1
/** Driver minor version */
#define  CY_LPDDR4_DRV_VERSION_MINOR   0

/** LPDDR driver ID */
#define CY_LPDDR_ID                     25u

/********************************************************** 
 *  Wait time definition. Used from Cy_Lpddr_PllEnable
 *  to wait for stabilization after PLL was enabled
 *  TODO: check with AVRH what is requirement
 **********************************************************/
/* distinguish between PSVP and FCV/SOC CDT 368813 #7*/
/* CDT 380742 200us -->500us */
#if(CY_USE_PSVP == 1)
    #define CY_LPDDR4_PLL_STABILIZATION_TIME (10ul)
#else
    #define CY_LPDDR4_PLL_STABILIZATION_TIME (500ul)
#endif



/** CY_LPDDR4_CLKPERIOD(clkfreq) calculates the clock period from the clock 
 *  frequency \a clkfreq*/
#define CY_LPDDR4_CLKPERIOD(clkfreq) ((double)(1.0 / clkfreq))

/** CY_LPDDR4_MAX(a,b) function like macro to deliver the max 
 *  from \a a and \a b */
#define CY_LPDDR4_MAX(a,b)  (a > b ? a : b)

/** CY_LPDDR4_MIN(a,b) function like macro to deliver the min 
 *  from \a a and  \a b */
#define CY_LPDDR4_MIN(a,b)  (a < b ? a : b)

/** CY_LPDDR4_CEILING(a,b) function like macro returns the smallest integer value greater than or equal to x.
 *  from \a a and  \a b */
#define CY_LPDDR4_CEILING(a,b)  (((a) + (b) - 1) / (b))


/** CY_LPDDR4_CEIL(a) function like macro returns the smallest integer value greater than or equal to a.
 *  from \a a */
#define CEIL(a) ( (a - (int)a)==0 ? (int)a : (int)a+1 )


//#define FLOOR(a) ((double)((long)(a)- ((a)<0.0)))


/** CY_LPDDR4_CLOCKCYCLES(t_nsec,clkfreq) converts given time in \a nsec. 
 *  to clock cycles based on the \a clkfreq round up to next full clk clock cycle*/
#define CY_LPDDR4_CLOCKCYCLES(t_nsec,clkfreq)  (uint32_t) (CEIL((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq)))) ))

/** CY_LPDDR4_CLOCKCYCLES_RD(t_nsec,clkfreq) converts given time in \a nsec. 
 *  to clock cycles based on the \a clkfreq round up to next full clk clock cycle*/
#define CY_LPDDR4_CLOCKCYCLES_RD(t_nsec,clkfreq)  (uint32_t) (CEIL((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq)))) ))

/** CY_LPDDR4_CLOCKCYCLES_TREFI(t_nsec,clkfreq) converts given time in \a nsec. 
 *  to clock cycles based on the \a clkfreq round down to next full clk clock cycle*/

#define CY_LPDDR4_CLOCKCYCLES_TREFI(t_nsec,clkfreq)  (((uint32_t)((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq))))))-((uint32_t)((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq))))))%4)
/*calculated the tREFI in clock cycles  from trefi value given in t_nsec based on the clock_freq and substracts the compensation value comp_clk */
#define CY_LPDDR4_CLOCKCYCLES_TREFI_COMPENSATED(t_nsec,clkfreq,comp_clks) (((uint32_t)((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq)))))-comp_clks)-((uint32_t)((((double)t_nsec)*0.000000001/( (1.0 / ((double)clkfreq)))))-comp_clks)%4)
/** CY_LPDDR4_UPPER_LIMIT(a,limit) limits \a a to max \a limit. 
 *  The result can´t be > \a limit*/
#define CY_LPDDR4_UPPER_LIMIT(a,limit)  (a > limit ? limit : a )

/** \brief CY_LPDDR4_LOWER_LIMIT(a,limit) limita \a a to \a limit. 
 *  The result is at least \a limit */
#define CY_LPDDR4_LOWER_LIMIT(a,limit)  ( a < limit ? limit : a )

/** refresh rate used in the driver status for 4*tREFI */
#define CY_LPDDR_FIXED_REFRESH_RATE_4_TREFI 4
/** refresh rate used in the driver status for 2*tREFI */
#define CY_LPDDR_FIXED_REFRESH_RATE_2_TREFI 2
/** refresh rate used in the driver status for 1*tREFI 
 *  corresponds to approx. 85°C  */
#define CY_LPDDR_FIXED_REFRESH_RATE_1_TREFI 1
/** refresh rate used in the driver status for 0.5*tREFI */
#define CY_LPDDR_FIXED_REFRESH_RATE_0p5_TREFI 0.5
/** refresh rate used in the driver status for 0.25*tREFI */
#define CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI 0.25
/**
 * maximum QoS Master ID 
 **/
#define CY_LPDDR_MAX_QOS_MASTERID 15

/** MC has 4  AXI ports 0..3 so highest idx is CY_LPDDR_MAX_ROUTEIDX */
#define CY_LPDDR_MAX_ROUTEIDX     3

/**  Read / Write  Address Queue High/Low Priority Threshold max value*/
#define CY_LPDDR4_ARQ_AWQ_PRIORITY_THRESHOLD 0x15
/** Priority Mask for clearing the prio of a master in the QoS registers*/
#define CY_LPDDR_MAX_QOS_PRIO_MASK 3

/*******************************************************************************
          Limitations for the Bitfields in the PLL800_CONFIG Register       
*******************************************************************************/
/* PLL 800M OUTPUT_DIV bitfield allowable range,configures the DCO Output Divider (N) */
/** The minimum PLL Output Divider according to 002-30271_0A*/
#define PLL_800M_MIN_OUTPUT_DIV     (2ul)
/** The maximum PLL Output Divider  */
#define PLL_800M_MAX_OUTPUT_DIV    (16ul)

/* PLL 800M REFERENCE_DIV bitfield allowable range,Configures the Input Reference Divider (Q)*/
/** The minimum PLL Reference Divider according to 002-30271_0A*/
#define PLL_800M_MIN_REF_DIV        (1ul)
/** The maximum PLL Reference Divider */
#define PLL_800M_MAX_REF_DIV       (18ul)

/* PLL 800M FEEDBACK_DIV bitfield allowable ranges, Configures the PLL Feedback Divider (P) PFD */

/** The minimum PLL Feedback Divider according to 002-30271_0A*/
#define PLL_800M_MIN_FB_DIV        (20ul)
/** The maximum PLL Feedback Divider according to 002-30271_0A*/
#define PLL_800M_MAX_FB_DIV       (200ul)

/*TODO check the limits for 800M PLL check 002-30271 */

/** PLL 800M minimum Fvco in MHz according to 002-30271_0A*/
#define PLL_800M_MIN_FVCO    (800000000ul)
/** PLL 800M maximum Fvco in MHz according to 002-30271_0A*/
#define PLL_800M_MAX_FVCO   (1600000000ul)

/* TODO check the limits for 800M PLL check 002-30271 */
/* PLL 800M input and output frequency limits */

/** PLL 800M minimum PFD frequency in Hz according to 002-30271_0A*/
#define PLL_800M_MIN_PFD_FREQ         (4000000ul)
/** PLL 800M maximum PFD frequency in Hz according to 002-30271_0A*/
#define PLL_800M_MAX_PFD_FREQ        (20000000ul)
/** PLL 800M minimum Output frequency in Hz according to 002-30271_0A*/
#define PLL_800M_MIN_OUT_FREQ        (50000000ul)
/** PLL 800M minimum Output frequency in Hz according to 002-30271_0A */
#define PLL_800M_MAX_OUT_FREQ      (1600000000ul)

/** TODO: Note: for DDR application no need of Fractional feature this is 
    additional feature for future Cypress requirement)*/
/*#define PLL_800M_FRAC_MIN_PFD_FREQ    (8000000ul)   */

/**
 * As per JEDEC209-4B allowed temperature margin for calculation of the 
 * MR4 Read Interval [°C]
*/
#define JEDEC_DEVICE_TEMPERATURE_MARGIN   2

/**
 * Temperature Sensor Interval as defined to JEDEC209-4B
 * used to calculate the cycle time for reading MR4 and updating 
 * the refresh rate [msec]
*/
#define JESD209_4B_MSEC_TTSI    32

/**
 * used to convert msec in seconds
*/
#define CY_LPDDR4_FACTOR_MSEC_IN_SEC     0.001

/**
 * used to convert sec in msec
*/
#define CY_LPDDR4_FACTOR_SEC_IN_MSEC     1000

/**
 * maximum allowed MR4 read interval as defined in JEDEC209-4B [ms]
 */
#define JESD209_4B_MAX_MR4_READINTERVAL     167 

/** \} group_lpddr_macro */

/***************************************
*       Enumeration
***************************************/
/**
* \addtogroup group_lpddr_enums
* \{
*/

/***************************************
*       ENUMS for the LPDDR4 PLL
***************************************/


/** 
* \enum     cy_en_refreshrate_strategy_change_state_t 
* \brief    enum for driver context field 
*/
typedef enum
{
    CY_LPDDR4_STRATEGY_UNCHANGED    = 0u, /*!< strategy was unchanged   */
    CY_LPDDR4_STRATEGY_CHANGED      = 1u  /*!< strategy was changed and is pending  */
} cy_en_refreshrate_strategy_change_state_t;

/** 
* \enum     cy_en_lpddr4_fsp_t 
* \brief    enum for frequency set point
*/
typedef enum
{
    CY_LPDDR4_FSP0      = 0u, /*!< frequency set point 0<br>PSVP=@80MHZ<br>SOC=@50MHz   */
    CY_LPDDR4_FSP1      = 1u  /*!< frequency set point 1<br>PSVP=@80MHZ<br>SOC=@800 MHz  */
} cy_en_lpddr4_fsp_t;

/** 
 * \enum     cy_en_lpddr4_moncycle_t 
 * \brief    enum for monitoring cycle
 */
typedef enum
{
    CY_LPDDR4_MONITORING_CYCLE      = 0u, /*!< cycle in which driver check the need for retraining  */
    CY_LPDDR4_RETRAIN_CYCLE         = 1u  /*!< cycle in which the retraining is perfromed when needed */
} cy_en_lpddr4_moncycle_t;




/** 
*   \enum   cy_en_lpddr4_vco_clk_sel_t
*   \brief  enums for VCO_CLKSEL PLL800 output multiplexer
*/
typedef enum
{
    CY_LPDDR4_VCOCLKSEL_FVCO_HALF           = 0u,   /*!< vco_clk divide by 2: fOUT=(fREF / REFERENCE_DIV) * FEEDBACK_DIV */
    CY_LPDDR4_VCOCLKSEL_DIRECT_OUTPUT       = 1u,   /*!< vco_clk direct output:fOUT=(fREF / REFERENCE_DIV) * FEEDBACK_DIV * 2 */
    CY_LPDDR4_VCO_CLKSEL_FVCO_HALF_OUT_DIV  = 2u    /*!< fOUT = (fREF / REFERENCE_DIV) * FEEDBACK_DIV / OUTPUT_DIV */
} cy_en_lpddr4_vco_clk_sel_t;

/** 
*   \enum   cy_en_lpddr4_pll_800M_lock_delay_rate_t
*   \brief  enums for LOCK_DELAY in PLL800_CONFIG Register 
*/
typedef enum
{
    CY_LPDDR4_PLL_LOCK_DELAY_INT_WO_SSCG        = 0u, /*!< Integer DIV without Spreading   */
    CY_LPDDR4_PLL_LOCK_DELAY_FRAC_OR_SSCG       = 1u  /*!< Fractional DIV or Spreading  */
} cy_en_lpddr4_pll_800M_lock_delay_rate_t;


/** 
* enums for WAIT_TIME in PLL800_CONFIG Register
* Configures the seqeuencer for starting and stopping the PLL. 
* Set the value depending on the external oscillator frequency before enabling the PLL.
*/
typedef enum
{
    CY_LPDDR4_PLL_WAIT_TIME_0_4MHZ       = 0u, /*!< for 4MHz                */
    CY_LPDDR4_PLL_WAIT_TIME_4_8MHZ       = 1u, /*!< for >4MHz up to 8MHz    */
    CY_LPDDR4_PLL_WAIT_TIME_8_16MHZ      = 2u, /*!< for >8MHz up to 16MHz   */
    CY_LPDDR4_PLL_WAIT_TIME_16_33MHZ     = 3u  /*!< for >16MHz up to 33MHz  */
} cy_en_lpddr4_pll_800M_wait_time_t;



/** 
*   \enum    cy_en_lpddr4_pll_800M_ssgc_depth_t
*   \brief   SSCG modulation depth of PLL 800M spreading mode.
*   See register PLL800_CONFIG2 SSCG_DEPTH 
*/
typedef enum
{
    CY_LPDDR4_PLL_SSCG_DEPTH_NO_SSCG               = 0u,    /*!< - value for none sscg to avoid failure during FCV */
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_0_5             = 41u,   /*!< - 0.5 [%] b0000101001=0x29h   (down spread)  */
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_1_0             = 82u,   /*!< - 1.0 [%] b0001010010=0x52h   (down spread)  */
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_2_0             = 164u,  /*!< - 2.0 [%] b0010100100=0xA4h   (down spread)  */
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_3_0             = 246u,  /*!< - 3.0 [%] b0011110110=0xF6h   (down spread)  */ 
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_4_0             = 328u,  /*!< - 4.0 [%] b0101001000=0x148h  (down spread)  */ 
    CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_5_0             = 410u   /*!< - 5.0 [%] b0110011010=0x19Ah  (down spread)  */ 

} cy_en_lpddr4_pll_800M_ssgc_depth_t;

/** 
*   \enum   cy_en_lpddr4_pll_800M_ssgc_rate_t  
*   \brief  Control bits for SSCG modulation rate as a factor of fPFD clock frequency.   
*   See register PLL800_CONFIG2 SSCG_RATE
*/
typedef enum
{
    CY_LPDDR4_PLL_SSCG_RATE_DIV_4096             = 0u, /*!<  Modulation rate = fPFD/4096  */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_2048             = 1u, /*!<  Modulation rate = fPFD/2048  */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_1024             = 2u, /*!<  Modulation rate = fPFD/1024  */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_512              = 3u, /*!<  Modulation rate = fPFD/512   */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_256              = 4u, /*!<  Modulation rate = fPFD/256   */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_128              = 5u, /*!<  Modulation rate = fPFD/128   */
    CY_LPDDR4_PLL_SSCG_RATE_DIV_744              = 6u  /*!<  Modulation rate = fPFD/744   */
} cy_en_lpddr4_pll_800M_ssgc_rate_t;


/** 
*   \enum   cy_en_lpddr4_pll_sscg_mode_t
*   \brief  Control bits for SSCG modulation rate as a factor of fPFD clock frequency.  
*   See register PLL800_CONFIG3 SSCG_RATE
*/
typedef enum
{
    CY_LPDDR4_PLL_SPREAD_MODE_DOWN = 0,     /*!<  Down Spreading always use down spreading */
    CY_LPDDR4_PLL_SPREAD_MODE_UP   = 1      /*!<  do not use RFU  Up Spreading */
} cy_en_lpddr4_pll_sscg_mode_t;



/***************************************
*       ENUMS for the LPDDR4 Controller
***************************************/

/**
 *  \enum   cy_en_lpddr4_inittype_t
 *  \brief  enum for Controller Init Type     
 */
typedef enum 
{
    CY_LPDDR4_INIT_WITH_TRAINING        = 0x1,  /*!< standard Init with training active */
    CY_LPDDR4_INIT_WITH_RELOAD          = 0x2   /*!< controller init with reloading the training values from PTSR registers */
}cy_en_lpddr4_inittype_t;


/** 
* \enum     cy_en_lpddr4_pom_request_t 
* \brief    enum for POM Register to set PHY requesta
*/
typedef enum
{
    CY_LPDDR4_POM_NOT_REQUESTED   = 0u, /*!< LPDDR4 POM Register not request  */
    CY_LPDDR4_POM_REQUESTED       = 1u  /*!< LPDDR4 POM Register requested  */
} cy_en_lpddr4_pom_request_t;


/** 
* \enum     cy_en_lpddr4_refreshrate_strategy_t 
* \brief    enum for refresh rate strategy. Dynamic when refresh rate is adapted 
*           based on MR4 reading or fixed when the refresh rate is set to the 
*           minimum valkue of 0.25 * tREFI 
*/
typedef enum
{
    CY_LPDDR_REFRESHRATE_DYNAMIC    = 0u, /*!< LPDDR4 POM Register not request */
    CY_LPDDR_REFRESHRATE_FIXED      = 1u  /*!< LPDDR4 POM Register requested  */
} cy_en_lpddr4_refreshrate_strategy_t;

/** 
* \enum     cy_en_deratingstrategy_state_t 
* \brief    enum for driver context timing derating strategy
*/
typedef enum
{
    CY_LPDDR4_DERATING_ALWAYS_ON    = 0u, /*!< timing derating is always active */
    CY_LPDDR4_DERATING_DYNAMIC      = 1u  /*!< derating based on MR4 request */
} cy_en_deratingstrategy_state_t;

/** 
* \enum     cy_en_lpddr4_cntrl_init_state_t 
* \brief    enum for controller state
*/
typedef enum
{
    CY_LPDDR4_CNTRL_NOT_INITIALIZED   = 0u,  /*!< controller not initialized */
    CY_LPDDR4_CNTRL_INITIALIZED       = 1u   /*!< controller is initialized */
    
}cy_en_lpddr4_cntrl_init_state_t;

/** 
* \enum     cy_en_lpddr4_cntrl_run_state_t 
* \brief    enum for controller run state
*/
typedef enum
{
    CY_LPDDR4_CNTRL_STOPPED     = 0u,  /*!< LPDDR4 controller in "STOP" state  */
    CY_LPDDR4_CNTRL_RUNNING     = 1u   /*!< LPDDR4 controller in "RUN" state */
    
} cy_en_lpddr4_cntrl_run_state_t;

/** 
* \enum     cy_en_lpddr4_cntrl_gsm_state_t 
* \brief    enum for controller gsm state
*/
typedef enum
{
    CY_LPDDR4_CNTRL_GSM_STATE_STOP          = 0u,  /*!< LPDDR4 controller in "RUNP" state  */
    CY_LPDDR4_CNTRL_GSM_STATE_RUN           = 1u,  /*!< LPDDR4 controller in "STRUN" state */
    CY_LPDDR4_CNTRL_GSM_STATE_SELFREFRESH   = 2u,  /*!< LPDDR4 controller in "STRUN" state */
    CY_LPDDR4_CNTRL_GSM_STATE_POWERDOWN     = 3u  /*!< LPDDR4 controller in "STRUN" state */
}cy_en_lpddr4_cntrl_gsm_state_t;

/** 
* \enum     cy_en_lpddr4_memory_operational_temperature_t 
* \brief    enum memory operational temperature mode
*/
typedef enum
{
    CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK            = 0u,   /*!< Temperature within normal limits  */
    CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_LOW    = 1u,   /*!< Temperature is below lower limit  */
    CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_HIGH   = 2u    /*!< Temperature is above upper limit  */
} cy_en_lpddr4_memory_operational_temperature_t;

/** 
* \enum     cy_en_lpddr4_refresh_adjust_t 
* \brief    enum for refresh adjustment type if done during controller init or when controller is already running.
*/
typedef enum
{
    CY_LPDDR4_REFRESHADJUST_INIT    = 0u, /*!< refresh adjustment is done during controller init */
    CY_LPDDR4_REFRESHADJUST_RUN     = 1u  /*!< refresh adjustment is done when cntrl is running ->controlller  must be stopped*/
} cy_en_lpddr4_refresh_adjust_t;


/** 
* \enum     cy_en_lpddr4_MR4_derate_state_t 
* \brief    enum for MR4 temperature depending timimg derating activation state 
*/
typedef enum
{
    CY_LPDDR4_DERATING_NOT_ACTIVE   = 0u, /*!<  temperature depending timing derating not active */
    CY_LPDDR4_DERATING_ACTIVE       = 1u  /*!<  temperature depending timing derating active */
} cy_en_lpddr4_MR4_derate_state_t;



/** 
* \enum     cy_en_lpddr4_route_ooo_state_t 
* \brief    enum for the out of order setting of a controller route
* <table class="doxtable">
*   <tr><th>rtcfg0_rtX_arq/awq_ooo_en</th><th>rtcfg0_rtX_arq/awq_lahead_en </th><th>Description</th></tr>
*   <tr> <td>0</td> <td>x</td> <td>Disable Out-of-Order </td> </tr> 
*   <tr> <td>1</td> <td>0</td> <td>Enable Level 1, Disable Level 2 </td></tr>
*   <tr> <td>1</td> <td>1</td> <td>Enable Level 1 and 2  </td></tr>
* </table>  
*
*/
typedef enum
{
    CY_LPDDR4_OUT_OF_ORDER_DISABLED  = 0u, /*!<  Out of order disabled */
    CY_LPDDR4_OUT_OF_ORDER_LEVEL_1   = 1u,  /*!< Out of Order Level1 active */
    CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2   = 2u,  /*!<  Out of Order Level1 and Level2 active */
} cy_en_lpddr4_route_ooo_state_t;

/** 
* \enum     cy_en_lpddr4_axiaddrbit_t 
* \brief    enum for address mapping
*/
typedef enum
{
    CY_LPDDR4_AXI_BIT_00        = 0,   /*!< axi bit 00 */
    CY_LPDDR4_AXI_BIT_01        = 1,   /*!< axi bit 01 */
    CY_LPDDR4_AXI_BIT_02        = 2,   /*!< axi bit 02 */
    CY_LPDDR4_AXI_BIT_03        = 3,   /*!< axi bit 03 */
    CY_LPDDR4_AXI_BIT_04        = 4,   /*!< axi bit 04 */
    CY_LPDDR4_AXI_BIT_05        = 5,   /*!< axi bit 05 */
    CY_LPDDR4_AXI_BIT_06        = 6,   /*!< axi bit 06 */
    CY_LPDDR4_AXI_BIT_07        = 7,   /*!< axi bit 07 */
    CY_LPDDR4_AXI_BIT_08        = 8,   /*!< axi bit 08 */
    CY_LPDDR4_AXI_BIT_09        = 9,   /*!< axi bit 09 */
    CY_LPDDR4_AXI_BIT_10        = 10,  /*!< axi bit 10 */
    CY_LPDDR4_AXI_BIT_11        = 11,  /*!< axi bit 11 */
    CY_LPDDR4_AXI_BIT_12        = 12,  /*!< axi bit 12 */
    CY_LPDDR4_AXI_BIT_13        = 13,  /*!< axi bit 13 */
    CY_LPDDR4_AXI_BIT_14        = 14,  /*!< axi bit 14 */
    CY_LPDDR4_AXI_BIT_15        = 15,  /*!< axi bit 15 */
    CY_LPDDR4_AXI_BIT_16        = 16,  /*!< axi bit 16 */
    CY_LPDDR4_AXI_BIT_17        = 17,  /*!< axi bit 17 */
    CY_LPDDR4_AXI_BIT_18        = 18,  /*!< axi bit 18 */
    CY_LPDDR4_AXI_BIT_19        = 19,  /*!< axi bit 19 */
    CY_LPDDR4_AXI_BIT_20        = 20,  /*!< axi bit 20 */
    CY_LPDDR4_AXI_BIT_21        = 21,  /*!< axi bit 21 */
    CY_LPDDR4_AXI_BIT_22        = 22,  /*!< axi bit 22 */
    CY_LPDDR4_AXI_BIT_23        = 23,  /*!< axi bit 23 */
    CY_LPDDR4_AXI_BIT_24        = 24,  /*!< axi bit 24 */
    CY_LPDDR4_AXI_BIT_25        = 25,  /*!< axi bit 25 */
    CY_LPDDR4_AXI_BIT_26        = 26,  /*!< axi bit 26 */
    CY_LPDDR4_AXI_BIT_27        = 27,  /*!< axi bit 27 */
    CY_LPDDR4_AXI_BIT_28        = 28,  /*!< axi bit 28 */
    CY_LPDDR4_AXI_BIT_29        = 29,  /*!< axi bit 29 */
    CY_LPDDR4_AXI_BIT_30        = 30,  /*!< axi bit 30 */
    CY_LPDDR4_AXI_BIT_31        = 31   /*!< axi bit 31 */
} cy_en_lpddr4_axiaddrbit_t;

/**
 *  \enum   cy_en_lpddr4_channel_status_t
 *  \brief  enums for Command channel status*/
typedef enum 
{
    CY_LPDDR4_CHANNEL_READY = 0x0,          /*!< Channel is ready to receive commands */
    CY_LPDDR4_CHANNEL_TIMEOUT = 0x1         /*!< Channel is still busy -> busy timeout */

} cy_en_lpddr4_channel_status_t;

/**
 *  \enum   cy_en_lpddr4_mr_t
 *  \brief  enum for Mode Register opcode 
 */
typedef enum 
{
    CY_LPDDR4_NONE_MR           = 0x0,      /*!< OP Code when no MR Register is needed in user command, as for example Start DSQ Oscillator*/
    CY_LPDDR4_MODEREGISTER_MR0  = 0x0,      /*!< OP code for MR0 register */
    CY_LPDDR4_MODEREGISTER_MR1  = 0x1,      /*!< OP code for MR1 register */
    CY_LPDDR4_MODEREGISTER_MR2  = 0x2,      /*!< OP code for MR2 register */
    CY_LPDDR4_MODEREGISTER_MR3  = 0x3,      /*!< OP code for MR3 register */
    CY_LPDDR4_MODEREGISTER_MR4  = 0x4,      /*!< OP code for MR4 register */
    CY_LPDDR4_MODEREGISTER_MR5  = 0x5,      /*!< OP code for MR5 register - LPDDR4 Manufacturer ID */
    CY_LPDDR4_MODEREGISTER_MR6  = 0x6,      /*!< OP code for MR6 register - Revision ID-1 */
    CY_LPDDR4_MODEREGISTER_MR7  = 0x7,      /*!< OP code for MR7 register - Revision ID-2 */
    CY_LPDDR4_MODEREGISTER_MR8  = 0x8,      /*!< OP code for MR8 register */
    CY_LPDDR4_MODEREGISTER_MR9  = 0x9,      /*!< OP code for MR8 register */
    CY_LPDDR4_MODEREGISTER_MR10  = 0xA,      /*!< OP code for MR8 register */
    CY_LPDDR4_MODEREGISTER_MR11  = 0xB,      /*!< OP code for MR8 register */
    CY_LPDDR4_MODEREGISTER_MR12  = 0xc,      /*!< OP code for MR12 register - */
    CY_LPDDR4_MODEREGISTER_MR13  = 0xd,      /*!< OP code for MR13 register - */
    CY_LPDDR4_MODEREGISTER_MR14  = 0xe,      /*!< OP code for MR14 register - */
    CY_LPDDR4_MODEREGISTER_MR15  = 0xf,      /*!< OP code for MR15 register Lower Byte Invert Register for DQ Calibration */
    CY_LPDDR4_MODEREGISTER_MR16  = 0x10,      /*!< OP code for MR16 register - PASR Mask */
    CY_LPDDR4_MODEREGISTER_MR17  = 0x11,      /*!< OP code for MR17 register - PASR Segment Mask */
    CY_LPDDR4_MODEREGISTER_MR18  = 0x12,     /*!< OP code for MR18 register - DQS Oscillator Count LSB */
    CY_LPDDR4_MODEREGISTER_MR19  = 0x13,     /*!< OP code for MR19 register   - DQS Oscillator Count MSB */
    CY_LPDDR4_MODEREGISTER_MR20  = 0x14,      /*!< OP code for MR20 register - Upper Byte for Invert Register for DQ Calibraton */
    CY_LPDDR4_MODEREGISTER_MR22  = 0x16,    /*!< OP code for MR22 register */
    CY_LPDDR4_MODEREGISTER_MR23  = 0x17,     /*!< OP code for MR23 register - DQS interval timer run time setting */
    CY_LPDDR4_MODEREGISTER_MR24  = 0x18,     /*!< OP code for MR24 register */
    CY_LPDDR4_MODEREGISTER_MR25  = 0x19,      /*!< OP code for MR25 register - PPR Resource*/
    CY_LPDDR4_MODEREGISTER_MR30  = 0x1E,     /*!< OP code for MR30 register - Reserved for testing - SDRAM will ignore */    
    CY_LPDDR4_MODEREGISTER_MR32  = 0x20,     /*!< OP code for MR32 register - DQ Calibration Pattern “A” (default = 5AH) */
    CY_LPDDR4_MODEREGISTER_MR39  = 0x27,     /*!< OP code for MR39 register - Reserved for testing - SDRAM will ignore */
    CY_LPDDR4_MODEREGISTER_MR40  = 0x28      /*!< OP code for MR40 register - DQ Calibration Pattern “B” (default = 3CH) */
} cy_en_lpddr4_mr_t; 

/***************************************
*       ENUMS for QoS Functionality
***************************************/

/**
 *  \enum   cy_en_lpddr4_qos_prio_t
 *  \brief  enum for QoS ( Quality of Servive ) Master ID
 */
typedef enum 
{
    CY_LPDDR4_QOS_PRIORITY_LOW       = 0x0,      /*!< Quality of Service ( QoS ) Priority  - Low  */
    CY_LPDDR4_QOS_PRIORITY_MEDIUM    = 0x1,      /*!< Quality of Service ( QoS ) Priority  - Medium */
    CY_LPDDR4_QOS_PRIORITY_HIGH      = 0x3       /*!< Quality of Service ( QoS ) Priority  - Higher */
} cy_en_lpddr4_qos_priority_t; 

 /*
 * Start of Mode Register 4 enums
 */

/**
 *  \enum   cy_en_lpddr4_mr4_refresh_rate_t
 *  \brief  Mode Register 4 - enum for Refresh rate
 */
typedef enum 
{
    CY_LPDDR4_MR4_REFRESH_RATE_LOW_TEMP_LIMIT = 0,              /*!< 000b: SDRAM low temperature operating limit exceeded  */
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_4 = 1,                     /*!< 001b: 4x refresh  (check if enabled in MR13 RRO Refresh rate option) */  
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_2 = 2,                     /*!< 010b: 2x refresh  (check if enabled in MR13 RRO Refresh rate option) */
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_1 = 3,                     /*!< 011b: 1x refresh (default)  */
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P5 = 4,                   /*!< 100b: 0.5x refresh  */
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_NO_DERATING = 5,      /*!< 101b: 0.25x refresh, no derating  */
    CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_WITH_DERATING = 6,    /*!< 110b: 0.25x refresh, with derating  */
    CY_LPDDR4_MR4_REFRESH_RATE_HIGH_TEMP_LIMIT = 7              /*!< 111b: SDRAM high temperature operating limit exceeded  */  
}cy_en_lpddr4_mr4_refresh_rate_t;

/**
 *  \enum   cy_en_lpddr4_refresh_rate_t
 *  \brief  enum for Refresh rate
 */
typedef enum 
{
    CY_LPDDR4_REFRESH_RATE_TIMES_4      = 1,    /*!< 4x refresh   */  
    CY_LPDDR4_REFRESH_RATE_TIMES_2      = 2,    /*!< 2x refresh   */
    CY_LPDDR4_REFRESH_RATE_TIMES_1      = 3,    /*!< 1x refresh (default)  */
    CY_LPDDR4_REFRESH_RATE_TIMES_0P5    = 4,    /*!< 0.5x refresh  */
    CY_LPDDR4_REFRESH_RATE_TIMES_0P25   = 5,    /*!< 0.25x refresh */
    CY_LPDDR4_REFRESH_RATE_USER_FIXED   = 6     /*!< user specified fix rate*/
}cy_en_lpddr4_refresh_rate_t;

/**
 *  \enum   cy_en_lpddr4_mr4_sr_abort_t
 *  \brief  Mode Register 4 - SR abort (Self refresh abort) 
 */
typedef enum 
{
    CY_LPDDR4_MR4_SR_ABORT_DISABLE = 0,     /*!< 0b: Disable (default)   */
    CY_LPDDR4_MR4_SR_ABORT_ENABLE  = 1      /*!< 1b: Device dependent  */
}cy_en_lpddr4_mr4_sr_abort_t;

/**
 *  \enum   cy_en_lpddr4_mr4_ppre_t
 *  \brief  Mode Register 4 - PPRE (Post-package repair entry/ exit) 
 */
typedef enum 
{
    CY_LPDDR4_MR4_PPRE_MODE_EXIT  = 0,     /*!< 0b: Exit PPR mode (default)   */
    CY_LPDDR4_MR4_PPRE_MODE_ENTER = 1      /*!< 1b: Enter PPR mode (Reference MR25 OP[7:0] for available PPR resources)*/
}cy_en_lpddr4_mr4_ppre_t;

/**
 *  \enum   cy_en_lpddr4_mr4_tcsr_offset_t
 *  \brief  Mode Register 4 - Thermal offset-controller offset to TCSR 
 *          TCSR (Temperature Compensated Self-Refresh)
 */
typedef enum 
{
    CY_LPDDR4_MR4_TCSR_NO_OFFSET = 0,       /*!< 00b: No offset, 0~5°C gradient (default)  */  
    CY_LPDDR4_MR4_TCSR_5_DEG_OFFSET = 1,    /*!< 01b: 5°C offset, 5~10°C gradient  */  
    CY_LPDDR4_MR4_TCSR_10_DEG_OFFSET = 2,   /*!< 10b: 10°C offset, 10~15°C gradient  */
    CY_LPDDR4_MR4_TCSR_RESERVED = 3         /*!< 11b: Reserved  */
}cy_en_lpddr4_mr4_tcsr_offset_t;

/**
 *  \enum   cy_en_lpddr4_mr4_tuf_t
 *  \brief  Mode Register 4 - TUF (Temperature update flag)
 */
typedef enum 
{
    CY_LPDDR4_MR4_TUF_NO_UPDATE = 0,    /*!< 0b: OP[2:0] No change in OP[2:0] since last MR4 read (default)  */
    CY_LPDDR4_MR4_TUF_UPDATED = 1       /*!< 1b: Change in OP[2:0] since last MR4 read  */
}cy_en_lpddr4_mr4_tuf_t;

/**
*  \enum   cy_en_lpddr4_mr23_dqsosc_runtime_t
*  \brief  Mode Register 23 - DQS interval  timer run time 
*  <table class="doxtable">
*   <tr><th>Function </th><th>Type</th><th>Operand</th><th>Data</th></tr>
*   <tr>
*     <td>DQS interval <br>timer run time </td>
*     <td>WO</td>
*     <td>OP[7:0]</td>
*     <td>  8’b00000000: DQS interval timer stop via MPC Command (Default) <br>
*           8’b00000001: DQS timer stops automatically at 16 th  clocks after timer starts  <br>
*           8’b00000010: DQS timer stops automatically at 32 nd  clocks after timer starts  <br>
*           8’b00000011: DQS timer stops automatically at 48 th  clocks after timer starts  <br>
*           8’b00000100: DQS timer stops automatically at 64 th  clocks after timer starts  <br>
*           -------------- Thru --------------  <br>
*           8’b00111111: DQS timer stops automatically at (63x16) th  clocks after timer starts  <br>
*           8’b01xxxxxx: DQS timer stops automatically at 2048 th  clocks after timer starts  <br>
*           8’b10xxxxxx: DQS timer stops automatically at 4096 th  clocks after timer starts  <br>
*           8’b11xxxxxx: DQS timer stops automatically at 8192 nd  clocks after timer starts </td>
*   </tr>
* </table>  
*/
typedef enum 
{
    CY_LPDDR4_MR23_STOP_BY_MPC_COMMAND  = 0 ,   /*!< (default) DQS interval timer stop via MPC Command (Default) */
    CY_LPDDR4_MR23_1X16CLK_COUNTS       = 1 ,   /*!< DQS timer stops automatically at  1 x 16 th  clocks after timer starts */
    CY_LPDDR4_MR23_2X16CLK_COUNTS       = 2 ,   /*!< DQS timer stops automatically at  2  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_3X16CLK_COUNTS       = 3 ,   /*!< DQS timer stops automatically at  3  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_4X16CLK_COUNTS       = 4 ,   /*!< DQS timer stops automatically at  4  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_5X16CLK_COUNTS       = 5 ,   /*!< DQS timer stops automatically at  5  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_6X16CLK_COUNTS       = 6 ,   /*!< DQS timer stops automatically at  6  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_7X16CLK_COUNTS       = 7 ,   /*!< DQS timer stops automatically at  7  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_8X16CLK_COUNTS       = 8 ,   /*!< DQS timer stops automatically at  8  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_9X16CLK_COUNTS       = 9 ,   /*!< DQS timer stops automatically at  9  x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_10X16CLK_COUNTS      = 10,   /*!< DQS timer stops automatically at  10 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_11X16CLK_COUNTS      = 11,   /*!< DQS timer stops automatically at  11 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_12X16CLK_COUNTS      = 12,   /*!< DQS timer stops automatically at  12 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_13X16CLK_COUNTS      = 13,   /*!< DQS timer stops automatically at  13 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_14X16CLK_COUNTS      = 14,   /*!< DQS timer stops automatically at  14 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_15X16CLK_COUNTS      = 15,   /*!< DQS timer stops automatically at  15 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_16X16CLK_COUNTS      = 16,   /*!< DQS timer stops automatically at  16 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_17X16CLK_COUNTS      = 17,   /*!< DQS timer stops automatically at  17 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_18X16CLK_COUNTS      = 18,   /*!< DQS timer stops automatically at  18 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_19X16CLK_COUNTS      = 19,   /*!< DQS timer stops automatically at  19 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_20X16CLK_COUNTS      = 20,   /*!< DQS timer stops automatically at  20 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_21X16CLK_COUNTS      = 21,   /*!< DQS timer stops automatically at  21 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_22X16CLK_COUNTS      = 22,   /*!< DQS timer stops automatically at  22 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_23X16CLK_COUNTS      = 23,   /*!< DQS timer stops automatically at  23 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_24X16CLK_COUNTS      = 24,   /*!< DQS timer stops automatically at  24 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_25X16CLK_COUNTS      = 25,   /*!< DQS timer stops automatically at  25 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_26X16CLK_COUNTS      = 26,   /*!< DQS timer stops automatically at  26 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_27X16CLK_COUNTS      = 27,   /*!< DQS timer stops automatically at  27 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_28X16CLK_COUNTS      = 28,   /*!< DQS timer stops automatically at  28 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_29X16CLK_COUNTS      = 29,   /*!< DQS timer stops automatically at  29 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_30X16CLK_COUNTS      = 30,   /*!< DQS timer stops automatically at  30 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_31X16CLK_COUNTS      = 31,   /*!< DQS timer stops automatically at  31 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_32X16CLK_COUNTS      = 32,   /*!< DQS timer stops automatically at  32 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_33X16CLK_COUNTS      = 33,   /*!< DQS timer stops automatically at  33 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_34X16CLK_COUNTS      = 34,   /*!< DQS timer stops automatically at  34 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_35X16CLK_COUNTS      = 35,   /*!< DQS timer stops automatically at  35 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_36X16CLK_COUNTS      = 36,   /*!< DQS timer stops automatically at  36 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_37X16CLK_COUNTS      = 37,   /*!< DQS timer stops automatically at  37 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_38X16CLK_COUNTS      = 38,   /*!< DQS timer stops automatically at  38 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_39X16CLK_COUNTS      = 39,   /*!< DQS timer stops automatically at  39 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_40X16CLK_COUNTS      = 40,   /*!< DQS timer stops automatically at  40 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_41X16CLK_COUNTS      = 41,   /*!< DQS timer stops automatically at  41 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_42X16CLK_COUNTS      = 42,   /*!< DQS timer stops automatically at  42 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_43X16CLK_COUNTS      = 43,   /*!< DQS timer stops automatically at  43 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_44X16CLK_COUNTS      = 44,   /*!< DQS timer stops automatically at  44 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_45X16CLK_COUNTS      = 45,   /*!< DQS timer stops automatically at  45 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_46X16CLK_COUNTS      = 46,   /*!< DQS timer stops automatically at  46 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_47X16CLK_COUNTS      = 47,   /*!< DQS timer stops automatically at  47 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_48X16CLK_COUNTS      = 48,   /*!< DQS timer stops automatically at  48 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_49X16CLK_COUNTS      = 49,   /*!< DQS timer stops automatically at  49 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_50X16CLK_COUNTS      = 50,   /*!< DQS timer stops automatically at  50 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_51X16CLK_COUNTS      = 51,   /*!< DQS timer stops automatically at  51 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_52X16CLK_COUNTS      = 52,   /*!< DQS timer stops automatically at  52 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_53X16CLK_COUNTS      = 53,   /*!< DQS timer stops automatically at  53 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_54X16CLK_COUNTS      = 54,   /*!< DQS timer stops automatically at  54 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_55X16CLK_COUNTS      = 55,   /*!< DQS timer stops automatically at  55 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_56X16CLK_COUNTS      = 56,   /*!< DQS timer stops automatically at  56 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_57X16CLK_COUNTS      = 57,   /*!< DQS timer stops automatically at  57 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_58X16CLK_COUNTS      = 58,   /*!< DQS timer stops automatically at  58 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_59X16CLK_COUNTS      = 59,   /*!< DQS timer stops automatically at  59 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_60X16CLK_COUNTS      = 60,   /*!< DQS timer stops automatically at  60 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_61X16CLK_COUNTS      = 61,   /*!< DQS timer stops automatically at  61 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_62X16CLK_COUNTS      = 62,   /*!< DQS timer stops automatically at  62 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_63X16CLK_COUNTS      = 63,   /*!< DQS timer stops automatically at  63 x 16 th  clocks after timer starts  */
    CY_LPDDR4_MR23_2048CLK_COUNTS       = 64,   /*!< DQS timer stops automatically at  2048 th  clocks after timer starts  */
    CY_LPDDR4_MR23_4096CLK_COUNTS       = 128,  /*!< DQS timer stops automatically at  4096 th  clocks after timer starts  */
    CY_LPDDR4_MR23_8192CLK_COUNTS       = 192   /*!< DQS timer stops automatically at  8192 th  clocks after timer starts  */
}cy_en_lpddr4_mr23_dqsosc_runtime_t;


/**
 *  \enum   cy_en_lpddr4_ecc_syndrom_t
 *  \brief  defines ECC syndrom type. ECC error bitposition  information. 
 */
typedef enum 
{
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_0     =   0xA4,    /*!< ECC error at Bit Position 0 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_1     =   0xC4,    /*!< ECC error at Bit Position 1 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_2     =   0xC2,    /*!< ECC error at Bit Position 2 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_3     =   0xA2,    /*!< ECC error at Bit Position 3 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_4     =   0x9E,    /*!< ECC error at Bit Position 4 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_5     =   0xC1,    /*!< ECC error at Bit Position 5 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_6     =   0xA1,    /*!< ECC error at Bit Position 6 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_7     =   0x91,    /*!< ECC error at Bit Position 7 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_8     =   0x52,    /*!< ECC error at Bit Position 8 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_9     =   0x62,    /*!< ECC error at Bit Position 9 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_10    =   0x61,    /*!< ECC error at Bit Position 10 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_11    =   0x51,    /*!< ECC error at Bit Position 11 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_12    =   0x4F,    /*!< ECC error at Bit Position 12 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_13    =   0xE0,    /*!< ECC error at Bit Position 13 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_14    =   0xD0,    /*!< ECC error at Bit Position 14 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_15    =   0xC8,    /*!< ECC error at Bit Position 15 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_16    =   0x29,    /*!< ECC error at Bit Position 16 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_17    =   0x31,    /*!< ECC error at Bit Position 17 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_18    =   0xB0,    /*!< ECC error at Bit Position 18 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_19    =   0xA8,    /*!< ECC error at Bit Position 19 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_20    =   0xA7,    /*!< ECC error at Bit Position 20 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_21    =   0x70,    /*!< ECC error at Bit Position 21 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_22    =   0x68,    /*!< ECC error at Bit Position 22 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_23    =   0x64,    /*!< ECC error at Bit Position 23 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_24    =   0x94,    /*!< ECC error at Bit Position 24 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_25    =   0x98,    /*!< ECC error at Bit Position 25 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_26    =   0x58,    /*!< ECC error at Bit Position 26 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_27    =   0x54,    /*!< ECC error at Bit Position 27 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_28    =   0xD3,    /*!< ECC error at Bit Position 28 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_29    =   0x38,    /*!< ECC error at Bit Position 29 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_30    =   0x34,    /*!< ECC error at Bit Position 30 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_31    =   0x32,    /*!< ECC error at Bit Position 31 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_32    =   0x4A,    /*!< ECC error at Bit Position 32 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_33    =   0x4C,    /*!< ECC error at Bit Position 33 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_34    =   0x2C,    /*!< ECC error at Bit Position 34 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_35    =   0x2A,    /*!< ECC error at Bit Position 35 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_36    =   0xE9,    /*!< ECC error at Bit Position 36 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_37    =   0x1C,    /*!< ECC error at Bit Position 37 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_38    =   0x1A,    /*!< ECC error at Bit Position 38 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_39    =   0x19,    /*!< ECC error at Bit Position 39 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_40    =   0x25,    /*!< ECC error at Bit Position 40 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_41    =   0x26,    /*!< ECC error at Bit Position 41 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_42    =   0x16,    /*!< ECC error at Bit Position 42 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_43    =   0x15,    /*!< ECC error at Bit Position 43 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_44    =   0xF4,    /*!< ECC error at Bit Position 44 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_45    =   0x0E,    /*!< ECC error at Bit Position 45 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_46    =   0x0D,    /*!< ECC error at Bit Position 46 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_47    =   0x8C,    /*!< ECC error at Bit Position 47 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_48    =   0x92,    /*!< ECC error at Bit Position 48 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_49    =   0x13,    /*!< ECC error at Bit Position 49 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_50    =   0x0B,    /*!< ECC error at Bit Position 50 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_51    =   0x8A,    /*!< ECC error at Bit Position 51 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_52    =   0x7A,    /*!< ECC error at Bit Position 52 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_53    =   0x07,    /*!< ECC error at Bit Position 53 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_54    =   0x86,    /*!< ECC error at Bit Position 54 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_55    =   0x46,    /*!< ECC error at Bit Position 55 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_56    =   0x49,    /*!< ECC error at Bit Position 56 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_57    =   0x89,    /*!< ECC error at Bit Position 57 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_58    =   0x85,    /*!< ECC error at Bit Position 58 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_59    =   0x45,    /*!< ECC error at Bit Position 59 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_60    =   0x3D,    /*!< ECC error at Bit Position 60 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_61    =   0x83,    /*!< ECC error at Bit Position 61 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_62    =   0x43,    /*!< ECC error at Bit Position 62 */
    CY_LPDDR_ECC_SYNDROM_DATA_BIT_63    =   0x23,    /*!< ECC error at Bit Position 63 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_0      =   0x80,    /*!< ECC error at Bit Position 0 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_1      =   0x40,    /*!< ECC error at Bit Position 1 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_2      =   0x20,    /*!< ECC error at Bit Position 2 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_3      =   0x10,    /*!< ECC error at Bit Position 3 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_4      =   0x08,    /*!< ECC error at Bit Position 4 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_5      =   0x04,    /*!< ECC error at Bit Position 5 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_6      =   0x02,    /*!< ECC error at Bit Position 6 */
    CY_LPDDR_ECC_SYNDROM_ECC_BIT_7      =   0x01    /*!< ECC error at Bit Position 7 */

} cy_en_lpddr4_ecc_syndrom_t;  

/**
 *  \enum   cy_en_lpddr4_retval_t
 *  \brief  defines general-purpose function return values. 
 */
typedef enum 
{
    CY_LPDDR4_SUCCESS       = 0u,   /*!< Command completed with no errors */
    CY_LPDDR4_BAD_PARAM     = 1u,   /*!< Invalid function input parameter */
    CY_LPDDR4_TIMEOUT       = 2u,   /*!< Timeout occurred */
    CY_LPDDR4_INVALID_STATE = 3u,   /*!< invalid state */
    CY_LPDDR4_ERROR         = 4u,   /*!< error occurred */
} cy_en_lpddr4_retval_t;  

/**
* \addtogroup group_lpddr_data_structures
* \{
*/

/** 
 * \brief structure for LPDDR4 PLL800 limits
 */
typedef struct 
{
    uint32_t  maxFpd;           /*!< Maximum value of PLL Fpd  */
    uint32_t  minFpd;           /*!< Minimum value of PLL Fpd  */
    uint32_t  maxFvco;          /*!< Maximum value of PLL Fvco  */
    uint32_t  minFvco;          /*!< Minimum value of PLL Fvco  */
    uint32_t  maxFoutput;       /*!< Maximum value of PLL Out put frequency  */
    uint32_t  minFoutput;       /*!< Minimum value of PLL Out put frequency  */
    uint32_t  maxRefDiv;        /*!< Maximum value of reference divider */
    uint32_t  minRefDiv;        /*!< Minimum value of reference divider */
    uint32_t  maxFeedBackDiv;   /*!< Maximum value of feedback divider */
    uint32_t  minFeedBackDiv;   /*!< Minimum value of feedback divider */
    uint32_t  maxOutputDiv;     /*!< Maximum value of output divider */
    uint32_t  minOutputDiv;     /*!< Minimum value of output divider */
} cy_stc_lpddr4_pll_limitation_t;


/** 
 * \brief Structure containing information for manually  configuring the PLL800
 * it contains the output divider/vcoclksel settings for both frequency set points 
 * FSP[0] and FSP[1] 
 **/
typedef struct
{
    uint32_t                                feedbackDiv;    /*!< FEEDBACK_DIV (P) bits */
    uint32_t                                referenceDiv;   /*!< REFERENCE_DIV (Q) bits */
    uint32_t                                outputDiv[2];   /*!< OUTPUT_DIV bits */
    cy_en_lpddr4_vco_clk_sel_t              vcoclksel[2];   /*!< PLL800 Output Multiplexer setting,select PLL output clock.*/
    cy_en_lpddr4_pll_800M_lock_delay_rate_t lockdelay;      /*!<  */
    cy_en_lpddr4_pll_800M_wait_time_t       waittime;       /*!<  */
    bool                                    sscgEn;         /*!< Enable spreading mode or disable it */
    cy_en_lpddr4_pll_800M_ssgc_depth_t      sscgDepth;      /*!< SSCG modulation depth  */
    cy_en_lpddr4_pll_800M_ssgc_rate_t       sscgRate;       /*!< SSCG modulation rate  */
    cy_en_lpddr4_pll_sscg_mode_t            sscgMode;       /*!< write always 0 down spreading   */
} cy_stc_lpddr4_pll800_manual_config_t;


/**
 * \brief structure for DRAM Mode Register 4
 */
typedef struct cy_stc_LPDDR4_DRAM_MR4_field {
    cy_en_lpddr4_mr4_refresh_rate_t     refreshRate:3;              /*!< Refresh Rate */
    cy_en_lpddr4_mr4_sr_abort_t         selrefreshAbort:1;          /*!< Self Refresh Abort */
    cy_en_lpddr4_mr4_ppre_t             postPackageRepair:1;        /*!< Post Package Repair */
    cy_en_lpddr4_mr4_tcsr_offset_t      thermalOffset:2;            /*!< Thermal Offset */
    cy_en_lpddr4_mr4_tuf_t              temperatureUpateFlag:1;     /*!< Temperature Update Flag */
}cy_stc_LPDDR4_DRAM_MR4_field_t;

/**
 *  \brief union DRAM Mode Register 4 
 */
typedef union un_dram_mr4 
{
  cy_stc_LPDDR4_DRAM_MR4_field_t stcField;     /*!< stcField for Bietfield access */
  uint8_t                    byteValue;     /*!< byteValue for byte wise access */
} un_dram_mr4_t;

/** \brief struct for the mode register data for or from a MRR or MRW Cmd*/
typedef struct 
{

    uint8_t  mrw_CHA_Data;           /*!< MR data of channel A */
    uint8_t  mrw_CHB_Data;           /*!< MR data of channel B */

} cy_stc_lpddr4_mrw_mrr_data_t;


/** \brief struct to catch DQS Counters from Cy_Lpddr_ReadDQSOscValue*/
typedef struct 
{
    uint32_t  value_CHA;           /*!< DQS Counter of CHA */
    uint32_t  value_CHB;           /*!< DQS Counter of CHB */
} cy_stc_lpddr4_dqsosc_read_data_t;

/** 
 * \brief Configuration structure of a LPDDR controller
 * 
 * This structure contains values for all the registers which need to be 
 * programmed during controller init to get the LPDDR4 controller up an running
 * 
 * All the registercan be grouped as following
 * 
 * \ref group_lpddr_config_struct_treg <br>
 * \ref group_lpddr_config_struct_lpmr <br>
 * \ref group_lpddr_config_struct_ptsr <br>
 * \ref group_lpddr_config_struct_rtcfg <br>
 * \ref group_lpddr_config_struct_addr <br>
 * \ref group_lpddr_config_struct_dllctlca <br>
 * \ref group_lpddr_config_struct_dllctldq <br>
 * 
 * 
 *  
 * \subsection group_lpddr_config_struct_treg Timing Registers
 * To avoid timing problems all timing Registers must be programmed with the 
 * corresonding JEDEC209-4B parameters as specified in Register TRM. 
 * For all timing registers the value programmed are clock cycles. The clock 
 * to calculate the clock cycles is FSP[1] with exception for the following 
 * parameters where FSP[0] must be used to calculate the clock cycles.  
 *  
 * Signals which must be initialized using FSP[0] frequency.
 * <ul>
 * <li>
 *      TREG7.stcField.u14T_INIT1 <br>
 * </li>
 * <li>
 *      TREG12.stcFieldu17T_INIT3 <br>
 * </li>
 * <li>
 *      TREG11.stcField.u7T_INIT5 <br>
 * </li>
 * <li>
 *      TREG0.stcField.u5T_CKEHDQS <br>
 * </li>
 * <li>
 *      TREG5.stcField.u2T_CKCKEH <br>
 * </li>
 * <li>
 *      TREG7.stcField.u4T_CKFSPE <br>
 * </li>
 * <li>
 *      TREG7.stcField.u4T_CKELCK <br>
 * </li>
 * <li>
 *      TREG11.stcField.u9T_CAENT <br>
 * </li>
 * 
 * 
 * 
 * \subsection group_lpddr_config_struct_lpmr Mode Registers
 * Before the LPDDR4 can be used with full bandwith the controller must be 
 * trained for this frequency. By default the controller and also the meomry are
 * low speed mode f<55MHz ( FSP[0 Frequency Setpoint 0]). During the Training 
 * the controller needs to switch between FSP[0] and FSP]1] f>55 MHz several times. 
 * In the external device the Parameters needed for FSP[0] and FSP[1] are 
 * located in 2 separate sets of mode registers. Switching between the different
 * FSP mode register sets can de done by programming corresponding bitfiled in 
 * mode registers MR13. In the memory controller the mode register settings are
 * combined in one single LPMRx register. To guarantee a defined startup point 
 * the MR / LPMR settings for FSP[0] are defined by JEDEC209-4B. So independant
 * on the used device the setting for FSP[0] should be equal and initilaiztion 
 * should be possible
 * 
 * 
 * \subsection group_lpddr_config_struct_ptsr PHY Training Setting Register
 * The PTSR ( PHY Training Setting Registers) PSTR0..25. These registers have 2 
 * functions. 
 * <ul>
 * <li> storing the PHY training data after a succefull training </li>
 * <li> registers to reload valid PHY Training data back to the PHY.
 *      e.g. after a powerdown. So software has to store the latest valid values 
 *      before entering power down mode. When repowering these values can be 
 *      reloaded by programming the PTSR registers and trigger a reload function
 *  </li>
 * </ul>
 * \subsection group_lpddr_config_struct_rtcfg Route Configuration Register
 * 
 * \subsection group_lpddr_config_struct_addr DRAM Address Registers
 * With the registers ADDR0..ADDR5 the user can 
 * 
 * \subsection group_lpddr_config_struct_dllctlca DLL Controll Register for Command Address Bus
 * 
 * \subsection group_lpddr_config_struct_dllctldq DLL Controll Registers for DQ Bits per Data Slice
 * 
 *
 **/
typedef struct 
{
un_LPDDR4_LPDDR4_WRAPPER_QOS_CPUSS_t unQOS_CPUSS;               /*!< QoS Configuration Register for CPUSS master */

un_LPDDR4_LPDDR4_WRAPPER_QOS_VIDEOSS_RD_t unQOS_VIDEOSS_RD;     /*!< QoS Configuration Register for VIDEOSS read master */

un_LPDDR4_LPDDR4_WRAPPER_QOS_VIDEOSS_WR_t unQOS_VIDEOSS_WR;     /*!< QoS Configuration Register for VIDEOSS write master */

cy_en_lpddr4_refreshrate_strategy_t   refreshRateStrategy;      /*!< defines the refresh rate strategy which shall be used to updated the refresh rate
                                                                     CY_LPDDR_REFRESHRATE_DYNAMIC --> refresh rate is set as on DRAM Request MR4[2:0]
                                                                     CY_LPDDR_REFRESHRATE_FIXED   --> fixed refresh rate CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI*/

uint32_t fixedRefreshRate;                                      /*!< Defines the Refresh rate which will be used when the user selected CY_LPDDR_REFRESHRATE_FIXED
                                                                     when set to 0 and refreshRateStrategy == CY_LPDDR_REFRESHRATE_FIXED  the resulting refresh 
                                                                     rate is fixed at smallest possible refresh rate (at 105°C) refresh rate CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI
                                                                     for values > 0 and refreshRateStrategy == CY_LPDDR_REFRESHRATE_FIXED its the numbers of clocks for tREFI*/

cy_stc_lpddr4_pll800_manual_config_t    pllconfig;              /*!< Configuration for the PLL used to setup the PLL800
                                                                     which is used to clock the LPDDR4 Controller. It contains all settings
                                                                     neccessary to setup the PLL for FSP[0] FSP[1] the settings for */
un_LPDDR4_LPDDR4_CORE_DMCTL_t           unDMCTL;                /*!< Dynamo Control Register 
                                                                     used to set the LPDDR Type,DFI Ratio,Channels Enable... 
                                                                     Refer Register TRM for further information */
un_LPDDR4_LPDDR4_CORE_DMCFG_t           unDMCFG;                /*!< Dynamo Configuration Register used to set
                                                                     Enable Refresh Postone/Pullin,ECC Enable, Auto ZQ Enbale...
                                                                     Refer Register TRM for further information */

un_LPDDR4_LPDDR4_CORE_LPMR1_t           unLPMR1;                /*!< LPDDR Mode Register 1. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR1 */

un_LPDDR4_LPDDR4_CORE_LPMR2_t           unLPMR2;                /*!< LPDDR Mode Register 2. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR2 */

un_LPDDR4_LPDDR4_CORE_LPMR3_t           unLPMR3;                /*!< LPDDR Mode Register 3. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR3 */

un_LPDDR4_LPDDR4_CORE_LPMR11_t          unLPMR11;               /*!< LPDDR Mode Register 11. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR11 */

un_LPDDR4_LPDDR4_CORE_LPMR12_t          unLPMR12;               /*!< LPDDR Mode Register 12. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR12 */

un_LPDDR4_LPDDR4_CORE_LPMR13_t          unLPMR13;               /*!< LPDDR Mode Register 13. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR13 */


un_LPDDR4_LPDDR4_CORE_LPMR14_t          unLPMR14;               /*!< LPDDR Mode Register 14. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR14 */

un_LPDDR4_LPDDR4_CORE_LPMR22_t          unLPMR22;               /*!< LPDDR Mode Register 22. Whereas the external device uses 2 sets of Mode Registers,
                                                                     the LPDDR Controller stores the values which will be programmed into MRs of the external device
                                                                     in one 32 Bit Register. So the LPMR1 contains all parameters  needed to setup 
                                                                     MR1 for FSP[0] and MR1 for FSP[1].For additional information refer Register TRM LPMR22 */



cy_en_lpddr4_mr23_dqsosc_runtime_t      dqsOscRuntimeClkCyls;   /*!< DQS2DQ Oscillator Timer interval value. This value will be programmed to 
                                                                     MR23. It defines how long the DQS Osc. runs. As longer the runtime is as 
                                                                     better is the accuracy */

uint32_t                             deltaDQSCountRetrain[2];   /*!< Delta DQS Counter Value which causes a DQS2DQ retraining . LPDDR4 driver reads 
                                                                     the DQS Oscillator Conuter directly after training. Then the 
                                                                     DQS Oscillator count is monitored on a cyclic base. If the absolute delta of
                                                                     actual DQS Osc Count and DQS Osc. Count last training > deltaDQSCountRetrain
                                                                     a retraining  request is set in the driver context */

un_LPDDR4_LPDDR4_CORE_RTCFG0_RT0_t      unRTCFG0_RT0;           /*!< Route Configuration 0 - Route 0 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG0_RT1_t      unRTCFG0_RT1;           /*!< Route Configuration 0 - Route 1 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG0_RT2_t      unRTCFG0_RT2;           /*!< Route Configuration 0 - Route 2 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG0_RT3_t      unRTCFG0_RT3;           /*!< Route Configuration 0 - Route 3 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG1_RT0_t      unRTCFG1_RT0;           /*!< Route Configuration 1 - Route 0 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG1_RT1_t      unRTCFG1_RT1;           /*!< Route Configuration 1 - Route 1 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG1_RT2_t      unRTCFG1_RT2;           /*!< Route Configuration 1 - Route 2 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_RTCFG1_RT3_t      unRTCFG1_RT3;           /*!< Route Configuration 1 - Route 3 
                                                                     For additional information refer Register TRM*/
un_LPDDR4_LPDDR4_CORE_ADDR0_t           unADDR0;                /*!< DRAM Address Register 0 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_ADDR1_t           unADDR1;                /*!< DRAM Address Register 1 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_ADDR2_t           unADDR2;                /*!< DRAM Address Register 2 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_ADDR3_t           unADDR3;                /*!< DRAM Address Register 3 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_ADDR4_t           unADDR4;                /*!< DRAM Address Register 4 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_ADDR5_t           unADDR5;                /*!< DRAM Address Register 5 used to setup the address mapping scheme
                                                                     For additional information refer Register TRM */

un_LPDDR4_LPDDR4_CORE_VTGC_t            unVTGC;                 /*!< PHY VREF Training General Control Register
                                                                     used to setup VREF Training settings: number of steps,step size of 
                                                                     VREF DQ / CA training For additional information refer Register TRM */
un_LPDDR4_LPDDR4_CORE_PHY_t             unPHY;                  /*!< PHY Register */
un_LPDDR4_LPDDR4_CORE_BISTCFG_CH0_t     unBISTCFG_CH0;          /*!< BIST Configuration Register - Channel 0 */
un_LPDDR4_LPDDR4_CORE_BISTCFG_CH1_t     unBISTCFG_CH1;          /*!< BIST Configuration Register - Channel 1 */
un_LPDDR4_LPDDR4_CORE_DQSDQCR_t         unDQSDQCR;              /*!< DQS2DQ Delay Control Register */
un_LPDDR4_LPDDR4_CORE_TREG0_t           unTREG0;                /*!< Timing Register 0 */
un_LPDDR4_LPDDR4_CORE_TREG1_t           unTREG1;                /*!< Timing Register 1 */
un_LPDDR4_LPDDR4_CORE_TREG2_t           unTREG2;                /*!< Timing Register 2 */
un_LPDDR4_LPDDR4_CORE_TREG3_t           unTREG3;                /*!< Timing Register 3 */
un_LPDDR4_LPDDR4_CORE_TREG4_t           unTREG4;                /*!< Timing Register 4 */
un_LPDDR4_LPDDR4_CORE_TREG5_t           unTREG5;                /*!< Timing Register 5 */
un_LPDDR4_LPDDR4_CORE_TREG6_t           unTREG6;                /*!< Timing Register 6 */
un_LPDDR4_LPDDR4_CORE_TREG7_t           unTREG7;                /*!< Timing Register 7 */

un_LPDDR4_LPDDR4_CORE_TREG8_t           unTREG8;                /*!<Timing Register 8 */
un_LPDDR4_LPDDR4_CORE_TREG9_t           unTREG9;                /*!<Timing Register 9 */
un_LPDDR4_LPDDR4_CORE_TREG10_t          unTREG10;               /*!<Timing Register 10 */  
un_LPDDR4_LPDDR4_CORE_TREG11_t          unTREG11;               /*!<Timing Register 11 */
un_LPDDR4_LPDDR4_CORE_TREG12_t          unTREG12;               /*!<Timing Register 12 */
un_LPDDR4_LPDDR4_CORE_TREG13_t          unTREG13;               /*!<Timing Register 13 */  
un_LPDDR4_LPDDR4_CORE_TREG14_t          unTREG14;               /*!<Timing Register 14 */
un_LPDDR4_LPDDR4_CORE_TREG15_t          unTREG15;               /*!<Timing Register 15 */
un_LPDDR4_LPDDR4_CORE_RTGC0_t           unRTGC0;                /*!<PHY Read Training General Control Register 0 */
un_LPDDR4_LPDDR4_CORE_RTGC1_t           unRTGC1;                /*!<PHY Read Training General Control Register 1 */
un_LPDDR4_LPDDR4_CORE_PTSR0_t           unPTSR0;                /*!<PHY Training Setting Register 0*/
un_LPDDR4_LPDDR4_CORE_PTSR1_t           unPTSR1;                /*!<PHY Training Setting Register 1*/
un_LPDDR4_LPDDR4_CORE_PTSR2_t           unPTSR2;                /*!<PHY Training Setting Register 2*/
un_LPDDR4_LPDDR4_CORE_PTSR3_t           unPTSR3;                /*!<PHY Training Setting Register 3*/
un_LPDDR4_LPDDR4_CORE_PTSR4_t           unPTSR4;                /*!<PHY Training Setting Register 4*/
un_LPDDR4_LPDDR4_CORE_PTSR5_t           unPTSR5;                /*!<PHY Training Setting Register 5*/
un_LPDDR4_LPDDR4_CORE_PTSR6_t           unPTSR6;                /*!<PHY Training Setting Register 6*/
un_LPDDR4_LPDDR4_CORE_PTSR7_t           unPTSR7;                /*!<PHY Training Setting Register 7*/
un_LPDDR4_LPDDR4_CORE_PTSR8_t           unPTSR8;                /*!<PHY Training Setting Register 8*/
un_LPDDR4_LPDDR4_CORE_PTSR9_t           unPTSR9;                /*!<PHY Training Setting Register 9*/
un_LPDDR4_LPDDR4_CORE_PTSR10_t          unPTSR10;               /*!<PHY Training Setting Register 10*/
un_LPDDR4_LPDDR4_CORE_PTSR11_t          unPTSR11;               /*!<PHY Training Setting Register 11*/
un_LPDDR4_LPDDR4_CORE_PTSR12_t          unPTSR12;               /*!<PHY Training Setting Register 12*/
un_LPDDR4_LPDDR4_CORE_PTSR13_t          unPTSR13;               /*!<PHY Training Setting Register 13*/
un_LPDDR4_LPDDR4_CORE_PTSR14_t          unPTSR14;               /*!<PHY Training Setting Register 14*/
un_LPDDR4_LPDDR4_CORE_PTSR15_t          unPTSR15;               /*!<PHY Training Setting Register 15*/
un_LPDDR4_LPDDR4_CORE_PTSR16_t          unPTSR16;               /*!<PHY Training Setting Register 16*/
un_LPDDR4_LPDDR4_CORE_PTSR17_t          unPTSR17;               /*!<PHY Training Setting Register 17*/
un_LPDDR4_LPDDR4_CORE_PTSR18_t          unPTSR18;               /*!<PHY Training Setting Register 18*/
un_LPDDR4_LPDDR4_CORE_PTSR19_t          unPTSR19;               /*!<PHY Training Setting Register 19*/
un_LPDDR4_LPDDR4_CORE_PTSR20_t          unPTSR20;               /*!<PHY Training Setting Register 20*/
un_LPDDR4_LPDDR4_CORE_PTSR21_t          unPTSR21;               /*!<PHY Training Setting Register 21*/
un_LPDDR4_LPDDR4_CORE_PTSR22_t          unPTSR22;               /*!<PHY Training Setting Register 22*/
un_LPDDR4_LPDDR4_CORE_PTSR23_t          unPTSR23;               /*!<PHY Training Setting Register 23*/
un_LPDDR4_LPDDR4_CORE_PTSR24_t          unPTSR24;               /*!<PHY Training Setting Register 24*/
un_LPDDR4_LPDDR4_CORE_PTSR25_t          unPTSR25;               /*!<PHY Training Setting Register 25*/
un_LPDDR4_LPDDR4_CORE_DLLCTLCA_CH0_t    unDLLCTLCA_CH0;         /*!<DLL Control Register for PHY Command Module - Channel 0*/
un_LPDDR4_LPDDR4_CORE_DLLCTLCA_CH1_t    unDLLCTLCA_CH1;         /*!<DLL Control Register for PHY Command Module - Channel 1*/
un_LPDDR4_LPDDR4_CORE_DLLCTLDQ_SL0_t    unDLLCTLDQ_SL0;         /*!<DLL Control Register for PHY Data Module Slice 0 */
un_LPDDR4_LPDDR4_CORE_DLLCTLDQ_SL1_t    unDLLCTLDQ_SL1;         /*!<DLL Control Register for PHY Data Module Slice 1 */
un_LPDDR4_LPDDR4_CORE_DLLCTLDQ_SL2_t    unDLLCTLDQ_SL2;         /*!<DLL Control Register for PHY Data Module Slice 2 */
un_LPDDR4_LPDDR4_CORE_DLLCTLDQ_SL3_t    unDLLCTLDQ_SL3;         /*!<DLL Control Register for PHY Data Module Slice 3 */
un_LPDDR4_LPDDR4_CORE_DIOR_SL0_t        unDIOR_SL0;             /*!<PHY Data Module IO Control Register for Slice 0*/
un_LPDDR4_LPDDR4_CORE_DIOR_SL1_t        unDIOR_SL1;             /*!<PHY Data Module IO Control Register for Slice 1*/
un_LPDDR4_LPDDR4_CORE_DIOR_SL2_t        unDIOR_SL2;             /*!<PHY Data Module IO Control Register for Slice 2*/
un_LPDDR4_LPDDR4_CORE_DIOR_SL3_t        unDIOR_SL3;             /*!<PHY Data Module IO Control Register for Slice 3*/
un_LPDDR4_LPDDR4_CORE_CIOR_CH0_t        unCIOR_CH0;             /*!<PHY Command Module IO Control Register - Channel 0 */
un_LPDDR4_LPDDR4_CORE_CIOR_CH1_t        unCIOR_CH1;             /*!<PHY Command Module IO Control Register - Channel 1 */
un_LPDDR4_LPDDR4_CORE_PTAR_t            unPTAR;                 /*!<PHY Sanity Check Address Register */
un_LPDDR4_LPDDR4_CORE_PCCR_CH0_t        unPCCR_CH0;             /*!<PHY Compensation Control Register - Channel 0 */
un_LPDDR4_LPDDR4_CORE_PCCR_CH1_t        unPCCR_CH1;             /*!<PHY Compensation Control Register - Channel 1 */
un_LPDDR4_LPDDR4_CORE_INECC0_t          unINECC0;               /*!<Inline ECC Register 0 - used to set the 
                                                                    ECC Region Base Address this is where the 
                                                                    ECC codes are stored the size depends on the size
                                                                    of the */
un_LPDDR4_LPDDR4_CORE_INECC1_t          unINECC1;               /*!<Inline ECC Register 1 - used to set the 
                                                                    ECC Protected Region Base Address */
un_LPDDR4_LPDDR4_CORE_INECC2_t          unINECC2;               /*!<Inline ECC Register 2 -used to program the 
                                                                     Protected Region Size */
} cy_stc_lpddr4_config_t;



/** 
 * \brief LPDDR4 controller status context variable. It stores internal driver 
 * information:<br>
 *  <ul>
 *  <li>channelActive - which channels are activated. The are the channels 
 *      which will be addresses when sending command over the UCI register 
 *      to the LPDDR4 controller.
 * </li>
 *  <li>
 *      retrainRequestCHA- retrain request flags for CHA 
 * </li> 
 *  <li>
 *      retrainRequestCHB- retrain request flags for CHB
 * </li>
 * <li>
 *      retrainThresholdCHA - delta DQS Osc count which will lead to a 
 *      re-train request for CHA
 * </li>
 * <li>
 *      retrainThresholdCHB - delta DQS Osc count which will lead to a 
 *      re-train request for CHB
 * </li>
 * <li>
 *      dqsOSCCountLastTraining - DQS OSC Counter after the last successfull
 *      training for each channel, used to detect re-training necessity
 * </li>
 * <li>
 *      dqsOSCCountTrigTrain - DQS OSC Counter Values which triggered the 
 *      last activated retraining request
 * </li>
 * <li>
 *      retrainDirection - the direction in which the next retraining 
 *      shall take place /ref cy_en_lpddr4_dqsdqcr_dir_t
 * </li>
 * <li>
 *      controllerRunning - if the controller is running or not 
 * </li>
 * <li>
 *      operationTemperatureState - temperature status of the external memory 
 *      device
 * </li>
 * <li>
 *      dqs2dqRetrainActive - status if currently a retraining is ongoing
 * </li>
 *  <li>
 *      refreshRate - actual refresh rate
 * </li>
 *  <li>
 *      refreshRateStrategy - current refresh rate Strategy
 * </li>
 *  <li>
 *      refreshStrategyChanged - if refresh rate strategy channge is pending
 * </li>
 *  <li>
 *      lastUCICommand - last command send to the controller using UCI Regsister 
 * </li>
 *  <li>
 *      deratingStatus - timing derating status active or not
 * </li>
 * </ul>
 **/
typedef struct
{
    /*!<shows the initialization status of the driver */
    volatile cy_en_lpddr4_cntrl_init_state_t           controllerInitState;
    
   /*!<shows which channels were activated during the initialization of controller 
    all commands send with Cy_Lpddr_SendCmd() will address teh channels shown here*/
    volatile cy_en_lpddr4_chan_sel_t                    channelActive; 
    

    /*!<flag signaling if a DQS2DQ retraining is needed for CHA */
    volatile bool                                           retrainRequestCHA;
    
    /*!<flag signaling if a DQS2DQ retraining is needed for CHB */
    volatile bool                                           retrainRequestCHB;
    
    /*!<delta DQS Osc Count which will trigger a retrain for CHA */
    volatile uint32_t                                       retrainThresholdCHA;
    
    /*!<delta DQS Osc Count which will trigger a retrain for CHB */
    volatile uint32_t                                       retrainThresholdCHB;
    
    /*!<struct contains the DQS Counter Value after last successfull 
    training for CHA and CHB */
    volatile cy_stc_lpddr4_dqsosc_read_data_t           dqsOSCCountLastTraining;

    /*!<struct contains the DQS Counter Value for CHA/B which triggered 
    current training */
    volatile cy_stc_lpddr4_dqsosc_read_data_t               dqsOSCCountTrigTrain;

    /*!<struct contains the actual DQS Counter Value for CHA/B */
    volatile cy_stc_lpddr4_dqsosc_read_data_t               actualDQSOsc;


    /*!<the direction for the next DQS2DQ training */
    volatile cy_en_lpddr4_dqsdqcr_dir_t                    retrainDirection;

     /*!<status if a DQS2DQ re-training is active or not. 
    Is active, if at least 1 channels has a training active  */
    volatile bool                                           dqs2dqRetrainActive;

    /*!<status if controller is running or not */
    volatile cy_en_lpddr4_cntrl_run_state_t                 controllerRunning;
    
    /* previous Controller statemaschine state */ 
    volatile cy_en_lpddr4_cntrl_gsm_state_t                 controllerStatePrevRecov;


    /* previous Controller statemaschine state */ 
    volatile cy_en_lpddr4_cntrl_gsm_state_t                 controllerStatePrev;

    /* actual Controller statemaschine state */ 
    volatile cy_en_lpddr4_cntrl_gsm_state_t                 controllerState;


    /*!< status if temperature derating is active. detail about derating refer
    JEDEC209-4B  10.3 Temperature Derating for AC timing*/
    volatile cy_en_lpddr4_MR4_derate_state_t                deratingStatus;


    /*  flag signaling that a timing derating is pending. During next MC stop
    the parameters will be changed accordingly. */
    volatile cy_en_lpddr4_MR4_derate_state_t                deratingPending;

    /*!< external Memory temperature state from MR4[2:0]*/
    volatile cy_en_lpddr4_memory_operational_temperature_t  operationTemperatureState;

    /*!< last command send over UCI Register to the controller */
    volatile un_LPDDR4_LPDDR4_CORE_UCI_t                   lastUCICommand;

    /*!< actual refresh rate setting */
    volatile cy_en_lpddr4_refresh_rate_t                    refreshRate;

    /*!< refresh rate strategy which is currently in use:<br>
        CY_LPDDR_REFRESHRATE_DYNAMIC--> refresh rate  based on MR4[2:0]<br>
        CY_LPDDR_REFRESHRATE_FIXED --> a fixed rate of CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI<br>*/
    volatile cy_en_lpddr4_refreshrate_strategy_t            refreshRateStrategy;

    /*!< driver context information of actual selected timing derating strategy 
    */
    volatile cy_en_deratingstrategy_state_t timingDeratingStrategy;

    /*!< flag indicating that the refresh rate strategy was changed and the refresh rate<br> 
         must be updated even if there was no update in the MR4 register (TUF)<br>
         ( Temperature Update Flag )*/
    volatile cy_en_refreshrate_strategy_change_state_t refreshStrategyChanged;



    /*!< the fixed user refresh rate the user has given in the lpddr4 config structure */
    volatile uint32_t fixedUserRefreshRate;


}cy_stc_lpddr4_context_t;



/* AXI Perf Count Config struct */

/** 
 * \brief AXI Performasnce Counter Measureing Unit Configuration StructureL  
 **/
typedef struct
{
    uint32_t ctlTmr:28;             /*!< 28 Bit Timer value */

    uint32_t ctlPreScale:4;         /*!< 4-bit prescaler value. The clock going to 
                                         the timer is divided by 2**PSC.*/

    uint32_t portSelect:3;          /*!< This field selects the AXI port of the 
                                         LPDDR4 controller on which the performance 
                                         measurement is done.*/
    
    uint32_t filterMID:4;           /*!< This field sets the filter for the Master ID. 
                                         Only bits i for which FILTER_MASK.MID[i] is 1 
                                         are compared.This field affects the following 
                                         counters: ADDR_CNT.CNT, ADDR_STALL.CNT, 
                                         DATA_CNT.CNT, DATA_STALL.CNT. Note: If 
                                         WRITE = 1, this field has no effect on 
                                         DATA_CNT and DATA_STALL_CNT.*/
    
    uint32_t filterTID:8;           /*!< This field sets the filter for the upper 
                                         transaction ID. Only bits i for which FILTER_MASK.
                                         TID[i] is 1 are compared. This field affects the 
                                         following counters: ADDR.CNT, ADDR_STALL.CNT, 
                                         DATA.CNT, DATA_STALL.CNT. Note: If WRITE = 1, 
                                         this field has no effect on DATA_CNT and DATA_STALL_CNT.
                                        Note: TID denotes bits [ID_WIDTH-1:8] of the 
                                        transaction ID. Different ports can have different 
                                        ID widths. ID_WIDTH is the ID width of the port 
                                        with the largest ID width. For ports with smaller 
                                        ID widths, the MSBs of TID are '0'.*/
    
    uint32_t filterWrite:1;         /*!< This field sets the filter for the direction. 
                                        If WRITE = 1, then only write transactions are 
                                        counted, and if WRITE = 0, then only read transactions 
                                        are counted. This field affects the following counters:
                                        OT_AC.CNT, ADDR_CNT.CNT, ADDR_STALL.CNT, DATA_CNT.CNT, 
                                        DATA_STALL.CNT. Note that this field does not have a 
                                        corresponding field in the FILTER_MASK register.*/
    
    uint32_t filterLenMin:4;        /*!< The value of this field + 1 defines the minimum 
                                         burst length to be counted. This field affects 
                                         the following counters: ADDR_CNT.CNT, ADDR_STALL.CNT.
                                         Note that this field does not have a corresponding 
                                         field in the FILTER_MASK register.*/
    
    uint32_t filterLenMax:4;        /*!< The value of this field + 1 defines the maximum burst
                                         length to be counted. This field affects the following
                                         counters:  ADDR_CNT.CNT, ADDR_STALL.CNT. Note that this 
                                         field does not have a corresponding field in the 
                                         FILTER_MASK register.*/
    
    uint32_t filtermaskMID:4;       /*!< This field sets the filter mask for the Master ID. Only
                                         bits i for which MID[i] is 1 are compared with FILTER.MID[i].
                                         This field affects the following counters: ADDR_CNT.CNT, 
                                         ADDR_STALL.CNT, DATA_CNT.CNT, DATA_STALL.CNT.Note: If 
                                         FILTER.WRITE = 1, this field has no effect on DATA_CNT.CNT 
                                         and DATA_STALL_CNT.*/
    
    uint32_t filtermaskTID:8;       /*!< This field sets the filter for the upper transaction ID.
                                         Only bits i for which TID[i] is 1 are compared with FILTER.TID[i].
                                         This field affects the following counters:  ADDR_CNT.CNT, 
                                         ADDR_STALL.CNT, DATA_CNT.CNT, DATA_STALL.CNT. Note: If FILTER.WRITE = 1,
                                         this field has no effect on DATA_CNT.CNT and DATA_STALL_CNT.
                                         Note: TID denotes bits [ID_WIDTH-1:8] of the transaction ID. 
                                         Different ports can have different ID widths. ID_WIDTH is the ID 
                                         width of the port with the largest ID width. For ports with smaller 
                                         ID widths, the MSBs of TID are '0'.*/
}stc_lpddr4_axi_perf_mu_config_t;







/** \} group_lpddr_data_structures */

/*******************************************************************************
*                           Global Variable
*******************************************************************************/
/** \} group_lpddr_enums */

/**
* \addtogroup group_lpddr_globals
* \{
*/

/*******************************************************************************
 *          global variables for the LPDDR4 driver
 ******************************************************************************/
/** driver internal context variable */
extern cy_stc_lpddr4_context_t g_stc_lpddr_context;


/** \} group_lpddr_globals */

/*******************************************************************************
*                               Constants
*******************************************************************************/

/** \cond INTERNAL */

/** \endcond */


/*******************************************************************************
*                          Function Prototypes
*******************************************************************************/
/**
* \addtogroup group_lpddr_functions
* \{
*/

/*******************************************************************************
*       Memory Address handling functions
*******************************************************************************/

/*******************************************************************************
 * Function Name: Cy_Lpddr_GetAxiToDramMapping
 **************************************************************************//***
 * 
 * \brief This function calculates the channel, bank, row, column addresses 
 * from a given AXI address based on the address mapping programmed in the controller
 * address mapping registers ADDR0 .. ADDR5.
 * 
 * \param axi_addr AXI address
 *
 * \param channel memory channel number
 * 
 * \param bank memory bank number
 *
 * \param row memory row address
 *
 * \param col memory column address
 *
 * \return None
 ******************************************************************************/
void Cy_Lpddr_GetAxiToDramMapping(volatile stc_LPDDR4_t * base, 
                                const uint32_t axi_addr,uint32_t *channel, 
                                uint32_t *bank, uint32_t *row, uint32_t *col);


/*******************************************************************************
 * Function Name: Cy_Lpddr_GetDramToAxiMapping
 *************************************************************************//****
 * 
 * \brief This function calculates the AXI address from given channel, bank, row,
 *         column addresses based on the addres mapping stored in the controller
 *         registers ADDR0 .. ADDR5.
 * 
 * \param axi_addr AXI address
 *
 * \param channel memory channel number
 * 
 * \param bank memory bank number
 *
 * \param row memory row address
 *
 * \param col memory column address
 *
 * \return None
 ******************************************************************************/
void Cy_Lpddr_GetDramToAxiMapping(volatile stc_LPDDR4_t * base, 
                                  uint32_t *axi_addr,const uint32_t channel, 
                                  const uint32_t bank, const uint32_t row, 
                                  const uint32_t col);





/***************************************
*       Start of PLL related stuff
***************************************/

/*******************************************************************************
 * Function Name: Cy_Lpddr_CalcMR4ReadInterval
 *************************************************************************//****
 * 
 * \brief
 * Function to calculate the cycle time to read the MR4 register for 
 * TCSR ( Temperature Compensated Self Refresh 
 * 
 * LPDDR4 devices feature a temperature sensor whose status can be read from MR4
 * see Table. This sensor can be used to determine an appropriate refresh rate, 
 * determine whether AC timing de-rating is required in the elevated temperature
 * range, and/or monitor the operating temperature. 
 * 
 * The MR4 Read  Interval is calculated with the formula iven in the 
 * JEDEC209-4C Spec .
 * 
 * 
 * \note
 * To assure proper operation using the temperature sensor, applications should 
 * consider the following factors:<br> TempGradient is the maximum temperature 
 * gradient experienced by the memory device at the temperature of interest <br>
 * over a range of 2°C \ref JEDEC_DEVICE_TEMPERATURE_MARGIN. ReadInterval is the
 *  time period between MR4 reads from the system.<br> TempSensorInterval (tTSI)
 * is maximum delay between internal updates of MR4.<br> SysRespDelay is the 
 * maximum time between a read of MR4 and the response by the system.<br>
 * In order to determine the required frequency of polling MR4, the system shall
 * use the maximum <br> TempGradient and the maximum response time of the system
 * using the following equation:<br>
 * <br>
 * TempGradient x (ReadInterval + tTSI + SysRespDelay)  <= 2°C<br>
 * 
 * <table class="doxtable">
 * <tr><th>Parameter</th>
 * <th> Symbol</th>
 * <th> Max/Min</th>
 * <th> Value</th>
 * <th> Unit</th></tr>
 * <tr><td>System Temperature Gradient</td>
 * <td> TempGradient</td>
 * <td> Max</td>
 * <td> System Dependent</td>
 * <td> °C/s</td></tr>
 * <tr><td>MR4 Read Interval</td>
 * <td> ReadInterval </td>
 * <td>Max </td>
 * <td>System Dependent</td>
 * <td> ms</td></tr>
 * <tr><td>Temperature Sensor Interval</td>
 * <td> tTSI</td>
 * <td> Max </td>
 * <td>32 </td>
 * <td>ms</td></tr>
 * <tr><td>System Response Delay</td>
 * <td> SysRespDelay</td>
 * <td> Max </td>
 * <td>System Dependent</td>
 * <td> ms</td></tr>
 * <tr><td>Device Temperature Margin</td>
 * <td> TempMargin </td>
 * <td>Max</td>
 * <td> 2 </td>
 * <td>°C</td></tr>
 * </table>
 * 
 * \note    For example, if TempGradient is 10°C/s and the SysRespDelay is 
 *          1 ms:<br> (10°C/s) x (ReadInterval + 32ms + 1ms)  <= 2°C<br>
 *          In this case, ReadInterval shall be no greater than 167 ms<br>
 * 
 * \param   sysResponseDelay System response of user system ( system dependend) 
 *          delay as specified in the JEDEC 209-4C in [msec]
 * 
 * \param   sysTemperatureGradient  is the maximum temperature gradient  of user
 *          system ( system dependend) experienced by the memory device at the 
 *          <br>temperature of interest over a range of 2°C [°C/s]
 * 
 * \return  mr4ReadInterval which is the ReadInterval as specified in 
 *          JEDEC 209-4C
*******************************************************************************/
uint32_t Cy_Lpddr_CalcMR4ReadInterval(double sysResponseDelay,double sysTemperatureGradient);



/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllEnabled();
 *************************************************************************//****
 *
 * \brief    Returns the enable status of the PLL800 for LPDDR4
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return    CY_LPDDR4_ONOFF_ENABLE  when PLL is enabled<br> 
 *            CY_LPDDR4_ONOFF_DISABLE when PLL is disabeld<br>
 * 
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsPllEnabled(volatile stc_LPDDR4_t * base);

/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllOutputEnabled();
 *************************************************************************//****
 *
 * \brief    Returns the output enable status of the PLL800 for LPDDR4
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return    CY_LPDDR4_ONOFF_ENABLE  when PLL output is enabled<br> 
 *            CY_LPDDR4_ONOFF_DISABLE when PLL output is disabeld<br>
 * 
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsPllOutputEnabled(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 * Function Name: Cy_Lpddr_HasPllLostLock
 *************************************************************************//****
 *
 * Reports whether or not the PLL lost its lock since the last time this 
 * function was called. 
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return 0 = didn't lose lock, 1 = lost lock
 *
 ******************************************************************************/
bool Cy_Lpddr_HasPllLostLock(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllLocked
 *************************************************************************//****
 * \brief  Returns lock status of the PLL
 *   
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return 0 = not locked, 1 = locked
 ******************************************************************************/
bool Cy_Lpddr_IsPllLocked(volatile stc_LPDDR4_t * base);

/***************************************
*       End of PLL related stuff
***************************************/

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_ControllerInit
 *************************************************************************//****
 * \brief  Function to initialize the LPDDR4 Controller
 *  
 * This function controlls the LPDDR4 controller initialisation process. Below 
 * list is a summary of what the function does to initialize the controller.<br>
 * The initialization process for the LPDDR4 inlcudes the following steps
 * <ol>
 *      <li> Enable the Main Enabler of the IP</li>
 *      <li> Setup the PLL ( set the divider, check pll for locking enable pll 
 *           output)</li>
 *      <li> Setup Arbiter</li>
 *      <li> Setup DRAM Registers ( LPMR1,LPMR2,LPMR3,LPMR11,LPMR12,LPMR13,
 *           LPMR14,LPMR22..</li>
 *      <li> Setup PHY DDR Mode (VTGC,PHY,DQSDQCR)</li>     
 *      <li> Setup Bist Configuration Register  (BISTCFG_CH0/unBISTCFG_CH1)</li>
 *      <li> Setup Phy Timing Registers (TREG0..15 )</li>
 *      <li> Setup DLL</li>
 *      <li> Setup Address Mapping</li>
 *      <li> Phy Initialization (DLL Init,DRAM,Init, PHY Training for FSP[0]</li>
 *      <li> Phy Reload(reload trained values from a former training run which 
 *           were stored in the PTSRx registers.) can be used for quickstart 
 *           without complete training to get the controller up and running 
 *           after a power down event.<br>
 *              <b>or</b><br>
 *           Perform high speed operation training (CBT,WRLEVL,RDLEVL,Gate 
 *           Training,...including switching the PLL fout several times)<br>
 *           and switch to high speed operation FSP[1]</li>
 *      <li> Start the Comntroller with Run Command </li>
 * </ol>
 * TODO: Configure ECC
 *
 *
 * \param  base Pointer to the LPDDR4 Base Structure from type 
           \ref stc_LPDDR4_t
 * 
 * \param  config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 * 
 * 
 * \param  inittype \ref cy_en_lpddr4_inittype_t
 * 
 * \return
 * CY_LPDDR4_SUCCESS if everything was ok<br>
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function <br>
 *
 *   @startuml"Controller Init Sequence"
 *   !include commons.iuml!ControllerInit
 *   @enduml
 *
 *
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ControllerInit(volatile stc_LPDDR4_t * base, 
                                              const cy_stc_lpddr4_config_t * config,
                                              cy_en_lpddr4_inittype_t init);



/*******************************************************************************
 * Function Name: Cy_Lpddr_GetInteruptStatusCHA
 ***************************************************************************//**
 *
 * \brief Return the interrupt Status of CHA
 *
 * Function to check if CHA caused a interrupt request. It returns the status
 * from Interrupt Status Register CH A (INTSTT_CH0) of the controller. 
 * 
 * \note Before calling this function the IP must be enabled as the IPs 
 *       CORE Registers  are not accessible when IP is disabled
 *  
 * \param  base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 * \return   bool
 *  0 - no interrupt pending
 *  1 - CH A has caused  a interrupt  
 ******************************************************************************/
__INLINE bool Cy_Lpddr_GetInteruptStatusCHA(volatile stc_LPDDR4_t * base)
{
    return (bool) base->LPDDR4_CORE.unINTSTT_CH0.stcField.u1INT_GC_FSM;
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_GetInteruptStatusCHB
 ***************************************************************************//**
 *
 * \brief Return the interrupt Status of CHB.
 * 
 * Function to check if CHB caused a interrupt request. It returns the status
 * from Interrupt Status Register CH B (INTSTT_CH1) of the controller.
 * 
 * \note Before calling this function the IP must be enabled as the IPs 
 *       CORE Registers  are not accessible when IP is disabled
 *
 * \param  base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 * \return   bool
 *  0 - no interrupt pending
 *  1 - CH B has caused  a interrupt  
 ******************************************************************************/
__INLINE bool Cy_Lpddr_GetInteruptStatusCHB(volatile stc_LPDDR4_t * base)
{
    return (bool) base->LPDDR4_CORE.unINTSTT_CH1.stcField.u1INT_GC_FSM;
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_ControllerDeInit
 *************************************************************************//****
 *
 *   \brief  Function to DeInitialize the LPDDR4 Memory controller
 *           Function is performing the following steps.
 *           -   Send Stop Command to the Controller 
 *           -   Switch off PLL Output_Enable
 *           -   Switch off the PLL ( not possible with PSVP)
 *               un_LPDDR4_LPDDR4_WRAPPER_CTL 
 *           -  set driver context state to
 *              .controllerInitState = CY_LPDDR4_CNTRL_NOT_INITIALIZED
 *               
 *               
 *
 *
 *   \param  base Pointer to the LPDDR4 Base Structure from type 
 *           \ref stc_LPDDR4_t
 * 
 *   \return \ref    cy_en_lpddr4_retval_t<br>
 *                   CY_LPDDR4_SUCCESS if everything was ok<br>
 *                   CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to 
 *                                       the function<br>
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ControllerDeInit(volatile stc_LPDDR4_t * base);


// /*******************************************************************************
//  *  Function Name:  Cy_Lpddr_PhyTraining
//  *************************************************************************//****
//  *  \brief Function to trigger trainings request to the LPDDR4 controller.
//  * 
//  *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
//  * 
//  *  \param config Pointer to the LPDDR4 config structure
//  * 
//  *  \param phyinit
//  * 
//  *  \param fs Target frequency set-point for PHY operations. Must be different 
//  *         from operating set point.This is the FSP for which the PHY will be 
//  *         trained 
//  * 
//  *  \param gten Gate training enable
//  * 
//  *  \param wrlvlen Write leveling enable. 
//  * 
//  *  \param vrefqdwren DRAM VREF-DQ training enable. 
//  * 
//  *  \param vrefdqrden PHY VREF-DQ training enable.
//  * 
//  *  \param rdlvlen Read data eye DQ training enable. 
//  * 
//  *  \param dlyevalen Delay evaluation enable. 
//  * 
//  *  \param sanchken Write/read sanity check enable.
//  * 
//  *  \param vrefcaen DRAM VREF-CA training enable.
//  * 
//  *  \return \ref    cy_en_lpddr4_retval_t<br>
//  *                  CY_LPDDR4_SUCCESS if everything was ok<br>
//  *                  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the 
//  *                  function<br>
//  ******************************************************************************/
// cy_en_lpddr4_retval_t Cy_Lpddr_PhyTraining(volatile stc_LPDDR4_t * base, 
//                                            const cy_stc_lpddr4_config_t * config,
//                                            bool phyinit, bool fs, bool gten, 
//                                            bool wrlvlen, bool vrefdqwren , 
//                                            bool vrefdqrden, bool rdlvlen, 
//                                            bool dlyevalen, bool sanchken, 
//                                            bool vrefcaen);

/*******************************************************************************
 * Function Name: Cy_Lpddr_Monitoring()
 *************************************************************************//****
 * 
 *  \brief Function to perform some LPDDR4 Memory Monitoring task 
 *      ( still under construction )
 * 
 *  Function which needs to be called cyclic with a interval calculated by 
 *  Cy_Lpddr_CalcMR4ReadInterval.As voltage and temperature on the SDRAM die 
 *  and Controller changes , some of the parameters need to be 
 *  adapted from time to time. 
 *  The parameters affected by PVT are : Refresh Rate and DQS2DQ timing skew
 * 
 * This function is performing the following steps to compensate for PVT changes
 * 
 *  For Refresh Rate compensation base on DRAM MR4 reading  OP[2:0]<br>
 *             -000b: SDRAM low temperature operating limit exceeded<br>
 *             -001b: 4x refresh<br>
 *             -010b: 2x refresh<br>
 *             -011b: 1x refresh (default)<br>
 *             -100b: 0.5x refresh<br>
 *             -101b: 0.25x refresh, no derating<br>
 *             -110b: 0.25x refresh, with derating<br>
 *             -111b: SDRAM high temperature operating limit exceeded <br>
 *          -adjust refresh rate accoring to MR4 readout<br>
 *          -if memory is requesting derating some timing parameters need to 
 *           be adapted with temperature penalty. Check Jedec 209-4C 10.3 
 *           Temperature Derating  for AC Timing
 * 
 *  <table class="doxtable">
 *      <tr><th>Parameter </th><th>Symbol</th><th>Min/Max</th><th>Unit</th></tr>

 *      <tr><td>DQS output access time <br>from CK_t/CK_c (derated)</td>
 *          <td>tDQSCK</td> <td>MAX</td>
 *          <td>3600</td>
 *           <td>ps </td> 
 *      </tr>
 *      <tr><td>RAS-to-CAS delay (derated)</td>
 *          <td>tRCD</td> <td>min</td>
 *          <td>tRCD + 1.875</td>
 *          <td>ns</td></tr>
 *      <tr><td>ACTIVATE-to- ACTIVATE<br>command period (derated) </td>
 *          <td>tRC</td> <td>min</td>
 *          <td>tRC + 3.75</td>
 *          <td>ns</td></tr>
 *      <tr><td>Row active time (derated)</td>
 *          <td>tRAS</td> <td>min</td>
 *          <td>tRAS + 1.875</td>
 *          <td>ns</td></tr>
 *      <tr><td>Row precharge time <br>(derated) </td>
 *          <td>tRP</td> <td>min</td>
 *          <td>tRP + 1.875</td>
 *          <td>ns</td></tr>
 *      <tr><td>Active bank A to <br>active bank B (derated) </td>
 *          <td>tRRD</td> <td>min</td>
 *          <td>tRRD + 1.875</td>
 *          <td>ns</td>
 *      </tr>
 *  </table>  
 * 
 *
 * 
 *  DQS2DQ Timing check necessity for retraining<br>
 *          <li> Stop the Controller</li>
 *          <li> Start DQS Oscillator</li>
 *          <li> Restart the controller</li>
 *          <li> Read the Result of DQS Oscillator (MRR MR19/MR18)</li>
 *          <li> calculate the need for retraining</li>
 * 
 *  
 ******************************************************************************/
 void Cy_Lpddr_MemoryMonitoring(void);


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_CheckForRetraining
 ***************************************************************************//**
 *  \brief Function feedbacks the necessity for retraining.
 * 
 *  This function must be called periodically after the LPDDR4 was initialized. 
 *  Function returns with true if at least one Channel requires a retraining.
 *  If retraining is required application software must trigger the retraining
 *  by calling \ref Cy_Lpddr_RequestDQS2DQRetrain()
 * 
 *  \return 
 * true retraining is necessary
 * false retraining not necessary
 * ****************************************************************************/
__INLINE bool Cy_Lpddr_CheckForRetraining(void)
{
    return ((g_stc_lpddr_context.retrainRequestCHA)||
            (g_stc_lpddr_context.retrainRequestCHB));
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WaitUserCMDReadyCHA
 ***************************************************************************//**
 * \brief Function which is waiting for the Channel A to become ready
 *        for receiving user commands. Channel must be activated in the
 *        LPDDR4 controller.Which channels should be activated during Controller
 *        Init can be set in .unDMCTL[12:11] in \ref cy_stc_lpddr4_config_t
 *        
 * Function is blocking untiL the CHA is ready for new commands
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type  \ref stc_LPDDR4_t
 * 
 * \return none
 ******************************************************************************/
__INLINE void Cy_Lpddr_WaitUserCMDReadyCHA(volatile stc_LPDDR4_t * base)
{
    while(base->LPDDR4_CORE.unOPSTT_CH0.stcField.u1USER_CMD_READY != 1);
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WaitUserCMDReadyCHB
 ***************************************************************************//**
 * \brief Function which is waiting for the Channel B to become ready
 *        for receiving user commands. Channel must be activated in the
 *        LPDDR4 controller.Which channels should be activated during Controller
 *        Init can be set in .unDMCTL[12:11] in \ref cy_stc_lpddr4_config_t
 *        
 * Function is blocking untiL the CHB is ready for new commands
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type  \ref stc_LPDDR4_t
 * 
 * \return none
 ******************************************************************************/
__INLINE void Cy_Lpddr_WaitUserCMDReadyCHB(volatile stc_LPDDR4_t * base)
{
    while(base->LPDDR4_CORE.unOPSTT_CH1.stcField.u1USER_CMD_READY != 1);
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WaitUserCMDReady
 ***************************************************************************//**
 * \brief Function which is waiting for the activated channels to become ready
 *        for receiving user commands. Which channels should be activated is set
 *        in .unDMCTL[12:11] in \ref cy_stc_lpddr4_config_t
 *  
 * Depending on the selected channel the function is waiting for the selected 
 * channel(s) to become ready for new commands. Each command/address channel 
 * has it´s own Operation Status Register ( opstt_ch0/opstt_ch1 ). This function
 * is waiting until the user_cmd_ready field of the corresponding Operation 
 * Status Register is set.<br>
 * 
 * <table class="doxtable">
 * <tr><th>Register Field </th>
 *      <th>Position</th><th>Reset </th>
 *      <th>Description </th></tr>
 * <tr><td>dram_pause</td>
 *      <td>[0] </td>
 *      <td>0</td>
 *      <td>DRAM Paused for PD - Channel x </td></tr>
 * <tr><td>user_cmd_ready</td>
 *      <td>[1] </td>
 *      <td>0</td>
 *      <td>User Command Ready - Channel x </td></tr>
 * <tr><td>bank_idle</td>
 *      <td>[9:2] </td>
 *       <td>0</td>
 *      <td>Bank Controller Idle (8 banks) - Channel x </td></tr>
 * <tr><td>xqr_empty </td>
 *      <td>[10] </td>
 *      <td>0</td>
 *      <td>Read Path Cross-over Queue Empty - Channel x </td></tr>
 * <tr><td>xqr_full</td>
 *      <td>[11] </td>
 *      <td>0</td>
 *      <td>Read Path Cross-over Queue Empty - Channel x </td></tr>
 * <tr><td>xqw_empty </td>
 *      <td>[12] </td>
 *      <td>0</td>
 *      <td>Write Path Cross-over Queue Empty - Channel x </td></tr>
 * <tr><td>xqw_full</td>
 *      <td>[13] </td>
 *      <td>0</td>
 *      <td>Write Path Cross-over Queue Empty - Channel x </td></tr>
 * </table>
 * 
 * 
 * \note This function is blocking as long as the channel is not free for
 *      new commands
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type 
 *       \ref stc_LPDDR4_t
 *
 * ****************************************************************************/
void Cy_Lpddr_WaitUserCMDReady(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_ReadModeRegister
 *************************************************************************//****
 *  \brief  This function is sending the request for a Mode Register Read the 
 *          specified channel of the LPDDR4 controller.
 *          
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 *
 *  \param  channel the channel to which the request shall be send 
 *          \ref cy_en_lpddr4_chan_sel_t. 
 * 
 *  \param  mr mode register to read . Trying to read mode registers which are 
 *          write only will result in invalid ( undefined ) data. User must take
 *          care that on mode registers are specified which are readable 
 * 
 *  \param  pointer to an struct \ref cy_stc_lpddr4_mrw_mrr_data_t for the 
 *          read data of ch0 and ch1 where the mode register value will be 
 *          written to 
 * 
 *  \return \ref cy_en_lpddr4_channel_status_t<br>
 *      CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *      CY_LPDDR4_SUCCESS   - if everything was ok<br>
 * ****************************************************************************/
 cy_en_lpddr4_retval_t Cy_Lpddr_ReadModeRegister(volatile stc_LPDDR4_t * base, 
                                                cy_en_lpddr4_mr_t mr, 
                                                cy_stc_lpddr4_mrw_mrr_data_t * registerValue);


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_RequestDQS2DQRetrain
 *************************************************************************//****
 *  \brief  This function is sending the request for DQS2DQ Retraining to the 
 *          LPDDR4 Controller
 *  
 *  After initial DRAM VREF-DQ training the tDQS2DQ delay in device 
 *  may drift due to voltage and temperature variation. Memory controller needs 
 *  to initiate DQS interval oscillator and then get the result to decide if 
 *  DQS2DQ retraining is needed. <br>If retraining is needed this function 
 *  triggers the request  to perform the controller internal retraining 
 *  algorithm.  The follwing steps will be performed to activate the retraining:
 *  DQS2DQ re-training 
 *  <ul>
 *   <li> Set appropriate DQSDQCR register fields: dir, dlymax, mpcrpt. 
 *   The value of dlymax must be greater than or equal to t_dqrpt. 
 *   (this is already done during  Controller init --> 
 *   \ref Cy_Lpddr_SetPhyDdrMode()</li>
 *   <li> Set following POM fields: dqsdqen=1, phyinit=1, and dfien=0 </li>
 *   <li> Waits for POS[phyinitc] and POS[r*_dqsdqc] for training complete </li>
 *   <li> Set POM: dqsdqen=0, phyinit=0, and dfien=1 for normal operation </li>
 *  </ul>
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 * 
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 * ****************************************************************************/
 cy_en_lpddr4_retval_t Cy_Lpddr_RequestDQS2DQRetrain(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_PSVPFakeTraining
 *************************************************************************//****
 *  \brief  This function only used on PSVP as on the PSVP no real 
 *          training can be performed. As during the real retraining 
 *          the controller is stoppped. the function wait approx 10 us
 *          to simulate time needed for retraining. Driver Context retrain
 *          request is reset and controller is restarted
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 ******************************************************************************/
 cy_en_lpddr4_retval_t Cy_LPDDR4_PSVPFakeTraining(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_InitECCMemoryWithVal
 *************************************************************************//****
 *  \brief  This function is used to Init the Protected Area when ECC is enabled
 *          It initialize the memory with value passed to initValue
 *
 *
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *  \param  initValue uint32_t value used to init the ECC Memory
 *
 ******************************************************************************/
void Cy_LPDDR4_InitECCMemoryWithVal(volatile stc_LPDDR4_t * base,uint32_t initValue);

/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_InitECCMemory
 *************************************************************************//****
 *  \brief  This function is used to Init the Protected Area when ECC is enabled
 *          It initialize the memory with 0xff
 *
 *
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *
 ******************************************************************************/
void Cy_LPDDR4_InitECCMemory(volatile stc_LPDDR4_t * base);

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WriteModeRegister
 *************************************************************************//****
 * \brief   This function is sending the request for a Mode Register Write 
 *          ( MRW ) to  the specified channel of the LPDDR4 controller.
 * 
 *  This function sends a MRW command to the selected channel. This is done by 
 *  programming the command to the UCI ( User Command Interface) register of the
 *  LPDDR4 controller. Command can be send to either a single channel or to 
 *  both channels at once. 
 * 
 * \note    It´s not allowed to send MRW commands when LPDDR4 controller is in 
 *          run mode. So before using this command the controller must be 
 *          stopped by sending a CY_LPDDR4_USER_CMD_STOP . During the time the 
 *          controller is stopped no memory acccesses are possible until the 
 *          controller was restarted with CY_LPDDR4_USER_CMD_RUN. 
 *
 * <table class="doxtable">
 * <tr>
 *   <th>Register Field</th>
 *   <th>Position</th>
 *   <th>Reset</th>
 *   <th>Description</th>
 * </tr>
 * <tr>
 *   <td>uci_cmd_op</td>
 *   <td>[3:0]</td>
 *   <td> 0</td>
 *   <td>User Command Opcode</td>
 * </tr>
 * <tr>
 *   <td>uci_cmd_chan</td>
 *   <td>[5:4] </td>
 *   <td>0</td>
 *   <td>User Command Channel <br>
 *       2’b01: Channel A <br>
 *       2’b10: Channel B <br>
 *       2’b11: Both Channels </td>
 * </tr>
 * <tr>
 *   <td>uci_mr_sel </td>
 *   <td>[11:6] </td>
 *   <td>0</td>
 *   <td>Register Select Argument for MRS/MRW/MRR Command </td>
 * </tr>
 * <tr>
 *   <td>uci_mr_op</td>
 *   <td>[19:12] </td>
 *   <td>0</td>
 *   <td>Register Opcode (MRW, MPC) </td>
 * </tr>
 * </table>
 *
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 *
 * \param  mr mode register to write to  
 * 
 * \param   registerValue pointer to the value which shall be programmed to the 
 *          mode register
 * 
 * \return \ref cy_en_lpddr4_channel_status_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 ******************************************************************************/
 cy_en_lpddr4_retval_t Cy_Lpddr_WriteModeRegister(volatile stc_LPDDR4_t * base, 
                                                cy_en_lpddr4_mr_t mr, 
                                                uint8_t * registerValue);

/*******************************************************************************
 * Function Name: Cy_Lpddr_SendCmd
 *************************************************************************//****
 * 
 * \brief Sends a Command to the LPDDR Controller
 *
 * This function is used to send a comand to the LPDDR4 controller. The command
 * is programmed to the controllers UCI (User Command Register) Register.Access
 * user command register is almost the same as writeable register. Only one 
 * difference is, users must check if user_cmd_ready register asserts 
 * \ref Cy_Lpddr_WaitUserCMDReady() before starting the transaction. 
 *
 * \note The commands: USER_CMD_STOP, USER_CMD_PDE (Enter power-down run mode) 
 *       and USER_CMD_MRR( for single channel) are the only commands allowed in 
 *       RUN mode. For all other commands user must stop the controller first.
 *       When controller is stopped no memory access is possible.
 *
 * 
 * <table class="doxtable">
 * <tr>
 *   <th>Register Field</th>
 *   <th>Position</th>
 *   <th>Reset</th>
 *   <th>Description</th>
 * </tr>
 * <tr>
 *   <td>uci_cmd_op</td>
 *   <td>[3:0]</td>
 *   <td> 0</td>
 *   <td>User Command Opcode</td>
 * </tr>
 * <tr>
 *   <td>uci_cmd_chan</td>
 *   <td>[5:4] </td>
 *   <td>0</td>
 *   <td>User Command Channel <br>
 *       2’b01: Channel A <br>
 *       2’b10: Channel B <br>
 *       2’b11: Both Channels </td>
 * </tr>
 * <tr>
 *   <td>uci_mr_sel </td>
 *   <td>[11:6] </td>
 *   <td>0</td>
 *   <td>Register Select Argument for MRS/MRW/MRR Command </td>
 * </tr>
 * <tr>
 *   <td>uci_mr_op</td>
 *   <td>[19:12] </td>
 *   <td>0</td>
 *   <td>Register Opcode (MRW, MPC) </td>
 * </tr>
 * </table>
 *
 *
 * \note Once a wrong user command is issued, controller will assert int_gc_fsm 
 *       signal if dmcfg_int_gc_fsm_en is  enabled. In this case, users must 
 *       clear the interrupt flag before issuing next user command. 
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 *
 * \param cmd user command to send to the controller over UCI Register 
 *        \ref  cy_en_lpddr4_uci_cmd_op_t
 *
 *
 * \param   mr if CMD is a MRR/MRW this parameter specifies the MR to read or 
 *          write \ref cy_en_lpddr4_mr_t .For none MRW / MRR command set this 
 *          parameter to CY_LPDDR4_NONE_MR
 *
 * \param mrval is the value which shall be written to mr in case of MRW.
 *        For none MRW cpommand set to "0"
 *
 * \return retval 
 *   CY_LPDDR4_SUCCESS if everything was ok<br>
 *   CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function <br>
 *   CY_LPDDR4_TIMEOUT if command channel is busy and command couldn´t be 
 *                     executed<br>
 *   CY_LPDDR4_ERROR   if the command caused a failure in the controller  
 *
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SendCmd(volatile stc_LPDDR4_t * base,
                                        cy_en_lpddr4_uci_cmd_op_t cmd,
                                        cy_en_lpddr4_mr_t mr,
                                        uint32_t mrval);

/*******************************************************************************
 * Function Name: void Cy_Lpddr_IsControllerEnabled()
 *************************************************************************//****
 *
 * \brief    Function returns the status of the main enabler for the LPDDR4 IP 
 *
 * \param    Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 * \return   \ref cy_en_lpddr4_onoff_t<br>
 *           CY_LPDDR4_ONOFF_ENABLE   when main enabler is set<br>
 *           CY_LPDDR4_ONOFF_DISABLE  when main enabler is not set<br>
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsControllerEnabled(volatile stc_LPDDR4_t * base);

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_IsAutoZQEnabled()
 ***************************************************************************//**
 *  \brief Function returns the Auto ZQ Calibration Feature activation status
 * 
 * Function returns the Auto ZQ Calibration Feature activation status. 
 * The status is read from the controllers DMCFG register.
 * 
 * \note Before calling this function the LPDDR4 IP must be enabled as the 
 *      IP Core Registers are not accessible otherwise. 
 * 
 * \param    Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref 
 *  true  - if automatic ZQ calibration feature is activated.
 *  false - if automatic ZQ calkiobration feature is de-activated
 ******************************************************************************/
__INLINE  bool Cy_Lpddr_IsAutoZQEnabled(volatile stc_LPDDR4_t * base)
{
    return (bool) (base->LPDDR4_CORE.unDMCFG.stcField.u1ZQ_AUTO_EN);
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_DisableAutoZQ()
 ***************************************************************************//**
 *  \brief Function to disable the Automatic ZQ Calibration Feature in 
 *         DMCFG Register
 * 
 * Function to disable the Automatic ZQ Calibration Feature in DMCFG 
 * Register of the Memory Controller.
 * 
 * \note Before calling this function the LPDDR4 IP must be enabled as the 
 *       IP Core Registers are not accessible otherwise.
 * 
 * \param  Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 *  CY_LPDDR4_SUCCESS  - if everything was ok
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed
 ******************************************************************************/
__INLINE cy_en_lpddr4_retval_t Cy_Lpddr_DisableAutoZQ(volatile stc_LPDDR4_t * base)
{
    if(base == NULL)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    base->LPDDR4_CORE.unDMCFG.stcField.u1ZQ_AUTO_EN = 0x0;
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_EnableAutoZQ()
 ***************************************************************************//**
 *  \brief Function to enable the Automatic ZQ Calibration Feature in 
 *         DMCFG Register
 * 
 * Function to disable the Automatic ZQ Calibration Feature in DMCFG Register 
 * of the Memory Controller.
 * 
 * \note Before calling this function the LPDDR4 IP must be enabled as the 
 *      IP Core Registers are not accessible otherwise.
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 *  CY_LPDDR4_SUCCESS  - if everything was ok
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed
 ******************************************************************************/
__INLINE cy_en_lpddr4_retval_t Cy_Lpddr_EnableAutoZQ(volatile stc_LPDDR4_t * base)
{
    if(base == NULL)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    base->LPDDR4_CORE.unDMCFG.stcField.u1ZQ_AUTO_EN = 0x1;
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
*                        LPDDR4 Driver context stuff starts here
*******************************************************************************/
/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetContextInitState
 ***************************************************************************//**
 * \brief  
 * Function to read the driver context controllerInitState
 * 
 * \return \ref cy_en_lpddr4_cntrl_init_state_t
 *  
 ******************************************************************************/
__INLINE  cy_en_lpddr4_cntrl_init_state_t Cy_Lpddr_GetContextInitState(void)
{
    return g_stc_lpddr_context.controllerInitState;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetContextChannelConfig
 ***************************************************************************//**
 * \brief This function can be used to read the ChannelConfig information 
 * from the driver context status.
 * This is the information which channels were initiliazed during the 
 * ControllerInit.  Which channels will be initialize can be set by the user 
 * \ref cy_stc_lpddr4_config_t config structure .unDMCTL[12:11]
 * 
 * 
 * \return \ref cy_en_lpddr4_chan_sel_t which channels where configured
 * during Controller Init
 * CY_LPDDR4_CHAN_SEL_A  - Channel A 
 * CY_LPDDR4_CHAN_SEL_B  - Channel B 
 * CY_LPDDR4_CHAN_SEL_AB - Both Channels 
 ******************************************************************************/
__INLINE cy_en_lpddr4_chan_sel_t Cy_Lpddr_GetContextChannelConfig(void)
{
    return g_stc_lpddr_context.channelActive;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetCurrenGSMState
 ***************************************************************************//**
 * \brief This function can be used to read the GSM status from the driver context
 * It returns the mode of the Controller Global State Machine
 * 
 * 
 * \return \ref cy_en_lpddr4_cntrl_gsm_state_t
 * 
 *  Refer State Diagram for more information 
 *  CY_LPDDR4_CNTRL_GSM_STATE_STOP          GSM STOP MODE 
 *  CY_LPDDR4_CNTRL_GSM_STATE_RUN           GSM RUN MODE
 *  CY_LPDDR4_CNTRL_GSM_STATE_SELFREFRESH   GSM SR MODE
 *  CY_LPDDR4_CNTRL_GSM_STATE_POWERDOWN     GSM PD MODE 
 ******************************************************************************/
__INLINE cy_en_lpddr4_cntrl_gsm_state_t Cy_Lpddr_GetCurrenGSMState(void)
{
    return (g_stc_lpddr_context.controllerState);
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetContextLastCommand
 ***************************************************************************//**
 * \brief This function returns the last User command which was send 
 * to the controller over UCI Register
 * 
 * \return  return the last command \ref cy_en_lpddr4_uci_cmd_op_t
 ******************************************************************************/
__INLINE cy_en_lpddr4_uci_cmd_op_t Cy_Lpddr_GetContextLastCommand(void)
{
    return (cy_en_lpddr4_uci_cmd_op_t) g_stc_lpddr_context.lastUCICommand.stcField.u4CMD_OP;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetPreviousGSMState
 ***************************************************************************//**
 * \brief This function can be used to read the GSM status from the driver context
 * It returns the mode of the Conntroller Global State Machine
 * 
 * 
 * \return \ref cy_en_lpddr4_cntrl_gsm_state_t
 * 
 *  Refer State Diagram for more information 
 *  CY_LPDDR4_CNTRL_GSM_STATE_STOP          GSM STOP MODE 
 *  CY_LPDDR4_CNTRL_GSM_STATE_RUN           GSM RUN MODE
 *  CY_LPDDR4_CNTRL_GSM_STATE_SELFREFRESH   GSM SR MODE
 *  CY_LPDDR4_CNTRL_GSM_STATE_POWERDOWN     GSM PD MODE 
 ******************************************************************************/
__INLINE cy_en_lpddr4_cntrl_gsm_state_t Cy_Lpddr_GetPreviousGSMState(void)
{
    return (g_stc_lpddr_context.controllerStatePrev);
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SyncGSMStateOnFault
 ***************************************************************************//**
 * \brief This function must be called in the fault handler for GSM. It 
 * resynchronizes GSM status to the previous GSM state as the last command which was
 * sent caused an error and the GSM State did not change as expected.
 * 
 ******************************************************************************/
__INLINE void Cy_Lpddr_SyncGSMStateOnFault(void)
{
    cy_en_lpddr4_uci_cmd_op_t locLastCommand = Cy_Lpddr_GetContextLastCommand();
    /*check if last command is one which should have changes the GSM State
      Stop=0,RUN=1,SRE=2,SRX=3,PDE=4,PDX=5*/
    if(locLastCommand <= CY_LPDDR4_UCI_CMD_OP_USER_CMD_PDX)
    {
        g_stc_lpddr_context.controllerState = g_stc_lpddr_context.controllerStatePrev;
        g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerStatePrevRecov; 
    }
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetTimingDeratingState
 ***************************************************************************//**
 * \brief This function can be used to read the timing derating status 
 *        from the driver context.
 * 
 * Timing derating will be activated once a temperature of external memory device 
 * is reached and the memory is reuesting derating over MR4.
 * 
 * 
 * \return \ref cy_en_lpddr4_MR4_derate_state_t
 * CY_LPDDR4_DERATING_NOT_ACTIVE
 * CY_LPDDR4_DERATING_ACTIVE
 ******************************************************************************/
__INLINE cy_en_lpddr4_MR4_derate_state_t Cy_Lpddr_GetTimingDeratingState(void)
{
    return (g_stc_lpddr_context.deratingStatus);
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_CheckMemoryTemperatureState
 ***************************************************************************//**
 *  \brief Function to read the external memory temperature operational state.
 * 
 * The functiomn return the external memory temperature operational state as 
 * read from memory mode register MR4 . The tempearture state is 
 * updated every time the interval when \ref Cy_Lpddr_MemoryMonitoring() is 
 * called.The intervall time  depends on the system temperature gradient of the 
 * system and system response delay  of the system. 
 * The resulting intervall for calling the \ref Cy_Lpddr_MemoryMonitoring() 
 * can be calculated with \ref Cy_Lpddr_CalcMR4ReadInterval(systemResponseDelay,
 * temperatureGradient)
 * 
 *  \return \ref cy_en_lpddr4_memory_operational_temperature_t<br>
 * 
 * CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK -> temperature is within Limits
 * CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_LOW -> temperature below lower limit
 * CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_HIGH -> above upper limit
 * ****************************************************************************/
__INLINE cy_en_lpddr4_memory_operational_temperature_t Cy_Lpddr_CheckMemoryTemperatureState(void)
{
    return ((g_stc_lpddr_context.operationTemperatureState));
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_GetContextRetrainDirection
 ***************************************************************************//**
 *  \brief Function to read the retraining direction from driver context
 * 
 *  \return \ref cy_en_lpddr4_dqsdqcr_dir_t<br>
 * CY_LPDDR4_DQSDQCR_DIR_DOWN <br>
 * CY_LPDDR4_DQSDQCR_DIR_UP   <br>
 * ****************************************************************************/
__INLINE  cy_en_lpddr4_dqsdqcr_dir_t Cy_Lpddr_GetContextRetrainDirection(void)
{
    return ((g_stc_lpddr_context.retrainDirection));
}



/*******************************************************************************
 *  Function Name:  Cy_Lpddr_GetContextRetrainActive
 ***************************************************************************//**
 *  \brief Function to check if a DQS2DQ re-training is active
 * 
 *  \return \ref retraining active state<br>
 *      false - DQS2DQ retraining not active<br>
 *      true  - DQS2DQ retraining active<br>
 * ****************************************************************************/
__INLINE  bool Cy_Lpddr_GetContextRetrainActive(void)
{
    return ((g_stc_lpddr_context.dqs2dqRetrainActive));
}






/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerContextRefreshRateStrategyFixed
 ***************************************************************************//**
 * \brief Function to set the refreshRateStrategy in the Driver Context
 *        to CY_LPDDR_REFRESHRATE_FIXED
 *
 * With refreshRateStrategy in driver context set to fixedc the refresh rate
 * is fixed at 0.25 * tREFI
 *  
 * \return none
 ******************************************************************************/
__INLINE void Cy_Lpddr_SetCntrlrCntxtRefRtStratFixed(void)
{
    g_stc_lpddr_context.refreshRateStrategy = CY_LPDDR_REFRESHRATE_FIXED;
    g_stc_lpddr_context.refreshStrategyChanged = CY_LPDDR4_STRATEGY_CHANGED;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetCntrlrCntxtRefRtStrat
 ***************************************************************************//**
 * \brief Function to read the refreshRateStrategy from the Driver Context
 *  
 * \return cy_en_lpddr4_refreshrate_strategy_t refresh rate Strategy used to 
 *         set tREFI
 ******************************************************************************/
__INLINE cy_en_lpddr4_refreshrate_strategy_t Cy_Lpddr_GetCntrlrCntxtRefRtStrat(void)
{
    return g_stc_lpddr_context.refreshRateStrategy;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetCntrlrCntxtRefRtStratDynamic
 ***************************************************************************//**
 * \brief Function to trigger a refreshRateStrategy change to dynamic
 * 
 * The refreshRateStrategy is set to CY_LPDDR_REFRESHRATE_DYNAMIC and the 
 * to refreshStrategyChanged = true . refreshStrategyChanged = true will force a 
 * update of the the refreshrate even if TUF ( temperature Update Flag ) flag 
 * in MR4 is not set.
 * 
 * With refreshRateStrategy in driver context set to dynamic the refresh rate
 * is updated based on DRAM Request  MR4[2:0]
 *
 *  
 * \return none
 ******************************************************************************/
__INLINE void Cy_Lpddr_SetCntrlrCntxtRefRtStratDynamic(void)
{
    g_stc_lpddr_context.refreshRateStrategy = CY_LPDDR_REFRESHRATE_DYNAMIC;
    g_stc_lpddr_context.refreshStrategyChanged = CY_LPDDR4_STRATEGY_CHANGED;
}

// /*******************************************************************************
//  *  Function Name: Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq
//  ***************************************************************************//**
//  * \brief Function clears the refreshStrategyChanged flag in the driver context
//  *        to CY_LPDDR4_STRATEGY_UNCHANGED. After refreshRateStrategy was 
//  *        changed and refreshRate was updated this function mut be called.
//  *        
//  * 
//  * This flag in the driver context force a update of the refresh rate based on
//  * DRAM MR4[2:0] request even if no TUF(temperature update flag )is set in 
//  * the MR4 register. After the 
//  *
//  *  
//  * \return none
//  ******************************************************************************/
// __STATIC_INLINE void Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq(void)
// {
//     g_stc_lpddr_context.refreshStrategyChanged = CY_LPDDR4_STRATEGY_UNCHANGED;
// }


/*******************************************************************************
 *  Function Name: Cy_Lpddr_ChngCntrlrCntxtRefRtStrat
 ****************************************************************************//**
 * \brief Function to set the refreshRateStrategy in the Driver Context
 *        to CY_LPDDR_REFRESHRATE_FIXED
 *
 * * The refreshRateStrategy is set to CY_LPDDR_REFRESHRATE_FIXED and the 
 * to refreshStrategyChanged = true . refreshStrategyChanged = true will force a 
 * update of the the refreshrate. 
 * 
 * With refreshRateStrategy in driver context set to fixed the refresh rate
 * is programmed to CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI
 *  
 * \return cy_en_lpddr4_retval_t
 * CY_LPDDR4_INVALID_STATE if another change request is still pending
 * CY_LPDDR4_SUCCESS if change request was set
 ******************************************************************************/
__INLINE cy_en_lpddr4_retval_t Cy_Lpddr_ChngCntrlrCntxtRefRtStrat(void)
{
    /*if there is a pending startegy request directly return with INVALID_STATE
     as the Pending request must be processed first */
    if ( CY_LPDDR4_STRATEGY_UNCHANGED != g_stc_lpddr_context.refreshStrategyChanged )
    {
        return CY_LPDDR4_INVALID_STATE;
    }
    if(CY_LPDDR_REFRESHRATE_FIXED == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
    {
        Cy_Lpddr_SetCntrlrCntxtRefRtStratDynamic();
    }else
    {
        Cy_Lpddr_SetCntrlrCntxtRefRtStratFixed();
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_GetContextFixedUsrRefRate
 ***************************************************************************//**
 * \brief Function to read the fixed user refresh rate from the driver 
 *        context status
 * 
 * \return fixedUserRefreshRate uint32_t tREFI intervall in number of clock cycle
 *         at the current Frequency Set Point
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr_GetContextFixedUsrRefRate(void)
{
    return (g_stc_lpddr_context.fixedUserRefreshRate);
}


/*******************************************************************************
*                        LPDDR4 Driver context stuff ends here
*******************************************************************************/

/*******************************************************************************
*                        LPDDR 4 AXI Perf Count
*******************************************************************************/


/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_EnableAXIPerfCnt
 *************************************************************************//****
 *  \brief Set the Main Enabler for AXI Performance Counter functionallity
 *  
  * Sets the Main Enable for the AXI Performance Counter in the CTL[:31] Register
 * 
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_EnableAXIPerfCnt(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_DisableAXIPerfCnt
 *************************************************************************//****
 *  \brief Clears the Main Enabler for AXI Performance Counter functionallity
 *  
  * Clears the Main Enabler for the AXI Performance Counter in the CTL[:31] Register
 * 
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_DisableAXIPerfCnt(volatile stc_LPDDR4_t * base);



/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_IsAXIPerfCntEnabled
 ***************************************************************************//**
 *  \brief Returns the Enable state of the AXI Performance Counters 
 *  
 *  Returns the Status of the AXI Perf Counter Main Enabler State 
 *  which can be read from CTL[31] Register.
 * 
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 *  false - not enabled
 *  true  - enabled
 ******************************************************************************/
__INLINE bool Cy_Lpddr4_IsAXIPerfCntEnabled(volatile stc_LPDDR4_t * base)
{
    return ((bool) base->AXI_PERF_CNT.unCTL.stcField.u1ENABLED);
}



/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_StartAXIPerfCnt
 *************************************************************************//****
 *  \brief Start the AXI Performance Counter Measureing window for all MUs
 *  
 *  Writes the START Bitfield in TMR_CMD Register . Writing 1 to this field triggers 
 *  the start of the measurement window. When this field is written to 1,
 *  MU_TMR_STATUS.MEAS in the measurement units are set to 1, indicating that the 
 *  measurement window starts, and START is automatically cleared to 0. In each 
 *  measurement unit, all counters are cleared to 0, and the current timer count is 
 *  set to 1 and is incremented with each cycle of the prescaled clock during the
 *  measurment window. START has a higher priority than STOP. Writing 1 to this field 
 *  during the measurement window restarts the measurement window, clearing the 
 *  counters and restarting the timer.
 * 
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_StartAXIPerfCnt(volatile stc_LPDDR4_t * base);


/*******************************************************************************
 *  Function Name: Cy_Lpddr4_StopAllAXIPerfCnt
 *************************************************************************//****
 *  \brief Stop the AXI Performance Count Measurement
 *  
 *  Writing 1 to this field stops the measurement at the next clock edge of the
 *  prescaled clock. At this time, the timer count and all counters in the measurement 
 *  units keep their current values. STOP is automatically cleared to 0 immediately. 
 *  STOP has a lower priority than START.
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_StopAXIPerfCnt(volatile stc_LPDDR4_t * base);



/*******************************************************************************
 *  Function Name: Cy_Lpddr4_IsAXIPerfMUIdxMeasure
 ***************************************************************************//**
 *  \brief Function to get the status of a AXI Perf Count MU unit 
 *  
 * It return the status of the measuring unit with index idx. The status is 
 * read from the MEAS-Bit in TMR_STATUS[31] Register  
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return 
 * true  - if MU is measuring
 * false - if MU is not measuring
 ******************************************************************************/
__INLINE bool Cy_Lpddr4_IsAXIPerfMUIdxMeasure(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((bool) base->AXI_PERF_CNT.MU[muIdx].unTMR_STATUS.stcField.u1MEAS);
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxTmr
 ***************************************************************************//**
 *  \brief Reads the TMR value from a MUs TMR_STATUS Register
 * 
 *  While the MU is running the function will read the TMR value with 0
 *  After the end of the measurement window (MEAS = 0), this field reads the 
 *  count of the timer at the end of the measurement window, unless the timer 
 *  is off (TMR_CTL.TMR = 0); in this case, this field reads 0. The duration of 
 *  the measurement window is TMR_STATUS.TMR * 2**TMR_CTL.PSC clock periods of 
 *  the selected AXI port.
 * 
 *
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t TMR value from TMR_STATUS[27:0] Register
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxTmr(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unTMR_STATUS.stcField.u28TMR);
}





/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxOTCnt
 ***************************************************************************//**
 *  \brief Function to read the Counter for Accumulated Outstanding Transactions 
 *         from the AXI Perf. MU with index muIDX
 * 
 *  As long as the MU is measuring TMR_STATUS.MEAS = 1 this function will read 0
 * 
 *  While TMR_STATUS.MEAS is 0, this field reads the accumulated number of 
 *  outstanding read or write transactions during the measurement window, 
 *  depending on FILTER.WRITE. This counter is cleared by TMR_CMD.START. During 
 *  the measurement window, this counter is incremented in each AXI clock cycle 
 *  by the current number of outstanding read or write transactions. If 
 *  FILTER.WRITE = 0, this counter accumulates the outstanding read 
 *  transactions.  If FILTER.WRITE = 1, this counter accumulates the 
 *  outstanding write transactions. Instead of overflowing, this counter is 
 *  clipped at 0xFFFFFFFF. This means that if at the end of the measurement 
 *  window, a count of 0xFFFFFFFF is read, then it is very likely that this 
 *  counter has overflowed. In this case, a shorter measurement window should 
 * be used.
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has 
 *         index LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t Counter Value of the respective counter of the selected MU
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxOTCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unOT_AC.stcField.u32CNT);
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxAddrCnt
 ***************************************************************************//**
 *  \brief  Function to read the Address Transfer Counter  from
 *         the AXI Perf. MU with index muIDX 
 * 
 *  As long as the MU is measuring TMR_STATUS.MEAS = 1 this function will read 0
 *  
 *  While TMR_STATUS.MEAS is 0, this field reads the value of the address 
 *  transfer counter. This counter shows the number of filtered address 
 *  transfers during the measurement window. This counter is cleared by 
 *  TMR_CMD.START. During the measurement window, this counter is incremented 
 *  by each address transfer that passes the filter defined by FILTER and 
 *  FILTER_MASK. Instead of overflowing, this counter is clipped at 
 *  0xFFFFFFFF. This means that if at the end of the measurement window, a 
 *  count of 0xFFFFFFFF is read, then it is very likely that this counter has 
 *  overflowed. In this case, a shorter measurement window should be used.
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t Counter Value of the respective counter of the selected MU
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxAddrCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unADDR_CNT.stcField.u32CNT);
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxAddrStallCnt
 ***************************************************************************//**
 *  \brief  Function to read the Address Stall  Counter  from
 *          the AXI Perf. MU with index muIDX 
 * 
 *  As long as the MU is measuring TMR_STATUS.MEAS = 1 this function will read 0
 *  
 *  While TMR_STATUS.MEAS is 0, this field reads the value of the address stall 
 *  counter. This counter shows the number of filtered address stall cycles 
 *  during the measurement window. This counter is cleared by TMR_CMD.START. 
 *  During the measurement window, this counter is incremented by each address 
 *  stall cycle that passes the filter defined by FILTER and FILTER_MASK. 
 *  Instead of overflowing, this counter is clipped at 0xFFFFFFFF. This means 
 *  that if at the end of the measurement window, a count of 0xFFFFFFFF is 
 *  read, then it is very likely that this counter has overflowed. In this case, 
 *  a shorter measurement window should be used.
 *  
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t Counter Value of the respective counter of the selected MU
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxAddrStallCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unADDR_STALL_CNT.stcField.u32CNT);
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxDataCnt
 ***************************************************************************//**
 *  \brief Function to read the Data Transfer Counter  from
 *          the AXI Perf. MU with index muIDX 
 * 
 *  As long as the MU is measuring TMR_STATUS.MEAS = 1 this function will read 0
 *  
 * While TMR_STATUS.MEAS is 0, this field reads the value of the data transfer 
 * counter. This counter shows the number of filtered data transfers during the 
 * measurement window. This counter is cleared by TMR_CMD.START. During the 
 * measurement window, this counter is incremented by each data transfer that 
 * passes the filter defined by FILTER and FILTER_MASK. It should be noted that 
 * if FILTER.WRITE = 1, the write data are not filtered by master ID and upper 
 * transaction ID. Instead of overflowing, this counter is clipped at 
 * 0xFFFFFFFF. This means that if at the end of the measurement window, a 
 * count of 0xFFFFFFFF is read, then it is very likely that this counter has 
 * overflowed. In this case, a shorter measurement window should be used.
 *  
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t Counter Value of the respective counter of the selected MU
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxDataCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unDATA_CNT.stcField.u32CNT);
}
/*******************************************************************************
 *  Function Name: Cy_Lpddr4_GetAXIPerfCntMUIdxDataStallCnt
 ***************************************************************************//**
 *  \brief Function to read the Data Transfer Counter from
 *          the AXI Perf. MU with index muIDX 
 * 
 *  As long as the MU is measuring TMR_STATUS.MEAS = 1 this function will read 0
 *  
 *  While TMR_STATUS.MEAS is 0, this field reads the value of the data stall 
 *  counter.This counter shows the number of filtered data stall cycles during 
 *  the measurement window. This counter is cleared by TMR_CMD.START. While 
 *  TMR_STATUS.MEAS = 1, this counter is incremented by each data stall cycle 
 *  that passes the filter defined by FILTER and FILTER_MASK. It should be 
 *  noted that if FILTER.WRITE = 1, the write data are not filtered by 
 *  master ID and upper transaction ID. Instead of overflowing, this counter is 
 *  clipped at 0xFFFFFFFF. This means that if at the end of the measurement 
 *  window, a count of 0xFFFFFFFF is read, then it is very likely that this 
 *  counter has overflowed. In this case, a shorter measurement window should 
 *  be used.
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param muIdx index of the Measuring Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 * \return uint32_t Counter Value of the respective counter of the selected MU
 * 
 ******************************************************************************/
__INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxDataStallCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx)
{
    return ((uint32_t) base->AXI_PERF_CNT.MU[muIdx].unDATA_STALL_CNT.stcField.u32CNT);
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr4_ConfigAXIPerfMUIdx
 *************************************************************************//****
 *  \brief Function to Configure a AXI Perf Counter Measureing Unit
 *  
 *  \note MU shall not be configured when AXI_PERF_CNT_CTL.ENABLED as this can 
 *        lead to unpredicted measurement results and can cause bus errors.
 *  
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param index of the Measureing Unit . First MU has index 0 last MU has index
 *         LPDDR4_AXI_PERF_CNT_MU_NR - 1.
 * 
 *  \param \ref stc_lpddr4_axi_perf_mu_config_t which is a structure holding all 
 *         values neccessary to configure the MU.
 *  
 *  \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 * CY_LPDDR4_INVALID_STATE when AXI_PERF_CNT_CTL.ENABLED
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_ConfigAXIPerfMUIdx(volatile stc_LPDDR4_t * base,
                                                    uint32_t muIdx, 
                                                    stc_lpddr4_axi_perf_mu_config_t* muConfig );


/*******************************************************************************
*                        END of LPDDR 4 AXI Perf Count
*******************************************************************************/


void Cy_Lpddr_ClearGCFSM_DMCFG(void);
/** \} group_lpddr_functions */



#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __CY_LPDDR4_H__ */

/** \} group_lpddr */

/* [] END OF FILE */
