/***************************************************************************//**
* \file cy_lpddr4.c
*   $Date: 2023-01-24 14:40:08 +0530 (Tue, 24 Jan 2023) $
*   $Revision: 320852 $
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




#include "cy_lpddr4.h"
#include "syslib/cy_syslib.h"
#include "systick/cy_systick.h"
#include "../../../../common/src/drivers/gpio/cy_gpio.h"
//#include "cy_dramconfsel.h"

#define SVN_REV_CY_LPDDR4_C 		"$Revision: 320852 $"
#define SVN_DATE_CY_LPDDR4_C		"$Date: 2023-01-24 14:40:08 +0530 (Tue, 24 Jan 2023) $"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/**
 *  \brief LPDDR4 driver context structure 
 **/
cy_stc_lpddr4_context_t g_stc_lpddr_context;


//cy_stc_lpddr4_dqsosc_read_data_t g_stc_lpddr_context.actualDQSOsc = {0,0};


// /** 
// *  structure holding the PLL800 Limits  
// * 
// */
// static const cy_stc_lpddr4_pll_limitation_t g_limPll800M =
// {
//     .maxFpd         = PLL_800M_MAX_PFD_FREQ,
//     .minFpd         = PLL_800M_MIN_PFD_FREQ,
//     .maxFvco        = PLL_800M_MAX_FVCO,
//     .minFvco        = PLL_800M_MIN_FVCO,
//     .maxFoutput     = PLL_800M_MAX_OUT_FREQ,
//     .minFoutput     = PLL_800M_MIN_OUT_FREQ,
//     .maxRefDiv      = PLL_800M_MAX_REF_DIV,
//     .minRefDiv      = PLL_800M_MIN_REF_DIV,
//     .maxFeedBackDiv = PLL_800M_MAX_FB_DIV,
//     .minFeedBackDiv = PLL_800M_MIN_FB_DIV,
//     .maxOutputDiv   = PLL_800M_MAX_OUTPUT_DIV,
//     .minOutputDiv   = PLL_800M_MIN_OUTPUT_DIV,
// };
/*Controller Related stuff*/
static cy_en_lpddr4_retval_t Cy_Lpddr_CheckPhyInitComplete(volatile stc_LPDDR4_t * base);
static cy_en_lpddr4_retval_t Cy_Lpddr_ControllerEnable(volatile stc_LPDDR4_t * base);
static cy_en_lpddr4_retval_t Cy_Lpddr_ControllerDisable(volatile stc_LPDDR4_t * base) __attribute__((unused));
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupModeRegisters(volatile stc_LPDDR4_t * base,
                                                  const cy_stc_lpddr4_config_t * config );
static void Cy_Lpddr_PerformTimingDerating(volatile stc_LPDDR4_t * base,
                                    cy_en_lpddr4_MR4_derate_state_t timingDerate)__attribute__((unused));
static cy_en_lpddr4_retval_t Cy_Lpddr_PhyInit(volatile stc_LPDDR4_t * base,  
                                                const cy_stc_lpddr4_config_t * config);
static cy_en_lpddr4_retval_t Cy_Lpddr_LPDDR4Training(volatile stc_LPDDR4_t * base,
                                                            const cy_stc_lpddr4_config_t * config)__attribute__((unused));
static cy_en_lpddr4_retval_t Cy_Lpddr_LPDDR4StepwiseTraining(volatile stc_LPDDR4_t * base,
                                                         const cy_stc_lpddr4_config_t * config)__attribute__((unused));
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupCAIOCntrlRegs(volatile stc_LPDDR4_t * base,
                                                  const cy_stc_lpddr4_config_t * config);
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupDataSliceIOCntrlRegs(volatile stc_LPDDR4_t * base,
                                                         const cy_stc_lpddr4_config_t * config);
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupDLL(volatile stc_LPDDR4_t * base,
                                        const cy_stc_lpddr4_config_t * config);
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyTiming(volatile stc_LPDDR4_t * base,
                                            const cy_stc_lpddr4_config_t * config );
static   cy_en_lpddr4_retval_t Cy_Lpddr_SetupBISTRegister(volatile stc_LPDDR4_t * base,
                                                 const cy_stc_lpddr4_config_t * config )__attribute__((unused));
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyDdrMode(volatile stc_LPDDR4_t * base,
                                             const cy_stc_lpddr4_config_t * config );
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyFrequency(volatile stc_LPDDR4_t * base,
                                               const cy_stc_lpddr4_pll800_manual_config_t *config,
                                               cy_en_lpddr4_fsp_t fsp);
static void Cy_Lpddr_PhyProceed(volatile stc_LPDDR4_t * base);
static cy_en_lpddr4_retval_t Cy_Lpddr_PhyReload(volatile stc_LPDDR4_t * base, 
                                        const cy_stc_lpddr4_config_t * config,
                                        bool phyinit,bool fs,bool gten,bool wrlvlen,
                                        bool vrefdqwren,bool vrefdqrden,bool rdlvlen,
                                        bool dlyevalen,bool sanchken,
                                        bool vrefcaen,bool clklocken); 

static cy_en_lpddr4_retval_t Cy_Lpddr_SetAdressConfig(volatile stc_LPDDR4_t * base,
                                               const cy_stc_lpddr4_config_t * config );
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupArbiterMode(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config );
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupECCRegisters(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config );
static cy_en_lpddr4_retval_t Cy_Lpddr_SetQoSPrios(volatile stc_LPDDR4_t * base,
                          const cy_stc_lpddr4_config_t * config);
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPTSRRegisters(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config );




extern __INLINE bool Cy_Lpddr_GetInteruptStatusCHA(volatile stc_LPDDR4_t * base);
extern __INLINE bool Cy_Lpddr_GetInteruptStatusCHB(volatile stc_LPDDR4_t * base);
/* Automatic ZQ Calibration Feature on/off */
extern __INLINE  bool Cy_Lpddr_IsAutoZQEnabled(volatile stc_LPDDR4_t * base);
extern __INLINE cy_en_lpddr4_retval_t Cy_Lpddr_DisableAutoZQ(volatile stc_LPDDR4_t * base);
extern __INLINE cy_en_lpddr4_retval_t Cy_Lpddr_EnableAutoZQ(volatile stc_LPDDR4_t * base);

/*User Command interface*/
extern __INLINE void Cy_Lpddr_WaitUserCMDReadyCHA(volatile stc_LPDDR4_t * base);
extern __INLINE void Cy_Lpddr_WaitUserCMDReadyCHB(volatile stc_LPDDR4_t * base);


/* PLL related stuff */
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllOutputDisable(volatile stc_LPDDR4_t * base);
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllOutputEnable(volatile stc_LPDDR4_t * base);
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_Pll800MManualConfigure(volatile stc_LPDDR4_t * base, 
                                                      const cy_stc_lpddr4_pll800_manual_config_t *config)
                                                       __attribute__((unused));
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_SetPllOutputDivider(volatile stc_LPDDR4_t * base,
                                                    uint32_t out_div) __attribute__((unused));
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_SelectPllVcoClock(volatile stc_LPDDR4_t * base,
                                                  cy_en_lpddr4_vco_clk_sel_t vcoclksel) 
                                                  __attribute__((unused));
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllDisable(volatile stc_LPDDR4_t * base)
                                                    __attribute__((unused));
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllEnable(volatile stc_LPDDR4_t * base, uint32_t timeoutus)
                                                     __attribute__((unused));




/* Driver Context Stuff */
static void Cy_Lpddr_DriverContextInit(const cy_stc_lpddr4_config_t * config);
__STATIC_INLINE void Cy_Lpddr_SetControllerStopped(void);
__STATIC_INLINE void Cy_Lpddr_SetContextInitState(void);
__STATIC_INLINE void Cy_Lpddr_ClrContextInitState(void);
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainRequestCHA(void);
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainRequestCHB(void);
__STATIC_INLINE void Cy_Lpddr_ClrControllerContextRetrainRequestCHA(void);
__STATIC_INLINE void Cy_Lpddr_ClrControllerContextRetrainRequestCHB(void);
__STATIC_INLINE void Cy_Lpddr_UpdateControllerContextLastTraining(void);
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainInactive(void);
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainActive(void);
__STATIC_INLINE void Cy_Lpddr_SetControllerRunning(void);
__STATIC_INLINE void Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq(void);
extern __INLINE  cy_en_lpddr4_cntrl_init_state_t Cy_Lpddr_GetContextInitState(void);
extern __INLINE cy_en_lpddr4_chan_sel_t Cy_Lpddr_GetContextChannelConfig(void);
extern __INLINE cy_en_lpddr4_MR4_derate_state_t Cy_Lpddr_GetTimingDeratingState(void);
extern __INLINE cy_en_lpddr4_dqsdqcr_dir_t Cy_Lpddr_GetContextRetrainDirection(void);
extern __INLINE bool Cy_Lpddr_GetContextRetrainActive(void);
extern __INLINE cy_en_lpddr4_cntrl_gsm_state_t Cy_Lpddr_GetCurrenGSMState(void);
extern __INLINE cy_en_lpddr4_cntrl_gsm_state_t Cy_Lpddr_GetPreviousGSMState(void);
extern __INLINE void Cy_Lpddr_SyncGSMStateOnFault(void);
extern __INLINE cy_en_lpddr4_uci_cmd_op_t Cy_Lpddr_GetContextLastCommand(void);
extern __INLINE bool Cy_Lpddr_CheckForRetraining(void);
extern __INLINE cy_en_lpddr4_refreshrate_strategy_t Cy_Lpddr_GetCntrlrCntxtRefRtStrat(void);
extern __INLINE void Cy_Lpddr_SetCntrlrCntxtRefRtStratDynamic(void);
extern __INLINE void Cy_Lpddr_SetCntrlrCntxtRefRtStratFixed(void);
extern __INLINE cy_en_lpddr4_retval_t Cy_Lpddr_ChngCntrlrCntxtRefRtStrat(void);
extern __INLINE uint32_t Cy_Lpddr_GetContextFixedUsrRefRate(void);

/* Memory Monitoring */
static cy_en_lpddr4_retval_t Cy_Lpddr_ReadDQSOscValue(volatile stc_LPDDR4_t * base,
                                                    volatile cy_stc_lpddr4_dqsosc_read_data_t* dqsOSCCount);
static void Cy_Lpddr_StartDQSOscillator(volatile stc_LPDDR4_t * base);
static void Cy_Lpddr_RefreshandTimingAdjust(volatile stc_LPDDR4_t * base); 

/* AXI Perf Count Stuff */
extern __INLINE bool Cy_Lpddr4_IsAXIPerfCntEnabled(volatile stc_LPDDR4_t * base);
extern __INLINE bool Cy_Lpddr4_IsAXIPerfMUIdxMeasure(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxTmr(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxOTCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxAddrCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxAddrStallCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxDataCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);
extern __INLINE uint32_t Cy_Lpddr4_GetAXIPerfCntMUIdxDataStallCnt(volatile stc_LPDDR4_t * base,
                                             uint32_t muIdx);




// /****************************************************************************
// *       Memory Address handling functions
// ****************************************************************************/

/*******************************************************************************
 * Function Name: Cy_Lpddr_GetAxiToDramMapping
 ***************************************************************************//**
 * 
 * \brief    This function calculates the channel, bank, row, column addresses 
 *           information from a given AXI address based on the configuration 
 *           programmed in the condrollers address mapping registers ADDR0..ADDR5 
 *
 * To gain higher DRAM bus efficiency, LPDDR memory controller supports a 
 * flexible method to configure DRAM channel, bank, row and column address.<br>
 * AXI4 address can be separated into 4 segments: <br>
 * <ul>
 * <li>DRAM Channel</LI> <br>
 * <li>DRAM Bank</LI> <br>
 * <li>DRAM Row</LI> <br>
 * <li>DRAM Column</LI> <br>
 * </ul>
 * For better flexibility, each bit in AXI4 Address can be assigned to one of 
 * these segments. The table shows the address mapping registers. 
 *
 * <table class="doxtable">
 * <tr>
 * <th>Register</th>
 * <th>Register Field</th>
 * <th>Description</th>
 * <th>Width</th>
 * <th>Reset Value</th>
 * </tr>
 * <tr><td>addr0</td><td>col_addr_map_b0 </td>
 * <td> Address mapping for column bit 0</td> <td>5</td> <td> 0</td></tr>
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
 *
 Some lowest bits in AXI4 Address are unused, 
 * they must be assigned to 0.<br> The number of unused bits depend on the DRAM 
 * data width (Total DQ bits): <br>
 * <ul> 
 * <li>x8 (8-bit DQ): No bit is unused </LI><br>
 * <li>x16 (16-bit DQ): 1 bit is unused (axi4 address[0])</LI> <br>
 * <li>x32 (32-bit DQ): 2 bits are unused (axi4 address[1:0])</LI> <br>
 * <li>x64 (64-bit DQ): 3 bits are unused (axi4 address[2:0]) </LI><br>
 * </ul>
 *  
 *
 * 
 * \param[in]    pointer to the LPDDR4 controller \ref stc_LPDDR4_t
 *
 * \param[in]    axi_addr AXI offset address
 *
 * \param[out]    channel - memory channel number of the given axi address
 * 
 * \param[out]    bank - memory bank number of the given axi address
 *
 * \param[out]    row - memory row address of the given axi address
 *
 * \param[out]    col - memory column address of the given axi address
 *
 ******************************************************************************/
void Cy_Lpddr_GetAxiToDramMapping(volatile stc_LPDDR4_t * base, 
                                  const uint32_t axi_addr, uint32_t *channel, 
                                  uint32_t *bank, uint32_t *row, uint32_t *col)
{
    uint32_t bank_addr, row_addr, col_addr;
    
    /* Calculate the column address */
    col_addr = 0;
    col_addr |=  (axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B0) & 1;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B1) & 1) << 1;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B2) & 1) << 2;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B3) & 1) << 3;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B4) & 1) << 4;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B5) & 1) << 5;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B6) & 1) << 6;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B7) & 1) << 7;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B8) & 1) << 8;
    col_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B9) & 1) << 9;
    *col = col_addr;
    
    /* Calculate the row address */
    row_addr = 0;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B0) & 1) << 0;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B1) & 1) << 1;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B2) & 1) << 2;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B3) & 1) << 3;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B4) & 1) << 4;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B5) & 1) << 5;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B6) & 1) << 6;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B7) & 1) << 7;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B8) & 1) << 8;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B9) & 1) << 9;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B10) & 1) << 10;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B11) & 1) << 11;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B12) & 1) << 12;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B13) & 1) << 13;
    row_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B14) & 1) << 14;
    *row = row_addr;

    /* Calculate the bank address */
    bank_addr = 0;
    bank_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B0) & 1) << 0;
    bank_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B1) & 1) << 1;
    bank_addr |= ((axi_addr >> base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B2) & 1) << 2;
    *bank = bank_addr;

    /* Calculate the channel address */
    *channel = ((axi_addr >> base->LPDDR4_CORE.unADDR5.stcField.u5CHAN_ADDR_MAP_B0) & 1);
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_GetDramToAxiMapping
 ***************************************************************************//**
 * 
 * \brief   This function calculates the AXI address <u> offset </u> from given
 *          channel, bank, row, column addresses based on the addressmapping 
 *          set in the controller register ADDR0..ADDR5 
 *
 * To gain higher DRAM bus efficiency, LPDDR memory controller supports a 
 * flexible method to configure DRAM channel, bank, row and column address.<br>
 * AXI4 address can be separated into 4 segments: <br>
 * <ul>
 * <li>DRAM Channel</LI> <br>
 * <li>DRAM Bank</LI> <br>
 * <li>DRAM Row</LI> <br>
 * <li>DRAM Column</LI> <br>
 * </ul>
 * For better flexibility, each bit in AXI4 Address can be assigned to one of 
 * these segments. The table shows the address mapping registers. 
 *
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
 *
 * Some lowest bits in AXI4 Address are unused, 
 * they must be assigned to 0.<br> The number of unused bits depend on the DRAM 
 * data width (Total DQ bits): <br>
 * <ul> 
 * <li>x8 (8-bit DQ): No bit is unused </LI><br>
 * <li>x16 (16-bit DQ): 1 bit is unused (axi4 address[0])</LI> <br>
 * <li>x32 (32-bit DQ): 2 bits are unused (axi4 address[1:0])</LI> <br>
 * <li>x64 (64-bit DQ): 3 bits are unused (axi4 address[2:0]) </LI><br>
 * </ul>
 * \note As the function returns the offset only for the absolute address 
 *       you have to add the 0x8000 0000 for the absolute address. 
 * 
 * \param[in]    pointer to the LPDDR4 controller \ref stc_LPDDR4_t
 * 
 * \param[out]   axi_addr AXI address offset
 *
 * \param[in]    channel memory channel number
 * 
 * \param[in]    bank memory bank number
 *
 * \param[in]    row memory row address
 *
 * \param[in]    col memory column address
 * 
 *
 *******************************************************************************/
void Cy_Lpddr_GetDramToAxiMapping(volatile stc_LPDDR4_t * base, 
                                    uint32_t *axi_addr, const uint32_t channel, 
                                    const uint32_t bank,const uint32_t row, 
                                    const uint32_t col)
{
    uint32_t address_val = 0;
    /* Map the column address bits */
    address_val |= ( col       & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B0;
    address_val |= ((col >> 1) & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B1;
    address_val |= ((col >> 2) & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B2;
    address_val |= ((col >> 3) & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B3;
    address_val |= ((col >> 4) & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B4;
    address_val |= ((col >> 5) & 1) << base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B5;
    address_val |= ((col >> 6) & 1) << base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B6;
    address_val |= ((col >> 7) & 1) << base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B7;
    address_val |= ((col >> 8) & 1) << base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B8;
    address_val |= ((col >> 9) & 1) << base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B9;

    /* Map the row address bits */
    address_val |= ( row       & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B0;
    address_val |= ((row >> 1) & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B1;
    address_val |= ((row >> 2) & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B2;
    address_val |= ((row >> 3) & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B3;
    address_val |= ((row >> 4) & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B4;
    address_val |= ((row >> 5) & 1) << base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B5;
    address_val |= ((row >> 6) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B6;
    address_val |= ((row >> 7) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B7;
    address_val |= ((row >> 8) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B8;
    address_val |= ((row >> 9) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B9;
    address_val |= ((row >> 10) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B10;
    address_val |= ((row >> 11) & 1) << base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B11;
    address_val |= ((row >> 12) & 1) << base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B12;
    address_val |= ((row >> 13) & 1) << base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B13;
    address_val |= ((row >> 14) & 1) << base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B14;

    /* Map the bank address bits */
    address_val |= ( bank       & 1) << base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B0;
    address_val |= ((bank >> 1) & 1) << base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B1;
    address_val |= ((bank >> 2) & 1) << base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B2;

    /* Map the channel address bit */
    address_val |= (channel     & 1) << base->LPDDR4_CORE.unADDR5.stcField.u5CHAN_ADDR_MAP_B0;

    *axi_addr = address_val;
}



/*******************************************************************************
*       End of Memory Address handling functions
*******************************************************************************/
/*******************************************************************************
 * Function Name: Cy_Lpddr_CalcMR4ReadInterval
 ***************************************************************************//**
 * 
 * \brief   Function to calculate the Mode Register 4 read interval. The 
 *          TCSR ( Temperature Compensated Self Refresh ) 
 * 
 *          LPDDR4 devices feature a temperature sensor whose status can be 
 *          read from MR4 see Table. This sensor can be used to determine an 
 *          appropriate refresh rate, determine whether AC  timing de-rating is 
 *          required in the elevated temperature range, and/or monitor the 
 *          operating temperature.
 *          
 * 
 * \note    To assure proper operation using the temperature sensor, 
 *          applications should consider the following factors:<br>
 •          TempGradient is the maximum temperature gradient experienced by the
 *          memory device at the temperature of interest <br>over a range 
 *          of 2°C \ref JEDEC_DEVICE_TEMPERATURE_MARGIN. ReadInterval is the 
 *          time period between MR4 reads from the system.<br>
 *          TempSensorInterval (tTSI) is maximum delay between internal updates
 *          of MR4.<br> SysRespDelay is the maximum time between a read of MR4
 *          and the response by the system.<br> In order to determine the 
 *          required frequency of polling MR4, the system shall use the 
 *          maximum <br> TempGradient and the maximum response time of the 
 *          system using the following equation:<br>
 *          <br>
 *          TempGradient x (ReadInterval + tTSI + SysRespDelay)  <= 2°C<br>
 * 
 *  \image html mr4readinterval.PNG "Temp Sensor Timing" width=800px
 * 
 * 
 * 
 * <table class="doxtable">
 * <tr>
 * <th>Parameter</th>
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
 * <td> ReadInterval </td><td>Max </td>
 * <td>System Dependent</td><td> ms</td></tr>
 * <tr><td>Temperature Sensor Interval</td>
 * <td> tTSI</td><td> Max </td>
 * <td>32 </td><td>ms</td></tr>
 * <tr><td>System Response Delay</td>
 * <td> SysRespDelay</td><td> Max </td>
 * <td>System Dependent</td><td> ms</td></tr>
 * <tr><td>Device Temperature Margin</td>
 * <td> TempMargin </td>
 * <td>Max</td><td> 2 </td>
 * <td>°C</td></tr>
 * </table>
 * 
 * 
 * 
 * 
 * \note    For example, if TempGradient is 10°C/s and the SysRespDelay is 1 
 *          ms:<br>(10°C/s) x (ReadInterval + 32ms + 1ms)  <= 2°C<br>
 *          In this case, ReadInterval shall be no greater than 167 ms<br>
 * 
 * \param   sysResponseDelay System response of user system ( system dependend ) 
 *          delay as specified in the JEDEC 209-4C in [msec]
 * 
 * \param   sysTemperatureGradient  is the maximum temperature gradient  of 
 *          user system ( system dependend) experienced by the memory device 
 *          at the <br> temperature of interest over a range of 2°C [°C/s]
 * 
 * \return  mr4ReadInterval which is the ReadInterval as specified in 
 *          JEDEC 209-4C
 ******************************************************************************/
uint32_t Cy_Lpddr_CalcMR4ReadInterval(double sysResponseDelay,double sysTemperatureGradient)
{
    // local variable for the MR4 Read Interval value
    double mr4ReadInterval = 0;
                        // 2°C - (10°C/sec*(32msec+1msec)*0.001 )
    mr4ReadInterval = (JEDEC_DEVICE_TEMPERATURE_MARGIN-
                        (sysTemperatureGradient*CY_LPDDR4_FACTOR_MSEC_IN_SEC*
                        (JESD209_4B_MSEC_TTSI + sysResponseDelay)))/sysTemperatureGradient;
    mr4ReadInterval =( mr4ReadInterval * CY_LPDDR4_FACTOR_SEC_IN_MSEC);
    /* as per JEDEC maximum allowed MR Read Intervall is 167 ms do a limitation here*/ 
    //misinterpreted spec. so commented out next line 167is not a general max
    //mr4ReadInterval = ( mr4ReadInterval < JESD209_4B_MAX_MR4_READINTERVAL ) ? mr4ReadInterval : JESD209_4B_MAX_MR4_READINTERVAL;
    return (uint32_t) (mr4ReadInterval);
}


/*******************************************************************************
*                        PLL800 Stuff starts here
*******************************************************************************/



/*******************************************************************************
 * Function Name: Cy_Lpddr_Pll800ManualConfigure
 ***************************************************************************//**
 *
 * 
 * \brief    Manually configures the PLL800M which is used to clock the LPDDR4 
 *           controller.
 *
 *   
 *   The functions programs the reference divider(Q = REFERENCE_DIV bitfield of 
 *   the PLL800_CONFIG register),feedback divider(P = FEEDBACK_DIV bitfield of 
 *   the PLL800_CONFIG register) and the output divider(N = OUTPUT_DIV bitfield 
 *   of the PLL800_CONFIG register) of the PLL as set in the PLL configuration 
 *   structure \ref cy_stc_lpddr4_pll800_manual_config_t. The output mutliplexer 
 *   VCO_CLK_SEL is set to 2b10 to output the FSP[0] f < 55MHz as used during 
 *   controller init. <br> Once the PLL is correctly setup further switching 
 *   between FSP[0] and FSP[1] shall be done by changing output multiplexer
 *   VCO_CLK_SEL and OUTPUT Dividier  without deactivating the PLL.
 *   
 *    Frequency channge process consists of the following steps:<br>
 *    <ul>
 *        <li>disable PLL output to guarantee glitch free switching without 
 *               deactivation the PLL</li>
 *        <li>change PLL output multiplexer VCO_CLK_SEL and PLL output divider 
 *               OUTPUT_DIV </li>
 *        <li>reenable the PLL output</li> 
 *   </UL>   
 *   To change the output frequency as in process described above use function 
 *   \ref Cy_Lpddr_SetPhyFrequency()
 * 
 * \image html pll_setting_overview.png width=800px
 *
 * \param    base \ref stc_LPDDR4_t
 *
 * \param    config \ref cy_stc_lpddr4_pll800_manual_config_t
 *
 * \return   \ref  cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS-PLL successfully configured<br>
 *  CY_LPDDR4_INVALID_STATE-PLL not configured because it is enabled<br>
 *  CY_LPDDR4_BAD_PARAM-invalid clock path number<br>
 *
 *
 * \note
 *   Do not call this function when the PLL is enabled. If it is, then this 
 *   function returns immmediately with an error return value and no register 
 *   updates.
 *
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_Pll800MManualConfigure(volatile stc_LPDDR4_t * base, 
                                                      const cy_stc_lpddr4_pll800_manual_config_t *config)
{
    un_LPDDR4_LPDDR4_WRAPPER_PLL800_CONFIG_t tempPLL800_CONFIG;
    un_LPDDR4_LPDDR4_WRAPPER_PLL800_CONFIG2_t tempPLL800_CONFIG2;
    
    if((NULL == base) || (NULL == config))
    {
        return(CY_LPDDR4_BAD_PARAM);
    }
    /* check for error */
    /* valid divider bitfield values */
    if((config->outputDiv[0] < PLL_800M_MIN_OUTPUT_DIV) || (PLL_800M_MAX_OUTPUT_DIV < config->outputDiv[0]))
    {
         return(CY_LPDDR4_BAD_PARAM);
    }
    /* check if reference Divider i in a valid range */
    if((config->referenceDiv < PLL_800M_MIN_REF_DIV) || (PLL_800M_MAX_REF_DIV < config->referenceDiv))
    {
         return(CY_LPDDR4_BAD_PARAM);
    }
    /* check if the feedback divider is in a valid range */ 
    if((config->feedbackDiv  < PLL_800M_MIN_FB_DIV) || (PLL_800M_MAX_FB_DIV < config->feedbackDiv))
    {
         return(CY_LPDDR4_BAD_PARAM);
    }

    
    //tempPLL800_CONFIG.u32Register =  base->LPDDR4_WRAPPER.unPLL800_CONFIG.u32Register;
    /* PLL must not be configured when already active */
    /*if (tempPLL800_CONFIG.stcField.u1ENABLE != 0ul)*/
    if (CY_LPDDR4_ONOFF_ENABLE == Cy_Lpddr_IsPllEnabled(base)) /* 1 = enabled */
    {
        return(CY_LPDDR4_INVALID_STATE);
    }
    tempPLL800_CONFIG.stcField.u8FEEDBACK_DIV = (uint32_t)config->feedbackDiv;
    tempPLL800_CONFIG.stcField.u5REFERENCE_DIV = (uint32_t)config->referenceDiv;
    tempPLL800_CONFIG.stcField.u5OUTPUT_DIV = (uint32_t)config->outputDiv[0];
    tempPLL800_CONFIG.stcField.u2VCO_CLK_SEL = (uint32_t)config->vcoclksel[0];
    tempPLL800_CONFIG.stcField.u2LOCK_DELAY = (uint32_t)config->lockdelay;
    tempPLL800_CONFIG.stcField.u2WAIT_TIME = (uint32_t)config->waittime;
    base->LPDDR4_WRAPPER.unPLL800_CONFIG.u32Register = tempPLL800_CONFIG.u32Register;

    
    /*tempPLL800_CONFIG2.u32Register = base->LPDDR4_WRAPPER.PLL800_CONFIG2.u32Register;*/
    if(config->sscgEn)
    {
        tempPLL800_CONFIG2.stcField.u10SSCG_DEPTH = (uint32_t)config->sscgDepth;
        tempPLL800_CONFIG2.stcField.u3SSCG_RATE =(uint32_t)config->sscgRate;
        tempPLL800_CONFIG2.stcField.u1SSCG_MODE    =(uint32_t)config->sscgMode;
        tempPLL800_CONFIG2.stcField.u1SSCG_EN = (uint32_t)config->sscgEn;
        

    }/* added else path to avoid error during FCV as the SCCG must be all 0 
        to avoid error during FCV even if they should be don´t care */
    else    
    {
        tempPLL800_CONFIG2.stcField.u10SSCG_DEPTH = CY_LPDDR4_PLL_SSCG_DEPTH_NO_SSCG;
        tempPLL800_CONFIG2.stcField.u3SSCG_RATE =CY_LPDDR4_PLL_SSCG_RATE_DIV_4096;
        tempPLL800_CONFIG2.stcField.u1SSCG_MODE    =CY_LPDDR4_PLL_SPREAD_MODE_DOWN;
        tempPLL800_CONFIG2.stcField.u1SSCG_EN = config->sscgEn ;
        base->LPDDR4_WRAPPER.unPLL800_CONFIG2.u32Register = tempPLL800_CONFIG2.u32Register;
    }
    return (CY_LPDDR4_SUCCESS);

}

/*******************************************************************************
 * Function Name: Cy_Lpddr_PllEnable()
 ***************************************************************************//**
 *
 * \brief    Enables the PLL800 which is used to clock the LPDDR4 IP. 
 *
 *   Function enables the PLL and waits for PLL to lock.  
 *   PLL Enabler is in PLL800_CONFIG register.
 * 
 *
 * \note Should only be called after Cy_Lpddr_PLL800_Configure() 
 *
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \param timeoutus amount of time in microseconds to wait for the PLL to lock.
 * To avoid waiting for lock set this to 0,and manually check for
 * lock using \ref Cy_Lpddr_IsPllLocked.
 *
 * \return   \ref cy_en_lpddr4_retval_t
 *           CY_LPDDR4_SUCCESS when PLL successfully enabled<br> 
 *           CY_LPDDR4_ERROR when error occurred<br>
 *           CY_LPDDR4_BAD_PARAM when base is not valid pointer<br>
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllEnable(volatile stc_LPDDR4_t * base, uint32_t timeoutus)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1ENABLE = 1UL;
        /* now do the timeout wait for PLL_STATUS, bit LOCKED */
        for (; (base->LPDDR4_WRAPPER.unPLL800_STATUS.stcField.u1LOCKED == 0ul) && (timeoutus != 0ul); timeoutus--)
        {
            Cy_SysLib_DelayUs(1u);
        }
        retval = ((timeoutus == 0ul) ? CY_LPDDR4_TIMEOUT : CY_LPDDR4_SUCCESS);
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_PllDisable();
 ***************************************************************************//**
 *
 * \brief    Disables the PLL800 for LPDDR4
 *
 * \note     Before disabling the PLL the PLL Output must be disabled
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return   \ref cy_en_lpddr4_retval_t
 *           CY_LPDDR4_SUCCESS when PLL successfully disabled<br> 
 *           CY_LPDDR4_ERROR when error occurred<br>
 *           CY_LPDDR4_BAD_PARAm when base is not valid pointer<br>
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllDisable(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if( NULL != base ) 
    {
        base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1ENABLE = 0UL;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllEnabled();
 ***************************************************************************//**
 *
 * \brief    Returns the enable status of the PLL800 for LPDDR4
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return   \ref cy_en_lpddr4_onoff_t
 *           CY_LPDDR4_ONOFF_ENABLE  when PLL is enabled<br> 
 *           CY_LPDDR4_ONOFF_DISABLE when PLL is disabeld<br>
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsPllEnabled(volatile stc_LPDDR4_t * base)
{
    //if( base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u1ENABLE == 0UL ) changed with CMR2
    if( base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1ENABLE == 0UL ) 
    {
        return CY_LPDDR4_ONOFF_DISABLE;
    }
    else 
    {
        return CY_LPDDR4_ONOFF_ENABLE;
    }
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllOutputEnabled();
 ***************************************************************************//**
 *
 * \brief    Returns the output enable status of the PLL800 for LPDDR4
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return   \ref cy_en_lpddr4_onoff_t
 *           CY_LPDDR4_ONOFF_ENABLE  when PLL is enabled<br> 
 *           CY_LPDDR4_ONOFF_DISABLE when PLL is disabeld<br>
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsPllOutputEnabled(volatile stc_LPDDR4_t * base)
{
    if(base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1OUTPUT_ENABLE == 0UL)
    {
        return CY_LPDDR4_ONOFF_DISABLE;
    }
    else 
    {
        return CY_LPDDR4_ONOFF_ENABLE;
    }
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_IsPllLocked
 ***************************************************************************//**
 * \brief    Returns lock status of the LPDDR4 PLL800
 *   
 *   Function returning the locked status of the PLL800 used for clocking the 
 *   LPDDR4 controller.The status is retrieved from PLL800_STATUS register 
 *   
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return   false = not locked<br>
 *           true = locked<br>
 ******************************************************************************/
bool Cy_Lpddr_IsPllLocked(volatile stc_LPDDR4_t * base)
{
    return ((bool) base->LPDDR4_WRAPPER.unPLL800_STATUS.stcField.u1LOCKED);
}



/*******************************************************************************
 * Function Name: Cy_Lpddr_HasPllLostLock
 ***************************************************************************//**
 *
 * \brief    Reports whether or not the PLL lost its lock since the last time 
 *           this  function was called. 
 *
 *   Function returning the information if PLL unlock has occured since the last 
 *   time this function was called. Unlock occured status is retrieved from 
 *   PLL800_STATUS register of the PLL800 used for clocking the LPDDR4 
 *   controller.
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return   false = didn't lose lock<br>
 *           true = lost lock<br>
 *
 ******************************************************************************/
bool Cy_Lpddr_HasPllLostLock(volatile stc_LPDDR4_t * base)
{
    uint32_t retval = base->LPDDR4_WRAPPER.unPLL800_STATUS.stcField.u1UNLOCK_OCCURRED;
        /* Lock occurred; we need to clear the unlock occurred bit.
       Do so by writing a 1 to it. RW1C*/
    base->LPDDR4_WRAPPER.unPLL800_STATUS.stcField.u1UNLOCK_OCCURRED = 1ul;
    return ((bool)retval);
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_PllOutputEnable
 ***************************************************************************//**
 *
 * \brief Function to Enable the Output of the PLL
 *
 * Sets the OUTPUT_ENABLE bit in the PLL800_CONFIG register to activate the 
 * output of the PLL. The output clock must only be enabled 
 * after PLL800_STATUS.LOCKED was confirmed to be 1. The output clock must be 
 * stopped before changing the OUTPUT_DIV or VCO_CLK_SEL setting
 * 
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return CY_LPDDR4_ERROR if error occured<br>
 *          CY_LPDDR4_SUCCESS if the output was successully enabled     <br>
 *            \ref cy_en_lpddr4_retval_t<br>
 *
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllOutputEnable(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval  = CY_LPDDR4_ERROR;
    if (NULL !=base )
    {
        /*only enable output when PLL is locked */
        if(Cy_Lpddr_IsPllLocked(base))
        {
            base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1OUTPUT_ENABLE=1ul;
            retval = CY_LPDDR4_SUCCESS;
        }
    }
    return retval;
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_PllOutputDisable
 ***************************************************************************//**
 *
 * \brief Function to Disable the Output of the PLL
 *
 * Clears the OUTPUT_ENABLE bit in the PLL800_CONFIG register to deactivate 
 * the output. The output clock must be stopped  before changing the 
 * OUTPUT_DIV or VCO_CLK_SEL setting.
 * 
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \return \ref cy_en_lpddr4_retval_t
 *
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_PllOutputDisable(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval  = CY_LPDDR4_ERROR;
    if (NULL !=base )
    {
        base->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1OUTPUT_ENABLE = 0ul;
        retval = CY_LPDDR4_SUCCESS;
    }
    return retval;
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_SelectPllVcoClock
 ***************************************************************************//**
 *
 * \brief Function to set the vcoclksel of the PLL
 *
 *  Before calling this function the PLL output must be disabled using 
 *  Cy_Lpddr_PllOutputDisable() to guarantee glitch free  switching. If the PLL 
 *  Output is still enabled function returns with error CY_LPDDR4_INVALID_STATE. 
 *  If base is not a valid pointer the function will directly return with 
 *  CY_LPDDR4_BAD_PARAM.This function is used during training when switching 
 *  between FSP[0] = 50MHz and FSP[1]=800MHz.
 *  
 *
 * \param  base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \param vcoclksel  vcoclksel to set in the  PLL800_CONFIG register 
 *       \ref cy_en_lpddr4_vco_clk_sel_t
 *
 * \return \ref cy_en_lpddr4_retval_t<br>
 *           CY_LPDDR4_INVALID_STATE if PLL output is still enabled<br>
 *           CY_LPDDR4_BAD_PARAM if base is not a valid pointer<br>
 *           CY_LPDDR4_SUCCESS if everything was ok<br>
 *
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_SelectPllVcoClock(volatile stc_LPDDR4_t * base,
                                                cy_en_lpddr4_vco_clk_sel_t vcoclksel)
{
    //cy_en_lpddr4_retval_t retval  = CY_LPDDR4_ERROR;
    if(NULL!=base)
    {
        /*PLL output must be disabled before changing configuration*/
        //base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u1OUTPUT_ENABLE
        if(CY_LPDDR4_ONOFF_ENABLE == Cy_Lpddr_IsPllOutputEnabled(base))
        {
            return(CY_LPDDR4_INVALID_STATE);
        }
        /* set the vcoclksel if all checks were ok*/
        base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u2VCO_CLK_SEL = vcoclksel;
        return(CY_LPDDR4_SUCCESS);
    }
    else
    {
        return(CY_LPDDR4_BAD_PARAM);
    }

}


/*******************************************************************************
 * Function Name: Cy_Lpddr_SetPllOutputDivider
 ***************************************************************************//**
 *
 * \brief    Function to sets the output divider of the PLL
 *
 * Before calling this function the PLL must be disabled to guarantee glitch 
 * free switching. If the PLL Output is still enabled function returns with 
 * error CY_LPDDR4_INVALID_STATE. If output divider is not within min/max 
 * limit or base is not a valid pointer the function will return with 
 * CY_LPDDR4_BAD_PARAM 
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 * \param    out_div output divider to set in the PLL800_CONFIG register must 
 *           be between PLL_800M_MIN_OUTPUT_DIV and PLL_800M_MAX_OUTPUT_DIV
 *
 * \return   \ref cy_en_lpddr4_retval_t<br>
 * CY_LPDDR4_INVALID_STATE if PLL output is was still enabled <br>
 * CY_LPDDR4_BAD_PARAM  if out_div was not in a valid range between 
 *                      PLL_800M_MIN_OUTPUT_DIV and PLL_800M_MAX_OUTPUT_DIV<br>
 * CY_LPDDR4_SUCCESS    if everything passed ok<br>
 *
 ******************************************************************************/
__STATIC_INLINE cy_en_lpddr4_retval_t Cy_Lpddr_SetPllOutputDivider(volatile stc_LPDDR4_t * base,
                                                   uint32_t out_div)
{
    //cy_en_lpddr4_retval_t retval  = CY_LPDDR4_ERROR;
    if(NULL!=base)
    {
        /*PLL output must be disabled before changing configuration*/
        //base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u1OUTPUT_ENABLE
        if(CY_LPDDR4_ONOFF_ENABLE == Cy_Lpddr_IsPllOutputEnabled(base))
        {
            return(CY_LPDDR4_INVALID_STATE);
        }
        /* check if out_div is within PLL limits*/
        if((out_div < PLL_800M_MIN_OUTPUT_DIV) || (PLL_800M_MAX_OUTPUT_DIV < out_div))
        {
            return(CY_LPDDR4_BAD_PARAM);
        }
        
        /* set the divider if all checks were ok*/
        base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u5OUTPUT_DIV = out_div;
        return(CY_LPDDR4_SUCCESS);
    }
    else
    {
        return(CY_LPDDR4_BAD_PARAM);
    }
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_SetPhyFrequency
 ***************************************************************************//**
 *
 * \brief Function to set the Phy Frequency by setting the appropriate output 
 *        dividers for FSP[0]/FSP[1] as set in 
 *        \ref cy_stc_lpddr4_pll800_manual_config_t During init the Phy 
 *        frequency must be <55MHz frequency set point  fsp[0]. After 
 *        initialization and during training the phy frequency is switched 
 *        to high speed mode fsp[1] 800 MHz .While the PLL stays active with 
 *        800MHz.The different frequencies can be achieved by selecting 
 *        different PLL output settings( vco_select /  out_div).
 *   
 *    
 *  \note    While the pll will stay active the output must be disabled to 
 *           change the output divider settings to guarantee glitch free 
 *           switching.
 *
 * \param    base pointer to LPDDR structure \ref stc_LPDDR4_t
 *
 *
 * \param config  pointer to a pll configuration structure 
 *       \ref cy_stc_lpddr4_pll800_manual_config_t
 *
 * \param fsp frequency set point \ref cy_en_lpddr4_fsp_t<br>
 * CY_LPDDR4_FSP0 :Pll output @50 MHZ ( depends on what is configured
 *                 in the lpddr_config. structure )<br>
 * CY_LPDDR4_FSP1 :PLL output @800 MHz( depends on what is configured 
 *                 in the lpddr_config. structure )<br>
 *
 * \return \ref cy_en_lpddr4_retval_t
 *
 *   @startuml"Frequency Change Process"
 *   !include commons.iuml!FSP_CHANGE_PROCESS
 *   @enduml
 *
 *
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyFrequency(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_pll800_manual_config_t *config,
                                                cy_en_lpddr4_fsp_t fsp)
{
    //cy_en_lpddr4_retval_t retval  = CY_LPDDR4_ERROR;
    if(NULL!=base)
    {
        Cy_Lpddr_PllOutputDisable(base);
        base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u2VCO_CLK_SEL=config->vcoclksel[fsp];
        base->LPDDR4_WRAPPER.unPLL800_CONFIG.stcField.u5OUTPUT_DIV=config->outputDiv[fsp]; 
        Cy_Lpddr_PllOutputEnable(base);
        return(CY_LPDDR4_SUCCESS);
    }
    else
    {
        return(CY_LPDDR4_BAD_PARAM);
    }
}

/*******************************************************************************
*                        PLL800 Stuff ends here
*******************************************************************************/


/*******************************************************************************
*                        LPDDR4 Controller Stuff starts here
*******************************************************************************/


/*******************************************************************************
 * Function Name: void Cy_Lpddr_ControllerEnable();
 ***************************************************************************//**
 *
 *   \brief  Function sets the Main Enabler for the LPDDR4 IP 
 *
 *   \param  Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 *
 *   \note   IP enable bit. When the IP is disabled, all retention registers  
 *           can be accessed (else results in AHB error response).
 *
 * \return CY_LPDDR4_SUCCESS when LPDDR4 Controller was successfully enabled<br> 
 *   CY_LPDDR4_ERROR when error occurred<br>
 *   CY-_LPDDR4_BAD_PARAM when base is not valid pointer<br>
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ControllerEnable(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->LPDDR4_WRAPPER.unCTL.stcField.u1ENABLED = 1ul;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 * Function Name: void Cy_Lpddr_ControllerDisable();
 ***************************************************************************//**
 *
 *   \brief  Function clears the Main Enabler for the LPDDR4 IP 
 *
 *   \param  Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 *   \note   IP enable bit. When the IP is disabled, all retention registers  
 *           can be accessed (else results in AHB error response).
 *
 * \return CY_LPDDR4_SUCCESS when LPDDR4 Controller was successfully enabled,<br> 
 *   CY_LPDDR4_ERROR when error occurred<br>
 *   CY-_LPDDR4_BAD_PARAM when base is not valid pointer<br>
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ControllerDisable(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->LPDDR4_WRAPPER.unCTL.stcField.u1ENABLED = 0ul;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 * Function Name: void Cy_Lpddr_IsControllerEnabled()
 ***************************************************************************//**
 *
 * \brief    Function returns the status of the main enabler for the LPDDR4 IP 
 *
 * \param    Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 * \return   \ref cy_en_lpddr4_onoff_t<br>
 *           CY_LPDDR4_ONOFF_ENABLE   when main enabler is set<br>
 *           CY_LPDDR4_ONOFF_DISABLE  when main enabler is not set<br>
 ******************************************************************************/
cy_en_lpddr4_onoff_t Cy_Lpddr_IsControllerEnabled(volatile stc_LPDDR4_t * base)
{
    if( base->LPDDR4_WRAPPER.unCTL.stcField.u1ENABLED == 1)
    {
        return CY_LPDDR4_ONOFF_ENABLE;
    }
    else 
    {
        return CY_LPDDR4_ONOFF_DISABLE;
    }
}




/*******************************************************************************
 *  Function Name:  Cy_Lpddr_ControllerInit
 ***************************************************************************//**
 * \brief  Function to initialize the LPDDR4 Controller
 *  
 * This function controlls the LPDDR4 controller initialisation process. Below 
 * list is a summary of what the function does to initialize the controller.<br>
 * The initialization process for the LPDDR4 inlcudes the following steps
 * <ol>
 *      <li>Program the QoS ( Quality fo Service ) Priorities for CPUSS / VIDEOSS
 *      <li>Enable the Main Enabler of the IP</li>
 *      <li>Setup the PLL ( set the divider, check pll for locking enable pll 
 *          output)</li>
 *      <li>Setup Arbiter</li>
 *      <li>Setup DRAM Registers ( LPMR1,LPMR2,LPMR3,LPMR11,LPMR12,LPMR13,
 *          LPMR14,LPMR22..</li>
 *      <li>Setup PHY DDR Mode (VTGC,PHY,DQSDQCR)</li>     
 *      <li>Setup Bist Configuration Register  (BISTCFG_CH0/unBISTCFG_CH1)</li>
 *      <li>Setup Phy Timing Registers (TREG0..15 )</li>
 *      <li>Setup DLL</li>
 *      <li>Setup Address Mapping</li>
 *      <li>Phy Initialization (DLL Init,DRAM,Init, PHY Training for FSP[0]</li>
 *      <li>Phy Reload(reload trained values from a former training run which 
 *          were stored in the PTSRx registers.) can be used for quickstart 
 *          without complete training to get the controller up and running 
 *          after a power down event.<br>
 *              <b>or</b><br>
 *          Perform high speed operation training (CBT,WRLEVL,RDLEVL,Gate 
 *          Training,...including switching the PLL fout several times)<br>
 *          and switch to high speed operation FSP[1]</li>
 *      <li>Check if refresh rate / timining derating must be adjusted because<br>
 *          (warm start)</li>
 *      <li>Programm DQS Oscillator Intervall MR23</li>
 *      <li>Start DQS Oscillator and Read a initial DQS Osc. Count as reference 
 *          to detect when retraining is necessary</li>
 *      <li>Start the Comntroller with Run Command </li>
 * </ol>
 * 
 * \param  base Pointer to the LPDDR4 Base Structure from type 
           \ref stc_LPDDR4_t
 * 
 * \param  config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 * 
 * 
 * \param  inittype \ref cy_en_lpddr4_inittype_t
 *         This parameters specifies if controller is 
 *         initialized by reloading data or performing a full training sequence 
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
                                              cy_en_lpddr4_inittype_t inittype)
{
    cy_en_lpddr4_retval_t result = CY_LPDDR4_ERROR;
    //cy_en_lpddr4_channel_status_t cmd_ch_status;
    
    /* Disable the PLL800 to disable LPDDR4 core */
    Cy_Lpddr_PllDisable(base);
    
    if(( NULL != base ) && (NULL != config))
    {
        /* update the driver context status with the actual channel selection */
        Cy_Lpddr_DriverContextInit(config);

        /* QOS must be set before Controller is Enabled */
        Cy_Lpddr_SetQoSPrios(base,config);

        /* set the main enabler for the IP */
        result = Cy_Lpddr_ControllerEnable(base);
        if(result != CY_LPDDR4_SUCCESS)
        {
            return result;
        }
        
        #if (CY_USE_PSVP == 1)
            /* since PSVP bitfile released with PAND#73 
            PLL Output must be enabled also on PSVP*/
            Cy_Lpddr_PllOutputEnable(base);
            /**  NO PLL topics at the PSVP as everything is fixed   */
        #else
            /** configure the PLL. The PLL frequency is set @800MHz switching 
             *  between FSP[0] and FSP[1] will be done by changing the pll 
             *  output divider setup vco_clk_sel and/or output_div without 
             *  stopping the PLL. To change the vco_clk_sel and / or  
             *  outputdivider the Pll output must be disabled for glitch free 
             * switching.                                                     */
            result = Cy_Lpddr_Pll800MManualConfigure(base, &(config->pllconfig));
            if(CY_LPDDR4_SUCCESS == result)
            {
                /* Enable the PLL and wait for lock */
                /*TODO: wait time as set to short value to reduce simulation time
                  PLL lock time according to 002-30271 
                  80us @f <= 7.92 MHz CY_LPDDR4_PLL_MAX_LOCKTIME_BELOW_7P92MHZ
                  90us @f >  7.92 MHz CY_LPDDR4_PLL_MAX_LOCKTIME_ABOVE_7P92MHZ*/
                result = Cy_Lpddr_PllEnable(base,CY_LPDDR4_PLL_STABILIZATION_TIME);
                if(CY_LPDDR4_SUCCESS == result)
                {
                    /*when PLL has sucessfully locked enable the output*/
                    result = Cy_Lpddr_PllOutputEnable(base);
                }
            /*  if either Cy_Lpddr_PllEnable or Cy_Lpddr_PllOutputEnable 
                failed return with error                                      */
                if(CY_LPDDR4_SUCCESS != result)
                {
                    return result;
                }

            }else
            {
                /*if Cy_Lpddr_Pll800MManualConfigure failed return with error*/
                return result;
            }
        #endif

        /*  Program the Registers which configure the ECC Memory Regions
            INECC0,INECC1,INECC2                                            */
        result = Cy_Lpddr_SetupECCRegisters(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        result = Cy_Lpddr_SetupArbiterMode(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /*Setup the Memory Mode Registers LPRMR registers in the 
        controller MR registers in ext. RAM                         */
        result = Cy_Lpddr_SetupModeRegisters(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        result = Cy_Lpddr_SetPhyDdrMode(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        result = Cy_Lpddr_SetupBISTRegister(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /* Setup the Memory Controller Timing Registers */
        result = Cy_Lpddr_SetPhyTiming(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }

        result = Cy_Lpddr_SetupDLL(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /*Setup the Command Module IO Control Register of the Memory Controller*/
        result = Cy_Lpddr_SetupCAIOCntrlRegs(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /*Setup the Data Modules SL0..SL3 IO Control Register of the 
        Memory Controller*/
        result = Cy_Lpddr_SetupDataSliceIOCntrlRegs(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /*Setup the Memory Controller Adress Registers*/
        result = Cy_Lpddr_SetAdressConfig(base,config);
        if(CY_LPDDR4_SUCCESS != result)
        {
            return result;
        }
        /*do the PHY init */
        result = Cy_Lpddr_PhyInit(base,config); 
        if(CY_LPDDR4_SUCCESS != result )
        {
            return result;
        }
        /*run the LPDDR4 training and exit with error if some error occured*/
        if( inittype == CY_LPDDR4_INIT_WITH_TRAINING)
        {
            //TRAINING can be set in cy_dram_config.h
            #if(LPDDR4_TRAINING == TRAINING_STEPWISE)
                result = Cy_Lpddr_LPDDR4StepwiseTraining(base, config);
            #elif(LPDDR4_TRAINING == TRAINING_ALL_AT_ONCE)
                result = Cy_Lpddr_LPDDR4Training(base,config);
            #endif

        }
        else if( inittype == CY_LPDDR4_INIT_WITH_RELOAD)
        {
        /*  program all controller PTSRx-Register 
        ( Phy Training Status Register) with values from last 
        successfully training */
            result = Cy_Lpddr_SetPTSRRegisters(base,config);
            if(CY_LPDDR4_SUCCESS != result)
            {
                return result;
            }
            /*  trigger a reload of the PTSRx values to the PHY for Quick start */
            result = Cy_Lpddr_PhyReload(base,config,1, 1, 1, 1, 1, 1, 1, 0, 1, 1,1); 
        }
        if(result != CY_LPDDR4_SUCCESS)
        {
            return result;
        }
        /*  adjust the tREFI in case of warm start . Means memory is still warmed from last run 
            and we are initilaizing new */ 
        Cy_Lpddr_RefreshandTimingAdjust(base); 
        /*  write the Mode Register MR23 ( DQS Oscillator Timer interval ) 
            must be programmed >0 for automatic DQS Oscillator stop
            DQS Oscillator itself is used for checking the need for retraining */
        result = Cy_Lpddr_WriteModeRegister(LPDDR40,CY_LPDDR4_MODEREGISTER_MR23,(uint8_t*)&(config->dqsOscRuntimeClkCyls));
        if( result != CY_LPDDR4_SUCCESS)
        {
            return result;
        }
        /* send run command to the controller */
       
	   
        /*  Start the DQS Oscillator to get a first value of the DQS Oscillator
            right after the initial training against which we later on can
            compare the DQS values once PVT has changed*/
         Cy_Lpddr_StartDQSOscillator(base);
		 
         /* send run command to the controller */
        result = Cy_Lpddr_SendCmd(base, CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN, CY_LPDDR4_NONE_MR, 0);
        if( result != CY_LPDDR4_SUCCESS)
        {
            return result;
        }
        /*Read the DQS Oscillator and store the reading in the driver context. 
          We just passed the training, spo this DQS Osc Count value will be the
          base for considering a re-training */
        result = Cy_Lpddr_ReadDQSOscValue(base,&(g_stc_lpddr_context.dqsOSCCountLastTraining));
        if( result != CY_LPDDR4_SUCCESS)
        {
            return result;
        }
        Cy_Lpddr_SetContextInitState();
    }
    else
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    return result;
}





/*******************************************************************************
 *  Function Name:  Cy_Lpddr_PhyReload
 ***************************************************************************//**
 *  \brief Function to trigger reload of the PHY trained values from 
 *         PTSR registers
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param config Pointer to the LPDDR4 config structure
 * 
 *  \param phyinit
 * 
 *  \param fs Target frequency set-point for PHY operations. Must be different 
 *         from operating set point. 
 * 
 *  \param gten reload request for Gate training values PTSR3
 * 
 *  \param wrlvlen reload request for Write leveling values. PTSR4
 * 
 *  \param vrefqdwren reload request for DRAM VREF-DQ training values
 *         (PTSR3,PTSR5..PTSR13). 
 * 
 *  \param vrefdqrden reload request for PHY VREF-DQ training values (PTSR23)
 * 
 *  \param rdlvlen reload request for Read data eye DQ training values( PTSR14-22) 
 * 
 *  \param dlyevalen reload request for Delay evaluation values. 
 * 
 *  \param sanchken reload request for Write/read sanity check values.
 * 
 *  \param vrefcaen reload request for DRAM VREF-CA training values.
 * 
 *  \return \ref    cy_en_lpddr4_retval_t<br>
 *                  CY_LPDDR4_SUCCESS if everything was ok<br>
 *                  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed 
 *                                      to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_PhyReload(volatile stc_LPDDR4_t * base, 
                        const cy_stc_lpddr4_config_t * config,
                        bool phyinit,bool fs,bool gten,bool wrlvlen,bool vrefdqwren,
                        bool vrefdqrden,bool rdlvlen,bool dlyevalen,bool sanchken,
                        bool vrefcaen,bool clklocken) 
{
    un_LPDDR4_LPDDR4_CORE_POS_t unPOSval;
    un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;
    unPOMval.u32Register = 0x0ul;
  
    /*  select the channels as configured by the user in the config 
        structure and is programmed during ControllerInit to driver 
        context.channelActive */
    unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();


    unPOMval.stcField.u1POM_PHYSETEN = 0x1;
    unPOMval.stcField.u1POM_PHYFSEN = 0x1;
    unPOMval.stcField.u1POM_PHYINIT = phyinit;
    unPOMval.stcField.u1POM_VREFDQRDEN = vrefdqrden;
    unPOMval.stcField.u1POM_VREFCAEN = vrefcaen;
    unPOMval.stcField.u1POM_GTEN = gten;
    unPOMval.stcField.u1POM_WRLVLEN = wrlvlen;
    unPOMval.stcField.u1POM_RDLVLEN = rdlvlen;
    unPOMval.stcField.u1POM_VREFDQWREN = vrefdqwren;
    unPOMval.stcField.u1POM_DLYEVALEN = dlyevalen;
    unPOMval.stcField.u1POM_SANCHKEN = sanchken;
    unPOMval.stcField.u1POM_FS = fs;
    unPOMval.stcField.u1POM_CLKLOCKEN = clklocken;

    base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
    while (1)
    {
    unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
        if (unPOSval.stcField.u1PHYINITC == 0x1)
        {
            unPOMval.u32Register = 0x0;                 /*  disable phy init process */
            /*  select the channels as configured by the user in the config 
                structure and is programmed during ControllerInit to driver 
                context.channelActive */
            unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();
            unPOMval.stcField.u1POM_DFIEN = 0x1;        /*  DFI interface enable.MC must write 1’b1 to
                                                            this field for normal DFI operation */
            unPOMval.stcField.u1POM_FS = 0x1;           /*  Target Frequency Set Point must be different 
                                                            from operation set point */
            /*  write the local POM to the controller register */
            base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
            return CY_LPDDR4_SUCCESS;
        }
        /* TODO:might be that we don´t need the branch for u1FS0REQ check simulation waveforms */
        else if(unPOSval.stcField.u1FS0REQ == 1)
        {
            /* change to FSP[0] f = 50 MHz */
            Cy_Lpddr_SetPhyFrequency(base,&config->pllconfig,CY_LPDDR4_FSP0);    
            /* give confirmation to PHY that frequency was changed an PHY can proceed*/
            Cy_Lpddr_PhyProceed(base);
        }
        else if(unPOSval.stcField.u1FS1REQ == 1) 
        {    
            /* change to FSP[1] f = 800 MHz */
            Cy_Lpddr_SetPhyFrequency(base,&config->pllconfig,CY_LPDDR4_FSP1);
            /* give confirmation to PHY that frequency was changed an PHY can proceed*/
            Cy_Lpddr_PhyProceed(base);
        }
    }
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_ControllerDeInit
 ***************************************************************************//**
 *
 *   \brief  Function to DeInitialize the LPDDR4 Memory controller
 *           Function is performing the following steps.
 *           -   Send Stop Command to the Controller 
 *           -   Send Power Down Entry Command to the Controller
 *           -   Switch off PLL Output_Enable
 *           -   Switch off the PLL ( not possible with PSVP)
 *           -  set driver context state to
 *              .controllerInitState = CY_LPDDR4_CNTRL_NOT_INITIALIZED
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
cy_en_lpddr4_retval_t Cy_Lpddr_ControllerDeInit(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if(( NULL != base ))
    {
        
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_STOP,CY_LPDDR4_NONE_MR,0);
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_PDE,CY_LPDDR4_NONE_MR,0); //Send Power Down Entry ==> CKE=L JEDEC 3.3.3 Power Off Sequence
        #if (CY_USE_PSVP == 1)
            /* PSVP in use */
            /* since PSVP bitfile released with PAND#73 
            PLL Output must be enabled also on PSVP*/
            Cy_Lpddr_PllOutputEnable(base);
        #else
            /* SOC or FCV in use */
            //TODO:
            Cy_Lpddr_PllOutputDisable(base);    /*  only possible with SOC */
            Cy_Lpddr_PllDisable(base);          /*  only possible with SOC */
        #endif
               /* --> CDT382711 Any time IP enable is turned off, an AXI reset has to follow.
                Deep sleep can be entered without turning off the LPDDR4 IP enable.
                Or
                Do not use the LPDDR4 IP enable to turn off the LPDDR4 enable. 
                It should only be used to sequence power up.*/
        //Cy_Lpddr_ControllerDisable(base); 
        Cy_Lpddr_ClrContextInitState();
    }
    else
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}




/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetPhyDdrMode
 ***************************************************************************//**
 *  \brief Function to set DRAM Mode 
 * 
 * This function programs the following registers with the values the user 
 * has set in the LPDDR4 config \ref cy_stc_lpddr4_config_t .
 * 
 * 
 *  <ul>
 *  <li>VTGC PHY VREF Training General Control Register </li>
 *  <li>PHY PHY Register </li>
 *  <li>DQSDQCR DQS2DQ Delay Control Register </li>
 *  </ul>
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type 
 *         \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 *
 * 
 *  \return \ref    cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS   if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to  the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyDdrMode(volatile stc_LPDDR4_t * base,
                                             const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;

    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unVTGC.u32Register = config->unVTGC.u32Register;
        base->LPDDR4_CORE.unPHY.u32Register = config->unPHY.u32Register;
        base->LPDDR4_CORE.unDQSDQCR.u32Register = config->unDQSDQCR.u32Register;
        base->LPDDR4_CORE.unPCCR_CH0.u32Register = config->unPCCR_CH0.u32Register;
        base->LPDDR4_CORE.unPCCR_CH1.u32Register = config->unPCCR_CH1.u32Register;
        
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetupBISTRegister
 ***************************************************************************//**
 *  \brief Function to initialize the BIST Registers
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 * 
 * 
 *  \return \ref    cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupBISTRegister(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;

    if(( NULL != base ) && (NULL != config))
    {

        base->LPDDR4_CORE.unBISTCFG_CH0.u32Register = config->unBISTCFG_CH0.u32Register;
        base->LPDDR4_CORE.unBISTCFG_CH1.u32Register = config->unBISTCFG_CH1.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}



/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetPhyTiming
 ***************************************************************************//**
 *  \brief Function programs the LPDDR4 Phy Timing Registers Registers
 * 
 *  Function programs the following LPDDR4 Phy Timing Registers Registers with 
 *  the values configured from the user in the the LPDDR4 config structure 
 *  \ref cy_stc_lpddr4_config_t <bR>
 * 
 *  <ul>
 *  <li>TREG0..TREG15 mainly timing parameters specified in the JEDEC 
 *      Spec 209-4C</li>
 *  <li>PTSR0,1,2,24..25 PhY Training Status Register for C/A BUs settings</li>
 *  <li>RTGC0,RTGC0 PHY Read Training General Control Register 0 for 
 *      FSP[0]/1 for FSP[1]</li>
 *  </ul>
 * 
 *  
 *  \param base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 * 
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 * 
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPhyTiming(volatile stc_LPDDR4_t * base,
                                            const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unTREG0.u32Register = config->unTREG0.u32Register;
        base->LPDDR4_CORE.unTREG1.u32Register = config->unTREG1.u32Register;
        base->LPDDR4_CORE.unTREG2.u32Register = config->unTREG2.u32Register;
        base->LPDDR4_CORE.unTREG3.u32Register = config->unTREG3.u32Register;
        base->LPDDR4_CORE.unTREG4.u32Register = config->unTREG4.u32Register;
        base->LPDDR4_CORE.unTREG5.u32Register = config->unTREG5.u32Register;
        base->LPDDR4_CORE.unTREG6.u32Register = config->unTREG6.u32Register;
        base->LPDDR4_CORE.unTREG7.u32Register = config->unTREG7.u32Register;
        base->LPDDR4_CORE.unTREG8.u32Register = config->unTREG8.u32Register;
        base->LPDDR4_CORE.unTREG9.u32Register = config->unTREG9.u32Register;
        base->LPDDR4_CORE.unTREG10.u32Register = config->unTREG10.u32Register;
        base->LPDDR4_CORE.unTREG11.u32Register = config->unTREG11.u32Register;
        base->LPDDR4_CORE.unTREG12.u32Register = config->unTREG12.u32Register;
        base->LPDDR4_CORE.unTREG13.u32Register = config->unTREG13.u32Register;
        base->LPDDR4_CORE.unTREG14.u32Register = config->unTREG14.u32Register;
        base->LPDDR4_CORE.unTREG15.u32Register = config->unTREG15.u32Register;

        base->LPDDR4_CORE.unRTGC0.u32Register = config->unRTGC0.u32Register;
        base->LPDDR4_CORE.unRTGC1.u32Register = config->unRTGC1.u32Register;
        
        //TODO: check Command Delay Load just reload 
        base->LPDDR4_CORE.unPTSR0.u32Register = config->unPTSR0.u32Register;
        base->LPDDR4_CORE.unPTSR1.u32Register = config->unPTSR1.u32Register;
        base->LPDDR4_CORE.unPTSR2.u32Register = config->unPTSR2.u32Register;
        base->LPDDR4_CORE.unPTSR24.u32Register = config->unPTSR24.u32Register;
        base->LPDDR4_CORE.unPTSR25.u32Register = config->unPTSR25.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;

}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetupDLL
 ***************************************************************************//**
 *  \brief  Function to program the LPDDR4 controller internal DLL registers. 
 * 
 *  Function to program the LPDDR4 controller internal registers to setup the 
 *  DLL in the PHY.The DLL related settings of the LPDDR4 config structure 
 * \ref cy_stc_lpddr4_config_t are programmed to the registers listed below:
 *
 *  <u>Command Module:</u><br><br>
 *  DLLCTLCA_CH0 - DLL Control Register for PHY Command Module - Channel 0<br>
 *  DLLCTLCA_CH1 - DLL Control Register for PHY Command Module - Channel 1<br>
 *  <br>
 *  <u>Data Module</u><br><br>
 *  DLLCTLDQ_SL0 - DLL Control Register for PHY Data Module Slice 0<br>
 *  DLLCTLDQ_SL1 - DLL Control Register for PHY Data Module Slice 1<br>
 *  DLLCTLDQ_SL2 - DLL Control Register for PHY Data Module Slice 2<br>
 *  DLLCTLDQ_SL3 - DLL Control Register for PHY Data Module Slice 3<br>
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 * 
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function <br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupDLL(volatile stc_LPDDR4_t * base,
                                        const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        //setup DLL register for command adress bus channel 0 / channel 1
        base->LPDDR4_CORE.unDLLCTLCA_CH0.u32Register = config->unDLLCTLCA_CH0.u32Register;
        base->LPDDR4_CORE.unDLLCTLCA_CH1.u32Register = config->unDLLCTLCA_CH1.u32Register;
        //setup the DLL controll registers per data slice
        base->LPDDR4_CORE.unDLLCTLDQ_SL0.u32Register = config->unDLLCTLDQ_SL0.u32Register;
        base->LPDDR4_CORE.unDLLCTLDQ_SL1.u32Register = config->unDLLCTLDQ_SL1.u32Register;
        base->LPDDR4_CORE.unDLLCTLDQ_SL2.u32Register = config->unDLLCTLDQ_SL2.u32Register;
        base->LPDDR4_CORE.unDLLCTLDQ_SL3.u32Register = config->unDLLCTLDQ_SL3.u32Register;
        
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetupDataSliceIOCntrlRegs
 ***************************************************************************//**
 *  \brief Function to setup the PHY Data Module IO Control Registers 
 * 
 *  This function programs the PHY Data Module IO Control Register for each 
 *  data slice. These are the registers dior_sl0..3
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type 
 *         \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 *
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *CY_LPDDR4_SUCCESS     if everything was ok<br>
 *CY_LPDDR4_BAD_PARAM   if a wrong parameter was passed to the 
 *                      function <br>
 * 
 *
 * <table class="doxtable">
 * <tr>
 *      <th>Register Field</th>
 *      <th>Position</th>
 *      <th>Reset</th>
 *      <th>Description</th>
 * </tr>
 * <tr>
 *      <td>dior_sl0_drvsel</td>
 *      <td>[2:0] </td>
 *      <td> 0</td>
 *      <td>3-bit value to select how many copies of driver need to be 
 *          turned on.<br>
 *          Driver Setting <br>
 *          000-turns on 1 driver copy<br>
 *          001- 2 driver copy<br>
 *          010- 3 driver copy<br>
 *          111- 8 driver copy<br>
 *      </td>
 *  </tr>
 * <tr>
 *      <td>dior_sl0_cmos_en</td>
 *      <td>[3]</td>
 *      <td> 0</td>
 *      <td>Enable CMOS Reciever in IO</td>
 * </tr>
 * <tr>
 *      <td>dior_sl0_fena_rcv</td>
 *      <td>[4]</td>
 *      <td> 0</td>
 *      <td>Driver output enables active high <br>
 *          (1: enable; 0: tristate); FENA has higher priority than <br>
 *          ODIS and OE when it is high. Enable = FENA + !ODIS & OE<br> </td>
 * </tr>
  * <tr>
 *      <td>dior_sl0_rtt_en</td>
 *      <td>[5]</td>
 *      <td>0</td>
 *      <td>Enable RTT compensation </td>
 * </tr>
   * <tr>
 *      <td>dior_sl0_rtt_sel </td>
 *      <td>[8:6]</td>
 *      <td>0</td>
 *      <td>rtt_sel pin is used to vary the RTT impedance when rtt_en=1 <br>
 *          rtt_sel=0 RTT impedance = R/2 [R = External Resistor]<br> 
 *          rtt_sel=1 RTT impedance = R/4 [R = External Resistor]<br> </td>
 * </tr>
 * <tr>
 *      <td>dior_sl0_odis_dq</td>
 *      <td>[16:9]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for dq <br>
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 * <tr>
 *      <td>dior_sl0_odis_dm</td>
 *      <td>[17]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for dm 
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 * <tr>
 *      <td>dior_sl0_odis_dqs </td>
 *      <td>[18]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for dqs 
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 *</table> 
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupDataSliceIOCntrlRegs(volatile stc_LPDDR4_t * base,
                                                         const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        // setup PHY Data Module IO Control Register per data slice 
        base->LPDDR4_CORE.unDIOR_SL0.u32Register = config->unDIOR_SL0.u32Register;
        base->LPDDR4_CORE.unDIOR_SL1.u32Register = config->unDIOR_SL1.u32Register;
        base->LPDDR4_CORE.unDIOR_SL2.u32Register = config->unDIOR_SL2.u32Register;
        base->LPDDR4_CORE.unDIOR_SL3.u32Register = config->unDIOR_SL3.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetupCAIOCntrlRegs
 ***************************************************************************//**
 *  \brief Function to setup the PHY Command Address Module IO Control Registers 
 * 
 *  This function programs the PHY Data Module IO Control Register for each 
 *  data slice. These are the registers CIOR_CH0 , CIOR_CH01.
 *  Each LPDDR4 I/O has 6 copies of drivers that are each tuned to 240Ω using 
 *  ZQ calibration. Drive strengths can be altered by changing cior_sl*.drvsel 
 * 
 * <table class="doxtable">
 * <tr>
 *      <th>Register Field</th>
 *      <th>Position</th>
 *      <th>Reset</th>
 *      <th>Description</th>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_drvsel</td>
 *      <td>[2:0] </td>
 *      <td> 0</td>
 *      <td>3-bit value to select how many copies of driver need to be turned 
 *          on.<br>Driver Setting <br>
 *          000-turns on 1 driver copy<br>
 *          001- 2 driver copy<br>
 *          010- 3 driver copy<br>
 *          111- 8 driver copy<br>
 *      </td>
 *  </tr>
 * <tr>
 *      <td>cior_ch0/1_cmos_en</td>
 *      <td>[3]</td>
 *      <td> 0</td>
 *      <td>Enable CMOS Reciever in IO Channel 0/1</td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_odis_resetn</td>
 *      <td>[4]</td>
 *      <td>0</td>
 *      <td>Driver output enables active high <br>
 *          Output force disable active high for mem_resetn - Channel 1<br> 
 *          When set to 1, the corresponding output bit is forced to high-Z<br> 
 *      </td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_odis_odt</td>
 *      <td>[5]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for mem_odt - Channel 1<br>
 *          When set to 1, the corresponding output bit is forced to high-Z 
 *      </td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_odis_cs</td>
 *      <td>[6]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for mem_cs - Channel 1<br> 
 *          When set to 1, the corresponding output bit is forced to high-Z 
 *      </td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_clk</td>
 *      <td>[7]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for mem_clk - Channel 1<br>
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_odis_cke</td>
 *      <td>[8]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for mem_cke - Channel 1<br>
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 * <tr>
 *      <td>cior_ch0/1_odis_ca </td>
 *      <td>[14:9]</td>
 *      <td>0</td>
 *      <td>Output force disable active high for mem_ca - Channel 1<br>
 *          When set to 1, the corresponding output bit is forced to high-Z</td>
 * </tr>
 *</table> 
 *  
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param config pointer to the LPDDR4 Initialization Structure 
 *         \ref cy_stc_lpddr4_config_t
 *
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *      CY_LPDDR4_SUCCESS if everything was ok<br>
 *      CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function <br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupCAIOCntrlRegs(volatile stc_LPDDR4_t * base,
                                                  const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        // setup PHY Command Address Module IO Control Register per channel 
        base->LPDDR4_CORE.unCIOR_CH0.u32Register = config->unCIOR_CH0.u32Register;
        base->LPDDR4_CORE.unCIOR_CH1.u32Register = config->unCIOR_CH1.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}







/*******************************************************************************
 *  Function Name:  Cy_Lpddr_PhyInit
 ***************************************************************************//**
 *  \brief Function to Initialize the LPDDR4 PHY  
 * 
 *  When system reset is completed and control registers are all programmed,
 *  this function triggers the PHY initialization process by writing the 
 *  corresponding bits  *  in the PHY Operational Mode Register ( POM ).<br>
 *  The PHY initialization includes the following steps:<br>
 *  <ul>
 *  <li>DLL init </li>
 *  <li>DRAM init --> programm DRAMs Mode Registers MR1,MR2,MR3,MR11,MR12,MR14 
 *      and MR22 for FSO[0]/[1]</li>
 *  <li>PHY Training (for FSP[0])</li>
 *  </ul>
 *  Phy Operational Status Register POS must be monitored for phyinitc=1.
 *  As soon as the PHY signals in POS ( PHY Operational Status ) that the Init 
 *  was completed  *  the function returns.
 * 
 * This function programs all requests at the same time but software could 
 * also request each step separately by writing other bits in this POM 
 * register individually.
 * 
 *  \note   Function shall not be called before all LPDDR4 controller 
 *          registers are programmed with the values as set in the lpddr4 
 *          config structure \ref cy_stc_lpddr4_config_t
 * 
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *  \param  config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 *
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_PhyInit(volatile stc_LPDDR4_t * base, 
                                      const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        /*  local variable to collect all requests in order to  
            write them at once to the controller */
        un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;
        unPOMval.u32Register = 0x0ul;

        /*  Program the Dynamo Control Register */
        base->LPDDR4_CORE.unDMCTL.u32Register = config->unDMCTL.u32Register;
        /*  Program the Dynamo Configuration Register */
        base->LPDDR4_CORE.unDMCFG.u32Register = config->unDMCFG.u32Register;

    /*  select the channels as configured by the user in the config 
        structure and is programmed during ControllerInit to driver 
        context.channelActive */
       unPOMval.stcField.u2POM_CHANEN =Cy_Lpddr_GetContextChannelConfig();
        /*  request PHY init */
        unPOMval.stcField.u1POM_PHYINIT = CY_LPDDR4_POM_REQUESTED;
        /*  request DLL initializstion */
        unPOMval.stcField.u1POM_DLLRSTEN = CY_LPDDR4_POM_REQUESTED;
        /*  request DRAM  initialization */
        unPOMval.stcField.u1POM_DRAMINITEN = CY_LPDDR4_POM_REQUESTED;
        /*  Dolphin UG p110 --> CLKLOCKEN must be set when Phy is 
            running in 1:4 */ 
        unPOMval.stcField.u1POM_CLKLOCKEN = CY_LPDDR4_POM_REQUESTED;
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        
        /*  Check if Phy Init was completed by reding the flags in the 
            POS register */
        retval = Cy_Lpddr_CheckPhyInitComplete(base);

        /*  clear all request currently active in the POM register */
        unPOMval.u32Register = 0x0ul;

        /*  select the channels as configured by the user in the config 
        structure and is programmed during ControllerInit to driver 
        context.channelActive */
        unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
    }
 return retval;
}



/*******************************************************************************
 * Function Name: Cy_Lpddr_RefreshandTimingAdjust
 ***************************************************************************//**
 *
 * \brief   This function is used once during LPDDR4 Controller Init and  
 *          cyclic in \ref Cy_Lpddr_MemoryMonitoring. The function checks the 
 *          MR4 DRAM Mode Register which is used to adapt the refresh rate . In 
 *          MR4 DRAM provides different temperature ranges based on which the 
 *          refresh rate can be adjusted.Since the there is no Temperature 
 *          Update Flag( update available since last MR4 reading ) set at 
 *          startup because the MR4 is not read before the function checks 
 *          driver Init State to detect if a refresh update must be performed 
 *         ( due to hot re-init ) even if TUF is not set during init.
 * 
 * 
 *          The continous MR4 monitoring is handled by 
 *          \ref Cy_Lpddr_MemoryMonitoring
 *         
 *
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static void Cy_Lpddr_RefreshandTimingAdjust(volatile stc_LPDDR4_t * base)
{
    #if(DERATING_STRATEGY == DERATING_DYNAMIC)
        /* static local used to detect if temperature is rising or falling. 
           It holds the worst case arbited temperature read from MR4 from last time 
           MR4 was checked */
        static cy_en_lpddr4_mr4_refresh_rate_t oldArbitedTempRange;
    #endif

    /* arbited refresh rate from CH_A / CH_B so worst case temperature */
    cy_en_lpddr4_mr4_refresh_rate_t arbitedTempRange = CY_LPDDR4_MR4_REFRESH_RATE_TIMES_1;
    
    /* local variabel for MR4 value CH_A */
    un_dram_mr4_t mr4ValueChA;
    
    /* local variabel for MR4 value CH_B */
    un_dram_mr4_t mr4ValueChB;

    /*local struct to catch reading MRR results */
    cy_stc_lpddr4_mrw_mrr_data_t mr4Value;
    /* local variable to update TREG5 ( refresh rate TREFI )  */
     un_LPDDR4_LPDDR4_CORE_TREG5_t timingRegister5;
    /* local variable for derating factor used to calculate new tREFI */
    double refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_1_TREFI;
    /*  Read MR4 determine whether tREFI adjustment is needed or AC timing 
        de-rating is required in the elevated temperature range */
    Cy_Lpddr_ReadModeRegister(LPDDR40,CY_LPDDR4_MODEREGISTER_MR4,&mr4Value);
    /* */
    mr4ValueChA.byteValue = mr4Value.mrw_CHA_Data;
    mr4ValueChB.byteValue = mr4Value.mrw_CHB_Data;
    /*timing adjustment only needed if there was a update from memory in MR4 register
     then TUF should be set or if we are not initialized and there is no TUF */
    if ((mr4ValueChA.stcField.temperatureUpateFlag == 1) || 
        (mr4ValueChB.stcField.temperatureUpateFlag == 1) || 
        (g_stc_lpddr_context.refreshStrategyChanged == CY_LPDDR4_STRATEGY_CHANGED) ||
        (Cy_Lpddr_GetContextInitState() == CY_LPDDR4_CNTRL_NOT_INITIALIZED ))
    {
        /*  first check which channels are activated then check which has 
            highest temperature for derating and refresh rate adaption */
        if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_A)
        {
            arbitedTempRange = mr4ValueChA.stcField.refreshRate;
        }
        else if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_B) 
        {
            arbitedTempRange = mr4ValueChB.stcField.refreshRate;
        }
        else if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_AB)
        {
            arbitedTempRange = (mr4ValueChA.stcField.refreshRate >= mr4ValueChB.stcField.refreshRate) ? 
                                                                    mr4ValueChA.stcField.refreshRate : 
                                                                    mr4ValueChB.stcField.refreshRate;
        }
        switch(arbitedTempRange)
        {
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_4:
            {
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_4;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_4_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_2:
            {
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_2;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_2_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_1:
            {
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_1;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_1_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P5:
        	{
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_0P5;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_0p5_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_NO_DERATING:
            {
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_0P25;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_WITH_DERATING:
            {
                
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_0P25;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI;
                }
                break;
            }
            /* exceeds allowed range -> appl sw must monitor the temperature 
               by checking context temperature from time to time */
            case CY_LPDDR4_MR4_REFRESH_RATE_HIGH_TEMP_LIMIT:
            {
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_HIGH ;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_0P25;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI;
                }
                break;
            }
            case CY_LPDDR4_MR4_REFRESH_RATE_LOW_TEMP_LIMIT:
            {
                g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_EXCEED_LOW ;
                if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
                {
                    g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_4;
                    refreshFactor = CY_LPDDR_FIXED_REFRESH_RATE_4_TREFI;
                }
                break;
            }
        }
        
        Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq();

        /**************** Adjust Refresh Rate *********************************/
        /*  read current TREG5 value  */
        timingRegister5.u32Register = LPDDR40->LPDDR4_CORE.unTREG5.u32Register;
        if(CY_LPDDR_REFRESHRATE_DYNAMIC == Cy_Lpddr_GetCntrlrCntxtRefRtStrat())
        {
             /* set new t_REFI according what Memory is requesting in MR4 OP[2:0] */
             /* a compensation value is applied to match the refresh requirements also during full load. If a refresh gets 
                pending a ongoing burst must be finished. The time needed to finish the burst until refresh can be issued must 
                be substracted from trefi an the value for TREFI must be MOD4 */
            timingRegister5.stcField.u14T_REFI = CY_LPDDR4_CLOCKCYCLES_TREFI_COMPENSATED(JESD209_4B_NS_TREFI*refreshFactor,CY_LPDDR4_FSP1_FREQ,CY_LPDDR4_TREFI_NCK_COMPENSATE);
            /*clear a possible refreshRateStrategy Change Request*/
           // g_stc_lpddr_context.refreshStrategyChanged == CY_LPDDR4_STRATEGY_UNCHANGED;

        }
        else
        {
            /* when user has selected to CY_LPDDR_REFRESHRATE_FIXED check if a value was provided. 
               If no value was provided .fixedUserRefreshRate==0   set refresh rate to 105°C  */
            if(Cy_Lpddr_GetContextFixedUsrRefRate()==0) 
            {
                /* set new t_REFI to 0.25 x Refresh */
                /* a compensation value is applied to match the refresh requirements also during full load. If a refresh gets 
                pending a ongoing burst must be finished. The time needed to finish the burst until refresh can be issued must 
                be substracted from trefi an the value for TREFI must be MOD4 */                
                timingRegister5.stcField.u14T_REFI = CY_LPDDR4_CLOCKCYCLES_TREFI_COMPENSATED((JESD209_4B_NS_TREFI*CY_LPDDR_FIXED_REFRESH_RATE_0p25_TREFI),CY_LPDDR4_FSP1_FREQ,CY_LPDDR4_TREFI_NCK_COMPENSATE);
                g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_TIMES_0P25;
            }
            /* if a value was provided use the value given */
            else
            {
                /* value for refresh rate is taken from the LPDDR4 configuration no further compensation is applied.
                   As this is for testing purpose only the user should know what he is doing */
                timingRegister5.stcField.u14T_REFI = Cy_Lpddr_GetContextFixedUsrRefRate();
                g_stc_lpddr_context.refreshRate = CY_LPDDR4_REFRESH_RATE_USER_FIXED;
            }
            /*clear a possible refreshRateStrategy Change Request*/
            //g_stc_lpddr_context.refreshStrategyChanged == CY_LPDDR4_STRATEGY_UNCHANGED;
        }
        /* write back the new value to TREG5 */
        LPDDR40->LPDDR4_CORE.unTREG5.u32Register = timingRegister5.u32Register;
        /**************** End of Adjust Refresh Rate *************/
        
        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            /************************** Timing derating ***************************/
            /*   temperature direction from cold to hot entering derating zone    */
            if( (oldArbitedTempRange <= CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_NO_DERATING) && 
                (arbitedTempRange > CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_NO_DERATING))
            {
                Cy_Lpddr_PerformTimingDerating(base,CY_LPDDR4_DERATING_ACTIVE);
                g_stc_lpddr_context.deratingStatus = CY_LPDDR4_DERATING_ACTIVE;
            }/*   temperature direction from hot to cold leaving derating zone    */
            else if((oldArbitedTempRange > CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_NO_DERATING) && 
                    (arbitedTempRange < CY_LPDDR4_MR4_REFRESH_RATE_TIMES_0P25_WITH_DERATING))
            {
                Cy_Lpddr_PerformTimingDerating(base,CY_LPDDR4_DERATING_NOT_ACTIVE);
                g_stc_lpddr_context.deratingStatus = CY_LPDDR4_DERATING_NOT_ACTIVE;
            }
            /**************** End of Timing derating *************/
            /*  store actual temp for next evaluation run */
            oldArbitedTempRange = arbitedTempRange;
        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
            g_stc_lpddr_context.deratingStatus = CY_LPDDR4_DERATING_ACTIVE;
        #endif
    }
}





/*******************************************************************************
 * Function Name: Cy_Lpddr_PerformTimingDerating
 ***************************************************************************//**
 *
 * \brief   This function is used to change timing parameters based on the 
 *          Memory temperature. The driver has to monitor the Memory internal 
 *          temperature by reading the Mode Register MR4 register. This is done
 *          by cylic calling \ref Cy_Lpddr_Monitoring() If a certain temperature 
 *          was exceeded the memory can request a de-rating of a specified set 
 *          of timing parameters. If during Cy_Lpddr_Monitoring() it is detected
 *          that timing derating is necessary this function is called .
 *          Depending on the parameter timingDerate the affected register will 
 *          be programmed with either the normal values or with the derated 
 *          values and the deratingStatus of driver context status 
 *          \ref cy_stc_lpddr4_context_t is set accordingly
 * 
 *          For further information please check JEDEC Spec 209-4C chapters:<br>
 *          3.4.1 MR4 Register definition<br>
 *          Refresh Rate:<br>
 *              000B: SDRAM Low temperature operating limit exceeded<br>
 *              001B: 4x refresh<br>
 *              010B: 2x refresh<br>
 *              011B: 1x refresh (default)<br>
 *              100B: 0.5x refresh<br>
 *              101B: 0.25x refresh, no de-rating<br>
 *              110B: 0.25x refresh, with de-rating<br>
 *              111B: SDRAM High temperature operating limit exceeded<br>
 *
 *          10.3 Temperature Derating for AC Timing <br>
 *              |tDQSCK  |MAX| 3600              |ps|<br>
 *              |tRCD    |MIN| tRCD + 1.875      |ns|<br>
 *              |tRC     |MIN| tRC + 3.75        |ns|<br>
 *              |tRAS    |MIN| tRAS + 1.875      |ns|<br>
 *              |tRP     |MIN| tRP + 1.875       |ns|<br>
 *              |tRRD    |MIN| tRRD + 1.875      |ns|<br>
 * 
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   timingDerate if derating is applied or not 
 *          \ref cy_en_lpddr4_MR4_derate_state_t<br>
 *              CY_LPDDR4_DERATING_ACTIVE       derating active<br>
 *              CY_LPDDR4_DERATING_NOT_ACTIVE   derating deactivated<br>
 *
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static void Cy_Lpddr_PerformTimingDerating(volatile stc_LPDDR4_t * base,
                                    cy_en_lpddr4_MR4_derate_state_t timingDerate)
{  
    /*  local variable to for the timing 
        registers which need to be adapted */
    un_LPDDR4_LPDDR4_CORE_TREG1_t tempTREG1;
    un_LPDDR4_LPDDR4_CORE_TREG2_t tempTREG2;
    un_LPDDR4_LPDDR4_CORE_TREG3_t tempTREG3;
    un_LPDDR4_LPDDR4_CORE_TREG7_t tempTREG7;
    un_LPDDR4_LPDDR4_CORE_TREG12_t tempTREG12;

    /* local variable for the actual controller state*/
    //cy_en_lpddr4_cntrl_run_state_t loc_cntrlState;
    /* local variable for the actual controller GSM state*/
    cy_en_lpddr4_cntrl_gsm_state_t lactualGSMState;

    /* read current registers content */
    tempTREG1.u32Register   =   base->LPDDR4_CORE.unTREG1.u32Register;
    tempTREG2.u32Register   =   base->LPDDR4_CORE.unTREG2.u32Register;
    tempTREG3.u32Register   =   base->LPDDR4_CORE.unTREG3.u32Register;
    tempTREG7.u32Register   =   base->LPDDR4_CORE.unTREG7.u32Register;
    tempTREG12.u32Register  =   base->LPDDR4_CORE.unTREG12.u32Register;

    if(timingDerate == CY_LPDDR4_DERATING_ACTIVE)
    {
        /* AC timing derating active */
        /*  According to JEDEC209-4C following parameter are underlying a derating
            tDQSCK      MAX     3600        ps
            tRCD        MIN     tRCD+1.875  ns
            tRC         MIN     tRC+3.75    ns
            tRAS        MIN     tRAS+1.875  ns
            tRP         MIN     tRP+1.875   ns
            tRRD        MIN     tRRD+1.875  ns
        */
        /* TODO check tDQSCK */
        tempTREG1.stcField.u6T_RCD  = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)-4):0;
        /* tRC = tRAS + tRPab */
        tempTREG2.stcField.u8T_RC   = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))>8?
                                        ((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))-8):0;
        
        tempTREG2.stcField.u8T_RAS  =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)-4):0;
        
        tempTREG3.stcField.u6T_RP   =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)-4):0;
        

        tempTREG12.stcField.u5T_RRD =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRRD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRRD);
            /*  additional registers which need to be updated as they are based on one 
            of the above timing parameters  tMPCWR=tRCD + 3*/
        tempTREG7.stcField.u6T_MPCWR = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD) + 3);
        
        // /*CDT 374734 additional parameter =WL + BL/2 + tWR-3 + 1*/
        // /* tWR=max(18ns, 6nCK) */
        // /* unLPMR1.stcField.u2FS0_BL = CY_LPDDR4_BL_16 --> +8
        //                                CY_LPDDR4_BL_32 --> +16*/
        // tempTREG4.stcField.u8T_WLBR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWR) - 3; //=tWR-3
        // if( base->LPDDR4_CORE.unLPMR1.stcField.u2FS1_BL == CY_LPDDR4_BL_16)
        // {
        //     tempTREG4.stcField.u8T_WLBR +=8;    //=+BL/2 + tWR-3
        // }
        // else if(base->LPDDR4_CORE.unLPMR1.stcField.u2FS1_BL == CY_LPDDR4_BL_32)
        // {
        //     tempTREG4.stcField.u8T_WLBR +=16;   //=+BL/2 + tWR-3
        // }
        // if(base->LPDDR4_CORE.unLPMR2.stcField.u1FS1_WLS==0)
        // {
        //     tempTREG4.stcField.u8T_WLBR+=CY_LPDDR_WRITELATENCY_FSP1_SETA + 1;//=WL + BL/2+ tWR-3 + 1
        // }
        // else
        // {
        //     tempTREG4.stcField.u8T_WLBR+=CY_LPDDR_WRITELATENCY_FSP1_SETB + 1;//=WL + BL/2+ tWR-3 + 1
        // }
    }
    else
    {
        /* AC timing derating not active */
        tempTREG1.stcField.u6T_RCD  =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)-4):0;
        
        /* tRC = tRAS + tRPab */
        tempTREG2.stcField.u8T_RC   =  ((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)))>8?
                                        ((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))-8):0; 
        
        tempTREG2.stcField.u8T_RAS  =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)-4):0;
        
        tempTREG3.stcField.u6T_RP   =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)>4?
                                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)-4):0;
       
        tempTREG12.stcField.u5T_RRD =   CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRRD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRRD);

        /*  additional registers which need to be updated as they are based on one 
            of the above timing parameters tMPCWR=tRCD + 3*/
        tempTREG7.stcField.u6T_MPCWR = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD) + 3);

        // /*CDT 374734 additional parameter =WL + BL/2 + tWR-3 + 1*/
        // /* tWR=max(18ns, 6nCK) */
        // /* unLPMR1.stcField.u2FS0_BL = CY_LPDDR4_BL_16 --> +8
        //                                CY_LPDDR4_BL_32 --> +16*/
        // tempTREG4.stcField.u8T_WLBR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWR) - 3; //=tWR-3
        // if( base->LPDDR4_CORE.unLPMR1.stcField.u2FS1_BL == CY_LPDDR4_BL_16)
        // {
        //     tempTREG4.stcField.u8T_WLBR +=8;    //=+BL/2 + tWR-3
        // }
        // else if(base->LPDDR4_CORE.unLPMR1.stcField.u2FS1_BL == CY_LPDDR4_BL_32)
        // {
        //     tempTREG4.stcField.u8T_WLBR +=16;   //=+BL/2 + tWR-3
        // }
        // if(base->LPDDR4_CORE.unLPMR2.stcField.u1FS1_WLS==0)
        // {
        //     tempTREG4.stcField.u8T_WLBR+=CY_LPDDR_WRITELATENCY_FSP1_SETA;//=WL + BL/2+ tWR-3
        // }
        // else
        // {
        //     tempTREG4.stcField.u8T_WLBR+=CY_LPDDR_WRITELATENCY_FSP1_SETB;//=WL + BL/2+ tWR-3
        // }
    }
    /*get actual Conntroller Run State*/
    lactualGSMState = Cy_Lpddr_GetCurrenGSMState();

    /*  if controller is running stop the controller to update the timings */
    //if(loc_cntrlState == CY_LPDDR4_CNTRL_RUNNING )
    if(lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN )
    {
        /* if controller is running stop it first */
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_STOP,CY_LPDDR4_NONE_MR,0);
    }
    /*  Update the Timing Regs with new values */
    base->LPDDR4_CORE.unTREG1.u32Register = tempTREG1.u32Register;
    base->LPDDR4_CORE.unTREG2.u32Register = tempTREG2.u32Register;
    base->LPDDR4_CORE.unTREG3.u32Register = tempTREG3.u32Register;
    base->LPDDR4_CORE.unTREG7.u32Register = tempTREG7.u32Register;
    base->LPDDR4_CORE.unTREG12.u32Register = tempTREG12.u32Register;
    
    /* if controller was running before - restart the controller */
    //if(loc_cntrlState == CY_LPDDR4_CNTRL_RUNNING )
    if(lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN )
    {
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN,CY_LPDDR4_NONE_MR,0);  
    }
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_MemoryMonitoring()
 ***************************************************************************//**
 * 
 *  \brief Function to perform some LPDDR4 Memory Monitoring tasks 
 *      ( still under construction )
 * 
 *  Function which needs to be called cyclic with a interval calculated by 
 *  Cy_Lpddr_CalcMR4ReadInterval( minimum interval). As voltage and temperature 
 *  on the SDRAM die and Controller changes , some of the parameters need to be 
 *  adapted from time to time. 
 *  The parameters affected by PVT are : Refresh Rate and DQS2DQ timing skew
 * 
 * This function is performing the following steps to compensate for PVT changes
 * 
 * For Refresh Rate compensation base on DRAM MR4 reading  OP[2:0]<br>
 *     -000b: SDRAM low temperature operating limit exceeded<br>
 *     -001b: 4x refresh<br>
 *     -010b: 2x refresh<br>
 *     -011b: 1x refresh (default)<br>
 *     -100b: 0.5x refresh<br>
 *     -101b: 0.25x refresh, no derating<br>
 *     -110b: 0.25x refresh, with derating<br>
 *     -111b: SDRAM high temperature operating limit exceeded <br>
 * -adjust refresh rate accoring to MR4 readout<br>
 * -if memory is requesting derating some timing parameters need to 
 *  be adapted with temperature penalty.
 *  \ref Cy_Lpddr_PerformTimingDerating gets called to update the timings.
 *  Check Jedec 209-4C 10.3 Temperature Derating  for AC Timing
 *  -if temperature of external device is out of range. Update the 
 *  driver context flags for \ref cy_stc_lpddr4_context_t 
 *  operationTemperatureState
 * 
 *  <table class="doxtable">
 *  <tr><th>Parameter</th>
 *      <th>Symbol</th>
 *      <th>Min/Max </th>
 *      <th>Value</th>
 *      <th>Unit</th>
 *  </tr>
 *      <tr><td>DQS output access time <br>from CK_t/CK_c (derated)</td>
 *          <td>tDQSCK</td> <td>MAX</td>
 *          <td>3600</td>
 *           <td>ps </td> 
 *  </tr>
 *      <tr><td>RAS-to-CAS delay (derated)</td>
 *          <td>tRCD</td> <td>min</td>
 *          <td>tRCD + 1.875</td>
 *          <td>ns</td>
 * </tr>
 *  <tr><td>ACTIVATE-to- ACTIVATE<br>command period (derated) </td>
 *          <td>tRC</td> <td>min</td>
 *          <td>tRC + 3.75</td>
 *          <td>ns</td>
 * </tr>
 *  <tr><td>Row active time (derated)</td>
 *          <td>tRAS</td> <td>min</td>
 *          <td>tRAS + 1.875</td>
 *          <td>ns</td>
 * </tr>
 *  <tr><td>Row precharge time <br>(derated) </td>
 *          <td>tRP</td> <td>min</td>
 *          <td>tRP + 1.875</td>
 *          <td>ns</td>
 * </tr>
 *  <tr><td>Active bank A to <br>active bank B (derated) </td>
 *          <td>tRRD</td> <td>min</td>
 *          <td>tRRD + 1.875</td>
 *          <td>ns</td>
 *  </tr>
 *  </table>  
 * 
 *
 * 
 *  DQS2DQ Timing check necessity for retraining<br>
 *          <li> Stop the Controller</li>
 *          <li> Start DQS Oscillator</li>
 *          <li> Restart the controller</li>
 *          <li> Read the Result of DQS Oscillator (MRR MR19/MR18)</li>
 *          <li> calculate the need for retraining and direction to train</li>
 * 
 ******************************************************************************/
void Cy_Lpddr_MemoryMonitoring(void)
{
    /* delta DQS Count between last successful training and actual value DQS2DQ drift check */
    uint32_t dqsOSCDeltaCHA = 0;
    uint32_t dqsOSCDeltaCHB = 0;
    
    /* Start the DQS Oscillator for the activated channels*/
    Cy_Lpddr_StartDQSOscillator(LPDDR40);
    
    /* as long as DQS Oscillator is running use the time do 
       something different */
    Cy_Lpddr_RefreshandTimingAdjust(LPDDR40);
    /* Read the DQS2DQ Oscillator */
    if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadDQSOscValue(LPDDR40,&(g_stc_lpddr_context.actualDQSOsc)))
    {
        CY_ASSERT(0);
    }
    
    /* check if the system was ever trained */
    if( (g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA != 0 ) &&
        (g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB != 0 ))
    {
        /*  
        check the delta drift of the DSQ Osc. count and set the trainings direction based on this. 
        - temperature increases --> DQS Counter value decreases --> retrainDirection = false
        - temperature decreases --> DQS Counter value increase  --> retrainDirection = true */

        //TODO check the assignment of the directions

        /*              Check DQS for CHANNEL A               */
        if(g_stc_lpddr_context.actualDQSOsc.value_CHA > g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA)
        {
            dqsOSCDeltaCHA  = g_stc_lpddr_context.actualDQSOsc.value_CHA - g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA;
            if(dqsOSCDeltaCHA > g_stc_lpddr_context.retrainThresholdCHA)
            {
                Cy_Lpddr_SetControllerContextRetrainRequestCHA();
                g_stc_lpddr_context.retrainDirection = CY_LPDDR4_DQSDQCR_DIR_UP;
            }
        }
        else
        {
            dqsOSCDeltaCHA = g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA - g_stc_lpddr_context.actualDQSOsc.value_CHA;
            if(dqsOSCDeltaCHA >  g_stc_lpddr_context.retrainThresholdCHA)
            {
                Cy_Lpddr_SetControllerContextRetrainRequestCHA();
                g_stc_lpddr_context.retrainDirection = CY_LPDDR4_DQSDQCR_DIR_DOWN;
            }
        }
        /*              Check DQS for CHANNEL B               */
        if(g_stc_lpddr_context.actualDQSOsc.value_CHB > g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB)
        {
            dqsOSCDeltaCHB  = g_stc_lpddr_context.actualDQSOsc.value_CHB - g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB;
            if(dqsOSCDeltaCHB > g_stc_lpddr_context.retrainThresholdCHB)
            {
                Cy_Lpddr_SetControllerContextRetrainRequestCHB();
                g_stc_lpddr_context.retrainDirection = CY_LPDDR4_DQSDQCR_DIR_UP;
            }
        }
        else
        {
            dqsOSCDeltaCHB = g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB - g_stc_lpddr_context.actualDQSOsc.value_CHB;
            if(dqsOSCDeltaCHB > g_stc_lpddr_context.retrainThresholdCHB)
            {
                Cy_Lpddr_SetControllerContextRetrainRequestCHB();
                g_stc_lpddr_context.retrainDirection = CY_LPDDR4_DQSDQCR_DIR_DOWN;
            }
        }
   
        if((g_stc_lpddr_context.retrainRequestCHA == true) || (g_stc_lpddr_context.retrainRequestCHB == true))
        {
                    g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHA = g_stc_lpddr_context.actualDQSOsc.value_CHA;
                    g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHB = g_stc_lpddr_context.actualDQSOsc.value_CHB;
        }
    }

}



/*******************************************************************************
 * Function Name: Cy_Lpddr_SetupModeRegisters
 ***************************************************************************//**
 *
 * \brief   Function to setup the mode registers of the LPDDR4 controller.
 * 
 * This function programs the Mode Registers of the LPDDR4 controller with the 
 * values for each Frequency set point as set by the user in the LPDDR4  
 * configuration  structure \ref cy_stc_lpddr4_config_t .
 * 
 * Frequency set points enable the CA bus to be switched between two differing 
 * operating frequencies with changes in voltage swings and termination values, 
 * without ever being  * in an untrained state, which could result in a loss of
 * communication to the device. This  * is accomplished by duplicating all CA 
 * bus mode register parameters, as well as other
 * mode register parameters commonly changed with operating frequency.
 * 
 * These duplicated registers form two sets that use the same mode register 
 * addresses, with read/write access controlled by MR bit FSP-WR (frequency 
 * set point write/read) and the operating point controlled by MR bit FSP-OP 
 * (FREQUENCY SET POINT operation). Changing the FSP-WR bit enables MR 
 * parameters to be changed for an alternate frequency set point without 
 * affecting the current operation.
 * Once all necessary parameters have been written to the alternate set point, 
 * changing  * the FSP-OP bit will switch operation to use all of the new 
 * parameters simultaneously (within tFC), eliminating the possibility of a 
 * loss of communication that could be caused by a partial configuration change.
 * Mode Registers with two physical 8 bit registers
 *  <ul>
 *  <li>MR 1 - Mode Register 1</li>
 *  <li>MR 2 - Mode Register 2</li>
 *  <li>MR 3 - Mode Register 1</li>
 *  <li>MR 11 - Mode Register 11</li>
 *  <li>MR 12 - Mode Register 12</li>
 *  <li>MR 14 - Mode Register 1<4/li>
 *  <li>MR 22 - Mode Register 12</li>
 *  </ul>
 * 
 * During DRAM init which is part of the PHY initialisation process 
 * ( draminiten = 1 set in POM PHY Operational Mode register ) the values 
 * programmed in the controllers LPMRx registers are programmed to the DRAMs 
 * Mode Registers for FSP[0]/[1]. The LPDDR4 controller registers as for 
 * example LPMR1 contain all neccessary value for FSP[0] and FSP[1].
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 *
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupModeRegisters(volatile stc_LPDDR4_t * base,
                                                  const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_BAD_PARAM;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unLPMR1.u32Register = config->unLPMR1.u32Register;
        base->LPDDR4_CORE.unLPMR2.u32Register = config->unLPMR2.u32Register;
        base->LPDDR4_CORE.unLPMR3.u32Register = config->unLPMR3.u32Register;
        base->LPDDR4_CORE.unLPMR11.u32Register = config->unLPMR11.u32Register;
        base->LPDDR4_CORE.unLPMR12.u32Register = config->unLPMR12.u32Register;
        base->LPDDR4_CORE.unLPMR13.u32Register = config->unLPMR13.u32Register;
        base->LPDDR4_CORE.unLPMR14.u32Register = config->unLPMR14.u32Register;
        base->LPDDR4_CORE.unLPMR22.u32Register = config->unLPMR22.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
 return retval;
}

/*******************************************************************************
 * Function Name: Cy_Lpddr_SetPTSRRegisters
 ***************************************************************************//**
 *
 * \brief   Function to setup the PTRS(PHY Training Status Registers) 
 *          registers of the LPDDR4 controller.
 * 
 * The PTSR registers are used for 2 purposes.
 *  <ol>
 *  <li>catch training data after successfull training</li>
 *  <li>load training data to the controller PHY for quick init</li>
 *  </ol>
 * 
 * Once LPDDR4 controller has successfully finished a training it automatically 
 * updates  * the corresponding PTSRx registers with the trained values. 
 * Storing these data to a permanent memory location provides the possibility 
 * to use it later on for a quick initialization without the need for a 
 * complete training.E.g. quick power up after a power down.<br>
 * 
 * This function reloads the PTSRx registers with the values from the LPDDR4 
 * config structure \ref  cy_stc_lpddr4_config_t. So they can be reloaded to 
 * the PHY using the function \ref Cy_Lpddr_PhyReload()
 * 
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 *
 * \return  \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetPTSRRegisters(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unPTSR0.u32Register = config->unPTSR0.u32Register;
        base->LPDDR4_CORE.unPTSR1.u32Register = config->unPTSR1.u32Register;
        base->LPDDR4_CORE.unPTSR2.u32Register = config->unPTSR2.u32Register;
        base->LPDDR4_CORE.unPTSR3.u32Register = config->unPTSR3.u32Register;
        base->LPDDR4_CORE.unPTSR4.u32Register = config->unPTSR4.u32Register;
        base->LPDDR4_CORE.unPTSR5.u32Register = config->unPTSR5.u32Register;
        base->LPDDR4_CORE.unPTSR6.u32Register = config->unPTSR6.u32Register;
        base->LPDDR4_CORE.unPTSR7.u32Register = config->unPTSR7.u32Register;
        base->LPDDR4_CORE.unPTSR8.u32Register = config->unPTSR8.u32Register;
        base->LPDDR4_CORE.unPTSR9.u32Register = config->unPTSR9.u32Register;
        base->LPDDR4_CORE.unPTSR10.u32Register = config->unPTSR10.u32Register;
        base->LPDDR4_CORE.unPTSR11.u32Register = config->unPTSR11.u32Register;
        base->LPDDR4_CORE.unPTSR12.u32Register = config->unPTSR12.u32Register;
        base->LPDDR4_CORE.unPTSR13.u32Register = config->unPTSR13.u32Register;
        base->LPDDR4_CORE.unPTSR14.u32Register = config->unPTSR14.u32Register;
        base->LPDDR4_CORE.unPTSR15.u32Register = config->unPTSR15.u32Register;
        base->LPDDR4_CORE.unPTSR16.u32Register = config->unPTSR16.u32Register;
        base->LPDDR4_CORE.unPTSR17.u32Register = config->unPTSR17.u32Register; 
        base->LPDDR4_CORE.unPTSR18.u32Register = config->unPTSR18.u32Register; 
        base->LPDDR4_CORE.unPTSR19.u32Register = config->unPTSR19.u32Register; 
        base->LPDDR4_CORE.unPTSR20.u32Register = config->unPTSR20.u32Register; 
        base->LPDDR4_CORE.unPTSR21.u32Register = config->unPTSR21.u32Register; 
        base->LPDDR4_CORE.unPTSR22.u32Register = config->unPTSR22.u32Register; 
        base->LPDDR4_CORE.unPTSR23.u32Register = config->unPTSR23.u32Register; 
        base->LPDDR4_CORE.unPTSR24.u32Register = config->unPTSR24.u32Register;
        base->LPDDR4_CORE.unPTSR25.u32Register = config->unPTSR25.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
 return retval;
}




/*******************************************************************************
 * Function Name: Cy_Lpddr_SetQoSPrios
 ***************************************************************************//**
 *
 * \brief   Function to program the Quality of Service Registers of the LPDDR4
 *          Wrapper Registers with the values specified in the LPDDR4 
 *          Config structure \ref cy_stc_lpddr4_config_t.
 *          
 * \note    The register must be configured when IP is Disabled (CTL.ENABLED =0). 
 *          Any attempt to modify the register after IP Enable (CTL.ENABLED =1) 
 *          would get  AHB error response. 
 *          
 *
 * \param   base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param   config pointer to the LPDDR4 Initialization Structure \ref cy_stc_lpddr4_config_t
 *
 * \return  \ref cy_en_lpddr4_retval_t<br>
 *          CY_LPDDR4_SUCCESS if everything was ok<br>
 *          CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 *          CY_LPDDR4_INVALID_STATE if controller was already activated
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetQoSPrios(volatile stc_LPDDR4_t * base,
                          const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval;
    if(( NULL != base ) && (NULL != config))
    {
        if(base->LPDDR4_WRAPPER.unCTL.stcField.u1ENABLED != 1ul)
        {
            base->LPDDR4_WRAPPER.unQOS_CPUSS.u32Register=config->unQOS_CPUSS.u32Register;
            base->LPDDR4_WRAPPER.unQOS_VIDEOSS_RD.u32Register=config->unQOS_VIDEOSS_RD.u32Register;
            base->LPDDR4_WRAPPER.unQOS_VIDEOSS_WR.u32Register=config->unQOS_VIDEOSS_WR.u32Register;
            retval = CY_LPDDR4_SUCCESS;
        }
        else
        {
            retval = CY_LPDDR4_INVALID_STATE;
        }    
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_SetupECCRegisters
 ***************************************************************************//**
 *
 * \brief Function to setup LPDDR4 Controller ECC Registers 
 * 
 *  Function to load the ECC Registers which are specifing the memory regions
 *  used in conjunction with ECC.
 * 
 *  -INECC0<br>
 *  -INECC1<br>
 *  -INECC2<br>
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 *
 *  \return \ref  cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupECCRegisters(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unINECC0.u32Register = config->unINECC0.u32Register; 
        base->LPDDR4_CORE.unINECC1.u32Register = config->unINECC1.u32Register;  
        base->LPDDR4_CORE.unINECC2.u32Register = config->unINECC2.u32Register;  
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
 return retval;
}



/*******************************************************************************
 * Function Name: Cy_Lpddr_SetupArbiter
 ***************************************************************************//**
 *
 * \brief Function to setup LPDDR4 Controller Arbiter 
 * 
 *  Function to load the following registers:<br>
 *  -RTCFG0_RT0,RTCFG1_RT0<br>
 *  -RTCFG0_RT1,RTCFG1_RT1<br>
 *  -RTCFG0_RT1,RTCFG1_RT2<br>
 *  -RTCFG0_RT1,RTCFG1_RT3<br>
 *  which are used to setup the controllers arbiter<br>
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 *
 *  \return \ref  cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetupArbiterMode(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config )
{
    cy_en_lpddr4_retval_t retval;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unRTCFG0_RT0.u32Register = config->unRTCFG0_RT0.u32Register;
        base->LPDDR4_CORE.unRTCFG0_RT1.u32Register = config->unRTCFG0_RT1.u32Register;
        base->LPDDR4_CORE.unRTCFG0_RT2.u32Register = config->unRTCFG0_RT2.u32Register;
        base->LPDDR4_CORE.unRTCFG0_RT3.u32Register = config->unRTCFG0_RT3.u32Register;

        base->LPDDR4_CORE.unRTCFG1_RT0.u32Register = config->unRTCFG1_RT0.u32Register;
        base->LPDDR4_CORE.unRTCFG1_RT1.u32Register = config->unRTCFG1_RT1.u32Register;
        base->LPDDR4_CORE.unRTCFG1_RT2.u32Register = config->unRTCFG1_RT2.u32Register;
        base->LPDDR4_CORE.unRTCFG1_RT3.u32Register = config->unRTCFG1_RT3.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}



/*******************************************************************************
 *  Function Name:  Cy_Lpddr_SetAdressConfig
 ***************************************************************************//**
 *  \brief Function is used to setup the LPDDR4 address configuration.
 *  
 *  The user has to set the address mapping in the LPDDR4 configuration structure
 *  \ref cy_stc_lpddr4_config_t. 
 *  This function programs the values given in the configuration structure
 *  to the LPDDR4 controller registers ADDR0..ADDR5
 * 
 *  Set the Adress Mapping scheme for the AXI PORT Adresses<br>
 *      1 bit for channel address           [29]<br>
 *      3 bits for bank address (8 banks)   [28:26]<br>
 *      15 bits for row address             [25:11]<br>
 *      10 bits for column address          [10:1]<br>
 *
 *      | ch | bank | row | coloumn |
 *
 *  The value programmed in the respective bitfield corresponds to the 
 *  bit position
 * 
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *  \param  config pointer to the LPDDR4 Initialization Structure 
 *          \ref cy_stc_lpddr4_config_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br> 
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_SetAdressConfig(volatile stc_LPDDR4_t * base,
                                                const cy_stc_lpddr4_config_t * config)
{
    cy_en_lpddr4_retval_t retval;
    if(( NULL != base ) && (NULL != config))
    {
        base->LPDDR4_CORE.unADDR0.u32Register = config->unADDR0.u32Register;
        base->LPDDR4_CORE.unADDR1.u32Register = config->unADDR1.u32Register;
        base->LPDDR4_CORE.unADDR2.u32Register = config->unADDR2.u32Register;
        base->LPDDR4_CORE.unADDR3.u32Register = config->unADDR3.u32Register;
        base->LPDDR4_CORE.unADDR4.u32Register = config->unADDR4.u32Register;
        base->LPDDR4_CORE.unADDR5.u32Register = config->unADDR5.u32Register;
        retval = CY_LPDDR4_SUCCESS;
    }
    else 
    {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}

    /*******************************************************************************
     *  Function Name:  Cy_Lpddr_LPDDR4StepwiseTraining
     ***************************************************************************//**
     *  \brief Function to trigger perform training stepwise
     *
     *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
     *
     *  \param config Pointer to the LPDDR4 config structure
     *
     *
     *  \return \ref    cy_en_lpddr4_retval_t<br>
     *                  CY_LPDDR4_SUCCESS if everything was ok<br>
     *                  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the
     *                  function<br>
     ******************************************************************************/
    static cy_en_lpddr4_retval_t Cy_Lpddr_LPDDR4StepwiseTraining(volatile stc_LPDDR4_t *base, const cy_stc_lpddr4_config_t *config)
    {
        bool fl_vrefdqrderr = false;
        bool fl_vrefcaerr = false;
        bool fl_gterr = false;
        bool fl_wrlvlerr = false;
        bool fl_rdlvldmerr = false;
        bool fl_vrefdqwrerr = false;
        // bool fl_dlyevalerr = false;
        bool fl_sanchkerr = false;

        bool vrefcaen_gten_Cmplt=false;
        bool vrefqdrdenCmplt=false;
        bool rdlvlCmplt=false;
        bool wrlvlCmplt=false;
        bool dlyevalenCmplt=false;
        bool vrefdqwrenCmplt=false;
        bool sanchkenCmplt=false;
        /*  local variable for training status.
            default no training error present */
        bool fl_error_training = false;

        /*  local to collect all request before programming them
            all at once to the controller */
        un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;

        //  local to catch the PHY Operational Status
        un_LPDDR4_LPDDR4_CORE_POS_t unPOSval;
        //  local to catch the PHY Training Status 0
        un_LPDDR4_LPDDR4_CORE_PTS0_t unPTS0val;
        
        uint32_t pomCHANEN = (uint32_t)Cy_Lpddr_GetContextChannelConfig();
 
        /* perform training stepwise refer CDT 373844*/
        /* to be impl. needed with Silicon must be reworked after having more*/
        //DFIEN=0

        /*  - POM_PHYINIT & POM_DLLRSTEN & POM_CLKLOCKEN    --> part of Cy_Lpddr_PhyInit()
             - POM_PHYINIT & POM_DRAMINITEN                  --> part of Cy_Lpddr_PhyInit()
             
             - POM_PHYINIT & POM_VREFCAEN & POM_GTEN
             - POM_PHYINIT & POM_VREFDQRDEN
             - POM_PHYINIT & POM_RDLVLEN
             - POM_PHYINIT & POM_WRLVLEN
             - POM_PHYINIT & POM_DLYEVALEN
             - POM_PHYINIT & POM_VREFDQWREN
             - POM_PHYINIT & POM_SANCHKEN
             - POM_CHANEN & POM_DFIEN & POM_FS (functional) */

        //DFIEN=1



    // __      _______  ______ ______ _____          ______ _   _             _____ _______ ______ _   _ 
    // \ \    / /  __ \|  ____|  ____/ ____|   /\   |  ____| \ | |   ___     / ____|__   __|  ____| \ | |
    //  \ \  / /| |__) | |__  | |__ | |       /  \  | |__  |  \| |  ( _ )   | |  __   | |  | |__  |  \| |
    //   \ \/ / |  _  /|  __| |  __|| |      / /\ \ |  __| | . ` |  / _ \/\ | | |_ |  | |  |  __| | . ` |
    //    \  /  | | \ \| |____| |   | |____ / ____ \| |____| |\  | | (_>  < | |__| |  | |  | |____| |\  |
    //     \/   |_|  \_\______|_|    \_____/_/    \_\______|_| \_|  \___/\/  \_____|  |_|  |______|_| \_|
        //Command Bus Training and Gate Training
        // POM_PHYINIT & POM_VREFCAEN & POM_GTEN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_VREFCAEN = 0x1;
        unPOMval.stcField.u1POM_GTEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x0;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(vrefcaen_gten_Cmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
                        /*  Command Bus Training    */
            if (unPOSval.stcField.u1VREFCAC == 1)
            {
                /*  VREFCA training error on rank 0. vrefcaerr[1] is for LPDDR4
                    channel B and vrefcaerr[0] is for LPDDR4 channel A. 1 means
                    VREFCA training for corresponding LPDDR4 channel is not
                    completed successfully  */
                fl_vrefcaerr = (unPTS0val.stcField.u2VREFCAERR >= 1);
            }
            /*  Gate Training   */
            if (unPOSval.stcField.u1GTC == 1)
            {
                /*  Gate training error on rank 0. One bit for each data slice.
                1 means gate training is not completed successfully   */
                fl_gterr = (unPTS0val.stcField.u4GTERR >= 1);
            }
            if ((unPOSval.stcField.u1PHYINITC == 0x1) && (unPOSval.stcField.u1VREFCAC == 0x01) && (unPOSval.stcField.u1GTC == 0x01))
            {
                vrefcaen_gten_Cmplt = true;
            }
            /*  check current Frequency set point 0 request  in Phy Operational
                Status Register  */
            else if (unPOSval.stcField.u1FS0REQ == 1)
            {
                /*  change to FSP[0] f = 50 MHz */
                Cy_Lpddr_SetPhyFrequency(base, &config->pllconfig, CY_LPDDR4_FSP0);
                /*  give confirmation to PHY that frequency was changed an PHY
                    can proceed  */
                Cy_Lpddr_PhyProceed(base);
            }
            else if (unPOSval.stcField.u1FS1REQ == 1)
            {
                /*  change to FSP[1] f = 800 MHz */
                Cy_Lpddr_SetPhyFrequency(base, &config->pllconfig, CY_LPDDR4_FSP1);
                /*  give confirmation to PHY that frequency was changed an PHY
                    can proceed  */
                Cy_Lpddr_PhyProceed(base);
            }
            // END OF CBT AND Gate Training
        }
        fl_error_training =  fl_gterr || fl_vrefcaerr;
        if (fl_error_training)
        {
            return CY_LPDDR4_ERROR;
        }

    // __      _______  ______ ______ _____   ____  _____  _____  ______ _   _ 
    // \ \    / /  __ \|  ____|  ____|  __ \ / __ \|  __ \|  __ \|  ____| \ | |
    //  \ \  / /| |__) | |__  | |__  | |  | | |  | | |__) | |  | | |__  |  \| |
    //   \ \/ / |  _  /|  __| |  __| | |  | | |  | |  _  /| |  | |  __| | . ` |
    //    \  /  | | \ \| |____| |    | |__| | |__| | | \ \| |__| | |____| |\  |
    //     \/   |_|  \_\______|_|    |_____/ \___\_\_|  \_\_____/|______|_| \_|
        // POM_PHYINIT & POM_VREFDQRDEN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_VREFDQRDEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(vrefqdrdenCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  PHY VREF-DQ training */
            if (unPOSval.stcField.u1VREFDQRDC == 1)
            {
                /*  Read VREF-DQ training error on rank 0. One bit for each data
                slice. 1 means PHY cannot find suitable VREF setting for its
                receiver  */
                fl_vrefdqrderr = (unPTS0val.stcField.u4VREFDQRDERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                vrefqdrdenCmplt=true;    
            }
            // END OF VREFDQRDEN Training
        }
      
        if (fl_vrefdqrderr)
        {
            return CY_LPDDR4_ERROR;
        }

    //  _____  _____  _ __      ___      ______ _   _ 
    // |  __ \|  __ \| |\ \    / / |    |  ____| \ | |
    // | |__) | |  | | | \ \  / /| |    | |__  |  \| |
    // |  _  /| |  | | |  \ \/ / | |    |  __| | . ` |
    // | | \ \| |__| | |___\  /  | |____| |____| |\  |
    // |_|  \_\_____/|______\/   |______|______|_| \_|
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_RDLVLEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(rdlvlCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  read leveling   */
            if (unPOSval.stcField.u1RDLVLC == 1)
            {
                /*  DM read data eye training error on rank 0. One bit for each DM
                    signal. 1 means data eye training for corresponding DM is not
                    completed successfully */
                fl_rdlvldmerr = (unPTS0val.stcField.u4RDLVLDMERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                rdlvlCmplt=true;    
            }
            // END OF VREFDQRDEN Training
        }
        if (fl_rdlvldmerr)
        {
            return CY_LPDDR4_ERROR;
        }

    // __          _______  _ __      ___      ______ _   _ 
    // \ \        / /  __ \| |\ \    / / |    |  ____| \ | |
    //  \ \  /\  / /| |__) | | \ \  / /| |    | |__  |  \| |
    //   \ \/  \/ / |  _  /| |  \ \/ / | |    |  __| | . ` |
    //    \  /\  /  | | \ \| |___\  /  | |____| |____| |\  |
    //     \/  \/   |_|  \_\______\/   |______|______|_| \_|
        //POM_PHYINIT & POM_WRLVLEN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_WRLVLEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(wrlvlCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  Write leveling  */
            if (unPOSval.stcField.u1WRLVLC == 1)
            {
                /*  Write leveling error on rank 0. One bit for each data slice.
                1 means write leveling is not completed successfully  */
                fl_wrlvlerr = (unPTS0val.stcField.u4WRLVLERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                wrlvlCmplt=true;    
            }
            // END OF WRLVLEN Training
        }
        if (fl_wrlvlerr)
        {
            return CY_LPDDR4_ERROR;
        }

    //  __      _______  ______ ______ _____   ______          _______  ______ _   _ 
    //  \ \    / /  __ \|  ____|  ____|  __ \ / __ \ \        / /  __ \|  ____| \ | |
    //   \ \  / /| |__) | |__  | |__  | |  | | |  | \ \  /\  / /| |__) | |__  |  \| |
    //    \ \/ / |  _  /|  __| |  __| | |  | | |  | |\ \/  \/ / |  _  /|  __| | . ` |
    //     \  /  | | \ \| |____| |    | |__| | |__| | \  /\  /  | | \ \| |____| |\  |
    //      \/   |_|  \_\______|_|    |_____/ \___\_\  \/  \/   |_|  \_\______|_| \_|
        //POM_PHYINIT & POM_VREFDQWREN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_VREFDQWREN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(vrefdqwrenCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  Write leveling  */
            if (unPOSval.stcField.u1VREFDQWRC == 1)
            {
                /* Write VREF-DQ training error on rank 0. One bit for each data
                   slice. 1 means VREF training for DRAM is not completed
                   successfully    */
                fl_vrefdqwrerr = (unPTS0val.stcField.u4VREFDQWRERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                vrefdqwrenCmplt=true;    
            }
            // END OF VREFDQRDEN Training
        }
        if (fl_vrefdqwrerr)
        {
            return CY_LPDDR4_ERROR;
        }        

    //   _____  _  __     __________      __     _      ______ _   _ 
    //  |  __ \| | \ \   / /  ____\ \    / /\   | |    |  ____| \ | |
    //  | |  | | |  \ \_/ /| |__   \ \  / /  \  | |    | |__  |  \| |
    //  | |  | | |   \   / |  __|   \ \/ / /\ \ | |    |  __| | . ` |
    //  | |__| | |____| |  | |____   \  / ____ \| |____| |____| |\  |
    //  |_____/|______|_|  |______|   \/_/    \_\______|______|_| \_|
        //POM_PHYINIT & POM_DLYEVALEN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_DLYEVALEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(dlyevalenCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  Delay evaluation    */
            if (unPOSval.stcField.u1DLYEVALC == 1)
            {
                /* check if there is a error status field available otherwise
                delete this line */
                // fl_dlyevalerr   =  ;
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                dlyevalenCmplt=true;    
            }
            // END OF DLYEVALEN
        } 
        // if (fl_vrefdqwrerr)
        // {
        //     return CY_LPDDR4_ERROR;
        // }          

    //  __      _______  ______ ______ _____   ______          _______  ______ _   _ 
    //  \ \    / /  __ \|  ____|  ____|  __ \ / __ \ \        / /  __ \|  ____| \ | |
    //   \ \  / /| |__) | |__  | |__  | |  | | |  | \ \  /\  / /| |__) | |__  |  \| |
    //    \ \/ / |  _  /|  __| |  __| | |  | | |  | |\ \/  \/ / |  _  /|  __| | . ` |
    //     \  /  | | \ \| |____| |    | |__| | |__| | \  /\  /  | | \ \| |____| |\  |
    //      \/   |_|  \_\______|_|    |_____/ \___\_\  \/  \/   |_|  \_\______|_| \_|
        //POM_PHYINIT & POM_VREFDQWREN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_VREFDQWREN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(vrefdqwrenCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  Write leveling  */
            if (unPOSval.stcField.u1VREFDQWRC == 1)
            {
                /* Write VREF-DQ training error on rank 0. One bit for each data
                   slice. 1 means VREF training for DRAM is not completed
                   successfully    */
                fl_vrefdqwrerr = (unPTS0val.stcField.u4VREFDQWRERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                vrefdqwrenCmplt=true;    
            }
            // END OF VREFDQRDEN Training
        }
        if (fl_vrefdqwrerr)
        {
            return CY_LPDDR4_ERROR;
        }                  

    //    _____         _   _  _____ _    _ _  ________ _   _ 
    //   / ____|  /\   | \ | |/ ____| |  | | |/ /  ____| \ | |
    //  | (___   /  \  |  \| | |    | |__| | ' /| |__  |  \| |
    //   \___ \ / /\ \ | . ` | |    |  __  |  < |  __| | . ` |
    //   ____) / ____ \| |\  | |____| |  | | . \| |____| |\  |
    //  |_____/_/    \_\_| \_|\_____|_|  |_|_|\_\______|_| \_|
        //POM_PHYINIT & POM_SANCHKEN
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_PHYINIT = 0x1;    
        unPOMval.stcField.u1POM_SANCHKEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        while(sanchkenCmplt == false){
            /*  check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            /*  Sanity Check    */
            if (unPOSval.stcField.u1SANCHKC == 1)
            {
                /*  Write/Read sanity check error on rank 0. One bit for each data
                    slice. 1 means read pattern and write pattern for corresponding
                    data slice are not match   */
                fl_sanchkerr = (unPTS0val.stcField.u4SANCHKERR >= 1);
            }
            if (unPOSval.stcField.u1PHYINITC == 0x1)
            {
                sanchkenCmplt=true;    
            }
            // END OF SANCHKEN
        }    
        if (fl_sanchkerr)
        {
            return CY_LPDDR4_ERROR;
        }                

    //   _____  ______ _____ ______ _   _ 
    //  |  __ \|  ____|_   _|  ____| \ | |
    //  | |  | | |__    | | | |__  |  \| |
    //  | |  | |  __|   | | |  __| | . ` |
    //  | |__| | |     _| |_| |____| |\  |
    //  |_____/|_|    |_____|______|_| \_|
        // POM_CHANEN & POM_DFIEN & POM_FS (functional) */
        unPOMval.u32Register = 0x0ul;
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        unPOMval.stcField.u1POM_DFIEN = 0x1;
        unPOMval.stcField.u1POM_FS = 0x1;
        unPOMval.stcField.u1POM_PHYFSEN=0x1;
        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
        return CY_LPDDR4_SUCCESS;
}



/*******************************************************************************
 *  Function Name:  Cy_Lpddr_LPDDR4Training
 ***************************************************************************//**
 *  \brief Function to trigger trainings request to the LPDDR4 controller.
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \param config Pointer to the LPDDR4 config structure
 * 
 * 
 *  \return \ref    cy_en_lpddr4_retval_t<br>
 *                  CY_LPDDR4_SUCCESS if everything was ok<br>
 *                  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the 
 *                  function<br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_LPDDR4Training(volatile stc_LPDDR4_t * base,const cy_stc_lpddr4_config_t * config)
{
    bool fl_vrefdqrderr = false;
    bool fl_vrefcaerr   = false;
    bool fl_gterr       = false;
    bool fl_wrlvlerr    = false;
    bool fl_rdlvldmerr  = false;
    bool fl_vrefdqwrerr = false;
    bool fl_dlyevalerr  = false;
    bool fl_sanchkerr   = false;
    
    /*  local variable for training status. 
        default no training error present */
    bool fl_error_training = false;

    /*  local to collect all request before programming them 
        all at once to the controller */
    un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;
    unPOMval.u32Register = 0x0ul;
    //  local to catch the PHY Operational Status 
    un_LPDDR4_LPDDR4_CORE_POS_t unPOSval;
    //  local to catch the PHY Training Status 0
    un_LPDDR4_LPDDR4_CORE_PTS0_t unPTS0val;

    /*  select the channels as configured by the user in the config 
        structure and is programmed during ControllerInit to driver 
        context.channelActive */
    unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();
    unPOMval.stcField.u1POM_PHYINIT = 0x1;      /*  set the PHY init Request in POM Phy Operational Mode Register  */
    unPOMval.stcField.u1POM_VREFDQRDEN = 0x1;   /*  set request for PHY VREF-DQ read training enable  */
    unPOMval.stcField.u1POM_VREFCAEN = 0x1;     /*  set request for DRAM VREF-CA training enable  */
    unPOMval.stcField.u1POM_GTEN = 0x1;         /*  set request for Gate training enable.  */
    unPOMval.stcField.u1POM_WRLVLEN = 0x1;      /*  set request for Write leveling enable  */
    unPOMval.stcField.u1POM_RDLVLEN = 0x1;      /*  set request for Read data eye DQ training enable  */
    unPOMval.stcField.u1POM_VREFDQWREN = 0x1;   /*  set request for DRAM VREF-DQ write training enable  */ 
    unPOMval.stcField.u1POM_DLYEVALEN = 0x0;    /*  set request for Delay evaluation enable  */
    unPOMval.stcField.u1POM_SANCHKEN = 0x1;     /*  set request for Write/read sanity check   */
    unPOMval.stcField.u1POM_FS = 0x1;           /*  Target frequency set-point for PHY operations. Must be different 
                                                    from operating set point. */

    /* write the local POM variable to the controller register */
    base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register; 

    /*  loop until phyinitc is set which is the signal from phy 
        that all training steps are finished */
    while (1) 
    {
        /*  check the completion flag in Phy Operational Status register */
        unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
        /*  read the PHY Training Status Register to check if some error 
            has occured during training */
        unPTS0val.u32Register = base->LPDDR4_CORE.unPTS0.u32Register;
        
        /*  PHY VREF-DQ training */
        if(unPOSval.stcField.u1VREFDQRDC == 1)
        {
            /*  Read VREF-DQ training error on rank 0. One bit for each data 
            slice. 1 means PHY cannot find suitable VREF setting for its 
            receiver  */
            fl_vrefdqrderr = ( unPTS0val.stcField.u4VREFDQRDERR >= 1);
        } 
        /*  Command Bus Training    */
        if(unPOSval.stcField.u1VREFCAC == 1)
        {
            /*  VREFCA training error on rank 0. vrefcaerr[1] is for LPDDR4 
                channel B and vrefcaerr[0] is for LPDDR4 channel A. 1 means 
                VREFCA training for corresponding LPDDR4 channel is not 
                completed successfully  */
            fl_vrefcaerr   = ( unPTS0val.stcField.u2VREFCAERR >= 1);
        }  
        /*  Gate Training   */      
        if(unPOSval.stcField.u1GTC == 1)
        {
            /*  Gate training error on rank 0. One bit for each data slice. 
            1 means gate training is not completed successfully   */
            fl_gterr = ( unPTS0val.stcField.u4GTERR >= 1);
        }
        /*  Write leveling  */
        if(unPOSval.stcField.u1WRLVLC == 1)
        {
            /*  Write leveling error on rank 0. One bit for each data slice. 
            1 means write leveling is not completed successfully  */
            fl_wrlvlerr = ( unPTS0val.stcField.u4WRLVLERR >= 1);
        }
        /*  read leveling   */
        if(unPOSval.stcField.u1RDLVLC == 1 )
        {
            /*  DM read data eye training error on rank 0. One bit for each DM 
                signal. 1 means data eye training for corresponding DM is not
                completed successfully */
            fl_rdlvldmerr = ( unPTS0val.stcField.u4RDLVLDMERR >= 1);
        } 

        if(unPOSval.stcField.u1VREFDQWRC == 1)
        {
            /* Write VREF-DQ training error on rank 0. One bit for each data 
               slice. 1 means VREF training for DRAM is not completed 
               successfully    */
            fl_vrefdqwrerr = ( unPTS0val.stcField.u4VREFDQWRERR >= 1);
        }

        /*  Delay evaluation    */
        if(unPOSval.stcField.u1DLYEVALC == 1)
        {
            /* check if there is a error status field available otherwise 
            delete this line */
            //fl_dlyevalerr   =  ;    
        } 

        /*  Sanity Check    */
        if(unPOSval.stcField.u1SANCHKC == 1 )
        {
            /*  Write/Read sanity check error on rank 0. One bit for each data 
                slice. 1 means read pattern and write pattern for corresponding 
                data slice are not match   */
            fl_sanchkerr = ( unPTS0val.stcField.u4SANCHKERR >= 1);
        } 
        
       
        fl_error_training= fl_sanchkerr || fl_dlyevalerr || 
                            fl_vrefdqwrerr || fl_rdlvldmerr || 
                            fl_wrlvlerr || fl_gterr || 
                            fl_vrefcaerr || fl_vrefdqrderr ;
        /*  check if initialization was successfull PHY initialization enable. 
        PHY initialization includes DLL init, DRAM init, and PHY training.  */
        if(unPOSval.stcField.u1PHYINITC == 0x1)
        {
            /* disable phy init process */
            unPOMval.u32Register = 0x0;
            /*  select the channels as configured by the user in the config 
                structure and is programmed during ControllerInit to driver 
                context.channelActive */
            //unPOMval.stcField.u2POM_CHANEN = g_stc_lpddr_context.channelActive;
            unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();
            /* DFI interface enable. After training, MC must write 1’b1 to this 
            field for normal DFI operation */
            unPOMval.stcField.u1POM_DFIEN = 0x1;
            /*  Target Frequency Set Point must be different from operation 
                set point */
            unPOMval.stcField.u1POM_FS = 0x1;
            /*  write the local POM to the controller register */
            base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
            /*  Training was successfull so return success :-) */
            if(fl_error_training)
            {
                return CY_LPDDR4_ERROR;
            }
            else
            {
                return CY_LPDDR4_SUCCESS;
            }
        }
        /*  check current Frequency set point 0 request  in Phy Operational 
            Status Register  */
        else if(unPOSval.stcField.u1FS0REQ == 1)
        {
            /*  change to FSP[0] f = 50 MHz */
            Cy_Lpddr_SetPhyFrequency(base,&config->pllconfig,CY_LPDDR4_FSP0);
            /*  give confirmation to PHY that frequency was changed an PHY 
                can proceed  */
            Cy_Lpddr_PhyProceed(base);
        }
        else if(unPOSval.stcField.u1FS1REQ == 1) 
        {
            /*  change to FSP[1] f = 800 MHz */
            Cy_Lpddr_SetPhyFrequency(base,&config->pllconfig,CY_LPDDR4_FSP1);
            /*  give confirmation to PHY that frequency was changed an PHY 
                can proceed  */
            Cy_Lpddr_PhyProceed(base);
        }
    }
}




/*******************************************************************************
 *  Function Name:  Cy_Lpddr_PhyProceed
 ***************************************************************************//**
 * \brief  Function to set the PHY Proceed bit in the POM Register to signal the 
 *         PHY that the frequency switch has happened and PHY can proceed
 *
 * This function is used during FSP ( Frequency Set Point) change. When the 
 * LPDDR4 controller is ready for FSP change it sets the request in the 
 * Phy Operational Status Register<br>
 *      POS[14] = fs0req<br>
 *      POS[15] = fs1req<br>
 * 
 * Software has to monitor these bits. If either fs0req or fs1req is set the 
 * software must switch the PLL output frequency 
 *( \ref Cy_Lpddr_SetPhyFrequency()) once this is done it uses this function 
 * Cy_Lpddr_PhyProceed()  to signal the PHY that the Frequency change was 
 * performed and PHY can proceed.
 * 
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return none
 * 
 *   @startuml"Controller Init Sequence"
 *   !include commons.iuml!FSP_CHANGE_PROCESS
 *   @enduml
 *
 ******************************************************************************/
static void Cy_Lpddr_PhyProceed(volatile stc_LPDDR4_t * base)
{
    /*  local for catching the PHY Operational Mode register    */
    un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;

    /*  get the register value POM Register */
    unPOMval.u32Register = base->LPDDR4_CORE.unPOM.u32Register;

    /*  set the PHY proceed bit and write it back to the register   */
    unPOMval.stcField.u1POM_PROC = 0x1;
    base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;

    /*  clear the PHY proceed bit and write it back to the register */
    unPOMval.stcField.u1POM_PROC = 0x0;
    base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;
}

/*******************************************************************************
 *  Function Name:  Cy_Lpddr_CheckInitComplete
 ***************************************************************************//**
 *  \brief  Function is checking the completeness of the PHY initialization 
 *          process by checking the POM and PTSO register
 *          
 * 
 * This function is checking the POS register ( Phy Operational Status ) for 
 * completeness of the PHY initialization process. The requests to init PHY is 
 * initiated using the POM register ( PHY Operational Mode).Once the request is 
 * send to the LPDDR4 controller the Phy init is done from the controller
 * without further firmware interaction. Only thing remains for the driver is 
 * checking the completeness which is signaled in the POS register 
 * ( PHY Operational Status ) of the controller. For PHY init completeness the 
 * flags PHYINITC,DLLRSTC,DRAMINITC and CLKLOCKC must be set. If these flags are 
 * all set and no error is signaled in the PTS0 ( Bitfield pts0_dllerr) 
 * register the PHY Init process is completed.
 *  
 * 
 * <table class="doxtable">
 * <tr><th>Register Field</th>
 *      <th>Position</th>
 *      <th>Reset</th>
 *      <th>Description</th></tr>
 *<tr><td>pos_physetc</td>
 *       <td>[0]</td>
 *      <td> 0</td>
 *      <td>PHY setting reload completed.</td></tr>
 * <tr><td>pos_phyfsc</td>
 *      <td>[1]</td>
 *      <td> 0</td>
 *      <td>PHY frequency change completed.</td></tr>
 * <tr><td>pos_phyinitc</td>
 *      <td>[2]</td>
 *      <td> 0</td>
 *      <td>PHY initialization completed</td></tr>
 * <tr><td>pos_dllrstc</td>
 *      <td>[3]</td>
 *      <td> 0</td>
 *      <td>DLL reset completed.</td></tr>
 * <tr><td>pos_draminitc </td>
 *      <td>[4]</td>
 *      <td> 0</td> <td>DRAM initialization completed.</td></tr>
 * <tr><td>pos_vrefdqrdc </td>
 *      <td>[5]</td>
 *      <td>0</td>
 *      <td>PHY VREF-DQ training completed.</td></tr>
 * <tr><td>pos_vrefcac</td><td>[6]</td>
 *      <td> 0</td>
 *      <td>DRAM VREF-CA training completed.</td></tr>
 * <tr><td>pos_gtc</td>
 *      <td>[7]  </td>
 *      <td> 0</td>
 *      <td>Gate training completed for rank 0</td></tr>
 * <tr><td>pos_wrlvlc</td>
 *      <td>[8]</td>
 *      <td> 0</td>
 *      <td>Write leveling completed for rank 0</td></tr>
 * <tr><td>pos_rdlvlc</td>
 *      <td>[9]</td>
 *      <td> 0</td>
 *      <td>Read data eye training completed for rank 0 </td></tr>
 * <tr><td>pos_vrefdqwrc </td>
 *      <td>[10] </td>
 *      <td> 0</td>
 *      <td>DRAM VREF-DQ training completed for rank 0  </td></tr>
 * <tr><td>pos_dlyevalc</td>
 *      <td>[11] </td>
 *      <td> 0</td> <td>Delay evaluation completed for rank 0</td></tr>
 * <tr><td>pos_sanchkc</td>
 *      <td>[12] </td>
 *      <td> 0</td>
 *      <td>Write/read sanity check completed for rank 0</td></tr>
 * <tr><td>pos_ofs</td>
 *      <td>[13] </td>
 *      <td> 0</td>
 *      <td>Operating frequency set point.</td></tr>
 * <tr><td>pos_fs0req</td>
 *      <td>[14] </td>
 *      <td> 0</td>
 *      <td>Frequency set point 0 request.</td></tr>
 * <tr><td>pos_fs1req</td>
 *      <td>[15] </td>
 *      <td> 0</td>
 *      <td>Frequency set point 1 request.</td></tr>
 * <tr><td>pos_clklockc  </td>
 *      <td>[16] </td>
 *      <td> 0</td>
 *      <td>MC-PHY clock phase locked.</td></tr>
 * <tr><td>pos_cmddlyc</td>
 *      <td>[17] </td>
 *      <td> 0</td> <td>PHY command bus delay loaded</td></tr>
 * <tr><td>pos_dqsdqc</td>
 *      <td>[18] </td>
 *      <td> 0</td> <td>tDQS2DQ training completed for rank 0</td></tr>
 * </table><br><br>
 * 
 * 
 * 
 * <table class="doxtable">
 * <tr>
 *   <th>Register Field</th><th>Position</th><th>Reset</th><th>Description</th>
 * </tr>
 * <tr>
 * <td>pts0_dllerr</td>  
 * <td>[5:0]   </td>  
 * <td>0</td> 
 * <td>
 *      DLL reset error<br>
 *      Dllerr[3:0] is for data module [3:0]<br>
 *      dllerr[5:4] is for control module[1:0]<br>
 *      1 means the DLL reset process of corresponding module is not 
 *      completed successfully<br>
 * </td>
 * </tr>
 * </table>
 *  <br>
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type 
 *         \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function <br>
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_CheckPhyInitComplete(volatile stc_LPDDR4_t * base)
{
    bool fl_phyiniterror = false;
    bool fl_initcompleted = false;
    bool fl_phyinitc = false;
    bool fl_draminitc = false;
    bool fl_clklockc = false;
    bool fl_dllrstc = false;
    bool fl_data_byte_dis = false;

    un_LPDDR4_LPDDR4_CORE_POS_t unPOSval;
    un_LPDDR4_LPDDR4_CORE_PTS0_t unPTS0val;
    //un_LPDDR4_LPDDR4_CORE_PTS1_t unPTS1val;
    

    // TODO: check if there is a better way other than while(1)
    // timeout is better but how long ?
    while (1)
    {
        unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
        unPTS0val.u32Register = base->LPDDR4_CORE.unPTS0.u32Register;
        
        fl_phyinitc = (unPOSval.stcField.u1PHYINITC == 1);
        fl_dllrstc = (unPOSval.stcField.u1DLLRSTC == 1 ) && (unPTS0val.stcField.u6DLLERR == 0);
        fl_draminitc = (unPOSval.stcField.u1DRAMINITC == 1);
        fl_clklockc = (unPOSval.stcField.u1CLKLOCKC == 1);

        /*  final initcomplete flag */
        fl_initcompleted = (fl_phyinitc && fl_dllrstc && fl_draminitc && fl_clklockc);
        
        
        /*TODO check if we need to take into account that only a single channel might be activated 
          so the maybe some bits of u6DLLERR and u4DTI_DATA_BYTE_DIS are don´t care */
        /*  check if there is Byte Lane deactivated */
        fl_data_byte_dis = (base->LPDDR4_CORE.unPHY.stcField.u4DTI_DATA_BYTE_DIS != 0);
        /*  set error flag if a byte lane is disabled or DLL shows error */
        fl_phyiniterror = ((unPTS0val.stcField.u6DLLERR != 0) || (fl_data_byte_dis));

        /*  exit with error  */
        if(fl_phyiniterror)
        {
            return CY_LPDDR4_ERROR;
        }
        if(fl_initcompleted)
        {
            return CY_LPDDR4_SUCCESS; 
        }
    }
}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WaitUserCMDReady
 ***************************************************************************//**
 * \brief Function which is waiting for the activated channels to become ready
 *        for receiving user commands. Which channels should be activated is set
 *        in .unDMCTL[12:11] in \ref cy_stc_lpddr4_config_t \ref cy_stc_lpddr4_config_t 
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
 void  Cy_Lpddr_WaitUserCMDReady(volatile stc_LPDDR4_t * base)
{
    /* get the information from driver context if controller is configures for CH0, CH1 or both */
        cy_en_lpddr4_chan_sel_t channelConfig = Cy_Lpddr_GetContextChannelConfig();
        
        if( channelConfig == CY_LPDDR4_CHAN_SEL_A)
        {
            /*wait for channel to be ready and no interrupt for channel is pending */
            while((base->LPDDR4_CORE.unOPSTT_CH0.stcField.u1USER_CMD_READY != 1)||
                    (base->LPDDR4_CORE.unINTSTT_CH0.stcField.u1INT_GC_FSM == 1));
        }
        else if(channelConfig == CY_LPDDR4_CHAN_SEL_B)
        {
            /*wait for channel to be ready and no interrupt for channel is pending */
            while((base->LPDDR4_CORE.unOPSTT_CH1.stcField.u1USER_CMD_READY != 1)||
                    (base->LPDDR4_CORE.unINTSTT_CH1.stcField.u1INT_GC_FSM == 1));
        }
        else if(channelConfig == CY_LPDDR4_CHAN_SEL_AB)
        {
            /*wait for channel to be ready and no interrupt for channel is pending */
            while((base->LPDDR4_CORE.unOPSTT_CH0.stcField.u1USER_CMD_READY == 0) || 
                        (base->LPDDR4_CORE.unINTSTT_CH0.stcField.u1INT_GC_FSM == 1) ||
                (base->LPDDR4_CORE.unOPSTT_CH1.stcField.u1USER_CMD_READY == 0) || 
                        (base->LPDDR4_CORE.unINTSTT_CH1.stcField.u1INT_GC_FSM == 1)); 
        }
 }


    /*******************************************************************************
     * Function Name: Cy_Lpddr_StartDQSOscillator()
     ***************************************************************************//**
     *
     * \brief
     * Function sends the request to start the DQS Oscillator to the LPDDR4
     * controller.The command is send to the channels which were activated with
     * Cy_LPDDR_ControllerInit(). See channelActive of driver context variable \ref
     * g_stc_lpddr_context.
     *
     * The DQS Oscillator is used to check the necessity of retraining.
     * The result of the DQS Oscillator can be read in MR19(MSB) and
     * MR18 (LSB). DQS Oscillator will stop either automatically when DQS Oscillator
     * intervall runtime value > 0 (programmed MR23) or when user sends Oscillator
     * stop request to the Memory Cy_Lpddr_StopDQSOscillator(). As soon as the
     * oscillator has stopped the MR18 and MR19 are ready for readout
     *
     *   Command is send over the UCI Register to the LPDDR4 Controller
     *       uci_cmd_op      [3:0]       User command Opcode<br>
     *       uci_cmd_chan    [5:4]       User Command Channel<br>
     *                       2b01: Ch A<br>
     *                       2b10: Ch B<br>
     *                       2b11: both channels<br>
     *       uci_mr_sel      [11:6]      Register Select Argument for MRS/MRW/MRR
     *                                   Command<br>
     *       uci_mr_op       [19:12]     Register Opcode ( MRW,MPC )<br>
     *
     *   @startuml"Start DQS Oscillator Sequence"
     *   !include commons.iuml!DQSSTART
     *   @enduml
     *
     *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
     *
     *  \note   As the DQS Oscillator start command can only be send when controller
     *          is not running this function has to stop the controller send DQS
     *          Start Comand and Restart the controller. During the time the
     *          controller is not running no memory access is possible.<br>
     ******************************************************************************/
    static void Cy_Lpddr_StartDQSOscillator(volatile stc_LPDDR4_t * base)
    {

        /* local variable for the actual controller state*/
        cy_en_lpddr4_cntrl_gsm_state_t lactualGSMState = g_stc_lpddr_context.controllerState;
        un_LPDDR4_LPDDR4_CORE_UCI_t usrCntrlCmdStop;
        un_LPDDR4_LPDDR4_CORE_UCI_t usrDQSStartCmd;
        un_LPDDR4_LPDDR4_CORE_UCI_t usrUserCmdRun;

        uint32_t pomCHANEN = (uint32_t)Cy_Lpddr_GetContextChannelConfig();

        // hardcoded DSQ Start Command to lower blackout time ( reduce API overhead)
        usrDQSStartCmd.stcField.u4CMD_OP = 0xC;
        usrDQSStartCmd.stcField.u2CMD_CHAN = pomCHANEN;
        usrDQSStartCmd.stcField.u6MR_SEL = 0x0;
        usrDQSStartCmd.stcField.u8MR_OP = 0x0;

        // hardcoded Controller STOP Command to lower blackout time ( reduce API overhead)
        usrCntrlCmdStop.stcField.u4CMD_OP = 0x0;
        usrCntrlCmdStop.stcField.u2CMD_CHAN = pomCHANEN;
        usrCntrlCmdStop.stcField.u6MR_SEL = 0x0;
        usrCntrlCmdStop.stcField.u8MR_OP = 0x0;

        // hardcoded DSQ Start Command to lower blackout time ( reduce API overhead)
        usrUserCmdRun.stcField.u4CMD_OP = 0x1;
        usrUserCmdRun.stcField.u2CMD_CHAN = pomCHANEN;
        usrUserCmdRun.stcField.u6MR_SEL = 0x0;
        usrUserCmdRun.stcField.u8MR_OP = 0x0;

        if (lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN)
        {
            /* if controller is running stop it first */
            // wait for command channel beeing ready
            while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
            // send USER Command Stop to the Controller and keep track of the GSM state
            base->LPDDR4_CORE.unUCI.u32Register = usrCntrlCmdStop.u32Register;

            g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_STOPPED;
            g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
            g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
            g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
        }
        /*Start the DQS Oscillator */
        // wait for command channel beeing ready
        while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
        // send DQS Osc start command
        base->LPDDR4_CORE.unUCI.u32Register = usrDQSStartCmd.u32Register;

        /*if controller was running at function entry restart it*/
        if (lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN)
        {
            // wait for command channel beeing ready
            while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
            // send USER Command RUN to the controller and keep track of the GSM state
            base->LPDDR4_CORE.unUCI.u32Register = usrUserCmdRun.u32Register;
            g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_RUNNING;
            g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
            g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
            g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_RUN;
        }
}



/*******************************************************************************
 * Function Name: Cy_Lpddr_ReadDQSOscValue
 ***************************************************************************//** 
 *  \brief  Function to read the DQS Oscillator results from MR19 and MR18
 *          
 *  Read the the result from the DQS Oscillator.Once the DQS Oscillator is 
 *  started the  MR18 and MR19 registers as cleared. When the DQS Oscillator is 
 *  either stopped automatically (MR23>0 )or by sending 
 *  CY_LPDDR4_USER_CMD_DQSOSC_STOP the MR19(MSB) and MR18 are updated with the 
 *  DQS Oscillator count.
 * 
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t 
 * 
 * \param dqsOSCCount pointer to a \ref cy_stc_lpddr4_dqsosc_read_data_t to 
 *        catch the DQS OScillator read data
 * 
 * If function returns with error the DQS Oscillator encountered a overflow 
 * So DQS OSC Counter reached 0xFFFFFFFF --> decrease MR23 OSC Intervall Runtime
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_SUCCESS if everythink was ok
 * CY_LPDDR4_ERROR if overflow of DQS Oscillator value occured 
 *  
 ******************************************************************************/
static cy_en_lpddr4_retval_t Cy_Lpddr_ReadDQSOscValue(volatile stc_LPDDR4_t * base,
                                                        volatile cy_stc_lpddr4_dqsosc_read_data_t* dqsOSCCount)
{   
    /*local struct to catch MR 18 register content which is the LSB of the OSC counter */
    cy_stc_lpddr4_mrw_mrr_data_t mr18Value = {0,0};
    /*local struct to catch MR 19 register content which is the MSB of the OSC counter */
    cy_stc_lpddr4_mrw_mrr_data_t mr19Value = {0,0};

    //locals for the DQS Osc Counts
    uint32_t value_CHA = 0;
    uint32_t value_CHB = 0;

    /*        CHANNEL A ONLY       */
    if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_A)
    {
        //wait for DQS Oscillator to stop and update the MR18 / MR19 registers
        while( value_CHA == 0)
        {
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR18,&mr18Value))
            {
                return CY_LPDDR4_ERROR;
            }
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR19,&mr19Value))
            {
                return CY_LPDDR4_ERROR;
            }
            value_CHA =(((uint32_t)mr19Value.mrw_CHA_Data) << 8) + mr18Value.mrw_CHA_Data;
            // for debugging
            dqsOSCCount->value_CHA = value_CHA;
        }
        /*check if DQS Osc interval was to long an OSC Count had overflow */
        if(value_CHA!=0xFFFFFFFF)
        {
            return CY_LPDDR4_SUCCESS;
        }
    }
    /*        CHANNEL B ONLY       */
    else if (Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_B)
    {
        //wait for DQS Oscillator to stop and update the MR18 / MR19 registers
        while( value_CHB == 0 )
        {
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR18,&mr18Value))
            {
                return CY_LPDDR4_ERROR;
            }
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR19,&mr19Value))
            {
                return CY_LPDDR4_ERROR;
            }
            value_CHB = (((uint32_t)mr19Value.mrw_CHB_Data) << 8) + mr18Value.mrw_CHB_Data;
            //for debugging
            dqsOSCCount->value_CHB = value_CHB;
        }
        /*check if DQS Osc interval was to long an OSC Count had overflow */
        if(value_CHB != 0xFFFFFFFF )
        {
            return CY_LPDDR4_SUCCESS;
        }
    }
    /*           BOTH CHANNELS               */
    else if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_AB)
    {
        //wait for DQS Oscillator to stop and update the MR18 / MR19 registers
        while(( value_CHA == 0) && ( value_CHB == 0 ))
        //while(( value_CHA == 0) || ( value_CHB == 0 ))
        {
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR18,&mr18Value))
            {
                return CY_LPDDR4_ERROR;
            }
            if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR19,&mr19Value))
            {
                return CY_LPDDR4_ERROR;
            }
            /*          CHANNEL A DQS Counter       */
            /*if MR18 already has valid data we can assume that MR19 was also 
            updated and the values are valid as we read them after MR18 */
            if( mr18Value.mrw_CHA_Data != 0 ) 
            { 
                value_CHA =(((uint32_t)mr19Value.mrw_CHA_Data) << 8) + mr18Value.mrw_CHA_Data;
                dqsOSCCount->value_CHA = value_CHA;
            }
            /*if MR19 has value !=0 it can be that MR18 is really = 0 or the registers were updated in 
              between reading MR18 and MR19. so we need to read MR18 again  */
            else if ((mr18Value.mrw_CHA_Data == 0) &&  (mr19Value.mrw_CHA_Data != 0))
            {
                if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR18,&mr18Value))
                {
                    return CY_LPDDR4_ERROR;
                }                
                value_CHA =(((uint32_t)mr19Value.mrw_CHA_Data) << 8) + mr18Value.mrw_CHA_Data;
                dqsOSCCount->value_CHA = value_CHA;
            }
            /*       CHANNEL B DQS Counter           */
            /*if MR18 already has valid data we can assume that MR19 was also 
            updated and the values are valid as we read them after MR18*/
            if( mr18Value.mrw_CHB_Data != 0 )
            {
                value_CHB =(((uint32_t)mr19Value.mrw_CHB_Data) << 8) + mr18Value.mrw_CHB_Data;
                dqsOSCCount->value_CHB = value_CHB;
            }
            /*if MR19 has value !=0 it can be that MR18 is really = 0 or the registers were updated in 
              between reading MR18 and MR19. so we need to read MR18 again  */
            else if ((mr18Value.mrw_CHB_Data == 0) &&  (mr19Value.mrw_CHB_Data != 0))
            {
                if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadModeRegister(base,CY_LPDDR4_MODEREGISTER_MR18,&mr18Value))
                {
                    return CY_LPDDR4_ERROR;
                }  
                value_CHB =(((uint32_t)mr19Value.mrw_CHB_Data) << 8) + mr18Value.mrw_CHB_Data;
                dqsOSCCount->value_CHB = value_CHB;
            }
        }
        /*check if DQS Osc interval was to long an OSC Count had overflow */
        if((value_CHA != 0xFFFFFFFF ) && (value_CHB != 0xFFFFFFFF ))
        {
            return CY_LPDDR4_SUCCESS;
        }
    }
    return CY_LPDDR4_ERROR;

}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_ReadModeRegister
 ***************************************************************************//**
 *  \brief  This function is sending the request for a Mode Register Read the 
 *          specified channel of the LPDDR4 controller.
 *          
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 *
 *  \param  mr mode register to read . Trying to read mode registers which are 
 *          write only will result in invalid ( undefined ) data. User must take
 *          care that only mode registers are specified which are readable 
 * 
 *  \param  pointer to an struct \ref cy_stc_lpddr4_mrw_mrr_data_t for the 
 *          read data of ch0 and ch1 where the mode register value will be 
 *          written to 
 * 
 *  \return \ref cy_en_lpddr4_channel_status_t<br>
 *      CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *      CY_LPDDR4_SUCCESS   - if everything was ok<br>
 * 
 ******************************************************************************/
 cy_en_lpddr4_retval_t Cy_Lpddr_ReadModeRegister(volatile stc_LPDDR4_t * base, 
                                                cy_en_lpddr4_mr_t mr, 
                                                cy_stc_lpddr4_mrw_mrr_data_t * registerValue)
{
    cy_en_lpddr4_retval_t ret;
    /*  directly return with error when no valid pointer for 
        LPDDR4 base is passed */
    if( NULL == base )
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*  directly return when no valid pointer for MR Value is given */
    if( NULL == registerValue )
    {
        return CY_LPDDR4_BAD_PARAM;
    }

    ret = Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_MRR,mr,0);
    if(ret == CY_LPDDR4_SUCCESS)
    {
        if(Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_A) 
        {

            while(base->LPDDR4_CORE.unMRR_CH0.stcField.u1DONE == 0);
            registerValue->mrw_CHA_Data = (uint8_t) base->LPDDR4_CORE.unMRR_CH0.stcField.u8READOUT;
        }
        else if (Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_B)
        {

            while(base->LPDDR4_CORE.unMRR_CH1.stcField.u1DONE == 0);
            registerValue->mrw_CHB_Data = (uint8_t) base->LPDDR4_CORE.unMRR_CH1.stcField.u8READOUT;
        }
        else if (Cy_Lpddr_GetContextChannelConfig() == CY_LPDDR4_CHAN_SEL_AB)
        {
                while((base->LPDDR4_CORE.unMRR_CH0.stcField.u1DONE == 0) || (base->LPDDR4_CORE.unMRR_CH1.stcField.u1DONE == 0));
                registerValue->mrw_CHA_Data = (uint8_t) base->LPDDR4_CORE.unMRR_CH0.stcField.u8READOUT;
                registerValue->mrw_CHB_Data = (uint8_t) base->LPDDR4_CORE.unMRR_CH1.stcField.u8READOUT;
        }
    }
    
    return ret;
}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr_RequestDQS2DQRetrain
 ***************************************************************************//**
 *  \brief  This function is sending the request for DQS2DQ Retraining to the 
 *          LPDDR4 Controller
 *  
 *  After initial DRAM VREF-DQ training the tDQS2DQ delay in device 
 *  may drift due to voltage and temperature variation. Memory controller needs 
 *  to initiate DQS interval oscillator and then get the result to decide if 
 *  DQS2DQ retraining is needed. <br>If retraining is needed this function 
 *  triggers the request to perform the controller internal retraining 
 *  algorithm. The following steps will be performed to activate the retraining:
 *  DQS2DQ re-training 
 *  <ul>
 *   <li> Set appropriate DQSDQCR register fields: dir, dlymax, mpcrpt. 
 *   The value of dlymax must be greater than or equal to t_dqrpt. 
 *   (this is already done during Controller init --> 
 *   \ref Cy_Lpddr_SetPhyDdrMode()</li>
 *   <li> Set following POM fields: dqsdqen=1, phyinit=1, and dfien=0 </li>
 *   <li> Waits for POS[phyinitc] and POS[r*_dqsdqc] for training complete </li>
 *   <li> Set POM: dqsdqen=0, phyinit=0, and dfien=1 for normal operation </li>
 *  </ul>
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 ******************************************************************************/
    cy_en_lpddr4_retval_t Cy_Lpddr_RequestDQS2DQRetrain(volatile stc_LPDDR4_t * base)
    {
        // if base is not a valid pointer return CY_LPDDR4_BAD_PARAM
        if( NULL == base )
        {
            return CY_LPDDR4_BAD_PARAM;
        }
        /* local variable to collect the request bits before programming it to
        the controller register   */
        un_LPDDR4_LPDDR4_CORE_POM_t unPOMval;
        // local variable to catch the controllers Phy Operational Status
        un_LPDDR4_LPDDR4_CORE_POS_t unPOSval;
        // local variable for the Phy Training Status 2 - DQS-DQ training error on rank 0
        un_LPDDR4_LPDDR4_CORE_PTS2_t unPTS2val;
        // local variable for the Phy Training Status 3 -
        un_LPDDR4_LPDDR4_CORE_PTS3_t unPTS3val;
        // local variable for the
        un_LPDDR4_LPDDR4_CORE_DQSDQCR_t unDQSDQCR;
        // local variable for the controller state
        cy_en_lpddr4_cntrl_gsm_state_t lactualGSMState;

        un_LPDDR4_LPDDR4_CORE_UCI_t usrDQSStartCmd;
        un_LPDDR4_LPDDR4_CORE_UCI_t usrUserCmdRun;
        un_LPDDR4_LPDDR4_CORE_UCI_t usrUserCmdStop;

        uint32_t pomCHANEN = (uint32_t)Cy_Lpddr_GetContextChannelConfig();

        //hardcoded DSQ Start Command to lower blackout time ( reduce API overhead)
        usrDQSStartCmd.stcField.u4CMD_OP = 0xC;
        usrDQSStartCmd.stcField.u2CMD_CHAN = pomCHANEN;
        usrDQSStartCmd.stcField.u6MR_SEL = 0x0;
        usrDQSStartCmd.stcField.u8MR_OP = 0x0;

        // hardcoded USER_CMD_RUN to lower blackout time ( reduce API overhead)
        usrUserCmdRun.stcField.u4CMD_OP = 0x1;
        usrUserCmdRun.stcField.u2CMD_CHAN = pomCHANEN;
        usrUserCmdRun.stcField.u6MR_SEL = 0x0;
        usrUserCmdRun.stcField.u8MR_OP = 0x0;

        // hardcoded USER_CMD_STOP Command to lower blackout time ( reduce API overhead)
        usrUserCmdStop.stcField.u4CMD_OP = 0x0;
        usrUserCmdStop.stcField.u2CMD_CHAN = pomCHANEN;
        usrUserCmdStop.stcField.u6MR_SEL = 0x0;
        usrUserCmdStop.stcField.u8MR_OP = 0x0;

        // local for Automatic ZQ Calibration feature enable state
        bool autoZQState;
        // local variable for catching training error
        bool fl_error_training = false;

        /*set the DQS2DQ Active flag in the driver context */
        Cy_Lpddr_SetControllerContextRetrainActive();
        /*
          New RQMTS with DOTP34 release: AUTO ZQ EN in DMCFG must be switched off during DQS2DQ Retraining
          "Before DQSDQ retraining, users must disable Auto-ZQ calibration by setting dmcfg_zq_auto_en to 0. "
          */
        autoZQState = Cy_Lpddr_IsAutoZQEnabled(base);
        /*if automatic ZQ Calibration Feature is active
          deactivate it before DQS2DQ training */
        if(autoZQState == true )
        {
            //Cy_Lpddr_DisableAutoZQ(base);
            base->LPDDR4_CORE.unDMCFG.stcField.u1ZQ_AUTO_EN = 0x0;
        }

        /* Get actual ontroller state as controller must be stopped before training */
        lactualGSMState = Cy_Lpddr_GetCurrenGSMState();

        /*read the current value from the controller*/
        unDQSDQCR.u32Register = base->LPDDR4_CORE.unDQSDQCR.u32Register;

        /*set the new direction to train as indicated set in the driver context*/
        unDQSDQCR.stcField.u1DIR = g_stc_lpddr_context.retrainDirection;

        /*write the new value back to the controller register */
        base->LPDDR4_CORE.unDQSDQCR.u32Register = unDQSDQCR.u32Register;

        /* programm the trainings request into a local variable */
        unPOMval.u32Register = 0x0ul;
        // unPOMval.stcField.u2POM_CHANEN = Cy_Lpddr_GetContextChannelConfig();
        unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
        /*  set the PHY init Request in POM Phy Operational Mode Register  */
        unPOMval.stcField.u1POM_PHYINIT = 1;
        /*  DFI interface enable. 0 = disable during training,
            will be re-enabled after retraining has finished */
        unPOMval.stcField.u1POM_DFIEN = 0;
        /*  bit to request the retraining */
        unPOMval.stcField.u1POM_DQSDQEN = 1;

        /* if controller is running stop it first */
        if (lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN)
        {
            /* wait for controller ready */
            while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
            base->LPDDR4_CORE.unUCI.u32Register = usrUserCmdStop.u32Register;
            g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_STOPPED;
            g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
            g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
            g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
        }

        /* write the local POM variable to the controller register */
        base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;

        /* loop until phyinitc is set */
        while (1)
        {
            /* check the completion flag in Phy Operational Status register */
            unPOSval.u32Register = base->LPDDR4_CORE.unPOS.u32Register;
            // unPTS2val.u32Register = base->LPDDR4_CORE.unPTS2.u32Register;

            /* check if retraining has finished */
            if(unPOSval.stcField.u1PHYINITC == 0x1 && unPOSval.stcField.u1DQSDQC == 0x1)
            {
                /*  disable phy init process */
                unPOMval.u32Register = 0x0;
                /*  channel which will recieve the command */
                unPOMval.stcField.u2POM_CHANEN = pomCHANEN;
                /*  disable init command */
                unPOMval.stcField.u1POM_PHYINIT = 0;
                /*  DFI interface enable. Reenable PHY after retraining finished */
                unPOMval.stcField.u1POM_DFIEN = 0x1;
                /*  disable the retraining */
                unPOMval.stcField.u1POM_DQSDQEN = 0;
                /*  write the local POM to the controller register */
                base->LPDDR4_CORE.unPOM.u32Register = unPOMval.u32Register;

                /* wait for controller ready */
                while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
                /* send MPC command Start DQS Oscillator */
                base->LPDDR4_CORE.unUCI.u32Register = usrDQSStartCmd.u32Register;

                /* wait for controller ready */
                while (((base->LPDDR4_CORE.unOPSTT_CH0.u32Register & 0x2) != 0x2) | ((base->LPDDR4_CORE.unOPSTT_CH1.u32Register & 0x2) != 0x2));
                // send User Command Run to controller
                base->LPDDR4_CORE.unUCI.u32Register = usrUserCmdRun.u32Register;
                g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_RUNNING;
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_RUN;

                /* if Automatic ZQ Calibration Feature was active
                   before DQS then re-enable the feature again. */
                if(autoZQState == true)
                {
                    base->LPDDR4_CORE.unDMCFG.stcField.u1ZQ_AUTO_EN = 0x1;
                }
                /* read PHY Training Status Register 2 for error status */
                unPTS2val.u32Register = base->LPDDR4_CORE.unPTS2.u32Register;
                /* read PHY Training Status Register 2 for error status */
                unPTS3val.u32Register = base->LPDDR4_CORE.unPTS3.u32Register;
                /*
                depending on the selected channel for training check the PTS2 / PTS3 for
                error occurence during training PTS2 pts2_dqsdqerr [31:0] DQS-DQ
                training error on rank 0. One bit for each DQ signal. 1 means data
                eye training for corresponding DQ signal is not completed successfully.
                PTS3 pts3_dqsdmerr[3:0] DQS-DM training error on rank 0. One bit for each
                DM signal. 1 means data eye training for corresponding DM signal is not
                completed successfully. */
                if (pomCHANEN == CY_LPDDR4_CHAN_SEL_A )
                {
                    /*  mask the bits for the channel which is
                        not trained and check if errors occured */
                    fl_error_training = (((unPTS2val.stcField.u32DQSDQERR & 0xFFFF) > 0) || ((unPTS3val.stcField.u4DQSDMERR & 0x3) > 0));
                    if(fl_error_training == true)
                    {
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_ERROR;
                    }else
                    {
                        /* DQS2DQ retrain was successful so clear the retrainRequestCHA */
                        g_stc_lpddr_context.retrainRequestCHA = false;
                        if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadDQSOscValue(base,&(g_stc_lpddr_context.dqsOSCCountLastTraining)))
                        {
                            return CY_LPDDR4_ERROR;
                        }
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_SUCCESS;
                    }
                }else if(pomCHANEN == CY_LPDDR4_CHAN_SEL_B )
                {
                    /*  mask the bits for the channel which is not trained and check
                        if errors occured */
                    fl_error_training = (((unPTS2val.stcField.u32DQSDQERR & 0xFFFF0000) > 0) || ((unPTS3val.stcField.u4DQSDMERR & 0xC) > 0));
                    if(fl_error_training == true)
                    {
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_ERROR;
                    }else
                    {
                        /* DQS2DQ retrain was successful so clear the retrainRequestCHB */
                        g_stc_lpddr_context.retrainRequestCHB = false;
                        if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadDQSOscValue(base,&(g_stc_lpddr_context.dqsOSCCountLastTraining)))
                        {
                            return CY_LPDDR4_ERROR;
                        }
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_SUCCESS;
                    }
                }else if(pomCHANEN == CY_LPDDR4_CHAN_SEL_AB )
                {
                    /* mask the bits for the channel which is not trained and check if errors occured */
                    fl_error_training =((unPTS2val.stcField.u32DQSDQERR > 0) || (unPTS3val.stcField.u4DQSDMERR) > 0);
                    if(fl_error_training == true)
                    {
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_ERROR;
                    }else
                    {
                        /* DQS2DQ retrain was successful so clear the retrainRequestCHA/CHB */
                        g_stc_lpddr_context.retrainRequestCHA = false;
                        g_stc_lpddr_context.retrainRequestCHB = false;
                        if(CY_LPDDR4_SUCCESS != Cy_Lpddr_ReadDQSOscValue(base,&(g_stc_lpddr_context.dqsOSCCountLastTraining)))
                        {
                            return CY_LPDDR4_ERROR;
                        }
                        /* reset the DQS2DQ Active flag in the driver context */
                        Cy_Lpddr_SetControllerContextRetrainInactive();
                        return CY_LPDDR4_SUCCESS;
                    }
                }
            }
        }
    }




/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_PSVPFakeTraining
 ***************************************************************************//**
 *  \brief  This function only used on PSVP as on the PSVP no real 
 *          training can be performed. As during the real retraining 
 *          the controller is stoppped. the function wait approx 10 us
 *          to simulate time needed for retraining. Driver Context retrain
 *          request is reset and controller is restarted
 * 
 *  \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 *  \return \ref cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 ******************************************************************************/
 cy_en_lpddr4_retval_t Cy_LPDDR4_PSVPFakeTraining(volatile stc_LPDDR4_t * base)
{
    //static uint8_t trainingCount=0;
    /* local variable for the controller GSM state */
    cy_en_lpddr4_cntrl_gsm_state_t lactualGSMState;

    // local for Automatic ZQ Calibration feature enable state
    bool autoZQState;

    // if base is not a valid pointer return CY_LPDDR4_BAD_PARAM
    if( NULL == base )
    {
        return CY_LPDDR4_BAD_PARAM;
    }

    /*New RQMTS with DOTP34 release: AUTO ZQ EN in DMCFG must be switched off
    during DQS2DQ Retraining "Before DQSDQ retraining, users must disable 
    Auto-ZQ calibration by setting dmcfg_zq_auto_en to 0. " */
    autoZQState = Cy_Lpddr_IsAutoZQEnabled(base);

    /* STOP the controller before DQS2DQ Retrain CDT 366974 Post #8 */
    lactualGSMState = Cy_Lpddr_GetCurrenGSMState();
    if(lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN)
    { 
        /* if controller is running stop it first */
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_STOP,CY_LPDDR4_NONE_MR,0);
    } 


    /*if automatic ZQ Calibration Feature is active 
      deactivate it before DQS2DQ training */
    if(autoZQState==true)
    {
        Cy_Lpddr_DisableAutoZQ(base);
    }

    /*set the retrain status of the driver context*/
    Cy_Lpddr_SetControllerContextRetrainActive();
    /* wait for about 6us*10 as were are on PSVP(as we are running 
    on 80MHz only) similar as the DQS2DQ retraining */
    Cy_SysLib_DelayUs(60);

    /*delete possible retrain request*/
    Cy_Lpddr_ClrControllerContextRetrainRequestCHA();
    Cy_Lpddr_ClrControllerContextRetrainRequestCHB();
    /* update dqsOscLastTraining of the driver context*/
    Cy_Lpddr_UpdateControllerContextLastTraining();
    //trainingCount++;

    /* if Automatic ZQ Calibration Feature was active 
       before DQS then re-enable the feature again. */
    if(autoZQState == true)
    {
        Cy_Lpddr_EnableAutoZQ(base);
    }
    /* Restart the controller after DQS2DQ has finished CDT 366974 Post #8 */
    if(lactualGSMState == CY_LPDDR4_CNTRL_GSM_STATE_RUN)
    { 
        /* if controller is running stop it first */
        Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN,CY_LPDDR4_NONE_MR,0);
    }
    Cy_Lpddr_SetControllerContextRetrainInactive();
    return CY_LPDDR4_SUCCESS;
}










/*******************************************************************************
 *  Function Name:  Cy_Lpddr_WriteModeRegister
 ***************************************************************************//**
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
 * \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 *
 * \param  mr mode register to write to  
 * 
 * \param  registerValue pointer to the value which shall be programmed to the 
 *         mode register
 * 
 * \return \ref cy_en_lpddr4_channel_status_t<br>
 *  CY_LPDDR4_BAD_PARAM - if a bad parameter was passed to the function<br>
 *  CY_LPDDR4_SUCCESS   - if everything was ok<br>
 ******************************************************************************/
 cy_en_lpddr4_retval_t Cy_Lpddr_WriteModeRegister(volatile stc_LPDDR4_t * base, 
                                                cy_en_lpddr4_mr_t mr, uint8_t * registerValue)
{
    cy_en_lpddr4_retval_t ret;
    if( NULL == base )
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    if( NULL == registerValue )
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    ret = Cy_Lpddr_SendCmd(base,CY_LPDDR4_UCI_CMD_OP_USER_CMD_MRW,mr,(uint32_t)*registerValue);
    return ret;
}


/*******************************************************************************
 * Function Name: Cy_Lpddr_SendCmd
 ***************************************************************************//**
 * 
 * \brief Sends a Command to the LPDDR Controller
 *
 *   This function is used to send a command to the LPDDR4 controller. The command
 *   is programmed to the controllers UCI (User Command Register) Register.Access
 *   user command register is almost the same as writeable register. Only one 
 *   difference is, users must check if user_cmd_ready register asserts 
 *   \ref Cy_Lpddr_WaitUserCMDReady() before starting the transaction. 
 *
 * \note The commands: USER_CMD_STOP, USER_CMD_PDE (Enter power-down run mode) 
 *       and USER_CMD_MRR are the only commands allowed in RUN mode.
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
 * \param mr if CMD is a MRR/MRW this parameter specifies the MR to read or 
 *          write \ref cy_en_lpddr4_mr_t .For none MRW / MRR command set this 
 *          parameter to CY_LPDDR4_NONE_MR
 *
 * \param mrval is the value which shall be written to mr in case of MRW.
 *        For none MRW command set to "0"
 *
 * 
 * \note Once the controller was started with CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN
 *       The only valid user commands which can be send are:
 *          CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN<br>
 *          CY_LPDDR4_UCI_CMD_OP_USER_CMD_PDE<br>
 *          CY_LPDDR4_UCI_CMD_OP_USER_CMD_MRR<br>
 *      Where CY_LPDDR4_UCI_CMD_OP_USER_CMD_MRR must be splitted in 2 separte
 *      Commands one for each channel.
 *          
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
                                        uint32_t mrval)
{
    cy_en_lpddr4_retval_t ret=CY_LPDDR4_BAD_PARAM;
    un_LPDDR4_LPDDR4_CORE_UCI_t usrcmd;
    if(NULL!=base)
    {  
            /* if it´s not a MRR command the command can/must be send to 
                both channels at the same time*/
            usrcmd.stcField.u4CMD_OP = cmd;
            usrcmd.stcField.u2CMD_CHAN = Cy_Lpddr_GetContextChannelConfig();
            usrcmd.stcField.u6MR_SEL = mr;
            usrcmd.stcField.u8MR_OP = mrval;
            Cy_Lpddr_WaitUserCMDReady(base);
            /*disable the interrupts to avoid getting out of sync with the controller status */
            __disable_irq();
            // update the lastUCICommand in the driver context
            g_stc_lpddr_context.lastUCICommand.u32Register = usrcmd.u32Register;
            base->LPDDR4_CORE.unUCI.u32Register = usrcmd.u32Register;
            /* perform the book keeping if the GSM state changes with the command */
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_STOP )
            {
                Cy_Lpddr_SetControllerStopped();
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
            }else
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_RUN  )
            {
                Cy_Lpddr_SetControllerRunning();
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_RUN;
            }else
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_PDE  )
            {
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_POWERDOWN;
            }else
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_PDX )
            {
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerState = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = CY_LPDDR4_CNTRL_GSM_STATE_POWERDOWN;
            }else
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_SRE )
            {
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_SELFREFRESH ;
            }else
            if( cmd == CY_LPDDR4_UCI_CMD_OP_USER_CMD_SRX )
            {
                g_stc_lpddr_context.controllerStatePrevRecov = g_stc_lpddr_context.controllerStatePrev;
                g_stc_lpddr_context.controllerStatePrev = g_stc_lpddr_context.controllerState;
                g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
            }

            /*re-enable the interrupt as the controllerState was updated*/
            __enable_irq();
            ret = CY_LPDDR4_SUCCESS;
    }else
    {
        ret = CY_LPDDR4_BAD_PARAM;
    }
    return ret;
}



/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_InitECCMemoryWithVal
 ***************************************************************************//**
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
void Cy_LPDDR4_InitECCMemoryWithVal(volatile stc_LPDDR4_t * base,uint32_t initValue)
{
    volatile uint32_t* ptrWriteAddress = (volatile uint32_t*) (CY_LPDDR4_BASE + base->LPDDR4_CORE.unINECC1.stcField.u30PROT_MEM_BASE);
    uint32_t sizeInUint32 = base->LPDDR4_CORE.unINECC2.stcField.u30PROT_MEM_SIZE/4;
    uint32_t cnt;
    un_LPDDR4_LPDDR4_CORE_INECCSTT4_t unINECCSTT4;
    unINECCSTT4.u32Register = 0;
    for (cnt=0;cnt<sizeInUint32;cnt++)
    {
        *ptrWriteAddress=initValue;
        ptrWriteAddress++;
    }
    /*Reset all error which were counted before ECC Init*/
    base->LPDDR4_CORE.unINECCSTT4.u32Register=unINECCSTT4.u32Register;
}

/*******************************************************************************
 *  Function Name:  Cy_LPDDR4_InitECCMemory
 ***************************************************************************//**
 *  \brief  This function is used to Init the Protected Area of ECC
 *          when ECC is enabled. It initialize the memory with 0xff
 *
 *
 *  \param  base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 *
 ******************************************************************************/
void Cy_LPDDR4_InitECCMemory(volatile stc_LPDDR4_t * base)
{
    volatile uint32_t* ptrWriteAddress = (volatile uint32_t*) (CY_LPDDR4_BASE + base->LPDDR4_CORE.unINECC1.stcField.u30PROT_MEM_BASE);
    /*local copy of the ECC Status Register 4 to reset all possible error before the ECC was init*/
    un_LPDDR4_LPDDR4_CORE_INECCSTT4_t unINECCSTT4;
    unINECCSTT4.u32Register = 0;

    //fill protected area with 0xFF
    memset((void*)ptrWriteAddress,0xff,(size_t)base->LPDDR4_CORE.unINECC2.stcField.u30PROT_MEM_SIZE);
    
    /*Reset all error which were counted before ECC Init*/
    base->LPDDR4_CORE.unINECCSTT4.u32Register=unINECCSTT4.u32Register;
}



/*******************************************************************************
*                        LPDDR4 Controller Stuff ends here
*******************************************************************************/

/*******************************************************************************
*                     Start of LPDDR4 Driver context stuff 
*******************************************************************************/
/*******************************************************************************
 * Function Name: Cy_Lpddr_DriverContextInit
 ***************************************************************************//**
 * \brief Function to Initialize the Driver Context Variable
 *
 * \param  config from type \ref cy_stc_lpddr4_config_t is a pointer to the 
 *         LPDDR4 config structure.
 * 
 * Function is called during Cy_LPDDR4_ControllerInit to initialize the
 * Driver Context Variable. The Driver Context Variable stores the internal
 * driver status. For additional information about driver context content 
 * \ref cy_stc_lpddr4_context_t.
 * 
 * \return none
 ******************************************************************************/
static void Cy_Lpddr_DriverContextInit(const cy_stc_lpddr4_config_t * config)
{
    /*!< initialization status of the memory controller*/
    g_stc_lpddr_context.controllerInitState = CY_LPDDR4_CNTRL_NOT_INITIALIZED;
    /*!<  */
    g_stc_lpddr_context.channelActive = (cy_en_lpddr4_chan_sel_t) config->unDMCTL.stcField.u2DRAM_CHAN_EN;
    /*!< set the DQS delta which will trigger a retraining for CHA */
    g_stc_lpddr_context.retrainThresholdCHA = config->deltaDQSCountRetrain[0];
    /*!< set the DQS delta which will trigger a retraining for CHB */
    g_stc_lpddr_context.retrainThresholdCHB = config->deltaDQSCountRetrain[1];
    /*!< flag signaling if retraining is needed for CHA */
    g_stc_lpddr_context.retrainRequestCHA = false;
    /*!< flag signaling if retraining is needed for CHB */
    g_stc_lpddr_context.retrainRequestCHB = false;
    /*!< DQS OSC Count after last successful Training CHA */
    g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA = 0;
    /*!< DQS OSC Count after last successful Training CHB */
    g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB = 0;
    /*!< DQS OSC Count which triggered actual retrainRequestCHA */
    g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHA = 0;
    /*!< DQS OSC Count which triggered actual retrainRequestCHB */
    g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHB = 0;
    /*!< the direction for the next training */
    g_stc_lpddr_context.retrainDirection = CY_LPDDR4_DQSDQCR_DIR_UP;
    /*!< if currently a retraining is active */
    g_stc_lpddr_context.dqs2dqRetrainActive = false;
    /*!< if controller is currently running */
    g_stc_lpddr_context.controllerRunning  = CY_LPDDR4_CNTRL_STOPPED;
    /*!< if controller state for recovering state in case of GSM error */
    g_stc_lpddr_context.controllerStatePrevRecov = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
    /*!< Controller statemaschine state */
    g_stc_lpddr_context.controllerState = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
    /*!< previous Controller statemaschine state */
    g_stc_lpddr_context.controllerStatePrev = CY_LPDDR4_CNTRL_GSM_STATE_STOP;
    /*!< temperature state of the external memory device */
    g_stc_lpddr_context.operationTemperatureState = CY_LPDDR4_OPERATIONAL_TEMPERATURE_OK;
    /*!< shows the last command which was send over UCI to MC */
    g_stc_lpddr_context.lastUCICommand.u32Register=0;
    /*!< currently active refresh rate strategy */
    g_stc_lpddr_context.refreshRateStrategy = config->refreshRateStrategy;
    /*!< if a refresh rate strategy is pending force a update of refresh 
    rate even if TUF in MR4 is not set */
    g_stc_lpddr_context.refreshStrategyChanged = CY_LPDDR4_STRATEGY_UNCHANGED;
    /*!< user specified fixed refresh rate */
    g_stc_lpddr_context.fixedUserRefreshRate = config->fixedRefreshRate;
    #if(DERATING_STRATEGY == DERATING_DYNAMIC)
        g_stc_lpddr_context.timingDeratingStrategy=CY_LPDDR4_DERATING_DYNAMIC;
    #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
        g_stc_lpddr_context.timingDeratingStrategy=CY_LPDDR4_DERATING_ALWAYS_ON;
    #endif
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetContextInitState
 ***************************************************************************//**
 * \brief  
 * Function to set the driver context controllerInitState = CY_LPDDR4_CNTRL_INITIALIZED
 * 
 * \return none
 *  
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetContextInitState(void)
{
    g_stc_lpddr_context.controllerInitState = CY_LPDDR4_CNTRL_INITIALIZED;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrContextInitState
 ***************************************************************************//**
 * \brief  
 * Function to set the driver context controllerInitState=CY_LPDDR4_CNTRL_NOT_INITIALIZED
 * 
 * 
 * \return none 
 *  
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_ClrContextInitState(void)
{
    g_stc_lpddr_context.controllerInitState = CY_LPDDR4_CNTRL_NOT_INITIALIZED;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerStopped
 ***************************************************************************//**
 * \brief  
 * Function to set the driver context status status CY_LPDDR4_CNTRL_STOPPED
 * 
 * Function must be called whenever the controller is stopped. 
 * 
 *******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerStopped(void)
{
    g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_STOPPED;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerRunning
 ***************************************************************************//**
 * \brief  
 * Function to set the driver context status status CY_LPDDR4_CNTRL_RUNNING
 * 
 * Function will be called whenever the driver starts the Controller. 
 * 
 *  
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerRunning(void)
{
    g_stc_lpddr_context.controllerRunning = CY_LPDDR4_CNTRL_RUNNING;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerContextRetrainActive
 ***************************************************************************//**
  * \brief Function sets the dqs2dqRetraining status of the driver context 
 *        \ref cy_stc_lpddr4_context_t to active 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainActive(void)
{
    g_stc_lpddr_context.dqs2dqRetrainActive = true;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerContextRetrainInactive
 ***************************************************************************//**
 * \brief Function sets the dqs2dqRetraining statsus of the driver context 
 *        \ref cy_stc_lpddr4_context_t to active 
 * 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainInactive(void)
{
    g_stc_lpddr_context.dqs2dqRetrainActive = false;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerContextRetrainRequestCHA
 ***************************************************************************//**
 * \brief Function to set a retrain request for CHA in the driver context 
 *        \ref cy_stc_lpddr4_context_t.
 * 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainRequestCHA(void)
{
    g_stc_lpddr_context.retrainRequestCHA = true;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrControllerContextRetrainRequestCHA
 ***************************************************************************//**
 * \brief Function to reset the retrain request for CHA in the driver context 
 *        \ref cy_stc_lpddr4_context_t. Typically used after a successfull 
 *        retraining  to clear the retrainRequest in the driver context.
 * 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_ClrControllerContextRetrainRequestCHA(void)
{
    g_stc_lpddr_context.retrainRequestCHA = false;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetControllerContextRetrainRequestCHB
 ***************************************************************************//**
 * \brief Function to set a retrain request for CHB in the driver context 
 *        \ref cy_stc_lpddr4_context_t.
 * 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_SetControllerContextRetrainRequestCHB(void)
{
    g_stc_lpddr_context.retrainRequestCHB = true;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq
 ***************************************************************************//**
 * \brief Function clears the refreshStrategyChanged flag in the driver context
 *        to CY_LPDDR4_STRATEGY_UNCHANGED. After refreshRateStrategy was 
 *        changed and refreshRate was updated this function mut be called.
 *        
 * 
 * This flag in the driver context force a update of the refresh rate based on
 * DRAM MR4[2:0] request even if no TUF(temperature update flag )is set in 
 * the MR4 register. After the 
 *
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_ClrCntrlContextRefRateStrategyChngReq(void)
{
    g_stc_lpddr_context.refreshStrategyChanged = CY_LPDDR4_STRATEGY_UNCHANGED;
}






/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrControllerContextRetrainRequestCHB
 ***************************************************************************//**
 * \brief Function to reset the retrain request for CHB in the driver context 
 *        \ref cy_stc_lpddr4_context_t. Typically used after a successfull 
 *        retraining to clear the retrainRequest in the driver context.
 * 
 * \note
 *  
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_ClrControllerContextRetrainRequestCHB(void)
{
    g_stc_lpddr_context.retrainRequestCHB = false;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_UpdateControllerContextLastTraining
 ***************************************************************************//**
 * \brief Function updates the dqsOSCCountLastTraining of the driver context 
 *        \ref cy_stc_lpddr4_context_t. It is the referece for detecting the 
 *        need to retrain. If the difference between actual DQS Osc. value
 *        and dqsOSCCountLastTraining exceed a limit a retraining is triggered.
 * 
 * \return none
 ******************************************************************************/
__STATIC_INLINE void Cy_Lpddr_UpdateControllerContextLastTraining(void)
{
    g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHA = g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHA;
    g_stc_lpddr_context.dqsOSCCountLastTraining.value_CHB = g_stc_lpddr_context.dqsOSCCountTrigTrain.value_CHB;
}






/*******************************************************************************
*                       End of LPDDR4 Driver Context
*******************************************************************************/

/*******************************************************************************
*                        LPDDR 4 AXI Perf Count
*******************************************************************************/



/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_EnableAXIPerfCnt
 ***************************************************************************//**
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
cy_en_lpddr4_retval_t Cy_Lpddr4_EnableAXIPerfCnt(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->AXI_PERF_CNT.unCTL.stcField.u1ENABLED = 0x1;
        retval = CY_LPDDR4_SUCCESS;
    }else {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_DisableAXIPerfCnt
 ***************************************************************************//**
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
cy_en_lpddr4_retval_t Cy_Lpddr4_DisableAXIPerfCnt(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->AXI_PERF_CNT.unCTL.stcField.u1ENABLED = 0x0;
        retval = CY_LPDDR4_SUCCESS;
    }else {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}




/*******************************************************************************
 *  Function Name:  Cy_Lpddr4_StartAXIPerfCnt
 ***************************************************************************//**
 * \brief Starts all AXI Performance Counter Measureing window
 * 
 * Writes the START Bitfiels in TMR_CMD Register . Writing 1 to this field 
 * triggers the start of the measurement window. When this field is written to 1,
 * MU_TMR_STATUS.MEAS in the measurement units are set to 1, indicating that the 
 * measurement window starts, and START is automatically cleared to 0. In each 
 * measurement unit, all counters are cleared to 0, and the current timer count is 
 * set to 1 and is incremented with each cycle of the prescaled clock during the
 * measurment window. START has a higher priority than STOP. Writing 1 to this 
 * field  during the measurement window restarts the measurement window, 
 * clearing the counters and restarting the timer.
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \return \ref cy_en_lpddr4_retval_t
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 *  CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr4_StartAXIPerfCnt(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->AXI_PERF_CNT.unTMR_CMD.stcField.u1START = 0x1;
        retval = CY_LPDDR4_SUCCESS;
    }else {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr4_StopAXIPerfCnt
 ***************************************************************************//**
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
cy_en_lpddr4_retval_t Cy_Lpddr4_StopAXIPerfCnt(volatile stc_LPDDR4_t * base)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {
        base->AXI_PERF_CNT.unTMR_CMD.stcField.u1STOP = 0x1;
        retval = CY_LPDDR4_SUCCESS;
    }else {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}




/*******************************************************************************
 *  Function Name: Cy_Lpddr4_ConfigAXIPerfMUIdx
 ***************************************************************************//**
 *  \brief Function to Configure a AXI Perf Counter Measureing Unit
 *  
 *  
 *   
 *  
 *  \note MU shall not be configured when AXI_PERF_CNT_CTL.ENABLED as this can 
 *        lead to unpredicted measurement results and can cause bus errors. 
 *        Check if AXI_PERF_CNT_CTL.ENABLED by calling 
 *        Cy_Lpddr4_IsAXIPerfCntEnabled() if it is already enabled call 
 *        Cy_Lpddr4_DisableAXIPerfCnt() for disabling.
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
                                                    stc_lpddr4_axi_perf_mu_config_t * muConfig)
{
    cy_en_lpddr4_retval_t retval = CY_LPDDR4_ERROR;
    if( NULL != base )
    {

        if( muIdx > (LPDDR4_AXI_PERF_CNT_MU_NR-1))
        {
            return CY_LPDDR4_BAD_PARAM;
        }
        /* MU shall not be configured when AXI_PERF_CNT_CTL.ENABLED
           doing so can cause unpredictablemeasurments results and bus errors*/
        if(base->AXI_PERF_CNT.unCTL.stcField.u1ENABLED==0x1)
        {
            return CY_LPDDR4_INVALID_STATE;
        }
        /* set 28 bit timer value and 4 bit pre-scaler. 
        The clock going to the timer is divided by 2**PSC. And depend on the selected AXI Port Clock*/
        base->AXI_PERF_CNT.MU[muIdx].unTMR_CTL.stcField.u4PSC = muConfig->ctlPreScale;
        /*set the timer value*/
        base->AXI_PERF_CNT.MU[muIdx].unTMR_CTL.stcField.u28TMR = muConfig->ctlTmr;
        
        /* set the filter parameter */
        base->AXI_PERF_CNT.MU[muIdx].unFILTER.stcField.u4MID = muConfig->filterMID;
        base->AXI_PERF_CNT.MU[muIdx].unFILTER.stcField.u8TID = muConfig->filterTID;
        base->AXI_PERF_CNT.MU[muIdx].unFILTER.stcField.u1WRITE = muConfig->filterWrite;
        base->AXI_PERF_CNT.MU[muIdx].unFILTER.stcField.u4LEN_MIN = muConfig->filterLenMin;
        base->AXI_PERF_CNT.MU[muIdx].unFILTER.stcField.u4LEN_MAX = muConfig->filterLenMax;
        
        /* set the filter mask*/
        base->AXI_PERF_CNT.MU[muIdx].unFILTER_MASK.stcField.u4MID = muConfig->filtermaskMID;
        base->AXI_PERF_CNT.MU[muIdx].unFILTER_MASK.stcField.u8TID = muConfig->filtermaskTID;

        /* set the AXI Port */
        base->AXI_PERF_CNT.MU[muIdx].unPORT_SELECT.stcField.u3SELECT = muConfig->portSelect;
        retval = CY_LPDDR4_SUCCESS;
    }else {
        retval = CY_LPDDR4_BAD_PARAM;
    }
    return retval;
}


/*******************************************************************************
*                        END of LPDDR 4 AXI Perf Count
*******************************************************************************/











/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClearGCFSM_DMCFG()
 ***************************************************************************//**
 *  \brief Clear the Global Control FSM Error Interrupt Status 
 *  
 * Must be called in the faulthandler to clear the interrupt request before 
 * sending the next command to the controller. Once a wrong user command is 
 * issued, controller will assert int_gc_fsm signal if dmcfg_int_gc_fsm_en is 
 * enabled. In this case, users must clear the interrupt flag before 
 * issuing next user command.
 * 
 * \return none 
 * 
 ******************************************************************************/
void Cy_Lpddr_ClearGCFSM_DMCFG(void)
{
    LPDDR40->LPDDR4_CORE.unDMCFG.stcField.u1INT_GC_FSM_CLR = 1;
    LPDDR40->LPDDR4_CORE.unDMCFG.stcField.u1INT_GC_FSM_CLR = 0;
    LPDDR40->LPDDR4_CORE.unDMCFG.stcField.u1INT_GC_FSM_CLR;
}










#if defined(__cplusplus)
}
#endif /* __cplusplus */


/* [] END OF FILE */
