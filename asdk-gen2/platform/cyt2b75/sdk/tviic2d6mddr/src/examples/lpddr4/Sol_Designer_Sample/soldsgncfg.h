/***************************************************************************//**
* \file soldsgncfg.h
*   $Date: 2022-12-16 20:49:47 +0530 (Fri, 16 Dec 2022) $
*   $Revision: 320224 $
*
* contains a lpddr4 configuration which can be used to initialize the 
* LPDDR4 Controller
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#ifndef SOLDSGNCFG_H
#define SOLDSGNCFG_H

//#include "drivers/lpddr4/cy_lpddr4.h"
//#include "drivers/lpddr4/cy_en_lpddr4.h"
#define SVN_REV_LPDDR4_CONFIG_FILE          "$Date: 2022-12-16 20:49:47 +0530 (Fri, 16 Dec 2022) $"
#define SVN_DATE_LPDDR4_CONFIG_FILE         "$Revision: 320224 $"
#define SVN_LPDDR4_CONFIG_FILE_NAME         "soldsgncfg.h"


cy_stc_lpddr4_config_t lpddr_config =  
{ 
    
    .unQOS_CPUSS.u32Register        =   LPDDR4_QOS_CPUSS,       /*!< QoS Configuration Register for CPUSS read master */    /* OK */
    .unQOS_VIDEOSS_RD.u32Register   =   LPDDR4_QOS_VIDEOSS_RD,  /*!< QoS Configuration Register for VIDEOSS read master */  /* OK */
    .unQOS_VIDEOSS_WR.u32Register   =   LPDDR4_QOS_VIDEOSS_WR,  /*!< QoS Configuration Register for VIDEOSS write master */ /* OK */
    .refreshRateStrategy = (cy_en_lpddr4_refreshrate_strategy_t )USERSETTING_REFRESH_STRATEGY,
    #if (CY_USE_PSVP == 1)
        /*.fixedRefreshRate ->value must be divisible by 4. If the value is not divisible by 4 next 
          lower integer which is divisible by 4 will be used.*/
        .fixedRefreshRate = USERSETTING_FIXED_REFRESH_RATE,      
    #else
        .fixedRefreshRate = USERSETTING_FIXED_REFRESH_RATE,      /*For IOP/FCV set TREFI at 105°C  0.25*tREFI*/
    #endif

    /*  PLL Configuration for both frequency setpoints FSP[0]/FSP[1] */
    .pllconfig =
    {
        .feedbackDiv = PLL800_CONFIG_PLL_FDBK_DIV,
        .referenceDiv = PLL800_CONFIG_PLL_REF_DIV,
        .outputDiv = {PLL800_CONFIG_PLL_OUT_DIV,PLL800_CONFIG_PLL_OUT_DIV},
        .vcoclksel =
            {
                CY_LPDDR4_VCO_CLKSEL_FVCO_HALF_OUT_DIV,     /*  Divider for FSP[0] */
                CY_LPDDR4_VCOCLKSEL_FVCO_HALF               /*  Divider for FSP[1] */
            },

        .lockdelay  = CY_LPDDR4_PLL_LOCK_DELAY_INT_WO_SSCG,     /*   */
        .waittime   = (cy_en_lpddr4_pll_800M_wait_time_t) LPDDR4_PLL_WAIT_TIME,/*   */
        .sscgEn     = CY_LPDDR4_ONOFF_DISABLE,                  /* NO SSCG  */
        .sscgDepth  = CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_0_5,       /* don´t care as no SSCG is used  */
        .sscgMode   = CY_LPDDR4_PLL_SPREAD_MODE_DOWN,           /* don´t care as no SSCG is used  */
        .sscgRate   = CY_LPDDR4_PLL_SSCG_RATE_DIV_4096          /* don´t care as no SSCG is used  */
    },
    .unDMCTL.u32Register = LPDDR4_DMCTL,                /*  Dynamo Control Register */
    .unDMCFG.u32Register = LPDDR4_DMCFG,                /*  Dynamo Control Register */
    .unLPMR1.u32Register = LPDDR4_LPMR1,                /*  LPDDR Mode Register 1   */
    .unLPMR2.u32Register = LPDDR4_LPMR2,                /*  LPDDR Mode Register 2   */
    .unLPMR3.u32Register = LPDDR4_LPMR3,                /*  LPDDR Mode Register 3   */
    .unLPMR11.u32Register = LPDDR4_LPMR11,              /*  LPDDR Mode Register 11  */
    .unLPMR12.u32Register = LPDDR4_LPMR12,              /*  LPDDR Mode Register 12  */
    .unLPMR13.u32Register = LPDDR4_LPMR13,              /*  LPDDR Mode Register 13  */
    .unLPMR14.u32Register = LPDDR4_LPMR14,              /*  LPDDR Mode Register 14  */
    .unLPMR22.u32Register = LPDDR4_LPMR22,              /*  LPDDR Mode Register 22  */
    .dqsOscRuntimeClkCyls = (cy_en_lpddr4_mr23_dqsosc_runtime_t) USERSETTING_DQS_OSC_RUNTIME,       /* DQS Oscillator Run Time = MR23 */
    .deltaDQSCountRetrain = {USERSETTING_DELTA_DQS_CHA_RETRAIN,USERSETTING_DELTA_DQS_CHB_RETRAIN}, /* DELTA DQS Count for triggering a DQS2DQ re-training CHACHB*/
    .unRTCFG0_RT0.u32Register = LPDDR4_RTCFG0_RT0,      /*  Route Configuration 0 - Route 0 */
    .unRTCFG0_RT1.u32Register = LPDDR4_RTCFG0_RT1,      /*  Route Configuration 0 - Route 1 */
    .unRTCFG0_RT2.u32Register = LPDDR4_RTCFG0_RT2,      /*  Route Configuration 0 - Route 2 */
    .unRTCFG0_RT3.u32Register = LPDDR4_RTCFG0_RT3,      /*  Route Configuration 0 - Route 3 */
    .unRTCFG1_RT0.u32Register = LPDDR4_RTCFG1_RT0,      /*  Route Configuration 1 - Route 0 */
    .unRTCFG1_RT1.u32Register = LPDDR4_RTCFG1_RT1,      /*  Route Configuration 1 - Route 1 */
    .unRTCFG1_RT2.u32Register = LPDDR4_RTCFG1_RT2,      /*  Route Configuration 1 - Route 2 */
    .unRTCFG1_RT3.u32Register = LPDDR4_RTCFG1_RT3,      /*  Route Configuration 1 - Route 3 */
    .unADDR0.u32Register = LPDDR4_ADDR0,                /*  DRAM Address Register 0 */
    .unADDR1.u32Register = LPDDR4_ADDR1,                /*  DRAM Address Register 5 */
    .unADDR2.u32Register = LPDDR4_ADDR2,                /*  DRAM Address Register 2 */
    .unADDR3.u32Register = LPDDR4_ADDR3,                /*  DRAM Address Register 3 */
    .unADDR4.u32Register = LPDDR4_ADDR4,                /*  DRAM Address Register 4 */
    .unADDR5.u32Register = LPDDR4_ADDR5,                /*  DRAM Address Register 5 */
    .unVTGC.u32Register = LPDDR4_VTGC,                  /*  PHY VREF Training General Control Register */
    .unPHY.u32Register = LPDDR4_PHY,                    /*  PHY Register */
    .unBISTCFG_CH0.u32Register = LPDDR4_BISTCFG_CH0,    /*  BIST Configuration Register - Channel 0 */
    .unBISTCFG_CH1.u32Register = LPDDR4_BISTCFG_CH1,    /*  BIST Configuration Register - Channel 1 */
    .unDQSDQCR.u32Register = LPDDR4_DQSDQCR,            /*  DQS2DQ Delay Control Register */
    .unTREG0.u32Register = LPDDR4_TREG0,                /*  Timing Register 0 */
    .unTREG1.u32Register = LPDDR4_TREG1,                /*  Timing Register 1 */
    .unTREG2.u32Register = LPDDR4_TREG2,                /*  Timing Register 2 */
    .unTREG3.u32Register = LPDDR4_TREG3,                /*  Timing Register 3 */
    .unTREG4.u32Register = LPDDR4_TREG4,                /*  Timing Register 4 */
    .unTREG5.u32Register = LPDDR4_TREG5,                /*  Timing Register 5 */
    .unTREG6.u32Register = LPDDR4_TREG6,                /*  Timing Register 6 */
    .unTREG7.u32Register = LPDDR4_TREG7,                /*  Timing Register 7 */
    .unTREG8.u32Register = LPDDR4_TREG8,                /*  Timing Register 8 */
    .unTREG9.u32Register = LPDDR4_TREG9,                /*  Timing Register 9 */
    .unTREG10.u32Register = LPDDR4_TREG10,              /*  Timing Register 10 */
    .unTREG11.u32Register = LPDDR4_TREG11,              /*  Timing Register 11 */
    .unTREG12.u32Register = LPDDR4_TREG12,              /*  Timing Register 12 */
    .unTREG13.u32Register = LPDDR4_TREG13,              /*  Timing Register 13 */
    .unTREG14.u32Register = LPDDR4_TREG14,              /*  Timing Register 14 */
    .unTREG15.u32Register = LPDDR4_TREG15,              /*  Timing Register 15 */
    .unRTGC0.u32Register = LPDDR4_RTGC0,                /*  PHY Read Training General Control Register 0 */
    .unRTGC1.u32Register = LPDDR4_RTGC1,                /*  PHY Read Training General Control Register 1 */
    .unPTSR0.u32Register = LPDDR4_PTSR0,                /*  PHY Training Status Register 0  */
    .unPTSR1.u32Register = LPDDR4_PTSR1,                /*  PHY Training Status Register 1  */
    .unPTSR2.u32Register = LPDDR4_PTSR2,                /*  PHY Training Status Register 2  */
    .unPTSR3.u32Register = LPDDR4_PTSR3,                /*  PHY Training Status Register 3  */
    .unPTSR4.u32Register = LPDDR4_PTSR4,                /*  PHY Training Status Register 4  */
    .unPTSR5.u32Register = LPDDR4_PTSR5,                /*  PHY Training Status Register 5  */
    .unPTSR6.u32Register = LPDDR4_PTSR6,                /*  PHY Training Status Register 6  */
    .unPTSR7.u32Register = LPDDR4_PTSR7,                /*  PHY Training Status Register 7  */
    .unPTSR8.u32Register = LPDDR4_PTSR8,                /*  PHY Training Status Register 8  */
    .unPTSR9.u32Register = LPDDR4_PTSR9,                /*  PHY Training Status Register 9  */
    .unPTSR10.u32Register = LPDDR4_PTSR10,              /*  PHY Training Status Register 10 */
    .unPTSR11.u32Register = LPDDR4_PTSR11,              /*  PHY Training Status Register 11 */
    .unPTSR12.u32Register = LPDDR4_PTSR12,              /*  PHY Training Status Register 12 */
    .unPTSR13.u32Register = LPDDR4_PTSR13,              /*  PHY Training Status Register 13 */
    .unPTSR14.u32Register = LPDDR4_PTSR14,              /*  PHY Training Status Register 14 */
    .unPTSR15.u32Register = LPDDR4_PTSR15,              /*  PHY Training Status Register 15 */
    .unPTSR16.u32Register = LPDDR4_PTSR16,              /*  PHY Training Status Register 16 */
    .unPTSR17.u32Register = LPDDR4_PTSR17,              /*  PHY Training Status Register 17 */
    .unPTSR18.u32Register = LPDDR4_PTSR18,              /*  PHY Training Status Register 18 */
    .unPTSR19.u32Register = LPDDR4_PTSR19,              /*  PHY Training Status Register 19 */
    .unPTSR20.u32Register = LPDDR4_PTSR20,              /*  PHY Training Status Register 20 */
    .unPTSR21.u32Register = LPDDR4_PTSR21,              /*  PHY Training Status Register 21 */
    .unPTSR22.u32Register = LPDDR4_PTSR22,              /*  PHY Training Status Register 22 */
    .unPTSR23.u32Register = LPDDR4_PTSR23,              /*  PHY Training Status Register 23 */
    .unPTSR24.u32Register = LPDDR4_PTSR24,              /*  PHY Training Status Register 24 */
    .unPTSR25.u32Register = LPDDR4_PTSR25,              /*  PHY Training Status Register 25 */
    .unDLLCTLCA_CH0.u32Register = LPDDR4_DLLCTLCA_CH0,  /* DLL Control Register for PHY Command Module - Channel 0 */
    .unDLLCTLCA_CH1.u32Register = LPDDR4_DLLCTLCA_CH1,  /* DLL Control Register for PHY Command Module - Channel 1 */
    .unDLLCTLDQ_SL0.u32Register = LPDDR4_DLLCTLDQ_SL0,  /* DLL Control Register for PHY Data Module Slice 0 */
    .unDIOR_SL0.u32Register = LPDDR4_DIOR_SL0,          /* PHY Data Module IO Control Register  Slice 0 */
    .unDLLCTLDQ_SL1.u32Register = LPDDR4_DLLCTLDQ_SL1,  /* DLL Control Register for PHY Data Module Slice 1 */
    .unDIOR_SL1.u32Register =LPDDR4_DIOR_SL1,           /* PHY Data Module IO Control Register  Slice 1 */
    .unDLLCTLDQ_SL2.u32Register = LPDDR4_DLLCTLDQ_SL2,  /* DLL Control Register for PHY Data Module Slice 2 */
    .unDIOR_SL2.u32Register = LPDDR4_DIOR_SL2,          /* PHY Data Module IO Control Register  Slice 2 */
    .unDLLCTLDQ_SL3.u32Register = LPDDR4_DLLCTLDQ_SL3,  /* DLL Control Register for PHY Data Module Slice 3 */
    .unDIOR_SL3.u32Register = LPDDR4_DIOR_SL3,          /* PHY Data Module IO Control Register  Slice 3*/
    .unCIOR_CH0.u32Register = LPDDR4_CIOR_CH0,          /*  PHY Command Module IO Control Register - Channel 0  */
    .unCIOR_CH1.u32Register = LPDDR4_CIOR_CH1,          /*  PHY Command Module IO Control Register - Channel 1  */
    .unPTAR.u32Register = LPDDR4_PTAR ,                 /*  PHY Sanity Check Address Register */
    .unPCCR_CH0.u32Register =LPDDR4_PCCR_CH0 ,          /*  PHY Compensation Control Register - Channel 0 */
    .unPCCR_CH1.u32Register =LPDDR4_PCCR_CH1 ,          /*  PHY Compensation Control Register - Channel 1 */
    .unINECC0.u32Register = LPDDR4_INECC0,              /*  Inline ECC Register 0 */
    .unINECC1.u32Register = LPDDR4_INECC1,              /*  Inline ECC Register 1 */
    .unINECC2.u32Register = LPDDR4_INECC2,              /*  Inline ECC Register 2 */
}; 
#endif // SOLDSGNCFG_H
