
#ifndef CONNFIG_02_RD_LLLH_WR_LLLM_1_AC2_H_INCLUDED
#define CONNFIG_02_RD_LLLH_WR_LLLM_1_AC2_H_INCLUDED

#include "drivers/lpddr4/cy_en_lpddr4.h"


cy_stc_lpddr4_config_t lpddr_config =  
{ 
    .unQOS_CPUSS.stcField =           /*!< QoS Configuration Register for CPUSS read master */
    {
        .u2QOS_MASTERID_0   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_1   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_2   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_3   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_4   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_5   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_6   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_7   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_8   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_9   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_10  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_11  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_12  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_13  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_14  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_15  =  CY_LPDDR4_QOS_PRIORITY_LOW
    },

    .unQOS_VIDEOSS_RD.stcField =     /*!< QoS Configuration Register for VIDEOSS read master */
    {
        .u2QOS_MASTERID_0   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_1   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetchdecode0
        .u2QOS_MASTERID_2   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetchlayer0
        .u2QOS_MASTERID_3   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetchwarp1
        .u2QOS_MASTERID_4   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetcheco1
        .u2QOS_MASTERID_5   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetcheco4
        .u2QOS_MASTERID_6   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetchlayer1
        .u2QOS_MASTERID_7   =  CY_LPDDR4_QOS_PRIORITY_HIGH,     //  videoss read fetchdecode4
        .u2QOS_MASTERID_8   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_9   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_10  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_11  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_12  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_13  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_14  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_15  =  CY_LPDDR4_QOS_PRIORITY_LOW
    },

    .unQOS_VIDEOSS_WR.stcField =     /*!< QoS Configuration Register for VIDEOSS write master */
    {
        .u2QOS_MASTERID_0   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_1   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_2   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_3   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_4   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_5   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_6   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_7   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_8   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_9   =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_10  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_11  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_12  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_13  =  CY_LPDDR4_QOS_PRIORITY_MEDIUM,       //videoss write store4
        .u2QOS_MASTERID_14  =  CY_LPDDR4_QOS_PRIORITY_LOW,
        .u2QOS_MASTERID_15  =  CY_LPDDR4_QOS_PRIORITY_LOW
    },
    /*  Select if RefreshRate Startegy to be used. 
            .refreshRateStrategy = CY_LPDDR_REFRESHRATE_DYNAMIC --> If refresh rate is dynamically updated based on MR4[2:0] (CY_LPDDR_REFRESHRATE_DYNAMIC)
            or  
            .refreshRateStrategy = CY_LPDDR_REFRESHRATE_FIXED -->  When set to CY_LPDDR_REFRESHRATE_FIXED paramter .fixedRefreshRate
            specifies the refresh interval. When set to  0 = fixed at 0.25*tREFI (highest refresh rate possible). 
            If .fixedRefreshRate >=0 it specifies then referesh interval in # of clks of CLK_PHY.Refer additional note below about
            value of .fixedRefreshRate*/
    .refreshRateStrategy =     CY_LPDDR_REFRESHRATE_FIXED,

    #if (CY_USE_PSVP == 1)
        
        /*.fixedRefreshRate ->value must be divisible by 4. If the value is not divisible by 4 next 
          higher integer which is divisible by 4 will be used. This could cause violation 
          of refresh interval.*/
        /* uncomment next line for TREFI  fixed rate @105°C freq=80MHz */
        //.fixedRefreshRate = 0x4c,  
        
        /*uncomment next line for TREFI  fixed rate 75°C tREFI scaling of times 8 compared to IOP */
        #if(PSVP_FREQUENCY == PSVP_FREQUENCY_80MHZ)
            .fixedRefreshRate = 0x270,      // @80MHz
        #elif(PSVP_FREQUENCY == PSVP_FREQUENCY_75P2MHZ)
            .fixedRefreshRate = 0x248,      // @75.2MHz
        #elif(PSVP_FREQUENCY == PSVP_FREQUENCY_72MHZ)
            .fixedRefreshRate = 0x230,      // @72MHz
        #elif(PSVP_FREQUENCY == PSVP_FREQUENCY_69P6MHZ)
            .fixedRefreshRate = 0x21C,      // @69.6MHz
        #endif
    #else
        .fixedRefreshRate = 0,      /*For IOP/FCV set TREFI at 105°C  0.25*tREFI*/
    #endif

    /*  PLL Configuration for both frequency setpoints FSP[0]/FSP[1] */
    .pllconfig =
    {
        .feedbackDiv = LPDDR4_PLL_FDBK_DIV,         /*  fpfd=8MHz andf FVCO  = (FRef/Q)*2P the P=100 ==> 1600MHz */
        .referenceDiv = LPDDR4_PLL_REF_DIV,          /*  fin=8MHZ / ref_div=1 ==> fpfd=8MHz */
        
                                    /*  switching between FSP[0]/[1] is only done by changing outputDIV and 
                                        vcoclksel so PLL can stay active but PLL ouput must be 
                                        deactivated when switching between FSPs */
        .outputDiv = {LPDDR4_PLL_OUT_DIV_FSP0,LPDDR4_PLL_OUT_DIV_FSP1},       /*  outputDIV for FSP[0],FSP[1] the 16 for FSP[1] is actually 
                                        a don´t care in our case as we bypass the  output_div*/

        .vcoclksel =
            {
                CY_LPDDR4_VCO_CLKSEL_FVCO_HALF_OUT_DIV,     /*  Divider for FSP[0] */
                CY_LPDDR4_VCOCLKSEL_FVCO_HALF               /*  Divider for FSP[1] */
            },

        .lockdelay  = CY_LPDDR4_PLL_LOCK_DELAY_INT_WO_SSCG,     /*   */
        .waittime   = CY_LPDDR4_PLL_WAIT_TIME_4_8MHZ,           /*   */
        .sscgEn     = CY_LPDDR4_ONOFF_DISABLE,                  /*   */
        .sscgDepth  = CY_LPDDR4_PLL_SSCG_DEPTH_MINUS_0_5,       /*   */
        .sscgMode   = CY_LPDDR4_PLL_SPREAD_MODE_DOWN,           /*   */
        .sscgRate   = CY_LPDDR4_PLL_SSCG_RATE_DIV_4096          /*   */
    },
    /*  Dynamo Control Register */
    .unDMCTL.stcField = 
    {
        .u2DRAM_CHAN_EN = CY_LPDDR4_CHAN_SEL_AB,                                /* Set which channels shall be activated */
        .u8RD_REQ_MIN = 0x8,                                                    /*  */
        .u8WR_REQ_MIN = 0x15,                                                   /*  */
        .u1CHAN_UNLOCK = 0x1,                                                   /*  Channel unlock. A bank requester FSM can
                                                                                    be unlocked while it is blocked by the other 
                                                                                    in the same port.   */
        .u1HI_PRI_IMM  = 0x1,                                                   /*  High priority immediately serving. The Arbiter
                                                                                    will grant for the high priority 
                                                                                    stream immediately.   */
        .u3DDRT = CY_LPDDR4_DMCTL_DDRT_LPDDR4,                                  /* DDR Type --> LPDDR4 */
        .u2DFI_FREQ_RATIO = CY_LPDDR4_DMCTL_DFI_FREQ_RATIO_DIV_4,               /* must be always Frequency Ratio 1:4 !!! do not change !!!!! */
        .u2DRAM_BANK_EN = CY_LPDDR4_DMCTL_DRAM_BANK_EN_BANKS8,                  /* Bank Controller Enable --> 8 Banks */
        .u1SWITCH_CLOSE = CY_LPDDR4_DMCTL_SWITCH_CLOSE_DONT_CLOSE_ON_RW_CHG,    /* Force close Opening Page when Direction changesForce close Opening 
                                                                                   Page when Direction changes */
        .u1BANK_POLICY = CY_LPDDR4_DMCTL_BANK_POLICY_OPEN_PAGE,                 /* Close Page Policy */
        #if(FS1_WDBI == WDBI_DISABLED)
            .u1WR_DBI = CY_LPDDR4_ONOFF_DISABLE,                                /* Write DBI Mode Enable --> disabled */
        #elif(FS1_WDBI == WDBI_ENABLED)
            .u1WR_DBI = CY_LPDDR4_ONOFF_ENABLE,                                 /* Write DBI Mode Enable --> enabled */
        #endif
         #if(FS1_RDBI == RDBI_DISABLED)
            .u1RD_DBI = CY_LPDDR4_ONOFF_DISABLE                                 /* Read DBI Mode Enable --> disabled */
        #elif(FS1_RDBI == RDBI_ENABLED)
            .u1RD_DBI = CY_LPDDR4_ONOFF_ENABLE                                  /* Read DBI Mode Enable --> enable */
        #endif
    }, 
    /*Dynamo Control Register */
    .unDMCFG.stcField = {
        .u1REF_POST_PULL_EN = CY_LPDDR4_ONOFF_DISABLE,                      /*  Postpone/Pull Automatic Refresh */
        .u1INT_GC_FSM_EN = CY_LPDDR4_ONOFF_ENABLE,                          /*  Global FSM Error interrupt enable signal. 0: Disable (Default). 1: Enable*/
        .u1INT_GC_FSM_CLR = CY_LPDDR4_DMCFG_INT_GC_FSM_CLR_DONT_CLEAR,      /*  Global FSM Error interrupt clear signal. 0: Disable (Default). 1: Clear.
                                                                                Set int_gc_fsm LOW. */
        .u1INT_ECC_EN = CY_LPDDR4_ONOFF_DISABLE,                            /*  Enable the ECC internupts*/   
        .u3REQ_TH = 0x7,                                                    /*  Look-ahead buffer request threshold. 
                                                                                The depth of Look-ahead Buffer (maximum: 8) */
        .u1ZQ_AUTO_EN = CY_LPDDR4_ONOFF_ENABLE,                             /*  enable/ disabled Automatic ZQ Calibration */
        .u1INLINE_ECC_EN = CY_LPDDR4_ONOFF_DISABLE                          /*  enable/disable inline ecc */
    },
    /*  LPDDR Mode Register 1 */
    .unLPMR1.stcField = {
        .u2FS0_BL = CY_LPDDR4_BL_16,                        /* 00b: BL = 16 sequential (default) see Memory datasheet */
        .u1FS0_WPRE = CY_LPDDR4_WPRE_WR_PREAMBLE_2,         /* 0: is reserved, so only  remains 1: WR Pre-Amble = 2*tCK */
        .u1FS0_RPRE = CY_LPDDR4_RPRE_RD_PREAMBLE_STATIC,    /* 0: RD Pre-amble = Static (default)*/
        .u3FS0_NWR = CY_LPDDR4_NWR_NWR6,                    /* 000b: nWR = 6 (default) see Memory datasheet */
        .u1FS0_RPST = CY_LPDDR4_RPST_RD_POSTAMBLE_0P5,      /* 0b: RD postamble = 0.5 × tCK (default) see Memory datasheet  */
        #if(BL==BL16)
            .u2FS1_BL = CY_LPDDR4_BL_16,                    /* 00b: BL = 16 sequential (default) see Memory datasheet */
        #elif(BL==BL32)
            .u2FS1_BL = CY_LPDDR4_BL_32,                    /* 00b: BL = 16 sequential (default) see Memory datasheet */
        #endif
        .u1FS1_WPRE = CY_LPDDR4_WPRE_WR_PREAMBLE_2,         /* 0: is reserved, so only  remains 1: WR Pre-Amble = 2*tCK */
        .u1FS1_RPRE = CY_LPDDR4_RPRE_RD_PREAMBLE_STATIC,    /* 0: RD Pre-amble = Static (default)*/
        .u3FS1_NWR = CY_LPDDR_NWR_FSP1,                     /* refer JESD209-4B Table 28 — Read and Write Latencies*/
        #if(READ_POST_AMBLE == READ_POST_AMBLE_0P5_NCLK)
            .u1FS1_RPST = CY_LPDDR4_RPST_RD_POSTAMBLE_0P5       /* 0B : RD Post-amble = 0.5×tCK (default) */
        #elif(READ_POST_AMBLE == READ_POST_AMBLE_1P5_NCLK)
            .u1FS1_RPST = CY_LPDDR4_RPST_RD_POSTAMBLE_1P5       /* 1B : RD Post-amble = 1.5×tCK  */
        #endif
    },
        /*  LPDDR Mode Register 2 */
    .unLPMR2.stcField = 
    { 
        .u3FS0_RL = JESD209_4B_FSP0_MR2_DEFAULT_RL,     /*   000b: RL = 6, nRTP = 8 (default) see JEDEC209-4B( do not change ) */
        .u3FS0_WL = JESD209_4B_FSP0_MR2_DEFAULT_WL,     /*   000b: WL = 4 (default) JEDEC209-4B ( do not change ) */
        .u1FS0_WLS = JESD209_4B_FSP0_MR2_DEFAULT_WLS,   /*   0b: Use WL set A (default) JEDEC209-4B ( do not change )*/
        .u1WRLEV = CY_LPDDR4_ONOFF_DISABLE,             /*  0B: Disabled (default) */
        .u3FS1_RL = CY_LPDDR_READLATENCY_FSP1_LPMR2,    /*  value depends on MR3 OP[6] setting. 
                                                            Check JEDEC209-4B 4.12 Read and Write Latencies */
        .u3FS1_WL = CY_LPDDR_WRITELATENCY_FSP1_LPMR2,   /*  value depends on MR2 OP[6] setting. 
                                                            Check JEDEC209-4B 4.12 Read and Write Latencies */
        #if(FS1_WLS==WL_SET_A)
            .u1FS1_WLS = CY_LPDDR4_WLS_WL_SET_A         /*   0b: Use WL set A (default) see Memory datasheet , 
                                                             impacts the setting of .u3FS1_WL */
        #elif(FS1_WLS==WL_SET_B)
            .u1FS1_WLS = CY_LPDDR4_WLS_WL_SET_B         /*   1b: Use WL set B (default) see Memory datasheet , 
                                                             impacts the setting of .u3FS1_WL */
        #endif
    }, 
    /*  LPDDR Mode Register 3 */
    .unLPMR3.stcField = 
    { 
        .u1FS0_PUCAL = CY_LPDDR4_PUCAL_VDDQ_DIV_3,          /*  1b: VDDQ × 0.5 (default) see Memory datasheet */
        .u1FS0_WPST = CY_LPDDR4_WPST_WR_POSTAMBLE_0P5,      /*  0b: WR postamble = 0.5 × tCK (default) see Memory datasheet */
        .u1PPRP = CY_LPDDR4_ONOFF_DISABLE,                  /*  0b: PPR protection disabled (default) see Memory datasheet */
        .u3FS0_PDDS = CY_LPDDR4_PDDS_RZQ_DIV_6,             /*  110b: RZQ/6 (default) see Memory datasheet */
        /* as on PSVP we are always on FSP[0] and are not able to change to FSP[1]
           we need to deviate from JEDEC default for MR3 OP[6] and MR3 OP[7] to be able to test 
           RDBI and WDBI feature. So for PSVP activate RDBI / WDBI when it is activated for FSP[1] */
        #if(CY_USE_PSVP==1)
            #if(FS1_RDBI == RDBI_DISABLED)
                .u1FS0_RDBI = CY_LPDDR4_ONOFF_DISABLE,
            #elif(FS1_RDBI == RDBI_ENABLED)
                .u1FS0_RDBI = CY_LPDDR4_ONOFF_ENABLE,
            #endif

            #if(FS1_WDBI == WDBI_DISABLED)
                .u1FS0_WDBI = CY_LPDDR4_ONOFF_DISABLE,
            #elif(FS1_WDBI == WDBI_ENABLED)
                .u1FS0_WDBI = CY_LPDDR4_ONOFF_ENABLE,
            #endif
        #else
            .u1FS0_RDBI = JESD209_4B_FSP0_MR3_DEFAULT_RDDBI,    /*  FSP0 (default) 0b: Read DBI Disabled see JESD209-4B ( do not change ) */
            .u1FS0_WDBI = JESD209_4B_FSP0_MR3_DEFAULT_WRDBI,    /*  FSP0 (default) 0b: Write DBI Disabled see JESD209-4B ( do not change ) */
        #endif
        .u1FS1_PUCAL = CY_LPDDR4_PUCAL_VDDQ_DIV_3,          /*  1B: VDDQ/3 (default) */
        #if(WRITE_POST_AMBLE==WRITE_POST_AMBLE_0P5_NCLK)
            .u1FS1_WPST = CY_LPDDR4_WPST_WR_POSTAMBLE_0P5,      /*  0B: WR Post-amble = 0.5×tCK (default) */
        #elif(WRITE_POST_AMBLE==WRITE_POST_AMBLE_1P5_NCLK)
            .u1FS1_WPST = CY_LPDDR4_WPST_WR_POSTAMBLE_1P5,      /*  0B: WR Post-amble = 0.5×tCK (default) */
        #endif
        .u3FS1_PDDS = CY_LPDDR4_PDDS_RZQ_DIV_6,             /*  110B: RZQ/6 (default) */
        #if(FS1_RDBI == RDBI_DISABLED)                      /*  activation RDDBI has effect on RL Read Latency. When activating it using FS1_RDBI  RL is updated accodingly */
            .u1FS1_RDBI = CY_LPDDR4_ONOFF_DISABLE,          /*  0b: Disabled (default) */
        #elif(FS1_RDBI == RDBI_ENABLED)
            .u1FS1_RDBI = CY_LPDDR4_ONOFF_ENABLE,           /*  1b: Enabled RDBI */
        #endif
        #if(FS1_WDBI == WDBI_DISABLED)
            .u1FS1_WDBI = CY_LPDDR4_ONOFF_DISABLE           /*  0b: Disabled (default)*/
        #elif(FS1_WDBI == WDBI_ENABLED)
            .u1FS1_WDBI = CY_LPDDR4_ONOFF_ENABLE            /*  1b: Enabled*/
        #endif
    }, 
    /*  LPDDR Mode Register 11 */
    .unLPMR11.stcField = 
    { 
        .u3FS0_DQODT = JESD209_4B_FSP0_MR11_DEFAULT_DQ_ODT,     /*  DQ bus receiver on-die termination 000b: Disable Jedec209-4B FSP[0] (default)  (do not change ) */
        .u3FS0_CAODT = JESD209_4B_FSP0_MR11_DEFAULT_CA_ODT,     /*  CA bus receiver on-die termination 000b: Disable Jedec209-4B FSP[0] (default)  (do not change ) */
        .u3FS1_DQODT = CY_LPDDR4_ODT_RZQ_DIV_5,                  /*  DQ bus receiver on-die termination for FSP[1] */
        .u3FS1_CAODT = CY_LPDDR4_ODT_RZQ_DIV_5                   /*  CA bus receiver on-die termination for FSP[1] */
    }, 

    /*  LPDDR Mode Register 12 */
    .unLPMR12.stcField = 
    {
        .u6FS0_VREFCAS = JESD209_4B_FSP0_MR12_DEFAULT_VREFCA_VALUE,     /* (default) 011101b: JESD209-4b default Range1 : 27.2% of VDD2 ( do not change ) */
        .u1FS0_VREFCAR = JESD209_4B_FSP0_MR12_DEFAULT_VREFCA_RANGE,     /*  VREF-CA range for FSP0 1b: VREF(CA) range[1] enabled (default) ( do not change ) */
        .u6FS1_VREFCAS = 0x1d,                                          /*  VREF-CA Settings for FSP1 011101b:Range1 : 27.2% of VDD2 (modify according user need  )check JESD209-4B-LPDDR4 Table 13*/
        .u1FS1_VREFCAR = CY_LPDDR4_VREF_RANGE_RANGE1                    /*  VREF-CA range for FSP1 (modify according user need )*/
    }, 
    /*  LPDDR Mode Register 13 */
    .unLPMR13.stcField = 
    {
        .u1CBT = CY_LPDDR4_ONOFF_DISABLE,                   /*  Command bus training  0b: Normal operation (default) see Memory datasheet */
        .u1RPT = CY_LPDDR4_ONOFF_DISABLE,                   /*  Read preamble training 0b: Disabled (default) see Memory datasheet */
        .u1VRO = CY_LPDDR4_ONOFF_DISABLE,                   /*  VREF output 0b: Normal operation (default) see Memory datasheet  */
        .u1VRCG = CY_LPDDR4_VRCG_NORMAL,                    /*  VREF current generator 0b: Normal operation (default) see Memory datasheet  */
        .u1RRO = CY_LPDDR4_RRO_DISABLE_RR4X2X,              /*  Refresh rate option, used to disable Disable codes 001 and 010 in MR4 OP[2:0] 
                                                                0: all codes active   */
        .u1DMD = CY_LPDDR4_DMD_ENABLED,                     /*  Data mask disable 0b: DATA MASK operation enabled (default) see Memory datasheet
                                                                crosscheck with u1WM_ENABLE in RTCFG0_RTx x=0,1,2,3 */
        .u1FSPWR = JESD209_4B_FSP0_MR13_DEFAULT_FSPWR,      /*  Frequency set point write read 0b: Frequency set point[0] (default)see Memory datasheet */
        .u1FSPOP = JESD209_4B_FSP0_MR13_DEFAULT_FSPOP       /*  FREQUENCY SET POINT operation mode 0b: Frequency set point[0] (default) see Memory datasheet */
    }, 
    /*  LPDDR Mode Register 14 */
    .unLPMR14.stcField = 
    { 
        .u6FS0_VREFDQS = JESD209_4B_FSP0_MR14_DEFAULT_VREFDQ_VALUE,     /*  VREF(DQ) value for FSP0 (default) 011101b: JESD209-4b default Range 1 : 27.2% of VDDQ ( do not change )*/
        .u1FS0_VREFDQR = JESD209_4B_FSP0_MR14_DEFAULT_VREFDQ_RANGE,     /*  VREF(DQ) range 1b: VREF(DQ) range[1] enabled (default) see Memory datasheet ( do not change )*/
        .u6FS1_VREFDQS = 0x1d,                                           /*  VREF(DQ) value for FSP1 (change according user needs) check JESD209-4B-LPDDR4 Table 14*/
        .u1FS1_VREFDQR = CY_LPDDR4_VREF_RANGE_RANGE1                    /*  VREF(DQ) range for FSP1 (change according user needs)*/
    },
    /*  LPDDR Mode Register 22 */
    .unLPMR22.stcField = 
    { 
        .u3FS0_SOCODT = CY_LPDDR4_SOCODT_RZQ_DIV_5,           /*  controller ODT value for VOH calibration 000b: Disable
                                                                (default) see Memory datasheet */
        .u1FS0_ODTECK = CY_LPDDR4_ODTE_OVERRIDE_ENABLED,   /*  CK ODT enabled for non-terminating rank0b: ODT-CK enable
                                                                (default) see Memory datasheet */
        .u1FS0_ODTECS = CY_LPDDR4_ODTE_OVERRIDE_ENABLED,   /*  CS ODT enabled for non-terminating rank 0b: ODT-CS enable
                                                                (default) see Memory datasheet */
        .u1FS0_ODTDCA = CY_LPDDR4_ODTDCA_ODT_CA_PAD,        /*  CA ODT termination disable 0b: CA ODT enable (default)
                                                                see Memory datasheet*/
        .u2ODTDX8 = 0x0,                                    /*  ODTD for x8_2ch (byte) mode */
        .u3FS1_SOCODT = CY_LPDDR4_SOCODT_RZQ_DIV_5,         /*  SoC ODT. Controller ODT value for VOH calibration
                                                                for frequency set 1 */
        .u1FS1_ODTECK = CY_LPDDR4_ODTE_OVERRIDE_ENABLED,    /*  ODTE-CK. CK ODT enable for non-terminating rank
                                                                for frequency set 1 */
        .u1FS1_ODTECS = CY_LPDDR4_ODTE_OVERRIDE_ENABLED,    /*  ODTE-CS. CS ODT enable for non-terminating rank
                                                                for frequency set 1 */
        .u1FS1_ODTDCA = CY_LPDDR4_ODTDCA_ODT_CA_PAD         /*  ODTD-CA. CA ODT termination disable for 
                                                                frequency set 1 */ 
    }, 
    /* DQS Oscilattor Run Time = MR23 */
    .dqsOscRuntimeClkCyls =  CY_LPDDR4_MR23_11X16CLK_COUNTS,   /*   when set to CY_LPDDR4_MR23_STOP_BY_MPC_COMMAND the DQS OSCillator must be stopped manually by MPC command
                                                                    CY_LPDDR4_USER_CMD_DQS_STOP . If dqsOscRuntimeClkCyls != CY_LPDDR4_MR23_STOP_BY_MPC_COMMAND  MPC command
                                                                    CY_LPDDR4_USER_CMD_DQS_STOP is illegal and will cause failure. According to JEDEC209-4C time must be at 
                                                                    least 200 nsec.200nsec/1.25nsec@800MHz= 160 => 160 /16 = min 10 */

    /* DELTA DQS Count for triggering a DQS2DQ re-training CHACHB*/
    .deltaDQSCountRetrain = {10,10},

    /*  Route Configuration 0 - Route 0 */
    .unRTCFG0_RT0.stcField = 
    {
        .u1ARQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in read channel - Route 0 */
        .u1AWQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in Write channel - Route 0 */
        .u1EXT_PRI = CY_LPDDR4_RTCFG0_EXT_PRIO_AXI_QOS,         /* Selects whether to use external AXI QoS field or internal 
                                                                   combination of static and dynamic priority.
                                                                   When external priority is enabled, all others configurations will be ignored */
        .u1MAX_PRI = CY_LPDDR4_ONOFF_DISABLE,                   /* Denotes Static Maximum Priority */
        .u4ARQ_LVL_HI = 0xf,                                    /* Denotes Read Address Queue High Priority Threshold */   
        .u4ARQ_LVL_LO = 0x7,                                    /* Denotes Read Address Queue Low Priority Threshold */ 
        .u4AWQ_LVL_HI = 0xf,                                    /* Denotes Write Address Queue High Priority Threshold */
        .u4AWQ_LVL_LO = 0x6,                                    /* Denotes Write Address Queue Low Priority Threshold */
        .u1ARQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Read Channel */
        .u1AWQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Write Channel */
        .u1ARQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Read Address Queue Operation Mode */
        .u1AWQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Write Address Queue Operation Mode  */
        .u1ACQ_REALTIME_EN = CY_LPDDR4_ONOFF_DISABLE,           /* Realtime priority enable for Read/Write */
        .u1WM_ENABLE = CY_LPDDR4_ONOFF_ENABLE                   /* Masked Write Support Enable (only supported with BL16)*/
    },
    /*  Route Configuration 0 - Route 1 */
    .unRTCFG0_RT1.stcField = 
    {
        .u1ARQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in read channel - Route 1 */
        .u1AWQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in Write channel - Route 1 */
        .u1EXT_PRI = CY_LPDDR4_RTCFG0_EXT_PRIO_AXI_QOS,         /* Selects whether to use external AXI QoS field or internal 
                                                                   combination of static and dynamic priority.
                                                                   When external priority is enabled, all others configurations will be ignored */
        .u1MAX_PRI = CY_LPDDR4_ONOFF_DISABLE,                   /* Denotes Static Maximum Priority */
        .u4ARQ_LVL_HI = 0xf,                                    /* Denotes Read Address Queue High Priority Threshold */   
        .u4ARQ_LVL_LO = 0x7,                                    /* Denotes Read Address Queue Low Priority Threshold */ 
        .u4AWQ_LVL_HI = 0xf,                                    /* Denotes Write Address Queue High Priority Threshold */
        .u4AWQ_LVL_LO = 0x6,                                    /* Denotes Write Address Queue Low Priority Threshold */
        .u1ARQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Read Channel */
        .u1AWQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Write Channel */
        .u1ARQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Read Address Queue Operation Mode */
        .u1AWQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Write Address Queue Operation Mode  */
        .u1ACQ_REALTIME_EN = CY_LPDDR4_ONOFF_DISABLE,           /* Realtime priority enable for Read/Write */
        .u1WM_ENABLE = CY_LPDDR4_ONOFF_ENABLE                   /* Masked Write Support Enable (only supported with BL16)*/
    },
    /*  Route Configuration 0 - Route 2 */
    .unRTCFG0_RT2.stcField = 
    {
        .u1ARQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in read channel - Route 2 */
        .u1AWQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in Write channel - Route 2 */
        .u1EXT_PRI = CY_LPDDR4_RTCFG0_EXT_PRIO_AXI_QOS,         /* Selects whether to use external AXI QoS field or internal 
                                                                   combination of static and dynamic priority.
                                                                   When external priority is enabled, all others configurations will be ignored */
        .u1MAX_PRI = CY_LPDDR4_ONOFF_DISABLE,                   /* Denotes Static Maximum Priority */
        .u4ARQ_LVL_HI = 0xf,                                    /* Denotes Read Address Queue High Priority Threshold */   
        .u4ARQ_LVL_LO = 0x7,                                    /* Denotes Read Address Queue Low Priority Threshold */ 
        .u4AWQ_LVL_HI = 0xf,                                    /* Denotes Write Address Queue High Priority Threshold */
        .u4AWQ_LVL_LO = 0x6,                                    /* Denotes Write Address Queue Low Priority Threshold */
        .u1ARQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Read Channel */
        .u1AWQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Write Channel */
        .u1ARQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Read Address Queue Operation Mode */
        .u1AWQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Write Address Queue Operation Mode  */
        .u1ACQ_REALTIME_EN = CY_LPDDR4_ONOFF_DISABLE,           /* Realtime priority enable for Read/Write */
        .u1WM_ENABLE = CY_LPDDR4_ONOFF_ENABLE                   /* Masked Write Support Enable (only supported with BL16)*/
    }, 
    
    /*  Route Configuration 0 - Route 3 */
    .unRTCFG0_RT3.stcField = 
    {
        .u1ARQ_LAHEAD_EN = CY_LPDDR4_ONOFF_ENABLE,              /* Out of order enable for Look-ahead buffer in read channel - Route 3 */
        .u1AWQ_LAHEAD_EN = CY_LPDDR4_ONOFF_DISABLE,             /* Out of order enable for Look-ahead buffer in Write channel - Route 3 */
        .u1EXT_PRI = CY_LPDDR4_RTCFG0_EXT_PRIO_AXI_QOS,         /* Selects whether to use external AXI QoS field or internal 
                                                                   combination of static and dynamic priority.
                                                                   When external priority is enabled, all others configurations will be ignored */
        .u1MAX_PRI = CY_LPDDR4_ONOFF_DISABLE,                   /* Denotes Static Maximum Priority */
        .u4ARQ_LVL_HI = 0xf,                                    /* Denotes Read Address Queue High Priority Threshold */   
        .u4ARQ_LVL_LO = 0x7,                                    /* Denotes Read Address Queue Low Priority Threshold */ 
        .u4AWQ_LVL_HI = 0xf,                                    /* Denotes Write Address Queue High Priority Threshold */
        .u4AWQ_LVL_LO = 0x6,                                    /* Denotes Write Address Queue Low Priority Threshold */
        .u1ARQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_ENABLE,         /* Maximum Latency Barrier Enable for Read Channel */
        .u1AWQ_LAT_BARRIER_EN = CY_LPDDR4_ONOFF_DISABLE,        /* Maximum Latency Barrier Enable for Write Channel */
        .u1ARQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Read Address Queue Operation Mode */
        .u1AWQ_OOO_EN = CY_LPDDR4_ONOFF_ENABLE,                 /* Write Address Queue Operation Mode  */
        .u1ACQ_REALTIME_EN = CY_LPDDR4_ONOFF_DISABLE,           /* Realtime priority enable for Read/Write */
        .u1WM_ENABLE = CY_LPDDR4_ONOFF_ENABLE                   /* Masked Write Support Enable (only supported with BL16)*/
    },
    /*  Route Configuration 1 - Route 0 */
    .unRTCFG1_RT0.stcField = 
    {
        .u8ARQ_LAT_BARRIER = 0x25,          /* Maximum Latency Barrier for Read Channel */
        .u8AWQ_LAT_BARRIER = 0x42,          /* Maximum Latency Barrier for Write Channel */
        .u8ARQ_STARV_TH = 0x8,             /* Denotes Read Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
        .u8AWQ_STARV_TH = 0x8              /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
    },
    /*  Route Configuration 1 - Route 1 */ 
    .unRTCFG1_RT1.stcField = 
    {
        .u8ARQ_LAT_BARRIER = 0x25,          /* Maximum Latency Barrier for Read Channel */
        .u8AWQ_LAT_BARRIER = 0x42,          /* Maximum Latency Barrier for Write Channel */
        .u8ARQ_STARV_TH = 0x8,             /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
        .u8AWQ_STARV_TH = 0x8              /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
    },
    /*  Route Configuration 1 - Route 2 */
    .unRTCFG1_RT2.stcField = 
    {
        .u8ARQ_LAT_BARRIER = 0x25,          /* Maximum Latency Barrier for Read Channel */
        .u8AWQ_LAT_BARRIER = 0x42,          /* Maximum Latency Barrier for Write Channel */
        .u8ARQ_STARV_TH = 0x8,             /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
        .u8AWQ_STARV_TH = 0x8              /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
    },
    /*  Route Configuration 1 - Route 3 */
    .unRTCFG1_RT3.stcField = 
    {
        .u8ARQ_LAT_BARRIER = 0x25,          /* Maximum Latency Barrier for Read Channel */
        .u8AWQ_LAT_BARRIER = 0x42,          /* Maximum Latency Barrier for Write Channel */
        .u8ARQ_STARV_TH = 0x8,             /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
        .u8AWQ_STARV_TH = 0x8              /* Denotes Write Address Queue Threshold for starvation preventing. Its 
                                               value represents the number of hits in a hit chain */
    },
    /* 15R_2C_3B_1L_8C_0 */
    /*  DRAM Address Register 0 */
    .unADDR0.stcField = 
    { 
        .u5COL_ADDR_MAP_B0 = CY_LPDDR4_AXI_BIT_01,      /*  column address bit 0 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B1 = CY_LPDDR4_AXI_BIT_02,      /*  column address bit 1 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B2 = CY_LPDDR4_AXI_BIT_03,      /*  column address bit 2 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B3 = CY_LPDDR4_AXI_BIT_04,      /*  column address bit 3 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B4 = CY_LPDDR4_AXI_BIT_05,      /*  column address bit 4 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B5 = CY_LPDDR4_AXI_BIT_06       /*  column address bit 5 is mapped to AXI address bit # */
    },
    /*  DRAM Address Register 5 */
    .unADDR1.stcField = 
    {
        .u5COL_ADDR_MAP_B6 = CY_LPDDR4_AXI_BIT_07,      /*  column address bit 6 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B7 = CY_LPDDR4_AXI_BIT_08,      /*  column address bit 7 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B8 = CY_LPDDR4_AXI_BIT_13,      /*  column address bit 13 is mapped to AXI address bit # */
        .u5COL_ADDR_MAP_B9 = CY_LPDDR4_AXI_BIT_14       /*  column address bit 14 is mapped to AXI address bit # */
    },
    /*  DRAM Address Register 2 */
    .unADDR2.stcField = 
    {  
        .u5ROW_ADDR_MAP_B0 = CY_LPDDR4_AXI_BIT_15,      /*  row address bit 0 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B1 = CY_LPDDR4_AXI_BIT_16,      /*  row address bit 1 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B2 = CY_LPDDR4_AXI_BIT_17,      /*  row address bit 2 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B3 = CY_LPDDR4_AXI_BIT_18,      /*  row address bit 3 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B4 = CY_LPDDR4_AXI_BIT_19,      /*  row address bit 4 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B5 = CY_LPDDR4_AXI_BIT_20       /*  row address bit 5 is mapped to AXI address bit # */
    },
    /*  DRAM Address Register 3 */
    .unADDR3.stcField = 
    {  
        .u5ROW_ADDR_MAP_B6  = CY_LPDDR4_AXI_BIT_21,     /*  row address bit 6 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B7  = CY_LPDDR4_AXI_BIT_22,     /*  row address bit 7 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B8  = CY_LPDDR4_AXI_BIT_23,     /*  row address bit 8 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B9  = CY_LPDDR4_AXI_BIT_24,     /*  row address bit 9 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B10 = CY_LPDDR4_AXI_BIT_25,     /*  row address bit 10 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B11 = CY_LPDDR4_AXI_BIT_26      /*  row address bit 11 is mapped to AXI address bit # */
    },
    /*  DRAM Address Register 4 */
    .unADDR4.stcField = 
    {  
        .u5ROW_ADDR_MAP_B12 = CY_LPDDR4_AXI_BIT_27,     /*  row address bit 12 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B13 = CY_LPDDR4_AXI_BIT_28,     /*  row address bit 13 is mapped to AXI address bit # */
        .u5ROW_ADDR_MAP_B14 = CY_LPDDR4_AXI_BIT_29      /*  row address bit 14 is mapped to AXI address bit # */
    },
    /*  DRAM Address Register 5 */
    .unADDR5.stcField = 
    {  
        .u5BANK_ADDR_MAP_B0 = CY_LPDDR4_AXI_BIT_10,     /* bank address bit 10 is mapped to AXI address bit # */
        .u5BANK_ADDR_MAP_B1 = CY_LPDDR4_AXI_BIT_11,     /* bank address bit 11 is mapped to AXI address bit # */
        .u5BANK_ADDR_MAP_B2 = CY_LPDDR4_AXI_BIT_12,     /* bank address bit 12 is mapped to AXI address bit # */
        .u5CHAN_ADDR_MAP_B0 = CY_LPDDR4_AXI_BIT_09       /* channel address bit 9 is mapped to AXI address bit # */
    },
    /*  PHY VREF Training General Control Register */
    .unVTGC.stcField = 
    {
        .u1IVREFR = CY_LPDDR4_VTGC_IVREFR_RANGE1,       /*  set always to CY_LPDDR4_VTGC_IVREFR_RANGE1  */
        .u8IVREFTS = CY_LPDDR4_VTGC_IVREFTS_STEPS64,    /*  PHY VREF-DQ training step, specify the number training 
                                                            patterns required by PHY set always to  CY_LPDDR4_VTGC_IVREFTS_STEPS64 */
        .u6VREFDQSW = 0x1,                              /*  DRAM VREF-DQ step width. This field defines the distance 
                                                            between 2 VREF steps in coarse training*/
        .u6VREFCASW = 0x1,                              /*  DRAM VREF-CA step width. This field defines the distance 
                                                            between 2 VREF steps in coarse training */
        .u1IVREFEN = CY_LPDDR4_VTGC_IVREFEN_ENABLE      /*  set always to CY_LPDDR4_VTGC_IVREFEN_ENABLE */
    },
    /*  PHY Register */
    .unPHY.stcField = 
    { 
        /* This register is used by the MC to inform the PHY when to enable/disable 
        the clock to the memory devices */
        .u1DTI_DRAM_CLK_DIS = CY_LPDDR4_ONOFF_DISABLE,
        /* Signal to disable operations on the specified byte-lane */
        .u4DTI_DATA_BYTE_DIS = 0x0
    },
    /*  BIST Configuration Register - Channel 0 */
    .unBISTCFG_CH0.stcField = 
    {  
        .u3START_BANK = 0x0,                        /* Start bank (Width = DRAM_BANK_WIDTH) - Channel 0 */
        .u3END_BANK = 0x0,                          /* End bank (Width = DRAM_BANK_WIDTH) - Channel 0 */
        .u3START_BACKGROUND = 0x0,                  /* Start background index - Channel 0 */
        .u3END_BACKGROUND = 0x0,                    /* End background index - Channel 0 */
        .u4ELEMENT = 0x0,                           /* March element number - Channel 0 */
        .u4OPERATION = 0x0,                         /* March operation number - Channel 0*/
        .u4RETENTION = 0x0,                         /* Count-to value of retention counter (Width = RETENTION_WIDTH) - Channel 0 */
        .u1DIAGNOSIS_EN = CY_LPDDR4_ONOFF_DISABLE   /* Enable diagnosis mode - Channel 0 */
    },
    /*  BIST Configuration Register - Channel 1 */
    .unBISTCFG_CH1.stcField = 
    {  
        .u3START_BANK = 0x0,                        /* Start bank (Width = DRAM_BANK_WIDTH) - Channel 1 */
        .u3END_BANK = 0x0,                          /* End bank (Width = DRAM_BANK_WIDTH) - Channel 1 */
        .u3START_BACKGROUND = 0x0,                  /* Start background index - Channel 1 */
        .u3END_BACKGROUND = 0x0,                    /* End background index - Channel 1 */
        .u4ELEMENT = 0x0,                           /* March element number - Channel 1 */
        .u4OPERATION = 0x0,                         /* March operation number - Channel 1*/
        .u4RETENTION = 0x0,                         /* Count-to value of retention counter (Width = RETENTION_WIDTH) - Channel 1 */
        .u1DIAGNOSIS_EN = CY_LPDDR4_ONOFF_DISABLE   /* Enable diagnosis mode - Channel 1 */
    },
    /*  DQS2DQ Delay Control Register */
    .unDQSDQCR.stcField = 
    { 
        .u8DLYOFFS =    0x0,                                /* DQS2DQ delay offset (unit of phase based on DRAM clock)*/
        .u4DQSEL =      0xf,                                /* Or-select data module to update. Here all 4 byte lanes are updated.*/
        .u1MUPD =       CY_LPDDR4_DQSDQCR_MUPD_AUTOMATIC,   /* DQS2DQ delay manual update enable. 0 = manual update disabled.*/
        .u3MPCRPT =     0x5,                                /* Maximum number of consecutive MPC FIFO Write/Read before checking data. Up to 5.*/
        .u8DLYMAX =     0xff,                               /* Maximum scan steps for a DQS2DQ training session. Default: 0xFF during PHY Init Training*/
        .u1DIR =        CY_LPDDR4_DQSDQCR_DIR_UP            /* DQS2DQ retraining direction control: 1: Up - Increase delay from old setting*/
    },
    /*  Timing Register 0 */
    .unTREG0.stcField = 
    { 
        /*Internal READ Command to PRECHARGE Command delay = tRTP + BL/2 – 12 =max(7.5ns, 8nCK) + 8 -12 */
        .u6T_ALRTP = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRTP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRTP)+(BL/2)-12),
        /*JESD209-4 table 17: tSR = MAX(15ns, 3nCK) = 15ns => 15ns / 1.25ns/tCK = 12 */
        .u5T_CKESR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TSR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TSR),
        /* UG CKE high to DQS T_CKEHDQS= tCKEHDQS + 12, tCKEHDQS=10ns */
        .u5T_CKEHDQS = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKEHDQS,CY_LPDDR4_FSP0_FREQ)+12,
        /* Jedec p.247 tCKE min=max(7.5nsec,4nCK)*/
        .u5T_PD = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKE,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TCKE),
        /* tDHtrain Jedec Spec p. 182 ==> 2nsec. .*/ 
        .u3T_DTRAIN = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_MAX_TDTRAIN,CY_LPDDR4_FSP1_FREQ),
        /* UG DLL enable time. Number of DTI_CLOCK that the DLL_EN must be asserted before DLL_RESET can be asserted  */
        .u8T_DLLEN = 0xa
    },
    /*  Timing Register 1 */
    .unTREG1.stcField = 
    {
        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            /* =tRCD-4,  Jedec Table 17 tRCD MAX(18ns, 4nCK) tRCD >= max(18ns, 4nCK) = 18ns. 18ns / 1.25 = 15. 15 - 4 = 11 = 0xB*/
            .u6T_RCD = (((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD))>4)?(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD))-4:0), 
        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
            .u6T_RCD  = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)>4)?
                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)-4):0,
        #endif

        /*  = tFAW-4 Jedec Spec p 182 tFAW min 40ns */
        .u7T_FAW = ((CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TFAW,CY_LPDDR4_FSP1_FREQ))>4?(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TFAW,CY_LPDDR4_FSP1_FREQ)-4):0),
        /* Jedec Spec p 132 TODO check Read to Write Command La(Bus Turnaround) = RL + BL/2 – WL +3 */
        /*RL + RU(tDQSCK_max/tCK) + BL/2 – WL + tWPRE + RD(tRPST)*/
        .u8T_RTW = (RL + CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TDQSCK_MAX,CY_LPDDR4_FSP1_FREQ) + (BL/2) - WL + JESD209_4B_NCLK_TWPRE + CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRPST,CY_LPDDR4_FSP1_FREQ)),
        //.u8T_RTW = (RL + (BL/2) -WL +3),
        // CAS_n to CAS_n command delay for Masked Write = JESD209-4 tCCDMW = 32 = 0x20
        #if(BL==BL16)
            .u6T_CCDWM = JESD209_4B_NCLK_tCCDMW_BL16,
        #elif(BL==BL32)
            .u6T_CCDWM = JESD209_4B_NCLK_tCCDMW_BL32,
        #endif
        /* T_MRS2LVLEN=tWLDQSEN+5*/
        .u5T_MRS2LVLEN = JESD209_4B_NCLK_TWLDQSEN+5
    },
    /*  Timing Register 2 */
    .unTREG2.stcField = 
    {
        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            // JESD209-4B Table 17 — Core AC Timing
            // tRAS >= MAX(42ns, 3nCK) = 42ns, tRPab >= MAX(21ns, 4nCK) = 21ns
            .u8T_RC =(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))>8?(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))-8:0,

            /* ACTIVE to PRECHARGE command period = tRAS_min - 4 */
            .u8T_RAS = ((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS))>4?(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)-4):0),

        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
        /* tRC = tRAS + tRPab */
            .u8T_RC   = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))>8?
                        ((CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)+
                        CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP))-8):0,

            .u8T_RAS  = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)>4)?
                        (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRAS_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRAS)-4):0,
        #endif

        /*  T_RDPDEN= RL+BL/2+1 =14+16/2+1=23=0x17 */
        .u8T_RDPDEN = (RL + (BL/2) + 1),
        /* DLL reset time. The number of DTI_CLOCKs for which the DLL_RESET must 
           remain asserted when reset is triggered though the POM register = 10 */
        .u8T_DLLRST = 0xa       
    },
    /*  Timing Register 3 */
    .unTREG3.stcField = 
    {
        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            /* PRE command period = tRP - 4. tRPpb >= MAX(18, 4nCK) */
            .u6T_RP = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)>4?(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)-4):0,
        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
            .u6T_RP = (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)>4)?
                      (CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRP_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRP)-4):0,
        #endif

        /* JEDEC p 157 min=8nCK */
        .u4T_MRR = JESD209_4B_NCLK_TMRR,
        /*Valid Strobe Requirement before CKE Low  =tDQSCKE */
        .u5T_DQSCKE = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TDQSCKE,CY_LPDDR4_FSP1_FREQ),
        /*This field defines the number of consecutive times of reading data correctly to confirm 
        the stability of DRAM read before DQS2DQ training finishes t_dqrpt = 8*/
        .u5T_DQRPT = 0x8,
        /* =tMRD+4 Mode register set command delay Jedec p 153 tMRD min=max(14ns,10nck) */
        .u5T_MRD = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TMRD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TMRD)+4,
        /* JEDEC MAX(30ns, 8tCK) Table 76 — ZQCal Timing Parameters */
        .u7T_ZQLAT = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TZQLAT,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TZQLAT)+4
    },
    /*  Timing Register 4 */
    .unTREG4.stcField = 
    {
        /*u8T_WLBR = WL + BL/2 – 3 + tWR,  tWR (Write Recovery time, JEDEC209-4B =max(18ns, 6nCK) = 8 + 16/2 -3 + 15 = 0x1c => OK*/
        /*WL + BL/2 – 3 + max(tWR, nWR)   tWR = min -->max(18ns, 6nCK) nWR Refer Jedec Table 28 --> CY_LPDDR_NWR_CLKCNT_FSP1 CDT38071 */
        .u8T_WLBR =( WL + (BL/2) - 3 + CY_LPDDR4_MAX(CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWR),CY_LPDDR_NWR_CLKCNT_FSP1)),
        /* Timing of WRA command to Power Down entry = WL + BL/2 +tWR +2. JESD209-4B Table 17 — Core AC Timing  -->tWR  */
        .u8T_WRAPDEN =(WL + (BL/2) + 2 + CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWR)),
        /* Jedec p 153 tXP min = MAX(7.5ns,5nCK) */
        .u5T_XP = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TXP,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TXP),
        /* JESD209-4B  Table 76 — ZQCal Timing Parameters tZQCAL MIN 1 us*/
        .u11T_ZQCAL = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TZQCAL,CY_LPDDR4_FSP1_FREQ)
    },
    /*  Timing Register 5 */
    .unTREG5.stcField = { 
        /* JESD09-4B Table 76 — ZQCal Timing Parameters Time needed to reset ZQ calibration after user Command USER_CMD_ZQRS   */
        .u8T_ZQRS = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TZQRESET,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TZQRESET),
        /* JEDEC p. 182 min=MAX(1.75nsec,3nck)*/
        .u2T_CKCKEH = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKCKEH,CY_LPDDR4_FSP0_FREQ),JESD209_4B_NCLK_TCKCKEH),
        /* JESD209-4B 4.20.1 Refresh Requirement value programmed must be %4*/
        .u14T_REFI = CY_LPDDR4_CLOCKCYCLES_TREFI(JESD209_4B_NS_TREFI,CY_LPDDR4_FSP1_FREQ),
        /* Jedec Spec page 127 tOSCO min = MAX(40ns,8nCK) */
        .u8T_OSCO = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TOSCO,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TOSCO)
    }, 
    /*  Timing Register 6 */
    .unTREG6.stcField = {
        /* default value 2 Maximum number of cycles that a low priority read 
        can wait before initiating the direction switch = 512 . UG READ_LOW must be greater than READ_HIGH*/
        .u10T_READ_LOW = 1023,
        /* default value Maximum number of cycles that a high priority read can wait before initiating the direction switch = 64*/
        .u10T_READ_HIGH = 31,
        /* Jedec page 60 depending on BL.BL=16 ==> tCCD is 8*tCK, BL=32 ==> tCCD is 16*tCK. */
        #if(BL==BL16)
            .u5T_CCD = JESD209_4B_NCLK_TCCD_BL16,
        #elif(BL==BL32)
            .u5T_CCD = JESD209_4B_NCLK_TCCD_BL32,
        #endif
        /* Delay from start of internal write transaction to internal read command -Different BG. 
        = WL + BL/2 + tWTR+1 = 8 +16/2 + max(10ns,8nCK)+1=  Jedec tWTR=max(10ns,8nCK) = 25 =19hex */ 
        .u7T_WLBTR = (WL + (BL/2) + 1 + CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWTR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWTR))
    },
    /*  Timing Register 7 */
    .unTREG7.stcField = 
    {
        /* JESD209-4B  Table 6 — Initialization Timing Parameters minimum 200 us ,  */
        .u14T_INIT1 = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TINIT1,CY_LPDDR4_FSP0_FREQ),

        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            /* tRCD + 3 Jedec page 60 Table 17 tRCD = max(18ns,4nCK) */
            .u6T_MPCWR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)+3,
        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
            .u6T_MPCWR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRCD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRCD)+3,
        #endif

        /* JEDEC Spec p 111 tCKFSPE min=max(7.5ns,4nCK) used durig FSP Switching*/
        .u4T_CKFSPE = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKFSPE,CY_LPDDR4_FSP0_FREQ),JESD209_4B_NCLK_TCKFSPE),
        /* JEDEC Spec p 111 tCKFSPX min=max(7.5ns,4nCK) used durig FSP Switching*/
        .u4T_CKFSPX = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKFSPX,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TCKFSPX),
        /* JEDEC Spec p 153 tCKELCK min=max(5ns, 5nCK) */
        .u4T_CKELCK = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCKELCK,CY_LPDDR4_FSP0_FREQ),JESD209_4B_NCLK_TCKELCK)+4
    },
    /*  Timing Register 8 */
    .unTREG8.stcField = 
    { 
        /* default value  Maximum number of cycles that a low priority write can 
        wait before initiating the direction switch = 512 */
        .u10T_WRITE_LOW = 1023,
        /* default value  Maximum number of cycles that a high priority write 
        can wait before initiating the direction switch  = 64 */
        .u10T_WRITE_HIGH = 127,
        /* JESD209-4B Table 48 — Mode Register Read/Write AC timing tMRW min = max(10ns,10nck) */
        .u5T_MRW = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TMRW,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TMRW),
        /* TODO:Check value. MPC write fifo to MPC read fifo  
        t_mpcwr2rd = WL + BL/2 + 1 + tWTR  = 8+16/2+1 +tWTR=max(10ns,8nCK) =25 =19hex*/
        // .u7T_MPCWR2RD = 0x19
        .u7T_MPCWR2RD =(WL + (BL/2) + 1 +  CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TWTR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TWTR))
    },
    /*  Timing Register 9 */
    .unTREG9.stcField = 
    { 
        /*ZQ Calibration interval corresponds to 32ms*/
        .u28T_ZQ_ITV = CY_LPDDR4_CLOCKCYCLES(ZQCAL_INTERVAL_NSEC,CY_LPDDR4_FSP1_FREQ),
        /* JEDEC spec p 153 min=max(1.75ns, 3nCK) */
        .u4T_CMDCKE = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCMDCKE,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TCMDCKE)
    },
    /*  Timing Register 10 */
    .unTREG10.stcField = 
    { 
        // Refresh Cycle Time (All Banks) = tRFCab - 4. 
        .u10T_RFC = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TREFCAB,CY_LPDDR4_FSP1_FREQ)-4,
        /* JESD209-4B Table 63 — Write Leveling Timing Parameters*/
        .u6T_MRS2ACT = JESD209_4B_NCLK_TWLMRD+10,
        /* check value according to UG >=8 with all conditions should be met (fine tune might be possible) */
        .u4T_LVLLOAD = 0x8,
        /* check value according to UG =15 with 15 all conditions should be met (fine tune might be possible)*/
        .u4T_LVLDLL = 0xf, 
        /* check value according to UG >=90 with 90 all conditions should be met (fine tune might be possible) */
        .u8T_LVLRESP = 0x5a 
    },
    /*  Timing Register 11 */
    .unTREG11.stcField = 
    { 
        /* Default Value  32000=0x7d00h  */
        .u16T_DLLLOCK = 0x7d00,
        /* JESD209-4B  Table 6 — Initialization Timing Parameters tINIT5=2us/12.5ns@80MHZ == >160. 
         Attention this parameter is limited as with the fixed PSVP clock the value is bigger as 
        it can be programmed in 7 Bits For SOC normal Init Clk is 50MHz==>100 no limit needed.*/
        /*TODO: romve CY_LPDDR4_UPPER_LIMIT() once we are not longer on PSVP */
        .u7T_INIT5 =  CY_LPDDR4_UPPER_LIMIT(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TINIT5,CY_LPDDR4_FSP0_FREQ),CY_LPDDR_INIT5_MAX_LIMIT),
        /*  JESD209-4B Table 57 — Command Bus Training AC Timing  tCAENT=250nsec*/
        .u9T_CAENT = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TCAENT,CY_LPDDR4_FSP0_FREQ)
    },
    /*  Timing Register 12 */
    .unTREG12.stcField = 
    {
        #if(DERATING_STRATEGY == DERATING_DYNAMIC)
            /*Jedec page 182 tRRD min=max(10ns,4nCK)*/
            .u5T_RRD = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRRD,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRRD),
        #elif(DERATING_STRATEGY == DERATING_ALWAYS_ON)
            .u5T_RRD = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TRRD_DERATED,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TRRD),
        #endif
        /* JESD209-4B  Table 6 — Initialization Timing Parameters
        parameter must be limited as with the 80MHz on PSVP the value is greater as 17 bit can hold */
        /*  
            TODO:remove the CY_LPDDR4_UPPER_LIMIT() as soon as the PSVP is not longer used as the PSVP with 80MHZ causes
            overflow in INIT3 this was implemented 
        */
        .u17T_INIT3 = CY_LPDDR4_UPPER_LIMIT(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TINIT3,CY_LPDDR4_FSP0_FREQ),CY_LPDDR_INIT3_MAX_LIMIT),
        /* JESD209-4b Table 17 — Core AC Timing tXSR min=max(tRFCab + 7.5ns, 2nCK) */
        .u10T_XSR = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TXSR,CY_LPDDR4_FSP1_FREQ),JESD209_4B_NCLK_TXSR)
    },
    /*  Timing Register 13 */
    .unTREG13.stcField = 
    {
        /* was initially 8 set to max value during learning x10 
        UG gives different values depending on working point */
        .u5T_LVLDIS = 0x8,
        /* TODO check value UG gives different values depending on working 
        point, 20 should be ok for all operating points */
        .u5T_LVLEXIT = 0x14,
        /* always 8 during each training no change.UG >=8 Leveling command-to-command. 
        Specifies the minimum number of DFI clock cycles after the assertion of a RD/MRR command 
        to the next RD/MRR command. */
        .u4T_LVLAA = 0x8,
        /* JESD209-4B Table 60 — AC Timing Table tFC_long 250ns */
        .u9T_FC = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TFCLONG,CY_LPDDR4_FSP1_FREQ)
    }, 
    /*  Timing Register 14 */
    .unTREG14.stcField = 
    {
        /* Micron DS p 263 tODTUP min=RU(20ns/tck(AVG)),
        Jedec 141 tODTUP min=RU(TBDns/tCK(avg) TODO:check newer version
         t_odtup=GetMax(RU(20ns/clk_period), 10)  */
        .u8T_ODTUP = CY_LPDDR4_MAX(CY_LPDDR4_CLOCKCYCLES(20,CY_LPDDR4_FSP1_FREQ),10),
        /* */ 
        .u8T_VREFTIMESHORT = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TVREFCASHORT,CY_LPDDR4_FSP1_FREQ), 
        /*  VREF CA TRAINING. During CBT >= RU(tVREFca_Long/tfs_perid)  */
        .u9T_VREFTIMELONG = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_TVREFCALONG,CY_LPDDR4_FSP1_FREQ)
    },
    /*  Timing Register 15 */
    .unTREG15.stcField = 
    {
        /* =tVRCG_ENABLE JEDEC Spec p.158 Table 17 max tVRCG_ENABLE=200nsec */
        /* added a offset of +6CLKs as without we saw errors  TODO  confirmation of actual offset value needed*/
        .u9T_VRCGEN = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_MAX_TVRCG_ENABLE,CY_LPDDR4_FSP1_FREQ)+6,
        /* VREF high current mode disable time max =tVRCG_DISABLE  */
        /* added a offset of +6CLKs as without we saw errors  TODO  confirmation of actual offset value needed*/
        .u8T_VRCGDIS = CY_LPDDR4_CLOCKCYCLES(JESD209_4B_NS_MAX_TVRCG_DISABLE,CY_LPDDR4_FSP1_FREQ)+6
    },
    /*  PHY Read Training General Control Register 0 */
    .unRTGC0.stcField = 
    {
        .u1GT_UPDT = 0x0,           /* Gate training setting update    */
        .u1GT_DIS = 0x0,            /* Auto gate training disable */
        /*Refer JEDEC 209-4B Chapter 4.12 Read and Write Latencies */
        .u6FS0_TWREN = 0x4,         /* Specifies the number of DFI PHY clock cycles between when a write command is sent on the DFI control interface 
                                       and when the dfi_wrdata_en signal is asserted. LPDDR4: WL   =*/
        .u6FS0_TRDEN = 0x6,         /*  RL*/
        .u7FS0_TRDENDBI = 0x6       /* default value */
    },
    /*  PHY Read Training General Control Register 1 */
    .unRTGC1.stcField = 
    {
        /* Specifies the number of DFI PHY clock cycles between when a write command is sent on the DFI 
        control interface and when the dfi_wrdata_ensignal is asserted. LPDDR4: WL. refer JESD209-4B  4.12 Read and Write Latencies*/
        .u6FS1_TWREN = WL,
        /* Specifies the number of DFI PHY clock cycles from the assertion of a read command on the DFI 
        to the assertion of the dfi_rddata_en signal LPDDR4: RL. Refer JESD209-4B  4.12 Read and Write Latencies*/
        .u6FS1_TRDEN = RL,
        /* RL for DBI Enabled refer JESD209-4B  4.12 Read and Write Latencies */
        .u7FS1_TRDENDBI = CY_LPDDR_READLATENCY_FSP1_DBI
    },
    
    /* all values for the PTSR Registers are initialize with values trained during FCV simulation . 
    For real silicon the values must be changed to values trained on the real target HW 
    before they can be used to reload the phy and skipping training */
    
    /*PHY Training Setting Register for VREFCA PTSR0,PTSR1,PTSR2,PTSR24,PTSR25 */
    /*TODO: check which init value to used for PTSR0, PTSR1, PTSR2, PTSR24, PTSR25 Dolphin 
      Driver Code or values shown after sucessfull learning   */
    
    /*  PHY Training Setting Register 0 */
    .unPTSR0.stcField = 
    {
        .u1VREFCAR  = CY_LPDDR4_VREF_RANGE_RANGE1, /*  VREF-CA range */
        .u6VREFCAS  = 0x0d,                     /*  VREF-CA setting */
        .u6CSC0     = 0x00,                     /*  CS setting for LPDDR4 channel 0 */
        .u6CSC1     = 0x00,                     /*  CS setting for LPDDR4 channel 1 */
        .u6CAC0B0   = 0x00,                     /*  CA setting for LPDDR4 channel 0 bit 0 */
        .u6CAC0B1   = 0x00                      /*  CA setting for LPDDR4 channel 0 bit 1 */
    },
    /*  PHY Training Setting Register 1 */
    .unPTSR1.stcField = 
    {
        .u6CAC0B2 = 0x00,           /*  CA setting for LPDDR4 channel 0 bit 2 */
        .u6CAC0B3 = 0x00,           /*  CA setting for LPDDR4 channel 0 bit 3 */
        .u6CAC0B4 = 0x00,           /*  CA setting for LPDDR4 channel 0 bit 4 */
        .u6CAC0B5 = 0x00,           /*  CA setting for LPDDR4 channel 0 bit 5 */
        .u6CAC1B0 = 0x00            /*  CA setting for LPDDR4 channel 1 bit 0 */
    },
    /*  PHY Training Setting Register 2 */
    .unPTSR2.stcField = 
    {
        .u6CAC1B1 = 0x00,           /*  CA setting for LPDDR4 channel 1 bit 1*/
        .u6CAC1B2 = 0x00,           /*  CA setting for LPDDR4 channel 1 bit 2*/
        .u6CAC1B3 = 0x00,           /*  CA setting for LPDDR4 channel 1 bit 3*/
        .u6CAC1B4 = 0x00,           /*  CA setting for LPDDR4 channel 1 bit 4*/
        .u6CAC1B5 = 0x00            /*  CA setting for LPDDR4 channel 1 bit 5*/
    },
    /*  PHY Training Setting Register 3 */
    .unPTSR3.stcField = 
    {
        .u6GTS0         = 0x15,     /*  Gate setting for slice 0 */
        .u6GTS1         = 0x15,     /*  Gate setting for slice 1 */
        .u6GTS2         = 0x15,     /*  Gate setting for slice 2 */
        .u6GTS3         = 0x15,     /*  Gate setting for slice 3 */
        .u1VREFDQWRR    = 0x1,      /*  VREF-DQ range*/
        .u6VREFDQWRS    = 0x0d      /*  VREF-DQ Setting*/
    },
    /*  PHY Training Setting Register 4 */
    .unPTSR4.stcField = 
    {
        .u8WRLVLS0 = 0x81,          /*  Write leveling setting for slice 0 */
        .u8WRLVLS1 = 0x81,          /*  Write leveling setting for slice 1 */
        .u8WRLVLS2 = 0x81,          /*  Write leveling setting for slice 2 */
        .u8WRLVLS3 = 0x81           /*  Write leveling setting for slice 3 */
    },
    /*  PHY Training Setting Register 5  tDQS2DQ Drift Compensation*/
    .unPTSR5.stcField = 
    {
        .u8DQSDQS0B0 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[0] */
        .u8DQSDQS0B1 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[1] */
        .u8DQSDQS0B2 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[2] */
        .u8DQSDQS0B3 = 0x51         /*  DQS-DQ setting for slice 0 – DQ[3] */
    },
    /*  PHY Training Setting Register 6  tDQS2DQ Drift Compensation*/
    .unPTSR6.stcField = 
    {
        .u8DQSDQS0B4 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[4] */
        .u8DQSDQS0B5 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[5] */
        .u8DQSDQS0B6 = 0x51,        /*  DQS-DQ setting for slice 0 – DQ[6] */
        .u8DQSDQS0B7 = 0x51         /*  DQS-DQ setting for slice 0 – DQ[7] */ 
    },
    /*  PHY Training Setting Register 7  tDQS2DQ Drift Compensation*/
    .unPTSR7.stcField = 
    {
        .u8DQSDQS1B0 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[0] */
        .u8DQSDQS1B1 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[1] */
        .u8DQSDQS1B2 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[2] */
        .u8DQSDQS1B3 = 0x51         /*  DQS-DQ setting for slice 1 – DQ[3] */
    },
    /*  PHY Training Setting Register 8  tDQS2DQ Drift Compensation*/
    .unPTSR8.stcField = 
    {
        .u8DQSDQS1B4 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[4] */
        .u8DQSDQS1B5 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[5] */
        .u8DQSDQS1B6 = 0x51,        /*  DQS-DQ setting for slice 1 – DQ[6] */
        .u8DQSDQS1B7 = 0x51         /*  DQS-DQ setting for slice 1 – DQ[7] */
    },
    /*  PHY Training Setting Register 9  tDQS2DQ Drift Compensation*/
    .unPTSR9.stcField = 
    {
        .u8DQSDQS2B0 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[0] */
        .u8DQSDQS2B1 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[1] */
        .u8DQSDQS2B2 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[2] */
        .u8DQSDQS2B3 = 0x51         /*  DQS-DQ setting for slice 2 – DQ[3] */
    },
    /*  PHY Training Setting Register 10  tDQS2DQ Drift Compensation*/
    .unPTSR10.stcField = 
    {
        .u8DQSDQS2B4 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[4] */
        .u8DQSDQS2B5 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[5] */
        .u8DQSDQS2B6 = 0x51,        /*  DQS-DQ setting for slice 2 – DQ[6] */
        .u8DQSDQS2B7 = 0x51         /*  DQS-DQ setting for slice 2 – DQ[7] */
    },
    /*  PHY Training Setting Register 11  tDQS2DQ Drift Compensation*/
    .unPTSR11.stcField = 
    {
        .u8DQSDQS3B0 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[0] */
        .u8DQSDQS3B1 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[1] */
        .u8DQSDQS3B2 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[2] */
        .u8DQSDQS3B3 = 0x51         /*  DQS-DQ setting for slice 3 – DQ[3] */
    },
    /*  PHY Training Setting Register 12  tDQS2DQ Drift Compensation*/
    .unPTSR12.stcField = 
    {
        .u8DQSDQS3B4 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[4] */
        .u8DQSDQS3B5 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[5] */
        .u8DQSDQS3B6 = 0x51,        /*  DQS-DQ setting for slice 3 – DQ[6] */
        .u8DQSDQS3B7 = 0x51         /*  DQS-DQ setting for slice 3 – DQ[7] */
    },
    /*  PHY Training Setting Register 13  tDQS2DQ Drift Compensation*/
    .unPTSR13.stcField = 
    {
        .u8DQSDMS0 = 0x51,          /*  DQS-DM setting for slice 0 */
        .u8DQSDMS1 = 0x51,          /*  DQS-DM setting for slice 1 */
        .u8DQSDMS2 = 0x51,          /*  DQS-DM setting for slice 2 */
        .u8DQSDMS3 = 0x51           /*  DQS-DM setting for slice 3 */
    },
    /*  PHY Training Setting Register 14 */
    .unPTSR14.stcField = 
    {
        .u8RDLVLDQS0B0 = 0x21,      /*  Read DQ setting for slice 0 bit 0  */
        .u8RDLVLDQS0B1 = 0x21,      /*  Read DQ setting for slice 0 bit 1  */
        .u8RDLVLDQS0B2 = 0x21,      /*  Read DQ setting for slice 0 bit 2  */
        .u8RDLVLDQS0B3 = 0x21       /*  Read DQ setting for slice 0 bit 3  */
    },
    /*  PHY Training Setting Register 15 */
    .unPTSR15.stcField = 
    {
        .u8RDLVLDQS0B4 = 0x21,      /*  Read DQ setting for slice 0 bit 4*/
        .u8RDLVLDQS0B5 = 0x21,      /*  Read DQ setting for slice 0 bit 5*/
        .u8RDLVLDQS0B6 = 0x21,      /*  Read DQ setting for slice 0 bit 6*/
        .u8RDLVLDQS0B7 = 0x21       /*  Read DQ setting for slice 0 bit 7*/
    },
    /*  PHY Training Setting Register 16 */
    .unPTSR16.stcField = 
    {
        .u8RDLVLDQS1B0 = 0x21,      /*  Read DQ setting for slice 1 bit 0 */
        .u8RDLVLDQS1B1 = 0x21,      /*  Read DQ setting for slice 1 bit 1 */
        .u8RDLVLDQS1B2 = 0x21,      /*  Read DQ setting for slice 1 bit 2 */
        .u8RDLVLDQS1B3 = 0x21       /*  Read DQ setting for slice 1 bit 3 */
    },
    /*  PHY Training Setting Register 17 */
    .unPTSR17.stcField =
    {
        .u8RDLVLDQS1B4 = 0x21,      /*  Read DQ setting for slice 1 bit 4 */
        .u8RDLVLDQS1B5 = 0x21,      /*  Read DQ setting for slice 1 bit 5 */
        .u8RDLVLDQS1B6 = 0x21,      /*  Read DQ setting for slice 1 bit 6 */
        .u8RDLVLDQS1B7 = 0x21       /*  Read DQ setting for slice 1 bit 7 */ 
    },
    /*  PHY Training Setting Register 18 */
    .unPTSR18.stcField = 
    {
        .u8RDLVLDQS2B0 = 0x21,      /*  Read DQ setting for slice 2 bit 0 */ 
        .u8RDLVLDQS2B1 = 0x21,      /*  Read DQ setting for slice 2 bit 1 */ 
        .u8RDLVLDQS2B2 = 0x21,      /*  Read DQ setting for slice 2 bit 2 */ 
        .u8RDLVLDQS2B3 = 0x21       /*  Read DQ setting for slice 2 bit 3 */
    },
    /*  PHY Training Setting Register 19 */
    .unPTSR19.stcField = 
    {
        .u8RDLVLDQS2B4 = 0x21,      /*  Read DQ setting for slice 2 bit 4 */
        .u8RDLVLDQS2B5 = 0x21,      /*  Read DQ setting for slice 2 bit 5 */
        .u8RDLVLDQS2B6 = 0x21,      /*  Read DQ setting for slice 2 bit 6 */
        .u8RDLVLDQS2B7 = 0x21       /*  Read DQ setting for slice 2 bit 7 */
    },
    /*  PHY Training Setting Register 20 */
    .unPTSR20.stcField = 
    {
        .u8RDLVLDQS3B0 = 0x21,      /*  Read DQ setting for slice 3 bit 0 */
        .u8RDLVLDQS3B1 = 0x21,      /*  Read DQ setting for slice 3 bit 1 */
        .u8RDLVLDQS3B2 = 0x21,      /*  Read DQ setting for slice 3 bit 2 */
        .u8RDLVLDQS3B3 = 0x21       /*  Read DQ setting for slice 3 bit 3 */
    },
    /*  PHY Training Setting Register 21 */
    .unPTSR21.stcField = 
    {
        .u8RDLVLDQS3B4 = 0x21,      /*  Read DQ setting for slice 3 bit 4 */
        .u8RDLVLDQS3B5 = 0x21,      /*  Read DQ setting for slice 3 bit 5 */
        .u8RDLVLDQS3B6 = 0x21,      /*  Read DQ setting for slice 3 bit 6 */
        .u8RDLVLDQS3B7 = 0x21       /*  Read DQ setting for slice 3 bit 7 */
    },
    /*  PHY Training Setting Register 22 */
    .unPTSR22.stcField = 
    {
        .u8RDLVLDMS0 = 0x21,        /*  Read DM setting for slice 0 */
        .u8RDLVLDMS1 = 0x21,        /*  Read DM setting for slice 1 */
        .u8RDLVLDMS2 = 0x21,        /*  Read DM setting for slice 2 */
        .u8RDLVLDMS3 = 0x21         /*  Read DM setting for slice 3 */
    },
    /*  PHY Training Setting Register 23 */
    .unPTSR23.stcField = 
    {
        .u6VREFDQRDS0 = 0x20,       /*  VREF-DQ setting for slice 0 READ path */
        .u6VREFDQRDS1 = 0x20,       /*  VREF-DQ setting for slice 1 READ path */
        .u6VREFDQRDS2 = 0x20,       /*  VREF-DQ setting for slice 2 READ path */
        .u6VREFDQRDS3 = 0x20,       /*  VREF-DQ setting for slice 3 READ path */
        .u1VREFDQRDR  = 0x1         /*  VREF-DQ range for READ path*/
    },
    /*  PHY Training Setting Register 24 */
    .unPTSR24.stcField = 
    {
        .u6CMDDLY_CKE0  = 0x20,     /*  MEM_CKE[0] delay setting */
        .u6CMDDLY_CKE1  = 0x20,     /*  MEM_CKE[1] delay setting */
        .u4PSCK         = 0x0,      /*  PHY-set 1CK exceed for each slice  */
        .u16SANPAT      = 0xd5d5    /*  Sanity check pattern*/
    },
    /*  PHY Training Setting Register 25 */
    .unPTSR25.stcField = 
    {
        .u6CMDDLY_ODT0  = 0x20,     /*  MEM_ODT[0] of MEM_ODT delay setting */
        .u6CMDDLY_ODT1  = 0x20,     /*  MEM_ODT[1] of MEM_ODT delay setting */
        .u6CMDDLY_RSTN0 = 0x20,     /*  MEM_RESET_N[0] delay setting */
        .u6CMDDLY_RSTN1 = 0x20,     /*  MEM_RESET_N[1] delay setting */
        .u4DQSLEADCK    = 0x0       /*  DQS leads CK flag. One bit for each slice. 
                                        0: CK leads DQS, 1: DQS leads CK.*/
    },
    /* DLL Control Register for PHY Command Module - Channel 0  */
    .unDLLCTLCA_CH0.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop. - Channel 0.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable. - Channel 0 */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off. - Channel 0*/
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. - Channel 0*/
        .u8BYPC = 0x0,                      /* DLL bypass code. - Channel 0*/
        .u6CLKDLY = 0x0                     /* Phase shifting for mem_clk: 6'b000000: No shifted 6'b111111: 0.5 cycle */
    },
    /* DLL Control Register for PHY Command Module - Channel 1  */
    .unDLLCTLCA_CH1.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop. - Channel 1.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable. - Channel 1 */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off. - Channel 1 */
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. - Channel 1 */
        .u8BYPC = 0x0,                      /* DLL bypass code. - Channel 1 */
        .u6CLKDLY = 0x0                     /* Phase shifting for mem_clk: 6'b000000: No shifted 6'b111111: 0.5 cycle - Channel 0 */
    },
    /* DLL Control Register for PHY Data Module Slice 0 */
    .unDLLCTLDQ_SL0.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off */
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. */
        .u8BYPC = 0x0                       /* DLL bypass code. */
    },
    /* PHY Data Module IO Control Register  Slice 0 */
    .unDIOR_SL0.stcField = 
    {
        .u3DRVSEL = 0x4,                            /* 3-bit value to select how many copies of driver need to be turned on. Driver Setting 
                                                        (000-turns on 1 driver copy, 001- 2 driver copy, 010- 3 driver copy,..., 101- 6 driver copy) */
        .u1CMOS_EN = CY_LPDDR4_ONOFF_DISABLE,       /* Enable CMOS Receiver in IO */
        .u1FENA_RCV = 0x0,                          /* Driver output enables active high (1: enable; 0: tristate); FENA has higher priority than ODIS and OE 
                                                       when it is high. Enable = FENA + !ODIS & OE  */
        .u1RTT_EN = CY_LPDDR4_ONOFF_ENABLE,        /* Enable RTT compensation switch on on-die termination resistor (RTT) on/off. Here: off.  */
        .u3RTT_SEL = 0x4,                           /* rtt_sel pin is used to vary the RTT impedance when rtt_en=1
                                                       rtt_sel=0 RTT impedance = R/2 [R = External Resistor]
                                                       rtt_sel=1 RTT impedance = R/4 [R = External Resistor] */
        .u8ODIS_DQ = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DM = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DQS = CY_LPDDR4_ODIS_NORMAL         /* When set to 1, the corresponding output bit is forced to high-Z. */
    },
    /* DLL Control Register for PHY Data Module Slice 1 */
    .unDLLCTLDQ_SL1.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off */
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. */
        .u8BYPC = 0x0                       /* DLL bypass code. */
    },

    /* PHY Data Module IO Control Register  Slice 1 */
    .unDIOR_SL1.stcField = 
    {
        .u3DRVSEL = 0x4,                            /* 3-bit value to select how many copies of driver need to be turned on. Driver Setting 
                                                    (000-turns on 1 driver copy, 001- 2 driver copy, 
                                                    010- 3 driver copy,..., 101- 6 driver copy) */
        .u1CMOS_EN = CY_LPDDR4_ONOFF_DISABLE,       /* Enable CMOS Receiver in IO */
        .u1FENA_RCV = 0x0,                          /* Driver output enables active high (1: enable; 0: tristate); FENA has higher priority than ODIS and OE 
                                                       when it is high. Enable = FENA + !ODIS & OE  */
        .u1RTT_EN = CY_LPDDR4_ONOFF_ENABLE,        /* Enable RTT compensation switches on on-die termination resistor (RTT) on/off. Here: off.  */
        .u3RTT_SEL = 0x4,                           /* rtt_sel pin is used to vary the RTT impedance when rtt_en=1
                                                       rtt_sel=0 RTT impedance = R/2 [R = External Resistor]
                                                       rtt_sel=1 RTT impedance = R/4 [R = External Resistor] */
        .u8ODIS_DQ = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DM = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DQS = CY_LPDDR4_ODIS_NORMAL         /* When set to 1, the corresponding output bit is forced to high-Z. */
    },
    /* DLL Control Register for PHY Data Module Slice 2 */
    .unDLLCTLDQ_SL2.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off */
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. */
        .u8BYPC = 0x0                       /* DLL bypass code. */
    },

    /*PHY Data Module IO Control Register  Slice 2*/
    .unDIOR_SL2.stcField = 
    {
        .u3DRVSEL = 0x4,                            /* 3-bit value to select how many copies of driver need to be turned on. Driver Setting 
                                                       (000-turns on 1 driver copy, 001- 2 driver copy, 010- 3 driver copy,..., 101- 6 driver copy) */
        .u1CMOS_EN = CY_LPDDR4_ONOFF_DISABLE,       /* Enable CMOS Receiver in IO */
        .u1FENA_RCV = 0x0,                          /* Driver output enables active high (1: enable; 0: tristate); FENA has higher priority than ODIS and OE 
                                                       when it is high. Enable = FENA + !ODIS & OE  */
        .u1RTT_EN = CY_LPDDR4_ONOFF_ENABLE,        /* Enable RTT compensation this bit switches on on-die termination resistor (RTT) on/off. Here: off.  */
        .u3RTT_SEL = 0x4,                           /* rtt_sel pin is used to vary the RTT impedance when rtt_en=1
                                                       rtt_sel=0 RTT impedance = R/2 [R = External Resistor]
                                                       rtt_sel=1 RTT impedance = R/4 [R = External Resistor] */
        .u8ODIS_DQ = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DM = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DQS = CY_LPDDR4_ODIS_NORMAL         /* When set to 1, the corresponding output bit is forced to high-Z. */
    },
    /* DLL Control Register for PHY Data Module Slice 3 */
    .unDLLCTLDQ_SL3.stcField = 
    {
        .u5LIMIT = 0x5,                     /* Bandwidth for the digital loop.Changed to 5 from 2 as recommended by Hai Anh */
        .u1EN = CY_LPDDR4_ONOFF_ENABLE,     /* DLL enable */
        .u1UPD = CY_LPDDR4_ONOFF_ENABLE,    /* DLL Update. When the PHY is in active operation, DLL update should be turn off */
        .u1BYP = CY_LPDDR4_ONOFF_DISABLE,   /* DLL bypass enable. */
        .u8BYPC = 0x0                       /* DLL bypass code. */
    },
    
    /*PHY Data Module IO Control Register  Slice 3*/
    .unDIOR_SL3.stcField = 
    {
        .u3DRVSEL = 0x4,                            /* 3-bit value to select how many copies of driver need to be turned on. Driver Setting 
                                                       (000-turns on 1 driver copy, 001- 2 driver copy, 010- 3 driver copy,..., 101- 6 driver copy) */
        .u1CMOS_EN = CY_LPDDR4_ONOFF_DISABLE,       /* Enable CMOS Receiver in IO */
        .u1FENA_RCV = 0x0,                          /* Driver output enables active high (1: enable; 0: tristate); FENA has higher priority than ODIS and OE 
                                                       when it is high. Enable = FENA + !ODIS & OE  */
        .u1RTT_EN = CY_LPDDR4_ONOFF_ENABLE,        /* Enable RTT compensation this bit switches on on-die termination resistor (RTT) on/off. Here: off.  */
        .u3RTT_SEL = 0x4,                           /* rtt_sel pin is used to vary the RTT impedance when rtt_en=1
                                                       rtt_sel=0 RTT impedance = R/2 [R = External Resistor]
                                                       rtt_sel=1 RTT impedance = R/4 [R = External Resistor] */
        .u8ODIS_DQ = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DM = CY_LPDDR4_ODIS_NORMAL,         /* When set to 1, the corresponding output bit is forced to high-Z. */
        .u1ODIS_DQS = CY_LPDDR4_ODIS_NORMAL         /* When set to 1, the corresponding output bit is forced to high-Z. */
    },

    /*  PHY Command Module IO Control Register - Channel 0  */
    .unCIOR_CH0.stcField = 
    {
        .u3DRVSEL       =   0x4,                          /*  3-bit value to select how many copies of driver need to be turned on. Driver 
                                                            Setting (000-turns on 1 driver copy, 001- 2 driver copy, 010- 3 driver copy,... 
                                                            ,101- 6 driver copy) - Channel 0*/
        .u1CMOS_EN      =   CY_LPDDR4_ONOFF_DISABLE,    /*  Enable CMOS Receiver in IO - Channel 0 */
        .u1ODIS_RESETN  =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_resetn - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_ODT     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_odt - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CS      =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_cs - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CLK     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_clk - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CKE     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_cke - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u6ODIS_CA      =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_ca - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z*/
    },

    /*  PHY Command Module IO Control Register - Channel 1  */
    .unCIOR_CH1.stcField = 
    {
        .u3DRVSEL       =   0x4,                          /*  3-bit value to select how many copies of driver need to be turned on. Driver 
                                                            Setting (000-turns on 1 driver copy, 001- 2 driver copy, 010- 3 driver copy,... 
                                                            ,101- 6 driver copy) - Channel 1*/
        .u1CMOS_EN      =   CY_LPDDR4_ONOFF_DISABLE,    /*  Enable CMOS Receiver in IO - Channel 0 */
        .u1ODIS_RESETN  =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_resetn - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_ODT     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_odt - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CS      =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_cs - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CLK     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_clk - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u1ODIS_CKE     =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_cke - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z */
        .u6ODIS_CA      =   CY_LPDDR4_ODIS_NORMAL,      /*  Output force disable active high for mem_ca - Channel 0 
                                                            When set to 1, the corresponding output bit is forced to high-Z*/
    },


    /*PHY Sanity Check Address Register */
    .unPTAR.stcField =
    {
        .u3BA = 0x0,            /*  Test bank address */
        .u15ROW = 0x0,          /*  Test row address */
        .u10COL = 0x0           /*  Test column address */
    },

    /*  PHY Compensation Control Register - Channel 0 */
    .unPCCR_CH0.stcField =
    {
        .u1PCCR_CH0_SRST        = CY_LPDDR4_ONOFF_ENABLE,       /*  Compensation soft-reset*/
        .u1PCCR_CH0_TPADEN      = CY_LPDDR4_ONOFF_ENABLE,       /*  Testpad enable*/
        .u1PCCR_CH0_MVG         = CY_LPDDR4_ONOFF_ENABLE,      /*  Enable moving average for compensation - Channel 0 
                                                                    If ZQ calibration is run continuously then setting MVG_EN=1 is recommended. 
                                                                    This turns on the moving average function which prevents the Drive strength 
                                                                    setting to not change by more than 1 setting from one cycle of ZQ calibration to the next. 
                                                                    If noise in the system has for some reason resulted in an anomalous setting, the moving 
                                                                    average function prevents that error to affect the drive strength by not more than a setting. */
        .u1PCCR_CH0_EN          = CY_LPDDR4_ONOFF_ENABLE,       /*  Enable compensation block*/
        .u1PCCR_CH0_UPD         = 0x1,                          /*  Update compensation block*/
        .u1PCCR_CH0_BYPEN       = CY_LPDDR4_ONOFF_DISABLE,                          /*  Bypass internal compensation setting */
        .u4PCCR_CH0_BYP_N       = 0x0,                          /*  Bypass setting*/
        .u4PCCR_CH0_BYP_P       = 0x0,                          /*  Bypass setting */
        .u11PCCR_CH0_INITCNT    = 0x400                         /*  PHY initialization counter always set to 1024 */
    },

    /*  PHY Compensation Control Register - Channel 1 */
    .unPCCR_CH1.stcField =
    {
        .u1PCCR_CH1_SRST        = CY_LPDDR4_ONOFF_ENABLE,       /*  Compensation soft-reset*/
        .u1PCCR_CH1_TPADEN      = CY_LPDDR4_ONOFF_ENABLE,       /*  Testpad enable*/
        .u1PCCR_CH1_MVG         = CY_LPDDR4_ONOFF_ENABLE,       /*  Enable moving average for compensation - Channel 1 
                                                                    If ZQ calibration is run continuously then setting MVG_EN=1 is recommended. 
                                                                    This turns on the moving average function which prevents the Drive strength 
                                                                    setting to not change by more than 1 setting from one cycle of ZQ calibration to the next. 
                                                                    If noise in the system has for some reason resulted in an anomalous setting, the moving 
                                                                    average function prevents that error to affect the drive strength by not more than a setting. */
        .u1PCCR_CH1_EN          = CY_LPDDR4_ONOFF_ENABLE,       /*  Enable compensation block*/
        .u1PCCR_CH1_UPD         = 0x1,                          /*  Update compensation block*/
        .u1PCCR_CH1_BYPEN       = CY_LPDDR4_ONOFF_DISABLE,      /*  Bypass internal compensation setting */
        .u4PCCR_CH1_BYP_N       = 0x0,                          /*  Bypass setting*/
        .u4PCCR_CH1_BYP_P       = 0x0,                          /*  Bypass setting */
        .u11PCCR_CH1_INITCNT    = 0x400                         /*  PHY initialization counter always set to 1024 */
    },
    /*  Inline ECC Register 0 */
    .unINECC0.stcField = 
    {
        .u30ECC_BASE = 0x20000,             /*  ECC Region Base Adddress. This is the region where the ECC 
                                                codes are stored. Size depends on the size of the Protected
                                                Memory Size . Size of the ECC Region =(Protected Region Size/8 ).
                                                When a ECC is configured here then ECC functionality 
                                                must be also enabled in DMCFG Register 
                                                dmcfg_int_ecc_en, dmcfg_inline_ecc_en 
                                                ==> ECC Region Base Address:
                                                = CY_LPDDR4_BASE + INECC1.stcField..u30PROT_MEM_BASE*/
    },
    /*  Inline ECC Register 1 */
    .unINECC1.stcField =
    {
        .u30PROT_MEM_BASE   = 0x00000000,        /*  Protected Region Base Address. Must be aligned
                                                prot_mem_base must be aligned to 256-byte. 
                                                PROT_MEM_BASE[7:0] must be programmed to 0. 
                                                When a ECC is configured here then ECC functionality 
                                                must be also enabled in DMCFG Register 
                                                dmcfg_int_ecc_en, dmcfg_inline_ecc_en. 
                                                ==> Protected Memory Base Address:
                                                = CY_LPDDR4_BASE + INECC1.stcField..u30PROT_MEM_BASE*/
    },
    /*  Inline ECC Register 2 */
    .unINECC2.stcField =
    {
        .u30PROT_MEM_SIZE       = 0x20000        /*  Protected Region Size. Size must be aligned to 256 byte.
                                                PROT_MEM_SIZE[7:0] must be programmed to 0 
                                                When a ECC is configured here then ECC functionality 
                                                must be also enabled in DMCFG Register 
                                                dmcfg_int_ecc_en, dmcfg_inline_ecc_en */

    }

}; 
#endif // CONFIG_02_RD_LLLH_WR_LLLM_1_AC2_H_INCLUDED
