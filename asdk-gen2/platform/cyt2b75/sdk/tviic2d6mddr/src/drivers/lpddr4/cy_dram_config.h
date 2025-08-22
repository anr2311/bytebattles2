/***************************************************************************//**
* \file cy_dram_config.h
*   $Date: 2023-01-24 14:39:41 +0530 (Tue, 24 Jan 2023) $
*   $Revision: 320851 $
*
* File in which user can set some external DRAM Parameter which have influence
* on timing parameter.So user has to check parameters of DRAM which will be 
* used.
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/
#if !defined(__CY_DRAM_CONFIG_H__)
#define __CY_DRAM_CONFIG_H__


#define SVN_REV_CY_DRAM_CONFIG_H   		"$Revision: 320851 $"
#define SVN_DATE_CY_DRAM_CONFIG_H		"$Date: 2023-01-24 14:39:41 +0530 (Tue, 24 Jan 2023) $"


#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

#define DISABLED    0
#define ENABLED     1


/* set the memory density per channel of your selected device
   ___ _                            _      ___               _ _         
  / __\ |__   __ _ _ __  _ __   ___| |    /   \___ _ __  ___(_) |_ _   _ 
 / /  | '_ \ / _` | '_ \| '_ \ / _ \ |   / /\ / _ \ '_ \/ __| | __| | | |
/ /___| | | | (_| | | | | | | |  __/ |  / /_//  __/ | | \__ \ | |_| |_| |
\____/|_| |_|\__,_|_| |_|_| |_|\___|_| /___,' \___|_| |_|___/_|\__|\__, |
                                                                    |___/ */
/*******************************************************************************
    Macros for different channel density in Gb as defined in the JEDEC209-4B
                            CHANNEL_DENSITY                                     
*******************************************************************************/
#define DENSITY_PER_CHANNEL_1GB         0
#define DENSITY_PER_CHANNEL_2GB         1
#define DENSITY_PER_CHANNEL_3GB         2
#define DENSITY_PER_CHANNEL_4GB         3
#define DENSITY_PER_CHANNEL_6GB         4
#define DENSITY_PER_CHANNEL_8GB         5
#define DENSITY_PER_CHANNEL_12GB        6
#define DENSITY_PER_CHANNEL_16GB        7

#define CHANNEL_DENSITY         DENSITY_PER_CHANNEL_4GB     //<---- set Channel Density of your DRAM Device


/*  _   _           _                   _                _   _              
   | | (_)         (_)                 | |              | | (_)             
   | |_ _ _ __ ___  _ _ __   __ _    __| | ___ _ __ __ _| |_ _ _ __   __ _  
   | __| | '_ ` _ \| | '_ \ / _` |  / _` |/ _ \ '__/ _` | __| | '_ \ / _` | 
   | |_| | | | | | | | | | | (_| | | (_| |  __/ | | (_| | |_| | | | | (_| | 
    \__|_|_| |_| |_|_|_| |_|\__, |  \__,_|\___|_|  \__,_|\__|_|_| |_|\__, | 
                             __/ |                                    __/ | 
                            |___/                                    |___/  */
/*******************************************************************************
 *  Macros for different derating strategy
 * 
 *  DERATING_ALWAYS_ON  --> derating is always on 
 * 
 *  DERATING_DYNAMIC    --> timing derating only active when external 
 *                          DRAM requests in MR4 is set
 * 
*******************************************************************************/
#define DERATING_ALWAYS_ON  0 
#define DERATING_DYNAMIC    1

#define DERATING_STRATEGY DERATING_ALWAYS_ON                //<---- set Timing derating strategy here



/******************************************************************* 
   Following Section defines parameters which depend on 
   the Data Density of the used external memory device
   Refer JESD209-4B-LPDDR4  Chapter 4.20.1 Table 44  
   
   User must set the device CHANNEL_DENSITY in cy_dram_config.h 
 ******************************************************************/
#ifndef CHANNEL_DENSITY 
    #error "CHANNEL_DENSITY not defined"
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_1GB )
    /** JEDEC tREFCAB 130 ns*/
    #define JESD209_4B_NS_TREFCAB           130
    /** JEDEC tREFCPB 60 ns*/
    #define JESD209_4B_NS_TREFCPB           60
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_2GB )
    /** JEDEC tREFCAB 130 ns*/
    #define JESD209_4B_NS_TREFCAB           130
    /** JEDEC tREFCPB 60 ns*/
    #define JESD209_4B_NS_TREFCPB           60
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_3GB )
    /** JEDEC tREFCAB 180 ns*/
    #define JESD209_4B_NS_TREFCAB           180
    /** JEDEC tREFCPB 90 ns*/
    #define JESD209_4B_NS_TREFCPB           90
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_4GB )
    /** JEDEC tREFCAB 180 ns*/
    #define JESD209_4B_NS_TREFCAB           180
    /** JEDEC tREFCPB 90 ns*/
    #define JESD209_4B_NS_TREFCPB           90
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_6GB )
    /** JEDEC tREFCAB 280 ns*/
    #define JESD209_4B_NS_TREFCAB           280
    /** JEDEC tREFCPB 140 ns*/
    #define JESD209_4B_NS_TREFCPB           140
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_8GB )
    /** JEDEC tREFCAB 280 ns*/
    #define JESD209_4B_NS_TREFCAB           280
    /** JEDEC tREFCPB 140 ns*/
    #define JESD209_4B_NS_TREFCPB           140
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_12GB )
    /** JEDEC tREFCAB 380 ns*/
    #define JESD209_4B_NS_TREFCAB           380
    /** JEDEC tREFCPB 140 ns*/
    #define JESD209_4B_NS_TREFCPB           190
#elif(CHANNEL_DENSITY == DENSITY_PER_CHANNEL_16GB )
    /** JEDEC tREFCAB 380 ns*/
    #define JESD209_4B_NS_TREFCAB           380
    /** JEDEC tREFCPB 140 ns*/
    #define JESD209_4B_NS_TREFCPB           190 
#elif
    error "unknown CHANNEL_DENSITY"
#endif

  /* Set the frequency your PSVP is running with  
     _____   _______      _______    ______              
    |  __ \ / ____\ \    / /  __ \  |  ____|             
    | |__) | (___  \ \  / /| |__) | | |__ _ __ ___  __ _ 
    |  ___/ \___ \  \ \/ / |  ___/  |  __| '__/ _ \/ _` |
    | |     ____) |  \  /  | |      | |  | | |  __/ (_| |
    |_|    |_____/    \/   |_|      |_|  |_|  \___|\__, |
                                                      | |
                                                      |_|*/
#define PSVP_FREQUENCY_69P6MHZ  69600000    /* 69.6 MHz */
#define PSVP_FREQUENCY_72MHZ    72000000    /* 72   MHz */
#define PSVP_FREQUENCY_75P2MHZ  75200000    /* 75.2 MHz */
#define PSVP_FREQUENCY_80MHZ    80000000    /* 80   MHz */


#define PSVP_FREQUENCY PSVP_FREQUENCY_75P2MHZ    /*<---- set PSVP Frequency here depends on FPGA Bitfile*/

/** Acticate to calculate  FSP0_USER_FREQUENCY_HZ and FSP0_USER_FREQUENCY_HZ based on PLL settings */
#define USE_PLL_FREQ_CALCULATION ENABLED 


#define USE_PLL_FREQ_400    1
#define USE_PLL_FREQ_496    2
#define USE_PLL_FREQ_552    3
#define USE_PLL_FREQ_600    4
#define USE_PLL_FREQ_696    5
#define USE_PLL_FREQ_720    6
#define USE_PLL_FREQ_736    7
#define USE_PLL_FREQ_744    8
#define USE_PLL_FREQ_752    9
#define USE_PLL_FREQ_800    10

#define USE_PLL_FREQ_MHZ        USE_PLL_FREQ_752
 

#if(USE_PLL_FREQ_CALCULATION==ENABLED)
/*LPDDR4 PLL Settings */
    //FSP[0]=50 MHz  and FSP[1]= 400 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_400)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         50          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     8          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     8          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=49.6 MHz  and FSP[1]= 496 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_496)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         62          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     10          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     10          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=50.18 MHz  and FSP[1]= 552 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_552)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         69          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     11          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     11          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=54.55 MHz  and FSP[1]= 600 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_600)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         75          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     11          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     11          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=53.54 MHz  and FSP[1]= 696 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_696)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         87          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     13          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     13          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif    

    //FSP[0]=51.43 MHz  and FSP[1]= 720 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_720)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         90          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     14          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     14          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=52.57 MHz  and FSP[1]= 736 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_736)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         92          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     14          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     14          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=53.14 MHz  and FSP[1]= 744 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_744)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         93          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     14          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     14          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=53.71 MHz  and FSP[1]= 752 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_752)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         94          //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     14          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     14          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    //FSP[0]=50 MHz  and FSP[1]= 800 MHz
    #if (USE_PLL_FREQ_MHZ == USE_PLL_FREQ_800)
        #define LPDDR4_PLL_FIN_HZ           16000000    //  PLL Input Frequency (CLK_HF14 )                 <--set PLL input freq.
        #define LPDDR4_PLL_REF_DIV          2           //  PLL Reference Divider  valid values 1..18       <--set PLL Reference Divider 
        #define LPDDR4_PLL_FDBK_DIV         100         //  PLL  Feedback Divider  valid values 16..200     <--set PLL Fdbk Divider 
        #define LPDDR4_PLL_OUT_DIV_FSP0     16          //  OutPut Divider for FSP[0] frequency             <--set PLL Output Divder for FSP[0]
                                                        //  valid values 2..16 
        #define LPDDR4_PLL_OUT_DIV_FSP1     16          //  OUTPUT_DIV for FSP[1] is actually don´t         <-- set PLL Output Divder for FSP[1]
                                                        //  (this is actually a don´t care) might be deleted later valid values 2..16 
    #endif

    #define FSP0_USER_FREQUENCY_HZ     LPDDR4_PLL_FIN_HZ/LPDDR4_PLL_REF_DIV*LPDDR4_PLL_FDBK_DIV/LPDDR4_PLL_OUT_DIV_FSP0
    #define FSP1_USER_FREQUENCY_HZ     LPDDR4_PLL_FIN_HZ/LPDDR4_PLL_REF_DIV*LPDDR4_PLL_FDBK_DIV

#endif

/** uncomment if you want to calculate values for IOP
 * then the frequency for calculation can be set in 
 * FSP0_USER_FREQUENCY_HZ, FSP1_USER_FREQUENCY_HZ 
 * Must be 0 for LPDDR4 running correctly on PSVP or SOC but 
 * can be set to 1 to calculate a IOP config with PSVP */
//#define CALCULATE_IOP_CONFIG



/*******************************************************************************************
 * set the FSP0 and FSP1 frequency ( for real silicon or simulation ), this setting has no 
 * effect on PSVP as long as CALCULATE_IOP_CONFIG == 0
 * These macro defines are only taken into account if the frequencies where not 
 * passed as compile time parameter with compiler flag -D and a none PSVP HW 
 * is in use . 
 *          _     ___ ___ ___   __    __    __     __  ___ ___ ___   __   _   __ 
 *  ___ ___| |_  | __/ __| _ \ | _|  /  \  |_ |   / / | __/ __| _ \ | _| / | |_ |
 * (_-</ -_)  _| | _|\__ \  _/ | |  | () |  | |  / /  | _|\__ \  _/ | |  | |  | |
 * /__/\___|\__| |_| |___/_|   | |   \__/   | | /_/   |_| |___/_|   | |  |_|  | |
 *                             |__|        |__|                     |__|     |__|
 ******************************************************************************************/
/*check if frequencies for FSP[0]/ FSP[1] were set using compile time parameter -D xxx
 * if so print error message and set both frequencies to 80MHz as PSVP only supports FSP[0]=FSP[1]=80MHz */
#ifndef CALCULATE_IOP_CONFIG
    /*For PSVP --> both frequencies ar fixed to 80MHz and can not be changed.*/
    #if(CY_USE_PSVP == 1)
            
            #ifdef CY_LPDDR4_FSP0_FREQ
                /*if someone uses compile time parameter -D undefine first*/
                #undef CY_LPDDR4_FSP0_FREQ
                #warning "PSVP only supports fixed settings please set PSVP_FREQUENCY in cy_dram_config.h"
                #warning "to avoid this errormsg "
                /** PSVP uses fixed clock 80MHz*/
            #endif
            #define CY_LPDDR4_FSP0_FREQ     PSVP_FREQUENCY
            #ifdef CY_LPDDR4_FSP1_FREQ
                /*if someone uses compile time parameter -D undefine first*/
                #undef CY_LPDDR4_FSP1_FREQ
               #warning "PSVP only supports fixed settings please set PSVP_FREQUENCY in cy_dram_config.h"
                #warning "comment out CALCULATE_IOP_CONFIG in cy_dram.config.h if you are not calculating a IOP config"
                /** PSVP uses fixed clock 80MHz*/
            #endif
            #define CY_LPDDR4_FSP1_FREQ     PSVP_FREQUENCY

    #elif(CY_USE_PSVP == 0)
        /* If real silicon is used, user can set the FSP[0] (FSP0_USER_FREQUENCY_HZ) / FSP[1](FSP1_USER_FREQUENCY_HZ) 
           frequencies in cy_dram_config.h If frequencies were passes as compile time parameter 
           with -D they will be kept. Based on this all timing related parameters will be correctly set. 
           PLL dividers must be set manually in the lpddr_config structure. */
    
        /*check if frequency was set using compile time parameter using -D keep this value 
          otherwise default to the values set in cy_dram_config.h */
        #ifndef CY_LPDDR4_FSP0_FREQ
            #define CY_LPDDR4_FSP0_FREQ     FSP0_USER_FREQUENCY_HZ    //freq of FSP[0] in [Hz]
        #endif
        #ifndef CY_LPDDR4_FSP1_FREQ
            #define CY_LPDDR4_FSP1_FREQ     FSP1_USER_FREQUENCY_HZ   //freq of FSP[1] in [Hz] 
        #endif
    #endif
#endif

/* you want to calculate a new IOP CONFIG setting using real HW or PSVP
 * uncomment //define CALCULATE_IOP_CONFIG in cy_dram_config.h
 * PLL settings must be calucalted manually all other timing parameter will be
 * calculated automatically. The resulting config might not run on your current setup
 * only activate CALCULATE_IOP_CONFIG if you know what you are doing 
 * */
#ifdef CALCULATE_IOP_CONFIG
    #ifndef CY_LPDDR4_FSP0_FREQ
        #define CY_LPDDR4_FSP0_FREQ     FSP0_USER_FREQUENCY_HZ    //freq of FSP[0] in [Hz]
        #warning "CALCULATE_IOP_CONFIG ACTIVATED in cy_dram_config.h !!! Config might not work on your current hardware"
    #endif
    #ifndef CY_LPDDR4_FSP1_FREQ
        #define CY_LPDDR4_FSP1_FREQ     FSP1_USER_FREQUENCY_HZ   //freq of FSP[1] in [Hz] 
        #warning "CALCULATE_IOP_CONFIG ACTIVATED in cy_dram_config.h!!! Config might not work on your current hardware"
    #endif
#endif

#if(USE_PLL_FREQ_CALCULATION==DISABLED)
#ifndef FSP0_USER_FREQUENCY_HZ
    #define FSP0_USER_FREQUENCY_HZ      53538461      //Frequency for FSP[0] in Hz  <--- set FSP0 Frequency if not set by compile option -D or USE_PLL_FREQ_CALCULATION==ENABLED
#endif
#ifndef FSP1_USER_FREQUENCY_HZ
    #define FSP1_USER_FREQUENCY_HZ     752000000     //Frequency for FSP[1] in Hz   <--- set FSP1 Frequency if not set by compile option -D or USE_PLL_FREQ_CALCULATION==ENABLED
#endif
#endif

/*  ____                 _     _                      _   _     
   |  _ \               | |   | |                    | | | |    
   | |_) |_   _ _ __ ___| |_  | |     ___ _ __   __ _| |_| |__  
   |  _ <| | | | '__/ __| __| | |    / _ \ '_ \ / _` | __| '_ \ 
   | |_) | |_| | |  \__ \ |_  | |___|  __/ | | | (_| | |_| | | |
   |____/ \__,_|_|  |___/\__| |______\___|_| |_|\__, |\__|_| |_|
                                                 __/ |          
                                                |___/          */
#define BL16            16  
#define BL32            32
/*set the BURST_LENGTH down below here          */
#define BL     BL16                             //<----Set Burst Length 
/* Masked Write is only supported for BL16 so when changing to BL32 
   check for MWR disabled unRTCFG0_RTx.stcField.u1WM_ENABLE */


/* _______        _       _             
  |__   __|      (_)     (_)            
     | |_ __ __ _ _ _ __  _ _ __   __ _ 
     | | '__/ _` | | '_ \| | '_ \ / _` |
     | | | | (_| | | | | | | | | | (_| |
     |_|_|  \__,_|_|_| |_|_|_| |_|\__, |
                                   __/ |
                                  |___/ */
#define TRAINING_ALL_AT_ONCE     0
#define TRAINING_STEPWISE        1
// defines if the neccessary training are triggered all at once or step by step (for testing purposes)
#define LPDDR4_TRAINING        TRAINING_ALL_AT_ONCE



/* __          __   _ _         _           _                           _____      _   
   \ \        / /  (_) |       | |         | |                         / ____|    | |  
    \ \  /\  / / __ _| |_ ___  | |     __ _| |_ ___ _ __   ___ _   _  | (___   ___| |_ 
     \ \/  \/ / '__| | __/ _ \ | |    / _` | __/ _ \ '_ \ / __| | | |  \___ \ / _ \ __|
      \  /\  /| |  | | ||  __/ | |___| (_| | ||  __/ | | | (__| |_| |  ____) |  __/ |_ 
       \/  \/ |_|  |_|\__\___| |______\__,_|\__\___|_| |_|\___|\__, | |_____/ \___|\__|
                                                                __/ |                  
                                                               |___/                */
/***** Write Latency Set used in MR2 (LPMR2) ********/
#define WL_SET_A    0
#define WL_SET_B    1
/*set Write Latency Set down below here             */
#define FS1_WLS      WL_SET_A                       //<---- Set Write Latency Set
/****************************************************/
#if (FS1_WLS == WL_SET_A)
    #define WL  CY_LPDDR_WRITELATENCY_FSP1_SETA 
#elif(FS1_WLS == WL_SET_B)
    #define WL  CY_LPDDR_WRITELATENCY_FSP1_SETB
#endif


/* _____  _____  ____ _____ 
  |  __ \|  __ \|  _ \_   _|
  | |__) | |  | | |_) || |  
  |  _  /| |  | |  _ < | |  
  | | \ \| |__| | |_) || |_ 
  |_|  \_\_____/|____/_____|*/
/********* Read Data Byte Inversion *************/
#define RDBI_DISABLED   0 
#define RDBI_ENABLED    1
/*set READ Data Byte Inversion down below here  */
#define FS1_RDBI        RDBI_ENABLED           //<---- set RDBI Mode
/************************************************/
#if (FS1_RDBI == RDBI_DISABLED)
    #define RL   CY_LPDDR_READLATENCY_FSP1_NODBI
#elif(FS1_RDBI == RDBI_ENABLED)
    #define RL   CY_LPDDR_READLATENCY_FSP1_DBI
#endif



/* __          _______  ____ _____ 
   \ \        / /  __ \|  _ \_   _|
    \ \  /\  / /| |  | | |_) || |  
     \ \/  \/ / | |  | |  _ < | |  
      \  /\  /  | |__| | |_) || |_ 
       \/  \/   |_____/|____/_____|*/
/********* Write Data Byte Inversion *************/
#define WDBI_DISABLED   0 
#define WDBI_ENABLED    1
/*set READ Data Byte Inversion down below here  */
#define FS1_WDBI        WDBI_ENABLED           //<---- set WDBI Mode
/************************************************/



/***************** Read POST-Amble *****************/
#define READ_POST_AMBLE_0P5_NCLK     0 
#define READ_POST_AMBLE_1P5_NCLK     1
//-----> set the Read Post-amble for FSP1 --> MR1 OP[7]
#define READ_POST_AMBLE  READ_POST_AMBLE_0P5_NCLK   //<---- set Read POST-AMBLE Typ
/***************************************************/

/* calculate JEDEC Parameter tRPST which depends on user settings above */

#if(READ_POST_AMBLE == READ_POST_AMBLE_0P5_NCLK)
    #define JESD209_4B_NCLK_TRPST       0.5
    #define JESD209_4B_NS_TRPST         JESD209_4B_NS_TRPST_MIN_0P5_PSTA 
#elif(READ_POST_AMBLE == READ_POST_AMBLE_1P5_NCLK)
    #define JESD209_4B_NCLK_TRPST       1.5
    #define JESD209_4B_NS_TRPST         JESD209_4B_NS_TRPST_MIN_1P5_PSTA
#endif

/***************** Write POST-Amble ****************/
#define WRITE_POST_AMBLE_0P5_NCLK     0
#define WRITE_POST_AMBLE_1P5_NCLK     1
/*set the correct write Post-amble for FSP1 --> MR3 OP[1]*/
#define WRITE_POST_AMBLE  WRITE_POST_AMBLE_0P5_NCLK //<---- set Write POST-AMBLE Typ
/***************************************************/

/* calculate JEDEC Parameter tWPST which depends on user settings above */
#if(WRITE_POST_AMBLE == WRITE_POST_AMBLE_0P5_NCLK)
    #define JESD209_4B_NCLK_TWPST       0.5
    #define JESD209_4B_NS_TWPST         JESD209_4B_NS_TWPST_MIN_0P5_PSTA
#elif(WRITE_POST_AMBLE == WRITE_POST_AMBLE_1P5_NCLK)
    #define JESD209_4B_NCLK_TWPST       1.5
    #define JESD209_4B_NS_TWPST         JESD209_4B_NS_TWPST_MIN_1P5_PSTA
#endif


/* set ZQ Calibration Interval in nsec */
#define ZQCAL_INTERVAL_NSEC  0x01E84800    //<---- set ZQ Calibration Interval 32msec=32000000nsec

/*********************************************************************************
 * Power- up, Initialization values for Mode Registers aka MR FSP[0] default values
 * Refer JEDEC209-4B-LPDDR4
 * Chapter 3.3
 * Table 6 — Initialization Timing Parameters
 * 
 *   FSP-OP/WR           MR13 OP[7:6]    00B FSP-OP/WR[0] are enabled
 *   WLS                 MR2 OP[6]       0B Write Latency Set 0 is selected
 *   WL                  MR2 OP[5:3]     000B WL = 4
 *   RL                  MR2 OP[2:0]     000B RL = 6, nRTP=8
 *   nWR                 MR1 OP[6:4]     000B nWR = 6
 *   DBI-WR/RD           MR3 OP[7:6]     00B Write & Read DBI are disabled
 *   CA ODT              MR11 OP[6:4]    000B CA ODT is disabled
 *   DQ ODT              MR11 OP[2:0]    000B DQ ODT is disabled
 *   VREF(CA) Setting    MR12 OP[6]      1B VREF(CA) Range[1] enabled
 *   VREF(CA) Value      MR12 OP[5:0]    001101B Range1 : 27.2% of VDD2
 *   VREF(DQ) Setting    MR14 OP[6]      1B VREF(DQ) Range[1] enabled
 *   VREF(DQ) Value      MR14 OP[5:0]    001101B Range1 : 27.2% of VDDQ
 ***********************************************************************************/
#define JESD209_4B_FSP0_MR2_DEFAULT_WLS             CY_LPDDR4_WLS_WL_SET_A
#define JESD209_4B_FSP0_MR2_DEFAULT_WL              0x0
#define JESD209_4B_FSP0_MR2_DEFAULT_RL              0x0
#define JESD209_4B_FSP0_MR1_DEFAULT_NWR             CY_LPDDR4_NWR_NWR6
#define JESD209_4B_FSP0_MR3_DEFAULT_WRDBI           CY_LPDDR4_ONOFF_DISABLE
#define JESD209_4B_FSP0_MR3_DEFAULT_RDDBI           CY_LPDDR4_ONOFF_DISABLE
#define JESD209_4B_FSP0_MR11_DEFAULT_CA_ODT         CY_LPDDR4_ODT_DISABLED
#define JESD209_4B_FSP0_MR11_DEFAULT_DQ_ODT         CY_LPDDR4_ODT_DISABLED
#define JESD209_4B_FSP0_MR12_DEFAULT_VREFCA_RANGE   CY_LPDDR4_VREF_RANGE_RANGE1
#define JESD209_4B_FSP0_MR12_DEFAULT_VREFCA_VALUE   0xd
#define JESD209_4B_FSP0_MR13_DEFAULT_FSPWR          CY_LPDDR4_FSP0
#define JESD209_4B_FSP0_MR13_DEFAULT_FSPOP          CY_LPDDR4_FSP0
#define JESD209_4B_FSP0_MR14_DEFAULT_VREFDQ_RANGE   CY_LPDDR4_VREF_RANGE_RANGE1
#define JESD209_4B_FSP0_MR14_DEFAULT_VREFDQ_VALUE   0xd

/**
 * u17T_INIT3 limit for TINIT3.As with the slow PSVP Clock frequency 
 * the u17T_INIT3 field in Timing Register can not hold the clock cycles it 
 * must be limited 2^17Bit =
 **/
#define CY_LPDDR_INIT3_MAX_LIMIT    0x1FFFF

/**
 * u7T_INIT5 limit for TINIT5.As with the slow PSVP Clock frequency 
 * the u7T_INIT5 field in Timing Register can not hold the clock cycles it 
 * must be limited
 **/
#define CY_LPDDR_INIT5_MAX_LIMIT 0x7F

/******************************************************************************
 * 
 * Macro defintions for Read-/Write Latency  (RL/WL)  depending on 
 * Data Byte Inversion DBI activation status and FSP1 frequency set point. 
 * Refer JEDEC 209-4B 
 * Chapter 4.12 Read and Write Latencies 
 * Table 28 — Read and Write Latencies
 *******************************************************************************/
/*                       10MHz < freq < 266MHz                       */
#if((CY_LPDDR4_FSP1_FREQ > 10000000)&&(CY_LPDDR4_FSP1_FREQ <= 266000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     6
    #define CY_LPDDR_READLATENCY_FSP1_DBI       6
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x0
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     4
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     4
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x0
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR6
    #define CY_LPDDR_NWR_CLKCNT_FSP1            6
    #define CY_LPDDR_NRTP_FSP1                  8
/*                       266MHz < freq < 533MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 266000000)&&(CY_LPDDR4_FSP1_FREQ <= 533000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     10
    #define CY_LPDDR_READLATENCY_FSP1_DBI       12
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x1    
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     6
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     8
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x1
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR10
    #define CY_LPDDR_NWR_CLKCNT_FSP1            10
    #define CY_LPDDR_NRTP_FSP1                  8
/*                       533MHz < freq < 800MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 533000000)&&(CY_LPDDR4_FSP1_FREQ <= 800000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     14
    #define CY_LPDDR_READLATENCY_FSP1_DBI       16
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x2
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     8
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     12
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x2
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR16
    #define CY_LPDDR_NWR_CLKCNT_FSP1            16
    #define CY_LPDDR_NRTP_FSP1                  8
/*                       800MHz < freq < 1066MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 800000000)&&(CY_LPDDR4_FSP1_FREQ <= 1066000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     20
    #define CY_LPDDR_READLATENCY_FSP1_DBI       22
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x3
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     10
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     18
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x3
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR20
    #define CY_LPDDR_NWR_CLKCNT_FSP1            20
    #define CY_LPDDR_NRTP_FSP1                  8
/*                       1066MHz < freq < 1333MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 1066000000)&&(CY_LPDDR4_FSP1_FREQ <= 1333000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     24
    #define CY_LPDDR_READLATENCY_FSP1_DBI       28
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x4
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     12
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     22
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x4
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR24
    #define CY_LPDDR_NWR_CLKCNT_FSP1            24
    #define CY_LPDDR_NRTP_FSP1                  10
/*                       1333MHz < freq < 1600MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 1333000000)&&(CY_LPDDR4_FSP1_FREQ <= 1600000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     28
    #define CY_LPDDR_READLATENCY_FSP1_DBI       32
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x5
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     14
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     26
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x5
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR30
    #define CY_LPDDR_NWR_CLKCNT_FSP1            30
    #define CY_LPDDR_NRTP_FSP1                  12
/*                       1600MHz < freq < 1866MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 1600000000)&&(CY_LPDDR4_FSP1_FREQ <= 1866000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     32
    #define CY_LPDDR_READLATENCY_FSP1_DBI       36
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x6
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     16
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     30
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x6
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR34
    #define CY_LPDDR_NWR_CLKCNT_FSP1            34
    #define CY_LPDDR_NRTP_FSP1                  14
/*                       1866MHz < freq < 2133MHz                       */
#elif((CY_LPDDR4_FSP1_FREQ > 1866000000)&&(CY_LPDDR4_FSP1_FREQ <= 2133000000)) 
    #define CY_LPDDR_READLATENCY_FSP1_NODBI     36
    #define CY_LPDDR_READLATENCY_FSP1_DBI       40
    #define CY_LPDDR_READLATENCY_FSP1_LPMR2     0x7
    #define CY_LPDDR_WRITELATENCY_FSP1_SETA     18
    #define CY_LPDDR_WRITELATENCY_FSP1_SETB     34
    #define CY_LPDDR_WRITELATENCY_FSP1_LPMR2    0x7
    #define CY_LPDDR_NWR_FSP1                   CY_LPDDR4_NWR_NWR40
    #define CY_LPDDR_NWR_CLKCNT_FSP1            40
    #define CY_LPDDR_NRTP_FSP1                  16
#endif


/** The frequency to determine the DATA_RATE as it is DDR Memory 2*freq */
#define DATA_RATE_FREQUENCY  CY_LPDDR4_FSP1_FREQ * 2


/*******************************************************************************
    Macros for different data rates as defined in the JEDEC209-4B
                                DATA_RATE
*******************************************************************************/
#define DATA_RATE_533                   1
#define DATA_RATE_1066                  2
#define DATA_RATE_1600                  3
#define DATA_RATE_2133                  4
#define DATA_RATE_2667                  5
#define DATA_RATE_3200                  6
#define DATA_RATE_3733                  7
#define DATA_RATE_4267                  8

/*******************************************************************************
    Macros for different data rates as defined in the JEDEC209-4B
                                DATA_RATE
*******************************************************************************/
#define DATA_RATE_FREQUENCY_533          533000000      /* Frequency in [Hz] -  533 MHz*/
#define DATA_RATE_FREQUENCY_1066        1066000000      /* Frequency in [Hz] - 1066 MHz*/
#define DATA_RATE_FREQUENCY_1600        1600000000      /* Frequency in [Hz] - 1600 MHz*/
#define DATA_RATE_FREQUENCY_2133        2133000000      /* Frequency in [Hz] - 2133 MHz*/
#define DATA_RATE_FREQUENCY_2667        2667000000      /* Frequency in [Hz] - 2667 MHz*/
#define DATA_RATE_FREQUENCY_3200        3200000000      /* Frequency in [Hz] - 3200 MHz*/
#define DATA_RATE_FREQUENCY_3733        3733000000      /* Frequency in [Hz] - 3733 MHz*/
#define DATA_RATE_FREQUENCY_4267        4267000000      /* Frequency in [Hz] - 4267 MHz*/


#if (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_533)
    #define DATA_RATE  DATA_RATE_533
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_533) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_1066))
    #define DATA_RATE  DATA_RATE_1066
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_1066) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_1600))
    #define DATA_RATE  DATA_RATE_1600
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_1600) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_2133))
    #define DATA_RATE  DATA_RATE_2133
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_2133) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_2667))
    #define DATA_RATE  DATA_RATE_2667
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_2667) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_3200))
    #define DATA_RATE  DATA_RATE_3200
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_3200) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_3733))
    #define DATA_RATE  DATA_RATE_3733
#elif ((DATA_RATE_FREQUENCY>DATA_RATE_FREQUENCY_3733) && (DATA_RATE_FREQUENCY<=DATA_RATE_FREQUENCY_4267))
    #define DATA_RATE  DATA_RATE_4267
#endif


/******************************************************************* 
   Following Section defines parameters which depend on 
   the Data Rate the external memory device is ruunning on. 
   Data Rate is directly depending on FSP1 frequency (2*FSP1)
 
   Refer JESD209-4B-LPDDR4
   Chapter 4.3 Core Timing Table 17 — Core AC Timing
   
   User must set the device FSP0_USER_FREQUENCY_HZ,FSP1_USER_FREQUENCY_HZ
   in cy_dram_config.h. For PSVP this has no effect as PSVP is 
   always runnning @80MHz
 ******************************************************************/
#ifndef DATA_RATE 
    #error "DATA_RATE not defined use compiler Option -D DATA_RATE=..... "
#elif(DATA_RATE == DATA_RATE_533)
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4
#elif(DATA_RATE == DATA_RATE_1066)
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4
#elif(DATA_RATE == DATA_RATE_1600)
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4
#elif(DATA_RATE == DATA_RATE_2133)
    /* JEDEC tRRD min=max(10ns,4nCK)*/
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4

#elif(DATA_RATE == DATA_RATE_2667)
    /* JEDEC tRRD min=max(10ns,4nCK)*/
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4

#elif(DATA_RATE == DATA_RATE_3200)
    /* JEDEC tRRD min=max(10ns,4nCK)*/
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4
#elif(DATA_RATE == DATA_RATE_3733)
    /* JEDEC tRRD min=max(10ns,4nCK)*/
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              10
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4

#elif(DATA_RATE == DATA_RATE_4267)
    /* JEDEC tRRD min=max(10ns,4nCK)*/
    /** tRRD value in nsec. as specified in JEDEC 209-4B  */
    #define JESD209_4B_NS_TRRD              7.5
    /** tRRD value in clock cycles as specified in JEDEC 209-4B  */
    #define JESD209_4B_NCLK_TRRD            4
#elif
    error "unknown DATA_RATE"
#endif


/** tFCshort value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TFCSHORT      200

/** tFCmiddle value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TFCMIDDLE     200

/** tFClong value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TFCLONG       250

/** tVREFCAlong value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TVREFCALONG   250

/** tVREFCAshort value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TVREFCASHORT  80

/** JEDEC tCKEHDQS = 10ns */
#define JESD209_4B_NS_TCKEHDQS          10

/** JEDEC tFAW = 40ns */
#define JESD209_4B_NS_TFAW      40

/** JEDEC tCCDMW In the case of BL = 16, tCCD is 8*tCK,
    tCCDMW = 32*tCK (4*tCCD at BL=16)*/
#define JESD209_4B_NCLK_tCCDMW_BL16      32

/** JEDEC tCCDMW In the case of BL = 32, tCCD is 8*tCK,
 * Write with BL=32 operation has 8*tCK longer than BL =16
    tCCDMW = 32*tCK + 8  */
#define JESD209_4B_NCLK_tCCDMW_BL32      40

/** JEDEC tWLDQSEN = 20 nCLK */
#define JESD209_4B_NCLK_TWLDQSEN      20

/** JEDEC209-4B ZQCAL 1us ==> 1000 ns */
#define JESD209_4B_NS_TZQCAL      1000

/** JEDEC209-4B tINIT1 = 200us ==> 200000ns */
#define JESD209_4B_NS_TINIT1    200000

/** JEDEC defines  min. tINIT3 = 2ms ==> 2000000ns 
 * set to 2.1ms as with 2ms we were border line
 * tINIT3 is min value so no issue to slightly increase the value */
#define JESD209_4B_NS_TINIT3    2100000

/** JEDEC209-4B tINIT5 = 2us ==> 2000 ns*/
#define JESD209_4B_NS_TINIT5    2000

/** JEDEC209-4B  max. tVRCG_ENABLE = 200ns*/
#define JESD209_4B_NS_MAX_TVRCG_ENABLE      200

/** JEDEC209-4B max tVRCG_DISABLE ==> 100 ns*/
#define JESD209_4B_NS_MAX_TVRCG_DISABLE     100

/** JEDEC209-4B max tDStrain defined as 2ns */
#define JESD209_4B_NS_MAX_TDSTRAIN    2

/** JEDEC209-4B max tDHtrain are both defined as 2ns */
#define JESD209_4B_NS_MAX_TDHTRAIN    2

/** JEDEC209-4B max tDStrain and tDHtrain are both defined as 2ns */
#define JESD209_4B_NS_MAX_TDTRAIN    JESD209_4B_NS_MAX_TDSTRAIN

/** JEDEC209-4B tWLMRD min 40tCK */
#define JESD209_4B_NCLK_TWLMRD      40

/** JEDEC209-4B tCAENT min 250 ns */
#define JESD209_4B_NS_TCAENT          250

/** JEDEC209-4B tCCD min 8nCK for BL16*/
#define JESD209_4B_NCLK_TCCD_BL16        8

/** JEDEC209-4B tCCD min 8nCK for BL32*/
#define JESD209_4B_NCLK_TCCD_BL32        16

/*For WRITE operations, a 2*tCK pre-amble is required at all operating frequencies.*/
#define JESD209_4B_NCLK_TWPRE       2

/*For READ operations the pre-amble is 2*tCK, but the pre-amble is static (no-toggle) or toggling, selectable
via mode register.JEDEC.209-4B Chapter 4.5*/
#define JESD209_4B_NCLK_TRPRE       2

/** JEDEC tMRR = 8nCK */
#define JESD209_4B_NCLK_TMRR    8

/** JEDEC tDQSCKE = 10 ns*/
#define JESD209_4B_NS_TDQSCKE   10

/** JEDEC tDQSCKmin = 1.5 ns*/
#define JESD209_4B_NS_TDQSCK_MIN   1.5

/** JEDEC tDQSCKmax = 3.5 ns*/
#define JESD209_4B_NS_TDQSCK_MAX   3.5


/*************************************************************/

/** JEDEC tRTP = MAX(7.5ns, 8nCK) */
/** tRTP value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRTP      7.5
/** tRTP value in clock cyles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TRTP        8


/* JEDEC tSR table 17 min=max(15ns, 3nCK)*/
/** tSR value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TSR               15
/** tSR value in clock cycles as specified in JEDEC 209-4B  */ 
#define JESD209_4B_NCLK_TSR             3



/* JEDEC tCKE min = Max(7.5ns,4nCK)*/
/** tCKE value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCKE              7.5
/** tCKE value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCKE            4

/* JEDEC tRCD min=max(18ns,4nCK)*/
/** tRCD value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRCD              18
#define JESD209_4B_NCLK_TRCD            4
/** tRCD value for hot temp derated in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRCD_DERATED      (18+1.875)

/** tRP in [ns] as defined in JEDEC209-4B without derating tRP = tRPpb >= MAX(18ns, 4nCK)only per bank 
 * is supported CDT 374734 */
#define JESD209_4B_NS_TRP           18
/** tRP in [ns] as defined in JEDEC when derating is active  */
#define JESD209_4B_NS_TRP_DERATED   (18+1.875)
/** tRP as defined in JEDEC in clk cycles */
#define JESD209_4B_NCLK_TRP         4

/* JEDEC Row active time tRAS min = max(42ns, 3nCK)*/
/** tRAS value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRAS      42
/** tRAS derated for hot temp */
#define JESD209_4B_NS_TRAS_DERATED      (42+1.875)
/** tRAS value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TRAS     3

/* JEDEC209-4B Mode register set command delay tMRD min = max(14ns, 10nCK)*/
/** tMRD value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TMRD       14
/** tMRD value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TMRD     10

/* JEDEC209-4B ZQ Calibration Latch Time TZQLAT min = max(30ns, 8nCK)*/
/** TZQLAT value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TZQLAT      30

/** TZQLAT value in clock cycles as specified in JEDEC 209-4B  */ 
#define JESD209_4B_NCLK_TZQLAT     8

/* JEDEC209-4B ZQRESET min = max(50ns, 3nCK)*/
/** ZQRESET value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TZQRESET      50   
/** ZQRESET value in clock cycles as specified in JEDEC 209-4B  */ 
#define JESD209_4B_NCLK_TZQRESET     3

/* JEDEC209-4B tCKCKEH min = max(1.75ns, 3nCK)*/
/** tCKCKEH value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCKCKEH     1.75
/** tCKCKEH value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCKCKEH      3

/** JEDEC209-4B tREFI = 3.904 us*/
#define JESD209_4B_NS_TREFI     3904
/* to fullfill the refresh requirement also with high data throughput
 * tREFI must be corrected as ongoing burst must be finished before being able to issue a refresh.
 * As a already started burst can not be stopped. So controller must finish the ongoing 
 * burst before refresh can be issued  tREFI-(4 + WL + (BL/2) + 1) */
#define CY_LPDDR4_TREFI_NCK_COMPENSATE  (4 + WL + BL/2 + 1)


/* JEDEC tOSCO = Max(40ns,8nCK) */
/** tOSCO value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TOSCO         40   
/** tOSCO value in clock cycles as specified in JEDEC 209-4B  */ 
#define JESD209_4B_NCLK_TOSCO        8

/** JEDEC tDQSCKmax (derated 105°C) = 3.6 ns*/
#define JESD209_4B_NS_TDQSCK_MAX_DERATED   3.6

/** tRPST for 0.5 Clk Read Post-Amble  */
#define JESD209_4B_NS_TRPST_MIN_0P5_PSTA        CY_LPDDR4_CLOCKCYCLES(0.4,CY_LPDDR4_FSP1_FREQ)

/** tRPST for 1.5 Clk Read Post-Amble  */
#define JESD209_4B_NS_TRPST_MIN_1P5_PSTA        CY_LPDDR4_CLOCKCYCLES(1.4,CY_LPDDR4_FSP1_FREQ)

/** tRPRE Read Pre-Amble time  1.8nCLK */
#define JESD209_4B_NS_TRPRE                     CY_LPDDR4_CLOCKCYCLES(1.8,CY_LPDDR4_FSP1_FREQ)

/** tWPST for 0.5 Clk Write Post-Amble  */
#define JESD209_4B_NS_TWPST_MIN_0P5_PSTA        CY_LPDDR4_CLOCKCYCLES(0.4,CY_LPDDR4_FSP1_FREQ)

/** tWPST for 1.5 Clk Write Post-Amble  */
#define JESD209_4B_NS_TWPST_MIN_1P5_PSTA        CY_LPDDR4_CLOCKCYCLES(1.4,CY_LPDDR4_FSP1_FREQ)

/** tRPRE Write Pre-Amble time  1.8nCLK */
#define JESD209_4B_NS_TWPRE                     CY_LPDDR4_CLOCKCYCLES(1.8,CY_LPDDR4_FSP1_FREQ)

/*JEDEC tXP Exit power- down to next valid command delay*/
/** tXP value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TXP       7.5 
/** tXP value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TXP     5

/* JEDEC tCKFSPE MIN max(7.5ns, 4nCK)*/
/** tCKFSPE value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCKFSPE       7.5
/** tCKFSPE value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCKFSPE       4

/*JEDEC tCKELCK min=max(5ns, 5nCK)*/
/** tCKELCK value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCKELCK       5
/** tCKELCK value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCKELCK     5

/* JEDEC tCKFSPX min=max(7.5ns,4nCK)*/
/** tCKFSPX value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCKFSPX       7.5
/** tCKFSPX value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCKFSPX     4

/* JEDEC tMRW min=max(10ns,10nCK)*/
/** tMRW value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TMRW          10
/** tMRW value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TMRW        10

/* tCMDCKE Min Max(1.75ns,3nCK) */
/** tCMDCKE value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TCMDCKE          1.75
/** tCMDCKE value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TCMDCKE        3

/** tRRD derated for hot temp in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRRD_DERATED          JESD209_4B_NS_TRRD + 1.875

/* JEDEC209-4B tRRD min=max(10ns,4nCK)*/
/** tRRD value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TRRD          10
/** tRRD value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TRRD        4

/* JEDEC209-4B tXSR min Max(tRFCab+7.5ns,2tCK) */
/** tXSR value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TXSR          (JESD209_4B_NS_TREFCAB + 7.5)
/** tXSR value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TXSR         2

/* tWR=max(18ns, 6nCK) */
/** tWR value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TWR               18
/** tWR value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TWR            6

/* tWTR=max(10ns, 8nCK) */
/** tWTR value in nsec. as specified in JEDEC 209-4B  */
#define JESD209_4B_NS_TWTR               10
/** tWR value in clock cycles as specified in JEDEC 209-4B  */
#define JESD209_4B_NCLK_TWTR            8



#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __CY_DRAM_CONFIG_H__ */

/** \} group_lpddr */

/* [] END OF FILE */
