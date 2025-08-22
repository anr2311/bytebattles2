/***************************************************************************//**
* \file main_cm7_0.c
*   $Date: 2022-12-16 20:51:16 +0530 (Fri, 16 Dec 2022) $
*   $Revision: 320225 $
* \brief
* Main file for CM7 #0
*
********************************************************************************
* \copyright
* Copyright 2016-2019, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/


#include "cy_project.h"
#include "cy_device_headers.h"
#include "drivers/lpddr4/cy_lpddr4.h"
#include "examples/lpddr4/commonfiles/func_validation.h"
#include "examples/lpddr4/commonfiles/basic_tests.h"
#include "drivers/sysclk/cy_sysclk.h"

#define SVN_REV_MAIN_CM7_0_C       "$Revision: 320225 $"
#define SVN_DATE_MAIN_CM7_0_C      "$Date: 2022-12-16 20:51:16 +0530 (Fri, 16 Dec 2022) $"

volatile bool pwrDwnRequest = false;

#if (CY_USE_PSVP == 1)
    // #define USER_LED1_PORT           CY_BSP_BB_USER_LED1_PORT
    // #define USER_LED1_PIN            CY_BSP_BB_USER_LED1_PIN
    // #define USER_LED1_PIN_MUX        CY_BSP_BB_USER_LED1_PIN_MUX

    #define USER_BUTTON_PORT        CY_BSP_BB_USER_BUTTON_1_PORT
    #define USER_BUTTON_PIN         CY_BSP_BB_USER_BUTTON_1_PIN
    #define USER_BUTTON_PIN_MUX     CY_BSP_BB_USER_BUTTON_1_PIN_MUX
    #define USER_BUTTON_IRQ         CY_BSP_BB_USER_BUTTON_1_IRQN
#else
    #define USER_BUTTON_PORT        CY_USER_BUTTON_LEFT_PORT
    #define USER_BUTTON_PIN         CY_USER_BUTTON_LEFT_PIN
    #define USER_BUTTON_PIN_MUX     CY_USER_BUTTON_LEFT_PIN_MUX
    #define USER_BUTTON_IRQ         CY_USER_BUTTON_LEFT_IRQN

    #define USER_LED_PORT           CY_USER_LED2_PORT
    #define USER_LED_PIN            CY_USER_LED2_PIN
    #define USER_LED_PIN_MUX        CY_USER_LED2_PIN_MUX
#endif

/*  config used GPIO Pin used for Power Down Button */
/*  as soon as button is pressed the LPDDR4 shutdown will be initiated */
const cy_stc_gpio_pin_config_t user_button_port_pin_cfg = {
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_HIGHZ,
    .hsiom     = USER_BUTTON_PIN_MUX,
    .intEdge   = CY_GPIO_INTR_FALLING,
    .intMask   = 1ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

/* Setup GPIO for BUTTON1 interrupt */
const cy_stc_sysint_irq_t irq_btncfg ={
    .sysIntSrc  = USER_BUTTON_IRQ,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

/* Setup GPIO for LED2 */
cy_stc_gpio_pin_config_t user_led_port_pin_cfg ={
    .outVal = 0x00,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom = USER_LED_PIN_MUX,
};
void ButtonIntHandler(void){
    uint32_t intStatus;

    /* If falling edge detected */
    intStatus = Cy_GPIO_GetInterruptStatusMasked(USER_BUTTON_PORT, USER_BUTTON_PIN);
    if(intStatus != 0ul)
    {
        Cy_GPIO_ClearInterrupt(USER_BUTTON_PORT, USER_BUTTON_PIN);
        pwrDwnRequest = true;
    }
}
/******************************************************************************/
/*                              INIT TYPE Definintion                         */
/******************************************************************************/
#define INIT_WITH_TRAINING          0
#define INIT_WITHOUT_TRAINING       1
/* 
    macro to set the LPDDR4 Controller initialization
    for PSVP: 
            #define INIT_TYPE       INIT_WITH_TRAINING

    for FCV or SOC both versions are possible. 
    When set to #define INIT_TYPE   INIT_WITHOUT_TRAINING
    the PTSRxx register in the lpddr_config structure must be set with training 
    data matching  user hardware as these values are used to load known 
    training values to the phy for initialization
*/
#define INIT_TYPE   INIT_WITH_TRAINING
//#define INIT_TYPE   INIT_WITHOUT_TRAINING

// define Timer-ID for cyclic MR4 reading / DQS2DQ check for retrain
#define MR4_DQS2DQ_TIMER_ID 0

/******************************************************************************* 
    set CONFIG_TYPE in cy_dramconfsel.h 
    #define CONFIG_TYPE     SOL_DESIGNER_CONFIG     //<------ set config type here
**********************************************************************************/
#include"soldsgncfg.h"


/** Interrupt Configuration for catching LPDDR4 Faults*/
cy_stc_sysint_irq_t irq_cfg = {
    .sysIntSrc  = cpuss_interrupts_fault_0_IRQn,
    .intIdx     = CPUIntIdx2_IRQn,
    .isEnabled  = true
};

/*******************************************************************************
 * Fault Handler for FAULT_STRUCT0 in which the LPDDR4 faults are mapped
 * **************************************************************************//*
 *
 *  LPDDR4 faults are mapped to Fault Structure 0
 *  In this Fault Handler the actions in case of fault can be handled
 * 
 ******************************************************************************/
void irqFaultReport0Handler(void){
    cy_en_sysflt_source_t status;
    
    uint32_t fltData = 0;
    status = Cy_SysFlt_GetErrorSource(FAULT_STRUCT0);

    if(status != CY_SYSFLT_NO_FAULT)
    {
        /* LPDDR4 FATAL Fault */
        if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_FATAL_FAULT) 
        {
            //add some usefull code for fatal handling 
            fltData = Cy_SysFlt_GetData0(FAULT_STRUCT0);
            switch(fltData)
            {
                /* DATA0[0]: 0: non-correctable ECC fault */
                case 0: 
                        /* accord. SAS Reset at least the LPDDR4 system. 
                           Reset at least the LPDDR4 system. */ 
                        Cy_SwTmr_Stop(MR4_DQS2DQ_TIMER_ID);
                        /* Deinitilize the LPDDR4 controller */
                        Cy_Lpddr_ControllerDeInit(LPDDR40);
                        /* Disable the LPDDR4 power sequencing */
                        #ifdef CY_SYS_LPDDR_POWER_EN    
                            Cy_SystemLpdd4PowerDown(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_US);
                        #endif  /* CY_SYS_LPDDR_POWER_EN */
                        // halt System
                        CY_ASSERT(0);
                        break;
            
                /*  DATA0[0]: 1: PLL unlock fault */
                case 1:
                        /*Reset at least the LPDDR4 system. */ 
                        Cy_SwTmr_Stop(MR4_DQS2DQ_TIMER_ID);
                        /* Deinitilize the LPDDR4 controller */
                        Cy_Lpddr_ControllerDeInit(LPDDR40);
                        /* Disable the LPDDR4 power sequencing */
                        #ifdef CY_SYS_LPDDR_POWER_EN    
                            Cy_SystemLpdd4PowerDown(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_US);
                        #endif  /* CY_SYS_LPDDR_POWER_EN */
                        // halt System
                        CY_ASSERT(0);
                        break;
            }
        /* LPDDR4 none fatal GSM ( Global Statemachine fault */
        }else if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_GSM_FAULT) 
        {
            Cy_Lpddr_SyncGSMStateOnFault();
             /*add something usefull here*/

            /* clear the interrupt flag before issuing next user command*/
             Cy_Lpddr_ClearGCFSM_DMCFG();
        }else if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_ECC_SEC_FAULT) 
        {   
            //add some usefull code for none fatal fault handling 
            /* correctable ECC Fault */
            __NOP();
        }
      Cy_SysFlt_ClearStatus(FAULT_STRUCT0);
    }
    Cy_SysFlt_ClearInterrupt(FAULT_STRUCT0);
}

int main(void)
{
    /**
    * specifies the system temperature gradient of your system
    * used to calculate the cycle time for reading MR4 and updating 
    * the refresh rate [°C/s] */
    uint32_t temperatureGradient = 30; 
    /**
     * system response delay of your system
     * used to calculate the cycle time for reading MR4 and updating 
     * the refresh rate [ms] */
    uint32_t systemResponseDelay = 1;

    /* *local variable for the MR4 Read Interval */
    uint32_t mr4ReadInterval=0;
    /* exit condition for the main loop  down below*/
    cy_en_lpddr4_retval_t cancelCondition = CY_LPDDR4_SUCCESS;  


    __enable_irq();
    SystemInit();
    
    /* Enable the LPDDR4 power sequencing */
    #ifdef CY_SYS_LPDDR_POWER_EN
        Cy_SystemLpdd4PowerUp(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_US);
    #endif  /* CY_SYS_LPDDR_POWER_EN */


    /* Init the GPIO used for the power on request button */
    Cy_GPIO_Pin_Init(USER_BUTTON_PORT, USER_BUTTON_PIN, &user_button_port_pin_cfg);

    /* Init the GPIO used to indicate end of operation */
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led_port_pin_cfg);
    /***************************************************************************
                            Fault report settings
    ***************************************************************************/
    /* clear status (typically this process is done by boot code)*/
    Cy_SysFlt_ClearStatus(FAULT_STRUCT0); 
    /* set the Mask bit for LPDDR fatal Fault DEC ECC Fault / PLL unlock  */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_FATAL_FAULT);
    /* set the Mask bit for LPDDR none fatal Global State Machine Fault */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_GSM_FAULT);
    /* set the Mask bit for LPDDR SEC ECCC none fatal fault */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_ECC_SEC_FAULT);
    Cy_SysFlt_SetInterruptMask(FAULT_STRUCT0);

    /***************************************************************************
                               Interrupt settings
    ***************************************************************************/
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, irqFaultReport0Handler);
    NVIC_SetPriority(irq_cfg.intIdx, 0);
    NVIC_EnableIRQ(irq_cfg.intIdx);

    Cy_SysInt_InitIRQ(&irq_btncfg);
    Cy_SysInt_SetSystemIrqVector(irq_btncfg.sysIntSrc, ButtonIntHandler);
    NVIC_SetPriority(irq_btncfg.intIdx, 3ul);
    NVIC_EnableIRQ(irq_btncfg.intIdx);
    /* Initialize LPDDR4 with either Reload or Training */
    #if ( INIT_TYPE == INIT_WITH_TRAINING )
    /* real training only possible on real silicon */
    cancelCondition = Cy_Lpddr_ControllerInit(LPDDR40,&lpddr_config,CY_LPDDR4_INIT_WITH_TRAINING);
    #elif ( INIT_TYPE == INIT_WITHOUT_TRAINING ) 
    cancelCondition=Cy_Lpddr_ControllerInit(LPDDR40,&lpddr_config,CY_LPDDR4_INIT_WITH_RELOAD);
    #else
        #error Init Type not set
    #endif
    if(cancelCondition != CY_LPDDR4_SUCCESS) 
    {
        Cy_Lpddr_ControllerDeInit(LPDDR40);
        /* Disable the LPDDR4 power sequencing */
        #ifdef CY_SYS_LPDDR_POWER_EN    
        Cy_SystemLpdd4PowerDown(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_US);
        #endif  /* CY_SYS_LPDDR_POWER_EN */
        CY_ASSERT(0);
    }
   
    // Initializes the HW timer used as SW Timer source (SysTick)
    Cy_SwTmr_Init();

    /* calculate the the interval for reading the Mode Register MR4 as specified in JEDEC209-4B */
    mr4ReadInterval = Cy_Lpddr_CalcMR4ReadInterval(systemResponseDelay,temperatureGradient);

    /*  Start a periodic SW timer with interval calculated above (running from IRQ context) 
        Cy_Lpddr4_MemoryMonitoring is the Callback Function which is 
        performing LPDDR4 monitoring :
                - read MR4 for temperature Monitoring 
                - refresh interval adjust based on temperature
                - timing derating based based on temperature
                - start DQS Oscillator ( used to detect the need for retraining)
                - read the DQS Oscillator values ( MR18/MR19 )
                - detect need for retrain based on DQS Oscillator value   */
    Cy_SwTmr_StartHighPrio(MR4_DQS2DQ_TIMER_ID,mr4ReadInterval, true,Cy_Lpddr_MemoryMonitoring);
    
    //Main Loop starts here 
    while((cancelCondition == CY_LPDDR4_SUCCESS)&&(pwrDwnRequest==false))
    {
        WriteAddressToMem(0x3FFFFFFF); // Write different pattern to whole memory
        /*check if retraining is needed*/
        if(true == Cy_Lpddr_CheckForRetraining())
        {
            /* stop the timer for memory monitoring during retraining */
            Cy_SwTmr_Stop(MR4_DQS2DQ_TIMER_ID);
            #if (CY_USE_PSVP == 1)
                /*as real re-training doesn´t work on PSVP 
                reset the retraining request manually */
                Cy_LPDDR4_PSVPFakeTraining(LPDDR40);
            #else
                /* on none PSVP HW perform the real training */
                cancelCondition = Cy_Lpddr_RequestDQS2DQRetrain(LPDDR40);
            #endif
            /* restart the timer for memory monitoring */
            Cy_SwTmr_StartHighPrio(MR4_DQS2DQ_TIMER_ID,mr4ReadInterval, true,Cy_Lpddr_MemoryMonitoring);
        }
    }
    Cy_SwTmr_Stop(MR4_DQS2DQ_TIMER_ID);
    /* Deinitilize the LPDDR4 controller */
    Cy_Lpddr_ControllerDeInit(LPDDR40);
    /* Disable the LPDDR4 power sequencing */
    #ifdef CY_SYS_LPDDR_POWER_EN    
        Cy_SystemLpdd4PowerDown(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_US);
    #endif  /* CY_SYS_LPDDR_POWER_EN */

    while(1)
    {
        /* Blink a LED after successful return */
        Cy_SysTick_DelayInUs(50000);
        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);
    }
}

/* [] END OF FILE */


