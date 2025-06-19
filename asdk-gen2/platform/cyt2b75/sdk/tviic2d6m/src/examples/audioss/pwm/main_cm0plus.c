/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_project.h"
#include "cy_device_headers.h"
#include "audioss/pwm/cy_pwm.h"
#include "wav.h"

/* PWM port configuration macro */
//------------------------------------------------------------------------------
#define AUDIO_PWM_TRIG_LEVEL (20U)

/* User led configuration */
//------------------------------------------------------------------------------
cy_stc_gpio_pin_config_t io_port_pin_cfg =
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = CY_USER_LED1_PIN_MUX,                           
    .intEdge = 0,                                  
    .intMask = 0,                                  
    .vtrip = 0,                                    
    .slewRate = 0,                                 
    .driveSel = 0,                                 
    .vregEn = 0,                                   
    .ibufMode = 0,                                 
    .vtripSel = 0,                                 
    .vrefSel = 0,                                  
    .vohSel = 0,                                   
};

/* Pwm tx configuration */
//------------------------------------------------------------------------------
static cy_stc_pwm_tx_config_t myPwmConfig =
{
    .txEnabled          = true,
    .txWordSize         = CY_PWM_WORD_LEN16,
    .txClkDiv           = 127,                       // 1.53M
    .txClkSel           = CY_PWM_SEL_SRSS_CLK0,
    .txLineOutPolarity  = CY_PWM_LINE1_P_OUT_LINE1_N_INV_OUT_LINE2_P_INV_OUT_LINE2_N_INV_OUT,
    .txDoublerEnabled   = false,
    .txDoublerMode      = CY_PWM_TX_DOUBLE_REPETITION,
    .txGainCode         = 20,
    .txFormat           = CY_PWM_TX_H_BRIDGE,//CY_PWM_TX_H_BRIDGE,
    .txGainScale        = 2,
    .pwmMinValue        = 0,
    .pwmMaxValue        = 65535,
    .pwmPeriod          = 15,  
    .pwmOffset          = 0,
    .pwmDeadTime        = 0,
    .txFifoTriggerLevel  = AUDIO_PWM_TRIG_LEVEL,
};

/* Audio pwm interrupt configuration */ 
//------------------------------------------------------------------------------
static const cy_stc_sysint_irq_t irqCfgPwm =
{
    .sysIntSrc  = CY_AUDIOSS_PWM0_IRQN,
    .intIdx     = CPUIntIdx2_IRQn, 
    .isEnabled  = true, 
};

/* Global function declaration */
//------------------------------------------------------------------------------
void InitPwmLines(void);
void AudioPwmInterruptHandler(void);

uint32_t WAVCounter = 0;
/* Audio pwm interrupt handler */
//------------------------------------------------------------------------------
static void AudioPwmInterruptHandler(void)
{
    uint32_t maskedIntrStatus;

    maskedIntrStatus = Cy_Pwm_Tx_GetInterruptStatusMasked(CY_AUDIOSS_PWM0_TYPE);
    
    if((maskedIntrStatus & CY_PWM_INTR_TX_FIFO_TRIGGER) != 0)
    {
        for(uint8_t data_i = 0; data_i < AUDIO_PWM_TRIG_LEVEL; data_i++)
        {
            Cy_Pwm_Tx_WriteData(CY_AUDIOSS_PWM0_TYPE, WAV_DATA[data_i + WAVCounter]);
        }
        WAVCounter += AUDIO_PWM_TRIG_LEVEL;

        if(WAVCounter >= sizeof(WAV_DATA)/sizeof(WAV_DATA[0]) )
        {
            WAVCounter = 0;
        }
    }
    Cy_Pwm_Tx_ClearInterrupt(CY_AUDIOSS_PWM0_TYPE, maskedIntrStatus);
}

/* Initialize Audio pwm io mux */
//------------------------------------------------------------------------------
void InitPwmLines(void)
{   
    // On PSVP : PWM0_LINE1_P0 -- TP357
    io_port_pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF;
    io_port_pin_cfg.hsiom = CY_AUDIOSS_PWM0_L1P1_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_AUDIOSS_PWM0_L1P1_PORT, CY_AUDIOSS_PWM0_L1P1_PIN, &io_port_pin_cfg);

    // On PSVP : PWM0_LINE1_N0 -- TP240
    io_port_pin_cfg.hsiom = CY_AUDIOSS_PWM0_L1N1_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_AUDIOSS_PWM0_L1N1_PORT, CY_AUDIOSS_PWM0_L1N1_PIN, &io_port_pin_cfg);

    // On PSVP : PWM0_LINE2_P0 -- TP307
    io_port_pin_cfg.hsiom = CY_AUDIOSS_PWM0_L2P1_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_AUDIOSS_PWM0_L2P1_PORT, CY_AUDIOSS_PWM0_L2P1_PIN, &io_port_pin_cfg);

    // On PSVP : PWM0_LINE2_N0 -- TP340
    io_port_pin_cfg.hsiom = CY_AUDIOSS_PWM0_L2N1_PIN_MUX;
    Cy_GPIO_Pin_Init(CY_AUDIOSS_PWM0_L2N1_PORT, CY_AUDIOSS_PWM0_L2N1_PIN, &io_port_pin_cfg);
}

/* Main app function */
//------------------------------------------------------------------------------
int main(void)
{
    /* System initialization */
    SystemInit();
  
    /* Enable irq */
    __enable_irq();

    /* Port setting for pwm channels */
    InitPwmLines();
    
    /* Initialization for pwm */
    Cy_Pwm_Tx_DeInit(CY_AUDIOSS_PWM0_TYPE);

    /* Initialize pwm transmittter interface */
    Cy_Pwm_Tx_Init(CY_AUDIOSS_PWM0_TYPE, &myPwmConfig);
    
    /* Interrupt setting for Pwm communication */
    Cy_SysInt_InitIRQ(&irqCfgPwm);
    Cy_SysInt_SetSystemIrqVector(irqCfgPwm.sysIntSrc, AudioPwmInterruptHandler);
    NVIC_SetPriority(irqCfgPwm.intIdx, 1);
    NVIC_ClearPendingIRQ(irqCfgPwm.intIdx);
    NVIC_EnableIRQ(irqCfgPwm.intIdx);

    /* Enable pwm */
    Cy_Pwm_Tx_Enable(CY_AUDIOSS_PWM0_TYPE);

    Cy_Pwm_Tx_SetTriggerInterruptMask(CY_AUDIOSS_PWM0_TYPE);

    for(;;);
}

//------------------------------------------------------------------------------

/* [] END OF FILE */


