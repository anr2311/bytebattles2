/***************************************************************************//**
* \file main_cm4.c
*
* \version 1.0
*
* \brief Main example file for CM4
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

/*******************************************************************************
*    Parameters definitions
*******************************************************************************/

#define USER_LED_PORT       CY_BSP_USER_LED2_PORT
#define USER_LED_PIN        CY_BSP_USER_LED2_PIN
#define USER_LED_PIN_MUX    CY_BSP_USER_LED2_PIN_MUX

const cy_stc_gpio_pin_config_t user_led3_port_pin_cfg =
{
    .outVal    = 0u,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = USER_LED_PIN_MUX,
    .intEdge   = 0u,
    .intMask   = 0u,
    .vtrip     = 0u,
    .slewRate  = 0u,
    .driveSel  = 0u,
};

// Initialize all SYSTEM_INT_STRUCTS
const cy_stc_sysint_irq_t irq_cfg =
{
    .sysIntSrc = srss_interrupt_backup_IRQn,
    .intIdx    = CPUIntIdx2_IRQn,
    .isEnabled = true,
};

void RTC_Handler(void)
{
    // CM0+ will handle
}

int main(void)
{
    SystemInit();

    __enable_irq(); /* Enable global interrupts. */

    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led3_port_pin_cfg);

    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, RTC_Handler);
    NVIC_SetPriority(irq_cfg.intIdx, 3ul);
    NVIC_EnableIRQ(irq_cfg.intIdx);

    uint8_t count = 0u;
    for(;;)
    {
        // wait 0.05 [s]
        Cy_SysTick_DelayInUs(50000ul);

        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);

        count++;
        if (count > 200u)
        {
            count = 0u;
            Cy_GPIO_Write(USER_LED_PORT, USER_LED_PIN, 0ul);
            Cy_SysPm_DeepSleep(CY_SYSPM_WAIT_FOR_INTERRUPT);
        }
    }
}

/* [] END OF FILE */
