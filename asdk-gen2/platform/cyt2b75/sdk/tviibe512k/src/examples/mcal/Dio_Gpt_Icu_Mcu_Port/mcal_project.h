/***************************************************************************//**
* \file mcal_project.h
*
* \brief
* Common file for inclusion in the project sources.
* Includes MCAL related header files and may be used for other purposes as well.
*
********************************************************************************
* \copyright
* Copyright 2016-2019, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _MCAL_PROJECT_H_
#define _MCAL_PROJECT_H_

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/

#include "mcal_notifications.h"

// MCAL module header files can only be included if they are part of the configuration,
// otherwise they may refer to non-present generated include files
#ifdef CY_MCAL_MODULE_ENABLED_ADC
    #include <Adc.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_CAN
    #include <Can.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_CORTST
    #include <CorTst.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_DIO
    #include <Dio.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_FEE
    #include <Fee.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_FLS
    #include <Fls.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_FLSTST
    #include <FlsTst.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_GPT
    #include <Gpt.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_ICU
    #include <Icu.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_LIN
    #include <Lin.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_MCU
    #include <Mcu.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_OCU
    #include <Ocu.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_PORT
    #include <Port.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_PWM
    #include <Pwm.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_RAMTST
    #include <RamTst.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_SPI
    #include <Spi.h>
#endif
#ifdef CY_MCAL_MODULE_ENABLED_WDG
    #include <Wdg.h>
#endif


/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define MCAL_PROJECT_TIMER_16BIT_1KHZ_GPT_IRQ_NUMBER    274
#define MCAL_PROJECT_TIMER_16BIT_1KHZ_GPT_IRQ_HANDLER   GPT_Isr_Vector_274_Cat2

#define MCAL_PROJECT_TIMER_32BIT_1MHZ_GPT_IRQ_NUMBER    350
#define MCAL_PROJECT_TIMER_32BIT_1MHZ_GPT_IRQ_HANDLER   GPT_Isr_Vector_350_Cat2

#define MCAL_PROJECT_CB_BUTTON_1_ICU_IRQ_NUMBER         27
#define MCAL_PROJECT_CB_BUTTON_1_ICU_IRQ_HANDLER        Icu_Isr_Vector_27_Cat2


#endif /* _MCAL_PROJECT_H_ */


/* [] END OF FILE */
