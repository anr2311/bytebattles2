/***************************************************************************//**
* \file system_tviic2d6mddr_cm7.c
* \version 1.0
*
* The device system-source file.
*
********************************************************************************
* \copyright
* Copyright 2018-2021, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include <stdint.h>
#include <stdbool.h>
#include "cy_device_headers.h"
#include "cy_project.h"

/* SCB->CPACR */
#define SCB_CPACR_CP10_CP11_ENABLE      (0xFUL << 20u)

/** Holds the FastClk system core clock, which is the system clock frequency supplied to the SysTick timer and the
* processor core clock. This variable can be used by debuggers to query the frequency of the debug timer or to configure
* the trace clock speed.
*
* \attention Compilers must be configured to avoid removing this variable in case the application program is not using
* it. Debugging systems require the variable to be physically present in memory so that it can be examined to configure
* the debugger. */
uint32_t SystemCoreClock  = 0UL;

uint32_t cy_delayFreqHz   = 0UL;

uint32_t cy_delayFreqKhz  = 0UL;

uint32_t cy_delayFreqMhz  = 0UL;

uint32_t cy_delay32kMs    = 0UL;


/*******************************************************************************
* Function Name: SystemInit
****************************************************************************//**
*
* Initializes the system
*
*******************************************************************************/
void SystemInit (void)
{
    // Ensure cache coherency (e.g. in case ROM-to-RAM copy of code sections happened during startup)
    SCB_CleanInvalidateDCache();
    SCB_InvalidateICache();
    
    /* Update the core clock parameters */
    SystemCoreClockUpdate();

#if defined (CY_SYS_LPDDR_POWER_EN)
    /* Configure the power pins used for LPDDR */
    Cy_SystemLpddrPowerConfig();
#endif  /* CY_SYS_LPDDR_POWER_EN */
}

/*******************************************************************************
* Function Name: SystemCoreClockUpdate
****************************************************************************//**
*
* Updates variables with current clock settings
*
*******************************************************************************/
void SystemCoreClockUpdate (void)
{
    cy_stc_base_clk_freq_t freqInfo = 
    {
        .clk_imo_freq  = CY_CLK_IMO_FREQ_HZ,
        .clk_ext_freq  = CY_CLK_EXT_FREQ_HZ,
        .clk_eco_freq  = CY_CLK_ECO_FREQ_HZ,
        .clk_ilo0_freq = CY_CLK_HVILO0_FREQ_HZ,
        .clk_ilo1_freq = CY_CLK_HVILO1_FREQ_HZ,
        .clk_wco_freq  = CY_CLK_WCO_FREQ_HZ,
    };

    cy_en_sysclk_status_t retVal;
    retVal = Cy_SysClk_InitGetFreqParams(&freqInfo);
    CY_ASSERT(retVal == CY_SYSCLK_SUCCESS);
    retVal = Cy_SysClk_GetCoreFrequency(&SystemCoreClock);
    CY_ASSERT(retVal == CY_SYSCLK_SUCCESS);
    (void) retVal; // avoid "unused" compiler warning if NDEBUG is set
	
    cy_delayFreqHz   = SystemCoreClock;
    cy_delayFreqMhz  = ((cy_delayFreqHz + CY_DELAY_1M_MINUS_1_THRESHOLD) / CY_DELAY_1M_THRESHOLD);
    cy_delayFreqKhz  = (cy_delayFreqHz + CY_DELAY_1K_MINUS_1_THRESHOLD) / CY_DELAY_1K_THRESHOLD;
    cy_delay32kMs    = CY_DELAY_MS_OVERFLOW_THRESHOLD * cy_delayFreqKhz;
}

/*******************************************************************************
* Function Name: Cy_SystemInitFpuEnable
****************************************************************************//**
*
* Enables the FPU if it is used. The function is called from the startup file.
*
*******************************************************************************/
void Cy_SystemInitFpuEnable(void)
{
    #if defined (__FPU_USED) && (__FPU_USED == 1U)
        uint32_t  interruptState;
        interruptState = Cy_SaveIRQ();
        SCB->CPACR |= SCB_CPACR_CP10_CP11_ENABLE;
        __DSB();
        __ISB();
        Cy_RestoreIRQ(interruptState);
    #endif /* (__FPU_USED) && (__FPU_USED == 1U) */
}


/* Case where system uses LPDDR4 memory sequencing */
#ifdef CY_SYS_LPDDR_POWER_EN

/* Port pin configuration type */
typedef struct
{
    volatile stc_GPIO_PRT_t*    pstcPort;
    uint8_t                     u8Pin;
    en_hsiom_sel_t              enMuxCfg;
    uint8_t                     u8DriveMode;
    uint8_t                     u8GpioOutVal;
} stc_portpin_cfg_t;

/* Port pin configuration */
static const stc_portpin_cfg_t cy_lpddrPowerSeqPins[] =
{
#if (CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_HW)
    { .pstcPort = CY_LPDDR_PWR_EN_PORT,     .u8Pin = CY_LPDDR_PWR_EN_PIN,       .enMuxCfg = CY_LPDDR_PWR_EN_PIN_MUX,    .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .pstcPort = CY_PG_VDDQ_PORT,          .u8Pin = CY_PG_VDDQ_PIN,            .enMuxCfg = CY_PG_VDDQ_PIN_MUX,         .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
#else /*(CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_SW) */
    { .pstcPort = CY_LPDDR_VDD1_EN_PORT,    .u8Pin = CY_LPDDR_VDD1_EN_PIN,      .enMuxCfg = CY_LPDDR_VDD1_EN_PIN_MUX,   .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .pstcPort = CY_LPDDR_VDD2_EN_PORT,    .u8Pin = CY_LPDDR_VDD2_EN_PIN,      .enMuxCfg = CY_LPDDR_VDD2_EN_PIN_MUX,   .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
#endif /* CY_SYS_USE_LPDDR4_SEQ */
    { .pstcPort = CY_PG_VDD1_PORT,          .u8Pin = CY_PG_VDD1_PIN,            .enMuxCfg = CY_PG_VDD1_PIN_MUX,         .u8DriveMode = CY_GPIO_DM_HIGHZ, .u8GpioOutVal = 0 },
    { .pstcPort = CY_PG_VDD2_PORT,          .u8Pin = CY_PG_VDD2_PIN,            .enMuxCfg = CY_PG_VDD2_PIN_MUX,         .u8DriveMode = CY_GPIO_DM_HIGHZ, .u8GpioOutVal = 0 },
};


/*******************************************************************************
* Function Name: Cy_SystemLpddrPowerConfig
****************************************************************************//**
*
* Configure the IO pins used to initiate the lpddr power up and down sequence.
*
*******************************************************************************/
void Cy_SystemLpddrPowerConfig(void)
{
    /* Initialize the power pins used for the sequencing */
    for(uint8_t u8Index = 0; u8Index < CY_GET_ARRAY_LEN(cy_lpddrPowerSeqPins); u8Index++)
    {
        Cy_GPIO_Pin_FastInit( cy_lpddrPowerSeqPins[u8Index].pstcPort,
                              cy_lpddrPowerSeqPins[u8Index].u8Pin,
                              cy_lpddrPowerSeqPins[u8Index].u8DriveMode,
                              cy_lpddrPowerSeqPins[u8Index].u8GpioOutVal,
                              cy_lpddrPowerSeqPins[u8Index].enMuxCfg      );
    }
}

/*******************************************************************************
* Function Name: Cy_SystemLpdd4PowerUp
****************************************************************************//**
*
* Enables the LPDDR4 memory power sequencing based on the selected scheme.
*
*******************************************************************************/
void Cy_SystemLpdd4PowerUp(uint8_t isPollPg, uint32_t delayUs)
{
    /* Sequencing based on the hardware sequencer */
    #if (CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_HW)

    /* Enable the hardware sequencer pin */
    Cy_GPIO_Write(CY_LPDDR_PWR_EN_PORT, CY_LPDDR_PWR_EN_PIN, 1u);

    /* If pooling is true, monitor all the PG signal until goes up */
    if (isPollPg == CY_SYS_LPDDR4_PG_POLL)
    {
        while (0 == Cy_GPIO_Read(CY_PG_VDD1_PORT, CY_PG_VDD1_PIN));
        while (0 == Cy_GPIO_Read(CY_PG_VDD2_PORT, CY_PG_VDD2_PIN));
        while (0 == Cy_GPIO_Read(CY_PG_VDDQ_PORT, CY_PG_VDDQ_PIN));
    }
    else
    {
        /* Wait for the defined us until power rails will stable */
        Cy_SysTick_DelayInUs(delayUs*3);
    }
    #endif /* CY_SYS_USE_LPDDR4_SEQ */

    /* Sequencing based on the software sequencer */
    #if (CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_SW)

    /* If pooling is true, monitor all the PG signal until goes up */
    if(isPollPg == CY_SYS_LPDDR4_PG_POLL)    
    {
        /* Power up VDD1 and wait for PG */
        Cy_GPIO_Write(CY_LPDDR_VDD1_EN_PORT, CY_LPDDR_VDD1_EN_PIN, 1u);
        while (0 == Cy_GPIO_Read(CY_PG_VDD1_PORT, CY_PG_VDD1_PIN));

        /* Power up VDD2 and wait for PG */
        Cy_GPIO_Write(CY_LPDDR_VDD2_EN_PORT, CY_LPDDR_VDD2_EN_PIN, 1u);
        while (0 == Cy_GPIO_Read(CY_PG_VDD2_PORT, CY_PG_VDD2_PIN));
    }
    else
    {
        /* Power up VDD1 and wait for the ms */
        Cy_GPIO_Write(CY_LPDDR_VDD1_EN_PORT, CY_LPDDR_VDD1_EN_PIN, 1u);
        Cy_SysTick_DelayInUs(delayUs);

        /* Power up VDD2 and wait for the ms */
        Cy_GPIO_Write(CY_LPDDR_VDD2_EN_PORT, CY_LPDDR_VDD2_EN_PIN, 1u);
        Cy_SysTick_DelayInUs(delayUs);
    }
    #endif /* CY_SYS_USE_LPDDR4_SEQ */
}

/*******************************************************************************
* Function Name: Cy_SystemLpdd4PowerDown
****************************************************************************//**
*
* Disables the LPDDR4 memory power sequencing based on the selected scheme.
*
*******************************************************************************/
void Cy_SystemLpdd4PowerDown(uint8_t isPollPg, uint32_t delayUs)
{
    /* Sequencing based on the hardware sequencer */
    #if (CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_HW)
    
    /* Enable the hardware sequencer pin */
    Cy_GPIO_Write(CY_LPDDR_PWR_EN_PORT, CY_LPDDR_PWR_EN_PIN, 0u);
    
    /* If pooling is true, monitor all the PG signal until goes down */
    if (isPollPg == CY_SYS_LPDDR4_PG_POLL)
    {
        while (1 == Cy_GPIO_Read(CY_PG_VDDQ_PORT, CY_PG_VDDQ_PIN));
        while (1 == Cy_GPIO_Read(CY_PG_VDD2_PORT, CY_PG_VDD2_PIN));
        while (1 == Cy_GPIO_Read(CY_PG_VDD1_PORT, CY_PG_VDD1_PIN));
    }
    else
    {
        /* Wait for the defined us until power rails will stable */
        Cy_SysTick_DelayInUs(delayUs*3);
    }

    #endif /* CY_SYS_USE_LPDDR4_SEQ */

    /* Sequencing based on the software sequencer */
    #if (CY_SYS_USE_LPDDR4_SEQ == CY_SYS_LPDDR4_SEQ_SW)

    /* If pooling is true, monitor all the PG signal until goes down */
    if(isPollPg == CY_SYS_LPDDR4_PG_POLL)    
    {
        /* Power down VDD2 and wait for PG */
        Cy_GPIO_Write(CY_LPDDR_VDD2_EN_PORT, CY_LPDDR_VDD2_EN_PIN, 0u);
        while (1 == Cy_GPIO_Read(CY_PG_VDD2_PORT, CY_PG_VDD2_PIN));

        /* Power down VDD1 and wait for PG */
        Cy_GPIO_Write(CY_LPDDR_VDD1_EN_PORT, CY_LPDDR_VDD1_EN_PIN, 0u);
        while (1 == Cy_GPIO_Read(CY_PG_VDD1_PORT, CY_PG_VDD1_PIN));
    }
    else
    {
        /* Power down VDD2 */
        Cy_GPIO_Write(CY_LPDDR_VDD2_EN_PORT, CY_LPDDR_VDD2_EN_PIN, 0u);
        Cy_SysTick_DelayInUs(delayUs);

        /* Power down VDD1 */
        Cy_GPIO_Write(CY_LPDDR_VDD1_EN_PORT, CY_LPDDR_VDD1_EN_PIN, 0u);
        Cy_SysTick_DelayInUs(delayUs);
    }
    #endif /* CY_SYS_USE_LPDDR4_SEQ */
}

#endif  /* CY_SYS_LPDDR_POWER_EN */
