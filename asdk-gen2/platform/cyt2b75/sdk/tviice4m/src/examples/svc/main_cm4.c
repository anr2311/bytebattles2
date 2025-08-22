
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

#define PRIVILEGED_LED_PORT CY_CB_LED_PORT
#define PRIVILEGED_LED_PIN  CY_CB_LED_PIN

cy_stc_gpio_pin_config_t gLedPortPinCfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = HSIOM_SEL_GPIO,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

#define BACKGROUND_REGION_ADDR       (0x00000000ul) // Back Ground Region Start Address
#define PERI_REGISTER_REGION_ADDR    (0x40000000ul) // Peripheral Register Region Start Address
#define ARM_SYS_REGISTER_REGION_ADDR (0xE0000000ul) // ARM System Registers Region Start Address

#define BACKGROUND_MPU_NO          (0)
#define PERI_REGISTER_MPU_NO       (1)
#define ARM_SYS_REGISTER_MPU_NO    (2)
#define PRIVILEGED_PERI_REG_MPU_NO (3)


cy_stc_mpu_region_cfg_t gMpuCfg[] =
{
    /*** Back Ground Region ***/
    {
        .addr       = BACKGROUND_REGION_ADDR,
        .size       = CY_MPU_SIZE_4GB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute  = CY_MPU_ATTR_NORM_MEM_WT,
        .execute    = CY_MPU_INST_ACCESS_EN,
        .srd        = 0x00u,
        .enable     = CY_MPU_ENABLE
    },

    /*** Peripheral Register Region ***/
    {
        .addr       = PERI_REGISTER_REGION_ADDR,
        .size       = CY_MPU_SIZE_64MB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute  = CY_MPU_ATTR_SHR_DEV,
        .execute    = CY_MPU_INST_ACCESS_DIS,
        .srd        = 0x00u,
        .enable     = CY_MPU_ENABLE
    },

    /*** ARM System Registers Region ***/
    {
        .addr       = ARM_SYS_REGISTER_REGION_ADDR,
        .size       = CY_MPU_SIZE_512MB,
        .permission = CY_MPU_ACCESS_P_FULL_ACCESS,
        .attribute  = CY_MPU_ATTR_STR_ORD_DEV,
        .execute    = CY_MPU_INST_ACCESS_DIS,
        .srd        = 0x00u,
        .enable     = CY_MPU_ENABLE
    },

    /*** Privileged LED port Region ***/
    {
        .addr       = 0ul,                             // Will be update in run time
        .size       = CY_MPU_SIZE_128B,
        .permission = CY_MPU_ACCESS_P_PRIV_RW_USER_RO, // Note: UESR software can read only.
        .attribute  = CY_MPU_ATTR_SHR_DEV,
        .execute    = CY_MPU_INST_ACCESS_DIS,
        .srd        = 0x00u,
        .enable     = CY_MPU_ENABLE
    },

};
#define SIZE_OF_MPU_SETTING (sizeof(gMpuCfg)/sizeof(gMpuCfg[0]))

/* This function changes the CPU mode to user mode. */
void SetUserMode(void)
{
    __ASM("MRS r0, CONTROL"); // Read CONTROL register into R0
    __ASM("ORR r0, r0, #1");  // nPRIV -> 1
    __ASM("MSR CONTROL, r0"); // Write R0 into CONTROL register
}

/* This function changes the CPU mode to privileged mode. */
void SetPrivilegedMode(void)
{
    __ASM("MRS r0, CONTROL"); // Read CONTROL register into R0
    __ASM("BIC r0, r0, #1");  // nPRIV -> 0
    __ASM("MSR CONTROL, r0"); // Write R0 into CONTROL register
}

/* Supervisor call function 
   SVC index 0: Initialize the LED port.
   SVC index 1: Turn the LED port on if R0 value == 1
                Turn the LED port off if R0 value == 0
   SVC index 2: Change the CPU mode to privileged mode
*/
void Cy_SysLib_SvcHandler(uint32_t* pSvcArgs)
{
    uint8_t svcIdx = ((char*)pSvcArgs[6])[-2];
    
    switch(svcIdx)
    {
    case 0:
        Cy_GPIO_Pin_Init(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, &gLedPortPinCfg);
        break;
    case 1:
        if(pSvcArgs[0] == 0)
        {
            Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 0);
        }
        else
        {
            Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 1);
        }
        break;
    case 2:
        SetPrivilegedMode();
        break;
    default:
        break;
    }
}

/* This function invokes SVC index 0. See Cy_SysLib_SvcHandler */
void SVC_InitPrivilegedLedPort(void)
{
    __ASM("SVC 0x00"); // SVC index = 0: Initialize LED port
}

/* This function invokes SVC index 1 with R0 = 1. See Cy_SysLib_SvcHandler */
void SVC_TurnLedPortOn(void)
{
    __ASM("LDR  r0, =1"); // arg0 = 1: turn on the LED
    __ASM("SVC 0x01"); // SVC index = 1: turn LED output
}

/* This function invokes SVC index 1 with R0 = 0. See Cy_SysLib_SvcHandler */
void SVC_TurnLedPortOff(void)
{
    __ASM("LDR  r0, =0"); // arg0 = 0: turn off the LED
    __ASM("SVC 0x01"); // SVC index = 1: turn LED output
}

/* This function invokes SVC index 2. See Cy_SysLib_SvcHandler */
void SVC_SetPrivilegedMode(void)
{
    __ASM("SVC 0x02"); // SVC index = 2: Get privileged mode
}

int main(void)
{
    SystemInit();

    __enable_irq();

    /* Setting MPU so that only privileged cores can access the GPIO registers  */
    gMpuCfg[PRIVILEGED_PERI_REG_MPU_NO].addr = (uint32_t)PRIVILEGED_LED_PORT;
    Cy_MPU_Setup(gMpuCfg, SIZE_OF_MPU_SETTING, CY_MPU_USE_DEFAULT_MAP_AS_BG, CY_MPU_DISABLED_DURING_FAULT_NMI);

    /* The CPU works in user mode from here */
    SetUserMode();

    /* Access the restricted registers via SVC */
    SVC_InitPrivilegedLedPort();
    SVC_TurnLedPortOn();
    SVC_TurnLedPortOff();

    /* The CPU works in privileged mode from here */
    SVC_SetPrivilegedMode();

    /* Access the restricted registers directly  */
    Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 1);
    Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 0);

    /* The CPU works in user mode again from here */
    SetUserMode();

    /* Access the restricted registers in user mode (it will cause the hard fault) */
    Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 1);
    Cy_GPIO_Write(PRIVILEGED_LED_PORT, PRIVILEGED_LED_PIN, 0);

    for(;;);
}



/* [] END OF FILE */
