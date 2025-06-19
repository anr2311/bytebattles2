/***************************************************************************//**
* \file main_cm7_0.c
*
* \version 1.0
*
* \brief Main example file for CM7_0
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include <stdio.h>

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/

//=============================================================================================
// The SMIF init step may cause issues and lock up the system if there are issues with the
// SMIF devices (e.g. not present on the connected board, or disabled by jumpers, etc.)
// With following defines you can quickly include/exclude SMIF init:
// Just set SMIF0_EXT_FREQ_MHZ/SMIF1_EXT_FREQ_MHZ to 0 or non-0
//=============================================================================================

#if (CY_USE_PSVP == 1)
    #define DISP0_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL

    #define SMIF0_EXT_FREQ_MHZ      0
    #define SMIF1_EXT_FREQ_MHZ      0
#else
    #define DISP0_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_FPD_VESA  // or CY_GFXENV_DISP_TYPE_800_480_60_FPD_JEIDA
  #ifdef VIDEOSS0_FPDLINK1
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_FPD_VESA  // or CY_GFXENV_DISP_TYPE_800_480_60_FPD_JEIDA
  #else
    #define DISP1_TYPE              CY_GFXENV_DISP_TYPE_800_480_60_TTL
  #endif

  // Notes:
  // - Frequency ratio of SMIF0_EXT_FREQ_MHZ and SMIF1_EXT_FREQ_MHZ needs to be 1, 2, 4 or 8 (or 1, 0.5, 0.25, 0.125 respectively)
  // - Try reducing frequency in case of issues, maximum frequencies may only be applicable for boards with soldered MCUs
  //   or depending on the board/revision some modifications may be necessary (e.g. resistor removal to reduce stubs)    

  #if defined(tviic2d6m)
  
    // Only revision C of tviic2d6m and the corresponding 327-BGA board is supported!
    #define SMIF0_EXT_FREQ_MHZ      185
    #define SMIF1_EXT_FREQ_MHZ      185
    
  #elif defined(tviic2d4m)

    #define SMIF0_EXT_FREQ_MHZ      133
    #define SMIF1_EXT_FREQ_MHZ      133
    
  #endif
#endif


#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_LED8_PORT
    #define USER_LED_PIN            CY_LED8_PIN
    #define USER_LED_PIN_MUX        CY_LED8_PIN_MUX
#else
    #define USER_LED_PORT           CY_USER_LED2_PORT
    #define USER_LED_PIN            CY_USER_LED2_PIN
    #define USER_LED_PIN_MUX        CY_USER_LED2_PIN_MUX 
#endif

#define LED_TOGGLE_TIMER_ID         0
#define EXTMEM_TEST_TIMER_ID        1

#define EXTMEM_TEST_SIZE_KB         512
#define EXTMEM_FLASH_BASE           0x60000000
#define EXTMEM_RAM_BASE             0x90000000

#define EXTMEM_FLASH_ENABLED        (SMIF0_EXT_FREQ_MHZ != 0)
#define EXTMEM_RAM_ENABLED          (SMIF1_EXT_FREQ_MHZ != 0)

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/
cy_stc_gpio_pin_config_t user_led_port_pin_cfg =             
{                                                  
    .outVal = 0x00,                                
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,    
    .hsiom = USER_LED_PIN_MUX,                           
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


cy_stc_mpu_region_cfg_t astcMpuCfg[] =
{
    {.addr = EXTMEM_FLASH_BASE, .size = CY_MPU_SIZE_256MB, .permission = CY_MPU_ACCESS_P_FULL_ACCESS, .attribute = CY_MPU_ATTR_NORM_MEM_WB, .execute = CY_MPU_INST_ACCESS_EN, .srd = 0x00u, .enable = CY_MPU_ENABLE},
    {.addr = EXTMEM_RAM_BASE,   .size = CY_MPU_SIZE_256MB, .permission = CY_MPU_ACCESS_P_FULL_ACCESS, .attribute = CY_MPU_ATTR_NORM_MEM_WB, .execute = CY_MPU_INST_ACCESS_EN, .srd = 0x00u, .enable = CY_MPU_ENABLE},
};


/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/

/*****************************************************************************
* Local function prototypes ('static')
*****************************************************************************/
static void TestExtMemories(void);
static void LedToggleCallback(void);
static void ButtonCallback(uint8_t u8ButtonId, cy_button_en_state_t enState);

/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/

// Init config struct for GfxEnv flexible init mode
static const cy_gfxenv_stc_cfg_t m_stcGfxEnvCfg = 
{
    .bInitSwTimer         = true,
    .bInitSemihosting     = true,    
    .pstcInitPortPins     = &(cy_gfxenv_stc_init_portpins_t)
                            {
                              #if (CY_USE_PSVP == 0)
                                .bInitDisplay0Ttl       = false, // usually the FPD-Link is used on silicon
                              #else
                                .bInitDisplay0Ttl       = true,  // FPD-Link #0 is not available on PSVP
                              #endif
                              #if (CY_USE_PSVP == 0) && defined(VIDEOSS0_FPDLINK1)
                                .bInitDisplay1Ttl       = false, // usually the FPD-Link is used on silicon (if it is available)
                              #else
                                .bInitDisplay1Ttl       = true,  // FPD-Link #1 is not available on PSVP or on silicon for this device
                              #endif
                                .bInitCapture0Ttl       = false,
                              #if (SMIF0_EXT_FREQ_MHZ == 0)
                                .bInitSmif0             = false,
                              #else
                                .bInitSmif0             = true,  
                              #endif
                              #if (SMIF1_EXT_FREQ_MHZ == 0)
                                .bInitSmif1             = false,
                              #else
                                .bInitSmif1             = true,  
                              #endif
                                .bInitBacklightDisp0    = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                .bInitBacklightDisp1    = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                .bInitBacklightFpdLink0 = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                .bInitBacklightFpdLink1 = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                .bInitButtonGpios       = true,
                            },
  #if (SMIF0_EXT_FREQ_MHZ == 0) && (SMIF1_EXT_FREQ_MHZ == 0)
    .pstcInitExtMem       = NULL,
  #else
    .pstcInitExtMem       = &(cy_gfxenv_stc_init_extmem_t)
                            {
                                .bInitMpuTemporarily = true,
                                .u8MpuRegionTemp     = CY_GFXENV_SMIF_TEMP_MPU_REGION,
                                .apstcInitSmif       = {
                                                          #if (SMIF0_EXT_FREQ_MHZ == 0)
                                                            NULL,
                                                          #else
                                                            &(cy_gfxenv_stc_smif_cfg_t)
                                                            {
                                                                .u8SmifExtClockMhz = SMIF0_EXT_FREQ_MHZ,
                                                                .apstcDevUsrCfg    = {
                                                                                         &(cy_gfxenv_stc_smif_device_cfg_t)
                                                                                         {
                                                                                             .u32BaseAddress           = EXTMEM_FLASH_BASE,
                                                                                             .u16SizeMegaBytes         = 64,
                                                                                             .enExtMemType             = CY_GFXENV_EXTMEM_TYPE_S26H,
                                                                                             .i32CalibrationDataOffset = -16, // use last 16 byte of memory
                                                                                             .bEraseIfNecessary        = true,
                                                                                             .bUseDifferentialClock    = false,
                                                                                         },
                                                                                         NULL,
                                                                                     },
                                                            },
                                                          #endif
                                                          #if (SMIF1_EXT_FREQ_MHZ == 0)
                                                            NULL,
                                                          #else
                                                            &(cy_gfxenv_stc_smif_cfg_t)
                                                            {
                                                                .u8SmifExtClockMhz = SMIF1_EXT_FREQ_MHZ,
                                                                .apstcDevUsrCfg    = {
                                                                                        NULL,
                                                                                        &(cy_gfxenv_stc_smif_device_cfg_t)
                                                                                        {
                                                                                            .u32BaseAddress           = EXTMEM_RAM_BASE,
                                                                                            .u16SizeMegaBytes         = 8,
                                                                                            .enExtMemType             = CY_GFXENV_EXTMEM_TYPE_S27KXXXX2,
                                                                                            .i32CalibrationDataOffset = -16, // use last 16 byte of memory
                                                                                            .bEraseIfNecessary        = false, // don't care for RAM
                                                                                            .bUseDifferentialClock    = false,
                                                                                        },
                                                                                    },
                                                            },
                                                          #endif
                                                        },
                            },
                          #endif
    .pstcInitButtons      = &(cy_gfxenv_stc_init_buttons_t)
                            {
                                .u8CySwTimerId = (CY_SWTMR_MAX_TIMERS - 1), // use last timer
                                .pfnCallback = ButtonCallback,
                            },                            
};

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/


/*******************************************************************************
* Function Name: main
****************************************************************************//**
*
* \brief  Main function
*
* \return does not return
*
*******************************************************************************/
int main(void)
{
    __enable_irq();

    SystemInit();    

    // Ensure that the address space for the external memories is assigned the desired memory attributes (e.g. cacheable)
    Cy_MPU_Setup(astcMpuCfg, CY_GET_ARRAY_LEN(astcMpuCfg), CY_MPU_USE_DEFAULT_MAP_AS_BG, CY_MPU_DISABLED_DURING_FAULT_NMI);

    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &user_led_port_pin_cfg);

    // Enable clocks/pin/buttons/semihosting/extmem
    cy_gfxenv_en_result_t result;
    result = Cy_GfxEnv_Init(&m_stcGfxEnvCfg);
    CY_ASSERT(result == CY_GFXENV_SUCCESS);
    
    // Output test picture on Display #0 and #1
    result = Cy_GfxEnv_EnableTestImage(DISP0_TYPE, DISP1_TYPE);
    CY_ASSERT(result == CY_GFXENV_SUCCESS);
    
    printf("Hello World via UART Semihosting!\n");
    
    Cy_SwTmr_Wait(0, 500, NULL); // Wait 500 ms
    
    // Start a periodic SW timer with 500ms interval
    Cy_SwTmr_Start(LED_TOGGLE_TIMER_ID, 500, true, LedToggleCallback);
    
    // Start a periodic SW timer with 100ms interval
    Cy_SwTmr_Start(EXTMEM_TEST_TIMER_ID, 100, true, TestExtMemories);

    for(;;)
    {
        // Poll and operate on expired (non high-prio) SW timers
        Cy_SwTmr_Main();
    }
}


/*******************************************************************************
* Function Name: TestExtMemories
****************************************************************************//**
*
* \brief  Calculates checksum of the content of the enabled memories and compares
*         with expected checksum. Updates test pattern in RAM connected to SMIF1
*
* \return none
*
*******************************************************************************/
static void TestExtMemories(void)
{
    static bool     bFirstRun = true;
    static bool     bFirstChecksumMismatchDetected = false;
    static uint32_t u32ExpectedChecksumExtFlash = 0;
    static uint32_t u32Counter = 0;
    
    uint32_t u32ExpectedChecksumExtRam = 0;    
    uint32_t u32ActualChecksumExtFlash = 0;
    uint32_t u32ActualChecksumExtRam   = 0;
    
    if(EXTMEM_FLASH_ENABLED)
    {
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            u32ActualChecksumExtFlash += ((uint32_t *) EXTMEM_FLASH_BASE)[u32Word];
        }        
    }
    
    if(bFirstRun == true)
    {
        bFirstRun = false;
        u32ExpectedChecksumExtFlash = u32ActualChecksumExtFlash;
        return;
    }    
    
    if(EXTMEM_RAM_ENABLED)
    {
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            uint32_t u32Pattern = ((uint16_t)(u32Word + u32Counter) << 16) + (uint16_t) (u32Word + u32Counter);
            ((uint32_t *) EXTMEM_RAM_BASE)[u32Word] = u32Pattern;
            u32ExpectedChecksumExtRam += u32Pattern;
        }
        
        for(uint32_t u32Word = 0; u32Word < (EXTMEM_TEST_SIZE_KB * 1024UL / 4); u32Word++)
        {
            u32ActualChecksumExtRam += ((uint32_t *) EXTMEM_RAM_BASE)[u32Word];
        }
    }
    
    if((u32ExpectedChecksumExtFlash != u32ActualChecksumExtFlash) ||
       (u32ExpectedChecksumExtRam   != u32ActualChecksumExtRam)     )
    {
        if(bFirstChecksumMismatchDetected == false)
        {
            bFirstChecksumMismatchDetected = true;
            Cy_SwTmr_Stop(LED_TOGGLE_TIMER_ID);
            // Restart timer with higher frequency to indicate the mismatch
            // Start a periodic SW timer with 100ms interval
            Cy_SwTmr_Start(LED_TOGGLE_TIMER_ID, 100, true, LedToggleCallback);
        }
    }
    
    u32Counter++;
}


/*******************************************************************************
* Function Name: LedToggleCallback
****************************************************************************//**
*
* \brief  Toggles an LED
*
* \return none
*
*******************************************************************************/
static void LedToggleCallback(void)
{
    Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);    
}



/*******************************************************************************
* Function Name: ButtonCallback
****************************************************************************//**
*
* \brief  Handles button events
*
* \param  u8ButtonId : ID of the button that caused the callback
* \param  enState    : New state of the button
*
* \return none
*
*******************************************************************************/
static void ButtonCallback(uint8_t u8ButtonId, cy_button_en_state_t enState)
{
    // Return if button was released only
    if (enState == StateHigh)
    {
        return;
    }

    // Handle the pressed button
    switch (u8ButtonId)
    {
        case CY_GFXENV_BUTTON_ID_AB_CB:  
            break;
        case CY_GFXENV_BUTTON_ID_CEB:
            break;         
        case CY_GFXENV_BUTTON_ID_WAKE:
        case CY_GFXENV_BUTTON_ID_BB_1:
            break;
        case CY_GFXENV_BUTTON_ID_LEFT:
        case CY_GFXENV_BUTTON_ID_BB_2:
            break;
        case CY_GFXENV_BUTTON_ID_RIGHT:
        case CY_GFXENV_BUTTON_ID_BB_3:
            break;
        case CY_GFXENV_BUTTON_ID_UP:
        case CY_GFXENV_BUTTON_ID_BB_4:
            break;
        case CY_GFXENV_BUTTON_ID_DOWN:
        case CY_GFXENV_BUTTON_ID_BB_5:
            break;
        default:
            break;
    }
}

/* [] END OF FILE */
