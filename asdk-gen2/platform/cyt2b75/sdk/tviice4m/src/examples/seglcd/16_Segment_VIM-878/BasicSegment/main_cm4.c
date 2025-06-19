/***************************************************************************//**
* \file main_cm0plus.c
*
* \brief
* Main file for CM0+
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
#include "cycfg_seglcd.h"


/* Scenario: user LED definition */
#if (CY_USE_PSVP == 1)
    #define USER_LED_PORT           CY_BSP_BB_USER_LED0_PORT
    #define USER_LED_PIN            CY_BSP_BB_USER_LED0_PIN
    #define USER_LED_PIN_MUX        CY_BSP_BB_USER_LED0_PIN_MUX
#else
    #define USER_LED_PORT           CY_BSP_USER_LED2_PORT
    #define USER_LED_PIN            CY_BSP_USER_LED2_PIN
    #define USER_LED_PIN_MUX        CY_BSP_USER_LED2_PIN_MUX 
#endif

/* Scenario: limit the char mapping by this macro */
#define AVAILABLE_CHAR_TO_DISPLAY   5

/* Scenario: application io pin structure */
typedef struct
{
    volatile stc_GPIO_PRT_t*  port;
    uint8_t                   pin;
    en_hsiom_sel_t            hsiom;
    uint32_t                  driveMode;
} cy_stc_app_port_t;

/* Scenario: application io pin details */
cy_stc_app_port_t appPortCfg[] =
{   
    /* User LED pin details */
    { USER_LED_PORT,            USER_LED_PIN,           USER_LED_PIN_MUX,               CY_GPIO_DM_STRONG_IN_OFF },
    /* LCD segment pin details */
    { CY_BSP_LCD_SEG_0_PORT,    CY_BSP_LCD_SEG_0_PIN,   CY_BSP_LCD_SEG_0_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_1_PORT,    CY_BSP_LCD_SEG_1_PIN,   CY_BSP_LCD_SEG_1_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_2_PORT,    CY_BSP_LCD_SEG_2_PIN,   CY_BSP_LCD_SEG_2_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_3_PORT,    CY_BSP_LCD_SEG_3_PIN,   CY_BSP_LCD_SEG_3_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_4_PORT,    CY_BSP_LCD_SEG_4_PIN,   CY_BSP_LCD_SEG_4_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_5_PORT,    CY_BSP_LCD_SEG_5_PIN,   CY_BSP_LCD_SEG_5_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_6_PORT,    CY_BSP_LCD_SEG_6_PIN,   CY_BSP_LCD_SEG_6_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_7_PORT,    CY_BSP_LCD_SEG_7_PIN,   CY_BSP_LCD_SEG_7_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_8_PORT,    CY_BSP_LCD_SEG_8_PIN,   CY_BSP_LCD_SEG_8_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_9_PORT,    CY_BSP_LCD_SEG_9_PIN,   CY_BSP_LCD_SEG_9_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_10_PORT,   CY_BSP_LCD_SEG_10_PIN,  CY_BSP_LCD_SEG_10_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_11_PORT,   CY_BSP_LCD_SEG_11_PIN,  CY_BSP_LCD_SEG_11_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_12_PORT,   CY_BSP_LCD_SEG_12_PIN,  CY_BSP_LCD_SEG_12_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_13_PORT,   CY_BSP_LCD_SEG_13_PIN,  CY_BSP_LCD_SEG_13_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_14_PORT,   CY_BSP_LCD_SEG_14_PIN,  CY_BSP_LCD_SEG_14_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_15_PORT,   CY_BSP_LCD_SEG_15_PIN,  CY_BSP_LCD_SEG_15_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_16_PORT,   CY_BSP_LCD_SEG_16_PIN,  CY_BSP_LCD_SEG_16_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_17_PORT,   CY_BSP_LCD_SEG_17_PIN,  CY_BSP_LCD_SEG_17_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_18_PORT,   CY_BSP_LCD_SEG_18_PIN,  CY_BSP_LCD_SEG_18_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_19_PORT,   CY_BSP_LCD_SEG_19_PIN,  CY_BSP_LCD_SEG_19_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_20_PORT,   CY_BSP_LCD_SEG_20_PIN,  CY_BSP_LCD_SEG_20_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_21_PORT,   CY_BSP_LCD_SEG_21_PIN,  CY_BSP_LCD_SEG_21_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_22_PORT,   CY_BSP_LCD_SEG_22_PIN,  CY_BSP_LCD_SEG_22_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_23_PORT,   CY_BSP_LCD_SEG_23_PIN,  CY_BSP_LCD_SEG_23_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_24_PORT,   CY_BSP_LCD_SEG_24_PIN,  CY_BSP_LCD_SEG_24_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_25_PORT,   CY_BSP_LCD_SEG_25_PIN,  CY_BSP_LCD_SEG_25_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_26_PORT,   CY_BSP_LCD_SEG_26_PIN,  CY_BSP_LCD_SEG_26_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_27_PORT,   CY_BSP_LCD_SEG_27_PIN,  CY_BSP_LCD_SEG_27_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_28_PORT,   CY_BSP_LCD_SEG_28_PIN,  CY_BSP_LCD_SEG_28_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_29_PORT,   CY_BSP_LCD_SEG_29_PIN,  CY_BSP_LCD_SEG_29_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_30_PORT,   CY_BSP_LCD_SEG_30_PIN,  CY_BSP_LCD_SEG_30_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_SEG_31_PORT,   CY_BSP_LCD_SEG_31_PIN,  CY_BSP_LCD_SEG_31_PIN_MUX,      CY_GPIO_DM_STRONG_IN_OFF },
    /* LCD com pin details */
    { CY_BSP_LCD_COM_0_PORT,    CY_BSP_LCD_COM_0_PIN,   CY_BSP_LCD_COM_0_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_COM_1_PORT,    CY_BSP_LCD_COM_1_PIN,   CY_BSP_LCD_COM_1_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_COM_2_PORT,    CY_BSP_LCD_COM_2_PIN,   CY_BSP_LCD_COM_2_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
    { CY_BSP_LCD_COM_3_PORT,    CY_BSP_LCD_COM_3_PIN,   CY_BSP_LCD_COM_3_PIN_MUX,       CY_GPIO_DM_STRONG_IN_OFF },
};
#define SIZE_OF_PORT (sizeof(appPortCfg)/sizeof(cy_stc_app_port_t))

/* Scenario: Enable an LCD block */
cy_stc_seglcd_config_t config =
{
    .speed      = CY_SEGLCD_SPEED_HIGH,
    .wave       = CY_SEGLCD_TYPE_B,
    .drive      = CY_SEGLCD_PWM,
    .bias       = CY_SEGLCD_BIAS_FOURTH,
    .lsClk      = CY_SEGLCD_LSCLK_LF,
    .comNum     = SegLCD_COM_NUM,
    .frRate     = 250,
    .contrast   = 70,
    .clkFreq    = 2000000, 
};

/* Scenario: available common */
const uint32_t commons[SegLCD_COM_NUM] =
{
    LCD_Seg_COM0_ROW,
    LCD_Seg_COM1_ROW,
    LCD_Seg_COM2_ROW,
    LCD_Seg_COM3_ROW,
};

/* Scenario: available character map common */
static const uint16_t fontMap16_custom[AVAILABLE_CHAR_TO_DISPLAY] =
{
    /* SegLCD_Display0_pixMap                                                                                                           DP CA N  M      L K J I      H G F E      D C B A       */
    {LCD_Seg_PIXEL_H | LCD_Seg_PIXEL_J},                                                                                // '\/'         0  0  0  0      0 0 1 0      1 0 0 0      0 0 0 0
    {LCD_Seg_PIXEL_L | LCD_Seg_PIXEL_N},                                                                                // '/\'         0  0  1  0      1 0 0 0      0 0 0 0      0 0 0 0
    {LCD_Seg_PIXEL_G | LCD_Seg_PIXEL_H | LCD_Seg_PIXEL_N},                                                              // '->'         0  0  1  0      0 0 0 0      1 1 0 0      0 0 0 0
    {LCD_Seg_PIXEL_J | LCD_Seg_PIXEL_K | LCD_Seg_PIXEL_L},                                                              // '<-'         0  0  0  0      1 1 1 0      0 0 0 0      0 0 0 0
    {LCD_Seg_PIXEL_B | LCD_Seg_PIXEL_C | LCD_Seg_PIXEL_E | LCD_Seg_PIXEL_F | LCD_Seg_PIXEL_I | LCD_Seg_PIXEL_M},        // '|||'        0  0  0  1      0 0 0 1      0 0 1 1      0 1 1 0
};

/* Scenario: application io init function */
static void Cy_AppPortInit(cy_stc_app_port_t cfg[], uint8_t size)
{
    cy_stc_gpio_pin_config_t pinCfg = {0};
    for(uint32_t i = 0; i < size; i++)
    {
        pinCfg.driveMode = cfg[i].driveMode;
        pinCfg.hsiom     = cfg[i].hsiom;
        Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
    }
}

/* Scenario: function to set character on display */
static void Cy_AppWriteCharOnDisplay(stc_LCD_t * base, uint8_t character, uint32_t position)
{
    if (character < AVAILABLE_CHAR_TO_DISPLAY)               /* if out of the font range */
    {
        for (uint8_t i = 0UL; i < SegLCD_Display0_TYPE; i++)
        {
            if(fontMap16_custom[character] & (1 << i))
            {
                Cy_SegLCD_SetPixel(base, SegLCD_Display0_pixMap[position][i]);
            }
        }
    } 
}


/* Scenario: application main */
int main(void)
{   
    char_t const * devString = "TVII-CPU";
    char_t const * brdString = "CE4M-176";
  
    SystemInit();
  
    __enable_irq();

    /* Place your initialization/startup code here (e.g. MyInst_Start()) */
    Cy_AppPortInit(appPortCfg, SIZE_OF_PORT);

    /* Scenario: Enable peripheral clock divider for LCD operation with 
     * the 2MHz clock frequency at the PeriClk frequency 80MHz
     */
    Cy_SysClk_PeriphAssignDivider(PCLK_LCD0_CLOCK, CY_SYSCLK_DIV_8_BIT, 0U);
    Cy_SysClk_PeriphSetDivider(CY_SYSCLK_DIV_8_BIT, 0U, 39U);
    Cy_SysClk_PeriphEnableDivider(CY_SYSCLK_DIV_8_BIT, 0U);
    
    /* Initialize LCD */
    if (CY_SEGLCD_SUCCESS == Cy_SegLCD_Init((stc_LCD_t *)SegLCD_HW, &config))
    {
        if (CY_SEGLCD_SUCCESS == Cy_SegLCD_ClrFrame((stc_LCD_t *)SegLCD_HW, commons))
        {
            Cy_SegLCD_Enable((stc_LCD_t *)SegLCD_HW);
            
           /* Now the block generates LCD signals (all the pixels are off) and is ready to turn on any pixel
            * (or many pixels) using any of the frame/pixel/character/display management API functions.
            */
        }
        else
        {
            /* error handling */
            while(1);
        }
    }
    else
    {
        /* error handling */
        while(1);
    }
    
    /* Scenario: Set a pixel (write a true value) */
    if (CY_SEGLCD_SUCCESS != Cy_SegLCD_SetPixel((stc_LCD_t *)SegLCD_HW, LCD_Seg_H16S1_DP))
    {
        /* error handling */
        while(1);
    }
    
    /* Scenario: Clear a pixel (write a false value) */
    if (CY_SEGLCD_SUCCESS != Cy_SegLCD_ClrPixel((stc_LCD_t *)SegLCD_HW, LCD_Seg_H16S1_DP))
    {
        /* error handling */
        while(1); 
    }
    
    /* Clear complete frame */
    Cy_SegLCD_ClrFrame((stc_LCD_t *)SegLCD_HW, commons);
    
    /* Scenario: Set characters on the display */
    for(uint8_t symbol = 0; symbol < SegLCD_Display0_SYM_NUM; symbol++)
    {
        Cy_SegLCD_WriteChar((stc_LCD_t *)SegLCD_HW, devString[symbol], symbol, &SegLCD_Display0);
    }
    
    /* Wait 0.5 [s] */
    Cy_SysTick_DelayInUs(2000000);
    
    /* Clear complete frame */
    Cy_SegLCD_ClrFrame((stc_LCD_t *)SegLCD_HW, commons);
    
    /* Scenario: Set string on the display */
    Cy_SegLCD_WriteString((stc_LCD_t *)SegLCD_HW, brdString, 0, &SegLCD_Display0);

    /* Wait 0.5 [s] */
    Cy_SysTick_DelayInUs(2000000);
    
    /* Clear complete frame */
    Cy_SegLCD_ClrFrame((stc_LCD_t *)SegLCD_HW, commons);

    for(;;)
    {
        /* Example of the handling custom segments */
        /* Scenario: start sequence of digits */
        for(uint8_t availChar = 0; availChar < AVAILABLE_CHAR_TO_DISPLAY; availChar++)
        {
            /* Clear complete frame */
            Cy_SegLCD_ClrFrame((stc_LCD_t *)SegLCD_HW, commons);
            
            /* Write digit on all symbol of display */
            for(uint8_t symbol = 0; symbol < SegLCD_Display0_SYM_NUM; symbol++)
            {
                /* write digit on each symbol */
                Cy_AppWriteCharOnDisplay((stc_LCD_t *)SegLCD_HW, availChar, symbol);
            }
            
            /* Wait 0.5 [s] */
            Cy_SysTick_DelayInUs(1000000);
        }

        /* Blink an user LED */
        Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);
    }
}


/* [] END OF FILE */
