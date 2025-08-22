/***************************************************************************//**
* \file system_tviic2d6mddr_cm0plus.c
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

#if CY_SYSTEM_USE_CLOCK == CY_SYSTEM_USE_ECO
    #define CY_SYSTEM_PLL_INPUT_SOURCE       CY_SYSCLK_CLKPATH_IN_ECO

    // PLL#0 -> PLL400#0 (CLK_HF1, CM7s: 320MHz)
  #if (CY_SYS_PLL400M_0_FREQ == CY_SYS_PLL400M_0_320MHz)
    // PLL_OUT = 16,000,000(Feco) / 2 * 80 / 2 = 320,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 80 / 2 = 640,000,000.
    #define CY_SYSTEM_PLL0_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL0_CONFIG_FEEDBACKDIV (80UL)
    #define CY_SYSTEM_PLL0_CONFIG_OUTDIV      (2UL)
  #else
    // PLL_OUT = 16,000,000(Feco) / 1 * 30 / 3 = 160,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 30 / 1 = 480,000,000.
    #define CY_SYSTEM_PLL0_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL0_CONFIG_FEEDBACKDIV (30UL)
    #define CY_SYSTEM_PLL0_CONFIG_OUTDIV      (3UL)
  #endif

    // PLL#1 -> PLL400#1 (CLK_HF5/6/7, SS#0/1/2: 24.575MHz)
    // PLL_OUT = 16,000,000(Feco) / 2 * 73.72799985 / 3 = 196607999.6Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 73.72799985 / 2 = 589824000.
    #define CY_SYSTEM_PLL1_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL1_CONFIG_FB_INT      (73UL)
    #define CY_SYSTEM_PLL1_CONFIG_FB_FRAC     (12213813UL)
    #define CY_SYSTEM_PLL1_CONFIG_OUTDIV      (3UL)

    // PLL#2 -> PLL400#2 (CLK_HF8/9, SMIF#0/1: 360MHz)
    // PLL_OUT = 16,000,000(Feco) / 1 * 45 / 2 = 360,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 45 / 1 = 720,000,000.
    #define CY_SYSTEM_PLL2_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL2_CONFIG_FEEDBACKDIV (45UL)
    #define CY_SYSTEM_PLL2_CONFIG_OUTDIV      (2UL)

    // PLL#3 -> PLL400#3 (CLK_HF10, VIDEOSS: 266MHz)
    // PLL_OUT = 16,000,000(Feco) / 4 * 133 / 2 = 266,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 133 / 4 = 532,000,000.
    #define CY_SYSTEM_PLL3_CONFIG_REFDIV      (4UL)
    #define CY_SYSTEM_PLL3_CONFIG_FEEDBACKDIV (133UL)
    #define CY_SYSTEM_PLL3_CONFIG_OUTDIV      (2UL)

    // PLL#4 -> PLL400#4 (CLK_HF11, DISP#0: 220MHz)
    // PLL_OUT = 16,000,000(Feco) / 2 * 55 / 4 = 220,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 16,000,000 * 55 / 2 = 440,000,000.
    #define CY_SYSTEM_PLL4_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL4_CONFIG_FEEDBACKDIV (55UL)
    #define CY_SYSTEM_PLL4_CONFIG_OUTDIV      (2UL)

    // PLL#5 -> PLL200#0 
    // ([CLK_HF0, CM0+/CLK_MEM: 160MHz], [CLK_HF3, EVTGEN: 160MHz], [CLK_HF4, ETH: 40MHz], 
    //  [CLK_HF13, SDHC: 80MHz], [CLK_HF14, LPDDR4: 20MHz])
    // PLL_OUT = 16,000,000(Feco) / 2 * 40 / 2 = 160,000,000Hz
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 16,000,000 * 40 / 2 = 320,000,000.
    #define CY_SYSTEM_PLL5_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL5_CONFIG_FEEDBACKDIV (40UL)
    #define CY_SYSTEM_PLL5_CONFIG_OUTDIV      (2UL)

    // PLL#6 -> PLL200#1 (CLK_HF2, CLK_PERI: 80MHz)
    // PLL_OUT = 16,000,000(Feco) / 2 * 40 / 4 = 80,000,000Hz (max. would be 100 MHz)
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 16,000,000 * 40 / 2 = 320,000,000.
    #define CY_SYSTEM_PLL6_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL6_CONFIG_FEEDBACKDIV (40UL)
    #define CY_SYSTEM_PLL6_CONFIG_OUTDIV      (4UL)

    // PLL#7 -> PLL200#2 (CLK_HF12, DISP#1: 200MHz)
    // PLL_OUT = 16,000,000(Feco) / 2 * 50 / 2 = 200,000,000Hz
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 16,000,000 * 50 / 2 = 400,000,000.
    #define CY_SYSTEM_PLL7_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL7_CONFIG_FEEDBACKDIV (50UL)
    #define CY_SYSTEM_PLL7_CONFIG_OUTDIV      (2UL)

#elif CY_SYSTEM_USE_CLOCK == CY_SYSTEM_USE_IMO
    #define CY_SYSTEM_PLL_INPUT_SOURCE CY_SYSCLK_CLKPATH_IN_IMO

    // PLL#0 -> PLL400#0 (CLK_HF1, CM7s: 320MHz)
  #if (CY_SYS_PLL400M_0_FREQ == CY_SYS_PLL400M_0_320MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 80 / 2 = 320,000,000Hz
    // Restriction: 400,000,000 < Fvco < 800,000,000
    // This time, Fvco = 8,000,000 * 80 / 1 = 640,000,000.
    #define CY_SYSTEM_PLL0_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL0_CONFIG_FEEDBACKDIV (80UL)
    #define CY_SYSTEM_PLL0_CONFIG_OUTDIV      (2UL)
  #else
    // PLL_OUT = 8,000,000(Fimo) / 1 * 60 / 3 = 160,000,000Hz
    // Restriction: 400,000,000 < Fvco < 800,000,000
    // This time, Fvco = 8,000,000 * 60 / 1 = 480,000,000.
    #define CY_SYSTEM_PLL0_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL0_CONFIG_FEEDBACKDIV (60UL)
    #define CY_SYSTEM_PLL0_CONFIG_OUTDIV      (3UL)
  #endif

    // PLL#1 -> PLL400#1 (CLK_HF5/6/7, SS#0/1/2: 24.575MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 73.72799985 / 3 = 196607999.6Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 8,000,000 * 73.72799985 / 1 = 589824000.
    #define CY_SYSTEM_PLL1_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL1_CONFIG_FB_INT      (73UL)
    #define CY_SYSTEM_PLL1_CONFIG_FB_FRAC     (12213813UL)
    #define CY_SYSTEM_PLL1_CONFIG_OUTDIV      (3UL)

    // PLL#2 -> PLL400#2 (CLK_HF8/9, SMIF#0/1: 360MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 90 / 2 = 360,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 8,000,000 * 90 / 1 = 720,000,000.
    #define CY_SYSTEM_PLL2_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL2_CONFIG_FEEDBACKDIV (90UL)
    #define CY_SYSTEM_PLL2_CONFIG_OUTDIV      (2UL)

    // PLL#3 -> PLL400#3 (CLK_HF10, VIDEOSS: 250MHz)
    // PLL_OUT = 8,000,000(Fimo) / 2 * 125 / 2 = 250,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 8,000,000 * 125 / 2 = 500,000,000.
    #define CY_SYSTEM_PLL3_CONFIG_REFDIV      (2UL)
    #define CY_SYSTEM_PLL3_CONFIG_FEEDBACKDIV (125UL)
    #define CY_SYSTEM_PLL3_CONFIG_OUTDIV      (2UL)

    // PLL#4 -> PLL400#4 (CLK_HF11, DISP#0: 220MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 55 / 2 = 220,000,000Hz
    // Restriction: 400,000,000 <= Fvco <= 800,000,000
    // This time, Fvco = 8,000,000 * 55 / 1 = 440,000,000.
    #define CY_SYSTEM_PLL4_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL4_CONFIG_FEEDBACKDIV (55UL)
    #define CY_SYSTEM_PLL4_CONFIG_OUTDIV      (2UL)

    // PLL#5 -> PLL200#0 
    // ([CLK_HF0, CM0+/CLK_MEM: 160MHz], [CLK_HF3, EVTGEN: 160MHz], [CLK_HF4, ETH: 40MHz], 
    //  [CLK_HF13, SDHC: 80MHz], [CLK_HF14, LPDDR4: 20MHz])
    // PLL_OUT = 8,000,000(Fimo) / 1 * 40 / 2 = 160,000,000Hz
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 8,000,000 * 50 / 1 = 320,000,000.
    #define CY_SYSTEM_PLL5_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL5_CONFIG_FEEDBACKDIV (40UL)
    #define CY_SYSTEM_PLL5_CONFIG_OUTDIV      (2UL)

    // PLL#6 -> PLL200#1 (CLK_HF2, CLK_PERI: 80MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 40 / 4 = 80,000,000Hz (max. would be 100 MHz)
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 8,000,000 * 40 / 1 = 320,000,000.
    #define CY_SYSTEM_PLL6_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL6_CONFIG_FEEDBACKDIV (40UL)
    #define CY_SYSTEM_PLL6_CONFIG_OUTDIV      (4UL)

    // PLL#7 -> PLL200#2 (CLK_HF12, DISP#1: 200MHz)
    // PLL_OUT = 8,000,000(Fimo) / 1 * 50 / 2 = 200,000,000Hz
    // Restriction: 170,000,000 <= Fvco <= 400,000,000
    // This time, Fvco = 8,000,000 * 50 / 1 = 400,000,000.
    #define CY_SYSTEM_PLL7_CONFIG_REFDIV      (1UL)
    #define CY_SYSTEM_PLL7_CONFIG_FEEDBACKDIV (50UL)
    #define CY_SYSTEM_PLL7_CONFIG_OUTDIV      (2UL)

#elif CY_SYSTEM_USE_CLOCK == CY_SYSTEM_USE_EXT

    #warning "Not implemented yet!!"

#endif


#define CY_SYS_PWR_CTL_KEY_OPEN  (0x05FAUL)
#define CY_SYS_PWR_CTL_KEY_CLOSE (0xFA05UL)


/** Holds the SlowClk system core clock, which is the system clock frequency supplied to the SysTick timer and the
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


#if ((CY_USE_PSVP == 0u) && (CY_SYS_VCCD_SOURCE == CY_SYS_VCCD_PMIC))
#define TIMING_MONZA_PMIC_ENABLE            (4)  // 0: Enable fastest timing, 8:Enable latest timing
#define WAIT_CYCLE_WHILE_DISTRIBUTING_CLOCK (50)
#endif

/*******************************************************************************
* Function Name: SystemInit
****************************************************************************//**
*
* Initializes the system
*
*******************************************************************************/
void SystemInit (void)
{
#if defined(CY_SYSTEM_WDT_DISABLE)
    /* disable WDT */
    Cy_WDT_Disable();
#endif /* CY_SYSTEM_WDT_DISABLE */
    
#if (CY_USE_PSVP == 0u)
      
    /*********** Setting wait state for ROM **********/
    CPUSS->unROM_CTL.stcField.u2SLOW_WS = 1u;
    CPUSS->unROM_CTL.stcField.u2FAST_WS = 0u;

    /*********** Setting wait state for RAM **********/
    CPUSS->unRAM0_CTL0.stcField.u2SLOW_WS = 1u;
    CPUSS->unRAM0_CTL0.stcField.u2FAST_WS = 0u;

    #if defined (CPUSS_RAMC1_PRESENT) && (CPUSS_RAMC1_PRESENT == 1UL)
    CPUSS->unRAM1_CTL0.stcField.u2SLOW_WS = 1u;
    CPUSS->unRAM1_CTL0.stcField.u2FAST_WS = 0u;
    #endif /* defined (CPUSS_RAMC1_PRESENT) && (CPUSS_RAMC1_PRESENT == 1UL) */

    #if defined (CPUSS_RAMC2_PRESENT) && (CPUSS_RAMC2_PRESENT == 1UL)
    CPUSS->unRAM2_CTL0.stcField.u2SLOW_WS = 1u;
    CPUSS->unRAM2_CTL0.stcField.u2FAST_WS = 0u;
    #endif /* defined (CPUSS_RAMC2_PRESENT) && (CPUSS_RAMC2_PRESENT == 1UL) */

    /*********** Setting wait state for FLASH **********/
    FLASHC->unFLASH_CTL.stcField.u4WS = 1u;

    /***    Set clock LF source        ***/
    SRSS->unCLK_SELECT.stcField.u3LFCLK_SEL = CY_SYSCLK_LFCLK_IN_ILO0;

#if CY_SYSTEM_USE_CLOCK == CY_SYSTEM_USE_ECO

    /***    ECO port settings        ***/
    Cy_SysClk_EcoDisable();
    /* Default settings should be OK. */

    /* Apply trim for the ECO */
//    cy_en_sysclk_status_t ecoStatus;
//    ecoStatus = Cy_SysClk_EcoConfigureWithMinRneg(
//                       CY_CLK_ECO_FREQ_HZ,
//                       SUM_LOAD_SHUNT_CAP_IN_PF,
//                       ESR_IN_OHM,
//                       MAX_DRIVE_LEVEL_IN_UW,
//                       MIN_NEG_RESISTANCE
//                       );
//    CY_ASSERT(ecoStatus == CY_SYSCLK_SUCCESS);
    
    // These values need to be confirmed
    SRSS->unCLK_ECO_CONFIG2.stcField.u3WDTRIM = 0u;
    SRSS->unCLK_ECO_CONFIG2.stcField.u4ATRIM  = 7u;
    SRSS->unCLK_ECO_CONFIG2.stcField.u2FTRIM  = 3u;
    SRSS->unCLK_ECO_CONFIG2.stcField.u2RTRIM  = 3u;
    SRSS->unCLK_ECO_CONFIG2.stcField.u3GTRIM  = 3u;
    SRSS->unCLK_ECO_CONFIG.stcField.u1AGC_EN  = 0u;

    SRSS->unCLK_ECO_CONFIG.stcField.u1ECO_EN = 1ul;
    while(SRSS->unCLK_ECO_STATUS.stcField.u1ECO_OK == 0ul);
    while(SRSS->unCLK_ECO_STATUS.stcField.u1ECO_READY == 0ul);
#endif

#if defined(CY_SYSTEM_WCO_ENABLE)

    /* Enable WCO */
    BACKUP->unCTL.stcField.u1WCO_EN = 1ul;

    /* Wait until WCO status becomes OK */
    while(BACKUP->unSTATUS.stcField.u1WCO_OK == 0ul);
    
#else /* Configure for LPECO */
    
    /* Enable LPECO */
	Cy_SysClk_ClkBak_LPECO_Enable(true);
    
    /* Wait until LPECO status becomes OK */
	while (0ul == Cy_SysClk_ClkBak_LPECO_Ready());
    
#endif

    /***  Set CPUSS dividers as required        ***/
    /* CLK_MEM */
    CPUSS->unMEM_CLOCK_CTL.stcField.u8INT_DIV     = 0u; /* no division */

    /* CLK_SLOW */
    CPUSS->unSLOW_CLOCK_CTL.stcField.u8INT_DIV    = 1u; /* divided by 2 */

    /* CLK_PERI */
    CPUSS->unPERI_CLOCK_CTL.stcField.u8INT_DIV    = 1u; /* divided by 2 */

    /* CLK_TRC_DBG */
    CPUSS->unTRC_DBG_CLOCK_CTL.stcField.u8INT_DIV = 0u; /* divided by 1 */

    // No setting for CLK_GRx requred. Initial values (no division) should be ok.

    /***     PLL setting and enabling        ***/
    /* Settings for PLL400M #0 (CLK_PATH1) */
    SRSS->unCLK_PATH_SELECT[1/*PLL400M #0*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->CLK_PLL400M[0].unCONFIG2.stcField.u1FRAC_EN      = 0ul;     /* Disabled */

    SRSS->CLK_PLL400M[0].unCONFIG3.stcField.u10SSCG_DEPTH  = 0x029ul; /* -0.5% (down-spread) */
    SRSS->CLK_PLL400M[0].unCONFIG3.stcField.u3SSCG_RATE    = 0ul;     /* Modulation rate = fPFD/4096 */
    SRSS->CLK_PLL400M[0].unCONFIG3.stcField.u1SSCG_DITHER_EN = 1ul;   /* Enabled */
    SRSS->CLK_PLL400M[0].unCONFIG3.stcField.u1SSCG_MODE    = 0ul;     /* Write "0" always */
    SRSS->CLK_PLL400M[0].unCONFIG3.stcField.u1SSCG_EN      = 1ul;

    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u5REFERENCE_DIV = CY_SYSTEM_PLL0_CONFIG_REFDIV; 
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u8FEEDBACK_DIV  = CY_SYSTEM_PLL0_CONFIG_FEEDBACKDIV; 
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u5OUTPUT_DIV    = CY_SYSTEM_PLL0_CONFIG_OUTDIV; 
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u2LOCK_DELAY    = 1ul;     /* Fractional division or spreading */
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u2BYPASS_SEL    = 0ul;     /* Auto */
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u1ENABLE        = 1ul;

    /* Settings for PLL400M #1 (CLK_PATH2) */
    SRSS->unCLK_PATH_SELECT[2/*PLL400M #1*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->CLK_PLL400M[1].unCONFIG2.stcField.u24FRAC_DIV    = CY_SYSTEM_PLL1_CONFIG_FB_FRAC;
    SRSS->CLK_PLL400M[1].unCONFIG2.stcField.u3FRAC_DITHER_EN  = 1ul; /* Enabled */
    SRSS->CLK_PLL400M[1].unCONFIG2.stcField.u1FRAC_EN      = 1ul;    /* Enabled */

    SRSS->CLK_PLL400M[1].unCONFIG3.stcField.u1SSCG_EN      = 0ul;    /* Disabled */

    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u5REFERENCE_DIV = CY_SYSTEM_PLL1_CONFIG_REFDIV; 
    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u8FEEDBACK_DIV  = CY_SYSTEM_PLL1_CONFIG_FB_INT; 
    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u5OUTPUT_DIV    = CY_SYSTEM_PLL1_CONFIG_OUTDIV; 
    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u2LOCK_DELAY    = 1ul;    /* Fractional division or spreading */
    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u2BYPASS_SEL    = 0ul;    /* Auto */
    SRSS->CLK_PLL400M[1].unCONFIG.stcField.u1ENABLE        = 1ul;

    /* Settings for PLL400M #2 (CLK_PATH3) */
    SRSS->unCLK_PATH_SELECT[3/*PLL400M #2*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->CLK_PLL400M[2].unCONFIG2.stcField.u1FRAC_EN      = 0ul;     /* Disabled */

    SRSS->CLK_PLL400M[2].unCONFIG3.stcField.u10SSCG_DEPTH  = 0x029ul; /* -0.5% (down-spread) */
    SRSS->CLK_PLL400M[2].unCONFIG3.stcField.u3SSCG_RATE    = 0ul;     /* Modulation rate = fPFD/4096 */
    SRSS->CLK_PLL400M[2].unCONFIG3.stcField.u1SSCG_DITHER_EN = 1ul;   /* Enabled */
    SRSS->CLK_PLL400M[2].unCONFIG3.stcField.u1SSCG_MODE    = 0ul;     /* Write "0" always */
    SRSS->CLK_PLL400M[2].unCONFIG3.stcField.u1SSCG_EN      = 1ul;

    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u5REFERENCE_DIV = CY_SYSTEM_PLL2_CONFIG_REFDIV; 
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u8FEEDBACK_DIV  = CY_SYSTEM_PLL2_CONFIG_FEEDBACKDIV; 
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u5OUTPUT_DIV    = CY_SYSTEM_PLL2_CONFIG_OUTDIV; 
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u2LOCK_DELAY    = 1ul;     /* Fractional division or spreading */
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u2BYPASS_SEL    = 0ul;     /* Auto */
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u1ENABLE        = 1ul;

    /* Settings for PLL400M #3 (CLK_PATH4) */
    SRSS->unCLK_PATH_SELECT[4/*PLL400M #3*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->CLK_PLL400M[3].unCONFIG2.stcField.u1FRAC_EN      = 0ul;     /* Disabled */

    SRSS->CLK_PLL400M[3].unCONFIG3.stcField.u10SSCG_DEPTH  = 0x029ul; /* -0.5% (down-spread) */
    SRSS->CLK_PLL400M[3].unCONFIG3.stcField.u3SSCG_RATE    = 0ul;     /* Modulation rate = fPFD/4096 */
    SRSS->CLK_PLL400M[3].unCONFIG3.stcField.u1SSCG_DITHER_EN = 1ul;   /* Enabled */
    SRSS->CLK_PLL400M[3].unCONFIG3.stcField.u1SSCG_MODE    = 0ul;     /* Write "0" always */
    SRSS->CLK_PLL400M[3].unCONFIG3.stcField.u1SSCG_EN      = 1ul;

    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u5REFERENCE_DIV = CY_SYSTEM_PLL3_CONFIG_REFDIV; 
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u8FEEDBACK_DIV  = CY_SYSTEM_PLL3_CONFIG_FEEDBACKDIV; 
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u5OUTPUT_DIV    = CY_SYSTEM_PLL3_CONFIG_OUTDIV; 
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u2LOCK_DELAY    = 1ul;     /* Fractional division or spreading */
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u2BYPASS_SEL    = 0ul;     /* Auto */
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u1ENABLE        = 1ul;

    /* Settings for PLL400M #4 (CLK_PATH5) */
    SRSS->unCLK_PATH_SELECT[5/*PLL400M #4*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->CLK_PLL400M[4].unCONFIG2.stcField.u1FRAC_EN      = 0ul;     /* Disabled */

    SRSS->CLK_PLL400M[4].unCONFIG3.stcField.u10SSCG_DEPTH  = 0x029ul; /* -0.5% (down-spread) */
    SRSS->CLK_PLL400M[4].unCONFIG3.stcField.u3SSCG_RATE    = 0ul;     /* Modulation rate = fPFD/4096 */
    SRSS->CLK_PLL400M[4].unCONFIG3.stcField.u1SSCG_DITHER_EN = 1ul;   /* Enabled */
    SRSS->CLK_PLL400M[4].unCONFIG3.stcField.u1SSCG_MODE    = 0ul;     /* Write "0" always */
    SRSS->CLK_PLL400M[4].unCONFIG3.stcField.u1SSCG_EN      = 1ul;

    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u5REFERENCE_DIV = CY_SYSTEM_PLL4_CONFIG_REFDIV; 
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u8FEEDBACK_DIV  = CY_SYSTEM_PLL4_CONFIG_FEEDBACKDIV; 
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u5OUTPUT_DIV    = CY_SYSTEM_PLL4_CONFIG_OUTDIV; 
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u2LOCK_DELAY    = 1ul;     /* Fractional division or spreading */
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u2BYPASS_SEL    = 0ul;     /* Auto */
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u1ENABLE        = 1ul;


    /* Settings for PLL200 #0 (CLK_PATH6) */
    SRSS->unCLK_PATH_SELECT[6/*PLL200 #0*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->unCLK_PLL_CONFIG[0].stcField.u7FEEDBACK_DIV      = CY_SYSTEM_PLL5_CONFIG_FEEDBACKDIV;
    SRSS->unCLK_PLL_CONFIG[0].stcField.u5REFERENCE_DIV     = CY_SYSTEM_PLL5_CONFIG_REFDIV;
    SRSS->unCLK_PLL_CONFIG[0].stcField.u5OUTPUT_DIV        = CY_SYSTEM_PLL5_CONFIG_OUTDIV;
    SRSS->unCLK_PLL_CONFIG[0].stcField.u2LOCK_DELAY        = 0ul;    /* Normal operation */
    SRSS->unCLK_PLL_CONFIG[0].stcField.u1PLL_LF_MODE       = 0ul;    /* VCO frequency is [200MHz, 400MHz] Fvco = 400,000,000[Hz] */
    SRSS->unCLK_PLL_CONFIG[0].stcField.u2BYPASS_SEL        = 0ul;    /* Auto */
    SRSS->unCLK_PLL_CONFIG[0].stcField.u1ENABLE            = 1ul;

    /* Settings for PLL200 #1 (CLK_PATH7) */
    SRSS->unCLK_PATH_SELECT[7/*PLL200 #1*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->unCLK_PLL_CONFIG[1].stcField.u7FEEDBACK_DIV      = CY_SYSTEM_PLL6_CONFIG_FEEDBACKDIV;
    SRSS->unCLK_PLL_CONFIG[1].stcField.u5REFERENCE_DIV     = CY_SYSTEM_PLL6_CONFIG_REFDIV;
    SRSS->unCLK_PLL_CONFIG[1].stcField.u5OUTPUT_DIV        = CY_SYSTEM_PLL6_CONFIG_OUTDIV;
    SRSS->unCLK_PLL_CONFIG[1].stcField.u2LOCK_DELAY        = 0ul;    /* Normal operation */
    SRSS->unCLK_PLL_CONFIG[1].stcField.u1PLL_LF_MODE       = 0ul;    /* VCO frequency is [200MHz, 400MHz] Fvco = 240,000,000[Hz] */
    SRSS->unCLK_PLL_CONFIG[1].stcField.u2BYPASS_SEL        = 0ul;    /* Auto */
    SRSS->unCLK_PLL_CONFIG[1].stcField.u1ENABLE            = 1ul;

    /* Settings for PLL200 #2 (CLK_PATH8) */
    SRSS->unCLK_PATH_SELECT[8/*PLL200 #2*/].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;

    SRSS->unCLK_PLL_CONFIG[2].stcField.u7FEEDBACK_DIV      = CY_SYSTEM_PLL7_CONFIG_FEEDBACKDIV;
    SRSS->unCLK_PLL_CONFIG[2].stcField.u5REFERENCE_DIV     = CY_SYSTEM_PLL7_CONFIG_REFDIV;
    SRSS->unCLK_PLL_CONFIG[2].stcField.u5OUTPUT_DIV        = CY_SYSTEM_PLL7_CONFIG_OUTDIV;
    SRSS->unCLK_PLL_CONFIG[2].stcField.u2LOCK_DELAY        = 0ul;    /* Normal operation */
    SRSS->unCLK_PLL_CONFIG[2].stcField.u1PLL_LF_MODE       = 0ul;    /* VCO frequency is [200MHz, 400MHz] Fvco = 240,000,000[Hz] */
    SRSS->unCLK_PLL_CONFIG[2].stcField.u2BYPASS_SEL        = 0ul;    /* Auto */
    SRSS->unCLK_PLL_CONFIG[2].stcField.u1ENABLE            = 1ul;

    /* Waiting for all PLLs being locked */
    while(SRSS->CLK_PLL400M[0].unSTATUS.stcField.u1LOCKED == 0ul);
    while(SRSS->CLK_PLL400M[1].unSTATUS.stcField.u1LOCKED == 0ul);
    while(SRSS->CLK_PLL400M[2].unSTATUS.stcField.u1LOCKED == 0ul);
    while(SRSS->CLK_PLL400M[3].unSTATUS.stcField.u1LOCKED == 0ul);
    while(SRSS->CLK_PLL400M[4].unSTATUS.stcField.u1LOCKED == 0ul);
    while(SRSS->unCLK_PLL_STATUS[0].stcField.u1LOCKED == 0ul);
    while(SRSS->unCLK_PLL_STATUS[1].stcField.u1LOCKED == 0ul);
    while(SRSS->unCLK_PLL_STATUS[2].stcField.u1LOCKED == 0ul);

    /***   Setting  PATH9  source, directly from IMO        ***/
    SRSS->unCLK_PATH_SELECT[9].stcField.u3PATH_MUX = CY_SYSCLK_CLKPATH_IN_IMO;

    /***   Setting  PATH10  source, directly from ECO       ***/
    SRSS->unCLK_PATH_SELECT[10].stcField.u3PATH_MUX = CY_SYSTEM_PLL_INPUT_SOURCE;
	
    /***  Setting for each CLK_HF        ***/
    struct {cy_en_hf_clk_dividers_t targetDiv; cy_en_hf_clk_sources_t source;} clkHfSetting[SRSS_NUM_HFROOT] = 
    {
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH6 /* PLL#5: PLL200#0 */ }, 	// setting for CLK_HF0,      CM0+/CLK_MEM:   160MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH1 /* PLL#0: PLL400#0 */ }, 	// setting for CLK_HF1,      CM7s:           320MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH7 /* PLL#6: PLL200#1 */ }, 	// setting for CLK_HF2,      CLK_PERI:       80MHz
        { .targetDiv = CY_SYSCLK_HFCLK_DIVIDE_BY_2, .source = CY_SYSCLK_HFCLK_IN_CLKPATH6 /* PLL#5: PLL200#0 */ }, 	// setting for CLK_HF3,      EVTGEN:         80MHz
        { .targetDiv = CY_SYSCLK_HFCLK_DIVIDE_BY_4, .source = CY_SYSCLK_HFCLK_IN_CLKPATH6 /* PLL#5: PLL200#0 */ }, 	// setting for CLK_HF4,      ETH:            40MHz
        { .targetDiv = CY_SYSCLK_HFCLK_DIVIDE_BY_8, .source = CY_SYSCLK_HFCLK_IN_CLKPATH2 /* PLL#1: PLL400#1 */ }, 	// setting for CLK_HF5,      SS#0:           24.575MHz
        { .targetDiv = CY_SYSCLK_HFCLK_DIVIDE_BY_8, .source = CY_SYSCLK_HFCLK_IN_CLKPATH2 /* PLL#1: PLL400#1 */ }, 	// setting for CLK_HF6,      SS#1:           24.575MHz
        { .targetDiv = CY_SYSCLK_HFCLK_DIVIDE_BY_8, .source = CY_SYSCLK_HFCLK_IN_CLKPATH2 /* PLL#1: PLL400#1 */ }, 	// setting for CLK_HF7,      SS#2:           24.575MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH3 /* PLL#2: PLL400#2 */ }, 	// setting for CLK_HF8,      SMIF#0:         360MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH3 /* PLL#2: PLL400#2 */ }, 	// setting for CLK_HF9,      SMIF#1:         360MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH4 /* PLL#3: PLL400#3 */ }, 	// setting for CLK_HF10,     VIDEOSS:        266MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH5 /* PLL#4: PLL400#4 */ }, 	// setting for CLK_HF11,     DISP#0:         220MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH8 /* PLL#7: PLL200#2 */ }, 	// setting for CLK_HF12,     DISP#1:         200MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH7 /* PLL#6: PLL200#1 */ }, 	// setting for CLK_HF13,     SDHC:           80MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH10 /* ECO */            },  // setting for CLK_HF14,     LPDDR4:         16MHz
        { .targetDiv = CY_SYSCLK_HFCLK_NO_DIVIDE,   .source = CY_SYSCLK_HFCLK_IN_CLKPATH9 /* IMO */             },  // setting for CLK_HF15
        };

#if (CY_SYS_VCCD_SOURCE == CY_SYS_VCCD_PMIC)
    // Restriction 1. Enabling MONZA PMIC when the current comsumption is from 20 [mA] to 300 [mA]
    //                Please adjust "TIMING_MONZA_PMIC_ENABLE" defined above.
    // Restriction 2. The current rising time has to be more than 10 [us]
    //                Please adjust "WAIT_CYCLE_WHILE_DISTRIBUTING_CLOCK" defined above.
    for(int8_t i_clkHfNo = 0ul; i_clkHfNo < SRSS_NUM_HFROOT; i_clkHfNo++)
    {
        if(i_clkHfNo == TIMING_MONZA_PMIC_ENABLE)
        {
            uint32_t primaskBackup = __get_PRIMASK(); /* Save current "IRQ enable" state for later restoration */
            __set_PRIMASK(0);                         /* Ensure that IRQs are enabled because the switch-over to PMIC uses a system call */

            /***   Enabling MONZA PMIC     ***/
            Cy_Power_SwitchToPmic(CY_SYSPMIC_VADJ_1V150, CY_POWER_PMIC_ENABLE_HIGH, CY_POWER_PMIC_STATUS_ABNORMAL_LOW, 0u);

            __set_PRIMASK(primaskBackup); /* Restore original "IRQ enable" state */
        }
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX   = clkHfSetting[i_clkHfNo].source;
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u2ROOT_DIV   = CY_SYSCLK_HFCLK_DIVIDE_BY_8;
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1DIRECT_MUX = 1u; /* Select ROOT_MUX */
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1ENABLE     = 1u; /* 1 = enable */
        /* Gradually decrease the current root clock divider until the target divider is reached */
        for(int8_t i_divRegValue = 2; i_divRegValue >= clkHfSetting[i_clkHfNo].targetDiv; i_divRegValue--)
        {
            Cy_SysTick_DelayCoreCycle(WAIT_CYCLE_WHILE_DISTRIBUTING_CLOCK);
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u2ROOT_DIV = i_divRegValue;
        }
        Cy_SysTick_DelayCoreCycle(WAIT_CYCLE_WHILE_DISTRIBUTING_CLOCK);
    }
#else
    for(int8_t i_clkHfNo = 0ul; i_clkHfNo < SRSS_NUM_HFROOT; i_clkHfNo++)
    {
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX   = clkHfSetting[i_clkHfNo].source;
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u2ROOT_DIV   = clkHfSetting[i_clkHfNo].targetDiv;
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1DIRECT_MUX = 1u; /* Select ROOT_MUX */
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1ENABLE     = 1u; /* 1 = enable */
    }
#endif

    /* CLK_FAST_0 */
    CPUSS->unFAST_0_CLOCK_CTL.stcField.u8INT_DIV  = 0u; /* no division */
    CPUSS->unFAST_0_CLOCK_CTL.stcField.u5FRAC_DIV = 0u; /* no division */

    /* CLK_FAST_1 */
    CPUSS->unFAST_1_CLOCK_CTL.stcField.u8INT_DIV  = 0u; /* no division */
    CPUSS->unFAST_1_CLOCK_CTL.stcField.u5FRAC_DIV = 0u; /* no division */

    /***     Enabling ILO0        ***/
    Cy_WDT_Unlock();
    SRSS->unCLK_ILO0_CONFIG.stcField.u1ENABLE = 1;        /* 1 = enable */
    SRSS->unCLK_ILO0_CONFIG.stcField.u1ILO0_BACKUP = 1ul; /* Ilo HibernateOn */
    Cy_WDT_Lock();

#else // CY_USE_PSVP == 0u

    /* For PSVP, PLL#0 should be enabled to switch from 8MHz(IMO) to 20MHz for HF0 */
    SRSS->CLK_PLL400M[0].unCONFIG.stcField.u1ENABLE = 1ul; 
    /* For PSVP, PLL#2 should be enabled to switch from 8MHz(IMO) to 32MHz for HF1 */
    SRSS->CLK_PLL400M[2].unCONFIG.stcField.u1ENABLE = 1ul; 
    /* For PSVP, PLL#3 should be enabled to switch from 8MHz(IMO) to 22MHz for HF11. 
       This PLL can be dynamically reconfigured (recommended from 11MHz to 22MHz) */
    SRSS->CLK_PLL400M[3].unCONFIG.stcField.u1ENABLE = 1ul; 
    /* For PSVP, PLL#4 should be enabled to switch from 8MHz(IMO) to 22MHz for HF12. 
       This PLL can be dynamically reconfigured (recommended from 11MHz to 22MHz)*/
    SRSS->CLK_PLL400M[4].unCONFIG.stcField.u1ENABLE = 1ul; 
	/* For PSVP, PLL#5 (PLL200#0) should be enabled to switch from 8Mhz(IMO) to 26.6MHz for all other HFs */
    SRSS->unCLK_PLL_CONFIG[0].stcField.u1ENABLE = 1ul;
    
    /* Configure all CLK_HFx to use the 24 MHz from this PLL */
    for(int8_t i_clkHfNo = 0ul; i_clkHfNo < SRSS_NUM_HFROOT; i_clkHfNo++)
    {
        if (i_clkHfNo == 0 ) 
        {
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX = CY_SYSCLK_HFCLK_IN_CLKPATH1;   /* PLL0: PLL400#0 */
        } 
        else if (i_clkHfNo == 1 ) 
        {
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX = CY_SYSCLK_HFCLK_IN_CLKPATH3;   /* PLL2: PLL400#0 */
        } 
        else if (i_clkHfNo == 11 ) 
        {
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX = CY_SYSCLK_HFCLK_IN_CLKPATH4;   /* PLL3: PLL400#0 */
        } 
        else if (i_clkHfNo == 12) 
        {
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX = CY_SYSCLK_HFCLK_IN_CLKPATH5;   /* PLL4: PLL400#0 */
        } 
        else 
        {
            SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u4ROOT_MUX = CY_SYSCLK_HFCLK_IN_CLKPATH6;   /* PLL5: PLL400#0 */
        }
        
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u2ROOT_DIV = CY_SYSCLK_HFCLK_NO_DIVIDE;
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1DIRECT_MUX = 1u;                              /* Select ROOT_MUX */
        SRSS->unCLK_ROOT_SELECT[i_clkHfNo].stcField.u1ENABLE = 1u;                                  /* 1 = Enable */
    }
    
#endif  // CY_USE_PSVP == 0u

    /* Enable VIDEOSS Power */
    PD->unCTL.stcField.u2PWR_MODE = 3ul;                                                            /* VIDEOSS Power Enable */
    while(PD->unSTATUS.stcField.u1PWR_DONE == 0ul);                                                 /* Wait for power enabled */
    VIDEOSS0_SUBSS0_VIDEOSSCFG->unCTL.stcField.u1ENABLED = 1;

    /* Update the core clock parameters */
    SystemCoreClockUpdate();

#ifdef CY_SYS_SMIF_POWER_EN
    /* Configure the power pins used for SMIF */
    Cy_SystemHsioPowerConfig();
    
    /* Enable the SMIF power sequencing */
    Cy_SystemHsioEnhPowerUp();
#endif  /* CY_SYS_SMIF_POWER_EN */
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
* Function Name: Cy_SysGetApplCoreStatus
****************************************************************************//**
*
* Gets the Cortex-M4/M7 core power mode.
*
* \return \ref group_system_config_cm_status_macro
*
*******************************************************************************/
uint32_t Cy_SysGetApplCoreStatus(uint8_t core)
{
    uint32_t regValue = 0;
    
    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        /* Get current power mode */
        regValue = CPUSS->unCM7_0_PWR_CTL.u32Register;
        regValue = (regValue >> CPUSS_CM7_0_PWR_CTL_PWR_MODE_Pos) & CPUSS_CM7_0_PWR_CTL_PWR_MODE_Msk;
    }
    else if(core == CORE_CM7_1)
    {
        /* Get current power mode */
        regValue = CPUSS->unCM7_1_PWR_CTL.u32Register;
        regValue = (regValue >> CPUSS_CM7_1_PWR_CTL_PWR_MODE_Pos) & CPUSS_CM7_1_PWR_CTL_PWR_MODE_Msk;
    }

    return (regValue);
}

/*******************************************************************************
* Function Name: Cy_SysEnableApplCore
****************************************************************************//**
*
* Enables the Cortex-M4/M7 core. The CPU is enabled once if it was in the disabled
* or retained mode. 
*
* \param vectorTableOffset The offset of the vector table base address from
* memory address 0x00000000. The offset should be multiple to 1024 bytes.
*
*******************************************************************************/
void Cy_SysEnableApplCore(uint8_t core, uint32_t vectorTableOffset)
{
    uint32_t cmStatus;
    uint32_t interruptState;
    un_CPUSS_CM7_0_PWR_CTL_t tPwrCtl0;
    un_CPUSS_CM7_1_PWR_CTL_t tPwrCtl1; 
    
    CY_ASSERT(core < CORE_MAX);

    interruptState = Cy_SaveIRQ();
    
    cmStatus = Cy_SysGetApplCoreStatus(core);
    if(cmStatus == CY_SYS_CM_STATUS_ENABLED)
    {
        // Set core into reset first, so that new settings can get effective
        // This branch is e.g. entered if a debugger is connected that would power-up the CM7,
        // but let it run in ROM boot or pause its execution by keeping CPU_WAIT bit set.
        Cy_SysResetApplCore(core);
    }
    
    // CLK_HF1, by default is disabled for use by CM7_0/1, hence enable
    SRSS->unCLK_ROOT_SELECT[1].stcField.u1ENABLE = 1;
        
    if(core == CORE_CM7_0)
    {           
        // Adjust the vector address
        CPUSS->unCM7_0_VECTOR_TABLE_BASE.u32Register = vectorTableOffset;
        
        // Enable the Power Control Key
        tPwrCtl0.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl0.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_ENABLED;
        CPUSS->unCM7_0_PWR_CTL.u32Register = tPwrCtl0.u32Register;
        
        CPUSS->unCM7_0_CTL.stcField.u1CPU_WAIT = 0;
    } 
    else if(core == CORE_CM7_1)
    {            
        // Adjust the vector address
        CPUSS->unCM7_1_VECTOR_TABLE_BASE.u32Register = vectorTableOffset;
        
        // Enable the Power Control Key
        tPwrCtl1.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl1.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_ENABLED;
        CPUSS->unCM7_1_PWR_CTL.u32Register = tPwrCtl1.u32Register;
        
        CPUSS->unCM7_1_CTL.stcField.u1CPU_WAIT = 0;
    }
    
    Cy_RestoreIRQ(interruptState);
}


/*******************************************************************************
* Function Name: Cy_SysDisableApplCore
****************************************************************************//**
*
* Disables the Cortex-M4/M7 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the
* CPU.
*
*******************************************************************************/
void Cy_SysDisableApplCore(uint8_t core)
{
    un_CPUSS_CM7_0_PWR_CTL_t tPwrCtl0;
    un_CPUSS_CM7_1_PWR_CTL_t tPwrCtl1; 

    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        tPwrCtl0.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl0.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_OFF;
        CPUSS->unCM7_0_PWR_CTL.u32Register = tPwrCtl0.u32Register; 
    }
    else if(core == CORE_CM7_1)
    {
        tPwrCtl1.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl1.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_OFF;
        CPUSS->unCM7_1_PWR_CTL.u32Register = tPwrCtl1.u32Register; 
    }
}


/*******************************************************************************
* Function Name: Cy_SysRetainApplCore
****************************************************************************//**
*
* Retains the Cortex-M4/M7 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the CPU.
*
*******************************************************************************/
void Cy_SysRetainApplCore(uint8_t core)
{
    uint32_t cmStatus;
    uint32_t  interruptState;
    un_CPUSS_CM7_0_PWR_CTL_t tPwrCtl0;
    un_CPUSS_CM7_1_PWR_CTL_t tPwrCtl1; 

    interruptState = Cy_SaveIRQ();
    
    cmStatus = Cy_SysGetApplCoreStatus(core);
    if(cmStatus == CY_SYS_CM_STATUS_ENABLED)
    {
        if(core == CORE_CM7_0)
        {
            tPwrCtl0.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
            tPwrCtl0.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_RETAINED;
            CPUSS->unCM7_0_PWR_CTL.u32Register = tPwrCtl0.u32Register;
        }
        else if(core == CORE_CM7_1)
        {
            tPwrCtl1.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
            tPwrCtl1.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_RETAINED;
            CPUSS->unCM7_1_PWR_CTL.u32Register = tPwrCtl1.u32Register;
        }
    }
    
    Cy_RestoreIRQ(interruptState);
}


/*******************************************************************************
* Function Name: Cy_SysResetApplCore
****************************************************************************//**
*
* Resets the Cortex-M4/M7 core.
*
* \warning Do not call the function while the Cortex-M4 is executing because
* such a call may corrupt/abort a pending bus-transaction by the CPU and cause
* unexpected behavior in the system including a deadlock. Call the function
* while the Cortex-M4 core is in the Sleep or Deep Sleep low-power mode. Use
* the \ref group_syspm Power Management (syspm) API to put the CPU into the
* low-power modes. Use the \ref Cy_SysPm_ReadStatus() to get a status of the CPU.
*
*******************************************************************************/
void Cy_SysResetApplCore(uint8_t core)
{
    un_CPUSS_CM7_0_PWR_CTL_t tPwrCtl0;
    un_CPUSS_CM7_1_PWR_CTL_t tPwrCtl1; 
    
    CY_ASSERT(core < CORE_MAX);

    if(core == CORE_CM7_0)
    {
        tPwrCtl0.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl0.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_RESET;
        CPUSS->unCM7_0_PWR_CTL.u32Register = tPwrCtl0.u32Register; 
    }
    else if(core == CORE_CM7_1)
    {
        tPwrCtl1.stcField.u16VECTKEYSTAT = CY_SYS_PWR_CTL_KEY_OPEN;
        tPwrCtl1.stcField.u2PWR_MODE = CY_SYS_CM_STATUS_RESET;
        CPUSS->unCM7_1_PWR_CTL.u32Register = tPwrCtl1.u32Register; 
    }
}


/* Case where system uses SMIF (HSIO_ENH) power */
#ifdef CY_SYS_SMIF_POWER_EN

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
static const stc_portpin_cfg_t cy_hsioPowerSeqPins[] =
{
    { .pstcPort = CY_SMIF_PWR_EN_PORT,      .u8Pin = CY_SMIF_PWR_EN_PIN,        .enMuxCfg = CY_SMIF_PWR_EN_PIN_MUX,     .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
};

/*******************************************************************************
* Function Name: Cy_SystemHsioPowerConfig
****************************************************************************//**
*
* Configure the IO pins used to initiate the system's power up and down sequence.
*
*******************************************************************************/
void Cy_SystemHsioPowerConfig(void)
{
    /* Initialize the power pins used for the sequencing */
    for(uint8_t u8Index = 0; u8Index < CY_GET_ARRAY_LEN(cy_hsioPowerSeqPins); u8Index++)
    {
        Cy_GPIO_Pin_FastInit( cy_hsioPowerSeqPins[u8Index].pstcPort,
                              cy_hsioPowerSeqPins[u8Index].u8Pin,
                              cy_hsioPowerSeqPins[u8Index].u8DriveMode,
                              cy_hsioPowerSeqPins[u8Index].u8GpioOutVal,
                              cy_hsioPowerSeqPins[u8Index].enMuxCfg      );
    }
}

/*******************************************************************************
* Function Name: Cy_SystemHsioEnhPowerUp
****************************************************************************//**
*
* Enables the SMIF IO or HSIO_ENH domain power .
*
*******************************************************************************/
void Cy_SystemHsioEnhPowerUp(void)
{
    /* Enable the SMIF IO power */
    Cy_GPIO_Write(CY_SMIF_PWR_EN_PORT, CY_SMIF_PWR_EN_PIN, 1u);
}

/*******************************************************************************
* Function Name: Cy_SystemHsioEnhPowerDown
****************************************************************************//**
*
* Disables the SMIF IO or HSIO_ENH domain power .
*
*******************************************************************************/
void Cy_SystemHsioEnhPowerDown(void)
{
    /* Disable the SMIF IO power */
    Cy_GPIO_Write(CY_SMIF_PWR_EN_PORT, CY_SMIF_PWR_EN_PIN, 0u);
}

#endif  /* CY_SYS_SMIF_POWER_EN */
