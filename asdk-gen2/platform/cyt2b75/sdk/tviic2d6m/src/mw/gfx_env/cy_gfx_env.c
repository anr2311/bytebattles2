/***************************************************************************//**
* \file cy_gfx_env.c
* \version 1.1
*
* \brief
* Provides an API implementation of the Graphics Environment (GfxEnv) middleware driver
*
********************************************************************************
* \copyright
* Copyright 2016-2017, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "cy_gfx_env.h"

#if defined(__cplusplus)
extern "C" {
#endif

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/

#define DISP0_CLK_PLL_MIN_HZ                      110000000UL
#define DISP0_CLK_PLL_MAX_HZ                      220000000UL
#define DISP1_CLK_PLL_MIN_HZ                      110000000UL
#define DISP1_CLK_PLL_MAX_HZ                      200000000UL  // 220 max if a PLL400M would be used

#define VIDEOSS_FIXED_DISP_CLK_DIV                2  // Fixed "divide-by-2" in the display clock path
#define CLK_PATH_DISP0_CLK                        5  // PLL400M #4
#define CLK_PATH_DISP1_CLK                        8  // PLL200M #2
#define CLK_PATH_SMIF_CLK                         3  // PLL400M #2
#define CLK_DIV_TO_REG_VAL(div)                   (((div) == 8) ? 3 : ((div) == 4) ? 2 : ((div) == 2) ? 1 : 0)

// Macros needed to generate the VIDEOSS TCON mapping values
//----
#define R(bit)                                    ((bit) +  0)
#define G(bit)                                    ((bit) +  8)
#define B(bit)                                    ((bit) + 16)
#define HSYNC                                     (24)
#define VSYNC                                     (25)
#define DEN                                       (26)
#define RES                                       (27)
#define MAP_VALUE(n0,n1,n2,n3)                    (((n3)<<24) + ((n2)<<16) + ((n1)<<8) +(n0))

#define MAP_INDEX_JEIDA                           0
#define MAP_INDEX_VESA                            1
//----

//====================================================================
// SMIF and HyperBus related macros

#define GFXENV_SMIF_TIMEOUT                       1000
#define GFXENV_S27K_INITIAL_LATENCY               CY_SMIF_HB_LC7
#define GFXENV_S27K_MAX_CS_LOW_NS                 1000UL          // t_CMS in nanoseconds

#define GFXENV_SMIF_MDL_TAP_XSPI400               6
#define GFXENV_SMIF_MDL_TAP_XSPI333               6
#define GFXENV_SMIF_MDL_TAP_XSPI266               6
#define GFXENV_SMIF_MDL_TAP_XSPI200               6

#define GFXENV_SMIF_SDL_TAP_XSPI400               0
#define GFXENV_SMIF_SDL_TAP_XSPI333               0
#define GFXENV_SMIF_SDL_TAP_XSPI266               1
#define GFXENV_SMIF_SDL_TAP_XSPI200               2

#define GFXENV_SMIF_GET_DLL_DIVIDER_FROM_ENUM(enum_val)    ((enum_val == 0) ? 2 : (enum_val == 1) ? 4 : (enum_val == 2) ? 8 : 16)
#define GFXENV_SMIF_GET_DLL_ENUM_FROM_DIVIDER(div_val)     ((div_val == 2) ? CY_SMIF_MDL_CLK_OUT_DIV2 : (div_val == 4) ? CY_SMIF_MDL_CLK_OUT_DIV4 : (div_val == 8) ? CY_SMIF_MDL_CLK_OUT_DIV8 : CY_SMIF_MDL_CLK_OUT_DIV16)

//====================================================================

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/

/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/

/*****************************************************************************
 ** \brief Port pin group enumeration type
 *****************************************************************************/
typedef enum
{
    Display0,
    Display1,
    Capture0,
    Smif0,
    Smif1,
    BlDisp0,
    BlDisp1,
    BlFpd0,
    BlFpd1,
    Button,
} en_portpin_group_t;

/*****************************************************************************
 ** \brief Port pin configuration type
 *****************************************************************************/
typedef struct
{
    en_portpin_group_t          enGroup;
    volatile stc_GPIO_PRT_t*    pstcPort;
    uint8_t                     u8Pin;
    en_hsiom_sel_t              enMuxCfg;
    uint8_t                     u8DriveMode;
    uint8_t                     u8GpioOutVal;
} stc_portpin_cfg_t;

/*****************************************************************************
 ** \brief Display presets type
 *****************************************************************************/
typedef struct
{
    uint16_t  u16HorizontalActive;
    uint16_t  u16HorizontalTotal;
    uint16_t  u16VerticalActive;
    uint16_t  u16VerticalTotal;
    uint8_t   u8RefreshRateHz;
    bool      bUseFpd;
    uint8_t   u8MapIndex;
} stc_display_presets_t;


/*****************************************************************************
 ** \brief Holds some config and status values for a SMIF instance with its
 **        devices that need to be preserved across function calls
 *****************************************************************************/
typedef struct
{
    volatile cy_stc_smif_reg_t *             pstcSmifRegBase;
    volatile cy_stc_smif_reg_device_t *      pstcDevRegBase;
    cy_stc_smif_context_t                    stcSmifCtx;
    cy_stc_ex_dev_context_t                  stcExDevCtx;
    cy_en_cy_smif_ddl_tap_sel_t              enMdlTap;
    cy_en_cy_smif_ddl_tap_sel_t              enSdlTap;
    uint8_t                                  u8SmifDllOutputDivider;
    uint8_t                                  u8DriveStrengthTrimVal;
    uint8_t                                  u8CoreNumber;
    uint8_t                                  u8DeviceNumber;
    cy_gfxenv_stc_smif_device_cfg_t *        pstcSmifDevUsrCfg;
} stc_smif_cfg_internal_t;


/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/

/*****************************************************************************
 ** \brief Display presets
 *****************************************************************************/
static const stc_display_presets_t m_astcDispPresets[] =
{
    { 0 }, // dummy for "CY_GFXENV_DISP_TYPE_NONE"
    { .u16HorizontalActive =  480, .u16HorizontalTotal =  592, .u16VerticalActive = 272, .u16VerticalTotal = 286, .u8RefreshRateHz = 60, .bUseFpd = false },
    { .u16HorizontalActive =  640, .u16HorizontalTotal =  800, .u16VerticalActive = 480, .u16VerticalTotal = 500, .u8RefreshRateHz = 60, .bUseFpd = false },
    { .u16HorizontalActive =  800, .u16HorizontalTotal =  992, .u16VerticalActive = 480, .u16VerticalTotal = 500, .u8RefreshRateHz = 60, .bUseFpd = false },
    { .u16HorizontalActive =  800, .u16HorizontalTotal =  992, .u16VerticalActive = 480, .u16VerticalTotal = 500, .u8RefreshRateHz = 60, .bUseFpd = true, .u8MapIndex = MAP_INDEX_JEIDA },
    { .u16HorizontalActive =  800, .u16HorizontalTotal =  992, .u16VerticalActive = 480, .u16VerticalTotal = 500, .u8RefreshRateHz = 60, .bUseFpd = true, .u8MapIndex = MAP_INDEX_VESA  },
    { .u16HorizontalActive = 1280, .u16HorizontalTotal = 1440, .u16VerticalActive = 768, .u16VerticalTotal = 790, .u8RefreshRateHz = 60, .bUseFpd = true, .u8MapIndex = MAP_INDEX_VESA },
    { .u16HorizontalActive = 1600, .u16HorizontalTotal = 1660, .u16VerticalActive = 480, .u16VerticalTotal = 528, .u8RefreshRateHz = 60, .bUseFpd = true, .u8MapIndex = MAP_INDEX_VESA },

};

/*****************************************************************************
 ** \brief FPD/LVDS Signal Mapping
 *****************************************************************************/
static const uint32_t m_aau32FpdMapping[7][2] =
{
//                    JEIDA standard                             VESA standard
//                +0     +1     +2     +3                    +0     +1     +2     +3
    { MAP_VALUE( G(2),  R(7),  R(6),  R(5)  ),   MAP_VALUE( G(0),  R(5),  R(4),  R(3)  ) },  // <-- Value for register MAPBIT3_0
    { MAP_VALUE( R(4),  R(3),  R(2),  B(3)  ),   MAP_VALUE( R(2),  R(1),  R(0),  B(1)  ) },  // <-- Value for register MAPBIT7_4
    { MAP_VALUE( B(2),  G(7),  G(6),  G(5)  ),   MAP_VALUE( B(0),  G(5),  G(4),  G(3)  ) },  // <-- Value for register MAPBIT11_8
    { MAP_VALUE( G(4),  G(3),  DEN,   VSYNC ),   MAP_VALUE( G(2),  G(1),  DEN,   VSYNC ) },  // <-- Value for register MAPBIT15_12
    { MAP_VALUE( HSYNC, B(7),  B(6),  B(5)  ),   MAP_VALUE( HSYNC, B(5),  B(4),  B(3)  ) },  // <-- Value for register MAPBIT19_16
    { MAP_VALUE( B(4),  RES,   B(1),  B(0)  ),   MAP_VALUE( B(2),  RES,   B(7),  B(6)  ) },  // <-- Value for register MAPBIT23_20
    { MAP_VALUE( G(1),  G(0),  R(1),  R(0)  ),   MAP_VALUE( G(7),  G(6),  R(7),  R(6)  ) },  // <-- Value for register MAPBIT27_24
};

/*****************************************************************************
 ** \brief Port pin configuration
 *****************************************************************************/
static const stc_portpin_cfg_t m_astcPortPinCfg[] =
{
//============================================================================================================================================================================================
#ifdef CY_BSP_HDR_DISP0_TTL_AVAILABLE
    { .enGroup = Display0, .pstcPort = CY_DISP0_CLK_PORT,        .u8Pin = CY_DISP0_CLK_PIN,        .enMuxCfg = CY_DISP0_CLK_PIN_MUX,        .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_CTRL0_PORT,      .u8Pin = CY_DISP0_CTRL0_PIN,      .enMuxCfg = CY_DISP0_CTRL0_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_CTRL1_PORT,      .u8Pin = CY_DISP0_CTRL1_PIN,      .enMuxCfg = CY_DISP0_CTRL1_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_CTRL2_PORT,      .u8Pin = CY_DISP0_CTRL2_PIN,      .enMuxCfg = CY_DISP0_CTRL2_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_0_PORT,  .u8Pin = CY_DISP0_DATA_A0_0_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_0_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_1_PORT,  .u8Pin = CY_DISP0_DATA_A0_1_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_1_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_2_PORT,  .u8Pin = CY_DISP0_DATA_A0_2_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_2_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_3_PORT,  .u8Pin = CY_DISP0_DATA_A0_3_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_3_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_4_PORT,  .u8Pin = CY_DISP0_DATA_A0_4_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_4_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_5_PORT,  .u8Pin = CY_DISP0_DATA_A0_5_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_5_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_6_PORT,  .u8Pin = CY_DISP0_DATA_A0_6_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_6_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_7_PORT,  .u8Pin = CY_DISP0_DATA_A0_7_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_7_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_8_PORT,  .u8Pin = CY_DISP0_DATA_A0_8_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_8_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_9_PORT,  .u8Pin = CY_DISP0_DATA_A0_9_PIN,  .enMuxCfg = CY_DISP0_DATA_A0_9_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_10_PORT, .u8Pin = CY_DISP0_DATA_A0_10_PIN, .enMuxCfg = CY_DISP0_DATA_A0_10_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A0_11_PORT, .u8Pin = CY_DISP0_DATA_A0_11_PIN, .enMuxCfg = CY_DISP0_DATA_A0_11_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_0_PORT,  .u8Pin = CY_DISP0_DATA_A1_0_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_0_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_1_PORT,  .u8Pin = CY_DISP0_DATA_A1_1_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_1_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_2_PORT,  .u8Pin = CY_DISP0_DATA_A1_2_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_2_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_3_PORT,  .u8Pin = CY_DISP0_DATA_A1_3_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_3_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_4_PORT,  .u8Pin = CY_DISP0_DATA_A1_4_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_4_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_5_PORT,  .u8Pin = CY_DISP0_DATA_A1_5_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_5_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_6_PORT,  .u8Pin = CY_DISP0_DATA_A1_6_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_6_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_7_PORT,  .u8Pin = CY_DISP0_DATA_A1_7_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_7_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_8_PORT,  .u8Pin = CY_DISP0_DATA_A1_8_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_8_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_9_PORT,  .u8Pin = CY_DISP0_DATA_A1_9_PIN,  .enMuxCfg = CY_DISP0_DATA_A1_9_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_10_PORT, .u8Pin = CY_DISP0_DATA_A1_10_PIN, .enMuxCfg = CY_DISP0_DATA_A1_10_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display0, .pstcPort = CY_DISP0_DATA_A1_11_PORT, .u8Pin = CY_DISP0_DATA_A1_11_PIN, .enMuxCfg = CY_DISP0_DATA_A1_11_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
#ifdef CY_BSP_HDR_DISP1_TTL_AVAILABLE
    { .enGroup = Display1, .pstcPort = CY_DISP1_CLK_PORT,        .u8Pin = CY_DISP1_CLK_PIN,        .enMuxCfg = CY_DISP1_CLK_PIN_MUX,        .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_CTRL0_PORT,      .u8Pin = CY_DISP1_CTRL0_PIN,      .enMuxCfg = CY_DISP1_CTRL0_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_CTRL1_PORT,      .u8Pin = CY_DISP1_CTRL1_PIN,      .enMuxCfg = CY_DISP1_CTRL1_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_CTRL2_PORT,      .u8Pin = CY_DISP1_CTRL2_PIN,      .enMuxCfg = CY_DISP1_CTRL2_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_0_PORT,  .u8Pin = CY_DISP1_DATA_A0_0_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_0_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_1_PORT,  .u8Pin = CY_DISP1_DATA_A0_1_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_1_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_2_PORT,  .u8Pin = CY_DISP1_DATA_A0_2_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_2_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_3_PORT,  .u8Pin = CY_DISP1_DATA_A0_3_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_3_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_4_PORT,  .u8Pin = CY_DISP1_DATA_A0_4_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_4_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_5_PORT,  .u8Pin = CY_DISP1_DATA_A0_5_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_5_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_6_PORT,  .u8Pin = CY_DISP1_DATA_A0_6_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_6_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_7_PORT,  .u8Pin = CY_DISP1_DATA_A0_7_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_7_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_8_PORT,  .u8Pin = CY_DISP1_DATA_A0_8_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_8_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_9_PORT,  .u8Pin = CY_DISP1_DATA_A0_9_PIN,  .enMuxCfg = CY_DISP1_DATA_A0_9_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_10_PORT, .u8Pin = CY_DISP1_DATA_A0_10_PIN, .enMuxCfg = CY_DISP1_DATA_A0_10_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A0_11_PORT, .u8Pin = CY_DISP1_DATA_A0_11_PIN, .enMuxCfg = CY_DISP1_DATA_A0_11_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_0_PORT,  .u8Pin = CY_DISP1_DATA_A1_0_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_0_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_1_PORT,  .u8Pin = CY_DISP1_DATA_A1_1_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_1_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_2_PORT,  .u8Pin = CY_DISP1_DATA_A1_2_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_2_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_3_PORT,  .u8Pin = CY_DISP1_DATA_A1_3_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_3_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_4_PORT,  .u8Pin = CY_DISP1_DATA_A1_4_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_4_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_5_PORT,  .u8Pin = CY_DISP1_DATA_A1_5_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_5_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_6_PORT,  .u8Pin = CY_DISP1_DATA_A1_6_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_6_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_7_PORT,  .u8Pin = CY_DISP1_DATA_A1_7_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_7_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_8_PORT,  .u8Pin = CY_DISP1_DATA_A1_8_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_8_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_9_PORT,  .u8Pin = CY_DISP1_DATA_A1_9_PIN,  .enMuxCfg = CY_DISP1_DATA_A1_9_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_10_PORT, .u8Pin = CY_DISP1_DATA_A1_10_PIN, .enMuxCfg = CY_DISP1_DATA_A1_10_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
    { .enGroup = Display1, .pstcPort = CY_DISP1_DATA_A1_11_PORT, .u8Pin = CY_DISP1_DATA_A1_11_PIN, .enMuxCfg = CY_DISP1_DATA_A1_11_PIN_MUX, .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
#ifdef CY_BSP_HDR_CAP0_TTL_AVAILABLE
    { .enGroup = Capture0, .pstcPort = CY_CAP0_CLK_PORT,         .u8Pin = CY_CAP0_CLK_PIN,         .enMuxCfg = CY_CAP0_CLK_PIN_MUX,         .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA0_PORT,       .u8Pin = CY_CAP0_DATA0_PIN,       .enMuxCfg = CY_CAP0_DATA0_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA1_PORT,       .u8Pin = CY_CAP0_DATA1_PIN,       .enMuxCfg = CY_CAP0_DATA1_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA2_PORT,       .u8Pin = CY_CAP0_DATA2_PIN,       .enMuxCfg = CY_CAP0_DATA2_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA3_PORT,       .u8Pin = CY_CAP0_DATA3_PIN,       .enMuxCfg = CY_CAP0_DATA3_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA4_PORT,       .u8Pin = CY_CAP0_DATA4_PIN,       .enMuxCfg = CY_CAP0_DATA4_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA5_PORT,       .u8Pin = CY_CAP0_DATA5_PIN,       .enMuxCfg = CY_CAP0_DATA5_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA6_PORT,       .u8Pin = CY_CAP0_DATA6_PIN,       .enMuxCfg = CY_CAP0_DATA6_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA7_PORT,       .u8Pin = CY_CAP0_DATA7_PIN,       .enMuxCfg = CY_CAP0_DATA7_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA8_PORT,       .u8Pin = CY_CAP0_DATA8_PIN,       .enMuxCfg = CY_CAP0_DATA8_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA9_PORT,       .u8Pin = CY_CAP0_DATA9_PIN,       .enMuxCfg = CY_CAP0_DATA9_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA10_PORT,      .u8Pin = CY_CAP0_DATA10_PIN,      .enMuxCfg = CY_CAP0_DATA10_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA11_PORT,      .u8Pin = CY_CAP0_DATA11_PIN,      .enMuxCfg = CY_CAP0_DATA11_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA12_PORT,      .u8Pin = CY_CAP0_DATA12_PIN,      .enMuxCfg = CY_CAP0_DATA12_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA13_PORT,      .u8Pin = CY_CAP0_DATA13_PIN,      .enMuxCfg = CY_CAP0_DATA13_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA14_PORT,      .u8Pin = CY_CAP0_DATA14_PIN,      .enMuxCfg = CY_CAP0_DATA14_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA15_PORT,      .u8Pin = CY_CAP0_DATA15_PIN,      .enMuxCfg = CY_CAP0_DATA15_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA16_PORT,      .u8Pin = CY_CAP0_DATA16_PIN,      .enMuxCfg = CY_CAP0_DATA16_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA17_PORT,      .u8Pin = CY_CAP0_DATA17_PIN,      .enMuxCfg = CY_CAP0_DATA17_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA18_PORT,      .u8Pin = CY_CAP0_DATA18_PIN,      .enMuxCfg = CY_CAP0_DATA18_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA19_PORT,      .u8Pin = CY_CAP0_DATA19_PIN,      .enMuxCfg = CY_CAP0_DATA19_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA20_PORT,      .u8Pin = CY_CAP0_DATA20_PIN,      .enMuxCfg = CY_CAP0_DATA20_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA21_PORT,      .u8Pin = CY_CAP0_DATA21_PIN,      .enMuxCfg = CY_CAP0_DATA21_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA22_PORT,      .u8Pin = CY_CAP0_DATA22_PIN,      .enMuxCfg = CY_CAP0_DATA22_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA23_PORT,      .u8Pin = CY_CAP0_DATA23_PIN,      .enMuxCfg = CY_CAP0_DATA23_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA24_PORT,      .u8Pin = CY_CAP0_DATA24_PIN,      .enMuxCfg = CY_CAP0_DATA24_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA25_PORT,      .u8Pin = CY_CAP0_DATA25_PIN,      .enMuxCfg = CY_CAP0_DATA25_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Capture0, .pstcPort = CY_CAP0_DATA26_PORT,      .u8Pin = CY_CAP0_DATA26_PIN,      .enMuxCfg = CY_CAP0_DATA26_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
#ifdef CY_BSP_HDR_SMIF0_AVAILABLE
  #ifdef CY_SMIF0_CLK_INV_PORT // inverted clock is not available on PSVP (and hence would not be defined in BSP header)
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_CLK_INV_PORT,    .u8Pin = CY_SMIF0_CLK_INV_PIN,    .enMuxCfg = HSIOM_SEL_GPIO,              .u8DriveMode = CY_GPIO_DM_STRONG_IN_OFF, .u8GpioOutVal = 0 },
  #endif
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_CLK_PORT,        .u8Pin = CY_SMIF0_CLK_PIN,        .enMuxCfg = CY_SMIF0_CLK_PIN_MUX,        .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_RWDS_PORT,       .u8Pin = CY_SMIF0_RWDS_PIN,       .enMuxCfg = CY_SMIF0_RWDS_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_PULLDOWN,      .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_SELECT0_PORT,    .u8Pin = CY_SMIF0_SELECT0_PIN,    .enMuxCfg = CY_SMIF0_SELECT0_PIN_MUX,    .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 1 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_SELECT1_PORT,    .u8Pin = CY_SMIF0_SELECT1_PIN,    .enMuxCfg = CY_SMIF0_SELECT1_PIN_MUX,    .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 1 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA0_PORT,      .u8Pin = CY_SMIF0_DATA0_PIN,      .enMuxCfg = CY_SMIF0_DATA0_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA1_PORT,      .u8Pin = CY_SMIF0_DATA1_PIN,      .enMuxCfg = CY_SMIF0_DATA1_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA2_PORT,      .u8Pin = CY_SMIF0_DATA2_PIN,      .enMuxCfg = CY_SMIF0_DATA2_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA3_PORT,      .u8Pin = CY_SMIF0_DATA3_PIN,      .enMuxCfg = CY_SMIF0_DATA3_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA4_PORT,      .u8Pin = CY_SMIF0_DATA4_PIN,      .enMuxCfg = CY_SMIF0_DATA4_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA5_PORT,      .u8Pin = CY_SMIF0_DATA5_PIN,      .enMuxCfg = CY_SMIF0_DATA5_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA6_PORT,      .u8Pin = CY_SMIF0_DATA6_PIN,      .enMuxCfg = CY_SMIF0_DATA6_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif0,    .pstcPort = CY_SMIF0_DATA7_PORT,      .u8Pin = CY_SMIF0_DATA7_PIN,      .enMuxCfg = CY_SMIF0_DATA7_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
#ifdef CY_BSP_HDR_SMIF1_AVAILABLE
  #ifdef CY_SMIF1_CLK_INV_PORT // inverted clock is not available on PSVP (and hence would not be defined in BSP header)
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_CLK_INV_PORT,    .u8Pin = CY_SMIF1_CLK_INV_PIN,    .enMuxCfg = HSIOM_SEL_GPIO,              .u8DriveMode = CY_GPIO_DM_STRONG_IN_OFF, .u8GpioOutVal = 0 },
  #endif
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_CLK_PORT,        .u8Pin = CY_SMIF1_CLK_PIN,        .enMuxCfg = CY_SMIF1_CLK_PIN_MUX,        .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_RWDS_PORT,       .u8Pin = CY_SMIF1_RWDS_PIN,       .enMuxCfg = CY_SMIF1_RWDS_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_PULLDOWN,      .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_SELECT0_PORT,    .u8Pin = CY_SMIF1_SELECT0_PIN,    .enMuxCfg = CY_SMIF1_SELECT0_PIN_MUX,    .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 1 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_SELECT1_PORT,    .u8Pin = CY_SMIF1_SELECT1_PIN,    .enMuxCfg = CY_SMIF1_SELECT1_PIN_MUX,    .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 1 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA0_PORT,      .u8Pin = CY_SMIF1_DATA0_PIN,      .enMuxCfg = CY_SMIF1_DATA0_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA1_PORT,      .u8Pin = CY_SMIF1_DATA1_PIN,      .enMuxCfg = CY_SMIF1_DATA1_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA2_PORT,      .u8Pin = CY_SMIF1_DATA2_PIN,      .enMuxCfg = CY_SMIF1_DATA2_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA3_PORT,      .u8Pin = CY_SMIF1_DATA3_PIN,      .enMuxCfg = CY_SMIF1_DATA3_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA4_PORT,      .u8Pin = CY_SMIF1_DATA4_PIN,      .enMuxCfg = CY_SMIF1_DATA4_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA5_PORT,      .u8Pin = CY_SMIF1_DATA5_PIN,      .enMuxCfg = CY_SMIF1_DATA5_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA6_PORT,      .u8Pin = CY_SMIF1_DATA6_PIN,      .enMuxCfg = CY_SMIF1_DATA6_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
    { .enGroup = Smif1,    .pstcPort = CY_SMIF1_DATA7_PORT,      .u8Pin = CY_SMIF1_DATA7_PIN,      .enMuxCfg = CY_SMIF1_DATA7_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG,        .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
  #ifdef CY_BL_DISP0_EN_PORT
    { .enGroup = BlDisp0,  .pstcPort = CY_BL_DISP0_EN_PORT,      .u8Pin = CY_BL_DISP0_EN_PIN,      .enMuxCfg = CY_BL_DISP0_EN_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 }, // NA
  #endif
  #ifdef CY_BL_DISP0_PWM_PORT
    { .enGroup = BlDisp0,  .pstcPort = CY_BL_DISP0_PWM_PORT,     .u8Pin = CY_BL_DISP0_PWM_PIN,     .enMuxCfg = CY_BL_DISP0_PWM_PIN_MUX,     .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
  #ifdef CY_FPDLINK0_BL_EN_PORT
    { .enGroup = BlFpd0,   .pstcPort = CY_FPDLINK0_BL_EN_PORT,   .u8Pin = CY_FPDLINK0_BL_EN_PIN,   .enMuxCfg = CY_FPDLINK0_BL_EN_PIN_MUX,   .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
  #ifdef CY_FPDLINK0_BL_PWM_PORT
    { .enGroup = BlFpd0,   .pstcPort = CY_FPDLINK0_BL_PWM_PORT,  .u8Pin = CY_FPDLINK0_BL_PWM_PIN,  .enMuxCfg = CY_FPDLINK0_BL_PWM_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
//============================================================================================================================================================================================
  #ifdef CY_BL_DISP1_EN_PORT
    { .enGroup = BlDisp1,  .pstcPort = CY_BL_DISP1_EN_PORT,      .u8Pin = CY_BL_DISP1_EN_PIN,      .enMuxCfg = CY_BL_DISP1_EN_PIN_MUX,      .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 }, // NA
  #endif
  #ifdef CY_BL_DISP1_PWM_PORT
    { .enGroup = BlDisp1,  .pstcPort = CY_BL_DISP1_PWM_PORT,     .u8Pin = CY_BL_DISP1_PWM_PIN,     .enMuxCfg = CY_BL_DISP1_PWM_PIN_MUX,     .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
  #ifdef CY_FPDLINK1_BL_EN_PORT
    { .enGroup = BlFpd1,   .pstcPort = CY_FPDLINK1_BL_EN_PORT,   .u8Pin = CY_FPDLINK1_BL_EN_PIN,   .enMuxCfg = CY_FPDLINK1_BL_EN_PIN_MUX,   .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
  #ifdef CY_FPDLINK1_BL_PWM_PORT
    { .enGroup = BlFpd1,   .pstcPort = CY_FPDLINK1_BL_PWM_PORT,  .u8Pin = CY_FPDLINK1_BL_PWM_PIN,  .enMuxCfg = CY_FPDLINK1_BL_PWM_PIN_MUX,  .u8DriveMode = CY_GPIO_DM_STRONG, .u8GpioOutVal = 1 },
  #endif
//============================================================================================================================================================================================
#if (CY_USE_PSVP == 0)  // --> Button availability may differ depending on used target board, so check whether button is defined in BSP header
  #ifdef CY_USER_BUTTON_WAKE_PORT
    { .enGroup = Button,   .pstcPort = CY_USER_BUTTON_WAKE_PORT, .u8Pin = CY_USER_BUTTON_WAKE_PIN, .enMuxCfg = CY_USER_BUTTON_WAKE_PIN_MUX, .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
  #endif
  #ifdef CY_USER_BUTTON_LEFT_PORT
    { .enGroup = Button,   .pstcPort = CY_USER_BUTTON_LEFT_PORT, .u8Pin = CY_USER_BUTTON_LEFT_PIN, .enMuxCfg = CY_USER_BUTTON_LEFT_PIN_MUX, .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
  #endif
  #ifdef CY_USER_BUTTON_RIGHT_PORT
    { .enGroup = Button,   .pstcPort = CY_USER_BUTTON_RIGHT_PORT,.u8Pin = CY_USER_BUTTON_RIGHT_PIN,.enMuxCfg = CY_USER_BUTTON_RIGHT_PIN_MUX,.u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
  #endif
  #ifdef CY_USER_BUTTON_UP_PORT
    { .enGroup = Button,   .pstcPort = CY_USER_BUTTON_UP_PORT,   .u8Pin = CY_USER_BUTTON_UP_PIN,   .enMuxCfg = CY_USER_BUTTON_UP_PIN_MUX,   .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
  #endif
  #ifdef CY_USER_BUTTON_DOWN_PORT
    { .enGroup = Button,   .pstcPort = CY_USER_BUTTON_DOWN_PORT, .u8Pin = CY_USER_BUTTON_DOWN_PIN, .enMuxCfg = CY_USER_BUTTON_DOWN_PIN_MUX, .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
  #endif
#else
    { .enGroup = Button,   .pstcPort = CY_AB_BUTTON_PORT,        .u8Pin = CY_AB_BUTTON_PIN,        .enMuxCfg = CY_AB_BUTTON_PIN_MUX,        .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_CEB_BUTTON_PORT,       .u8Pin = CY_CEB_BUTTON_PIN,       .enMuxCfg = CY_CEB_BUTTON_PIN_MUX,       .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_BUTTON1_PORT,          .u8Pin = CY_BUTTON1_PIN,          .enMuxCfg = CY_BUTTON1_PIN_MUX,          .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_BUTTON2_PORT,          .u8Pin = CY_BUTTON2_PIN,          .enMuxCfg = CY_BUTTON2_PIN_MUX,          .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_BUTTON3_PORT,          .u8Pin = CY_BUTTON3_PIN,          .enMuxCfg = CY_BUTTON3_PIN_MUX,          .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_BUTTON4_PORT,          .u8Pin = CY_BUTTON4_PIN,          .enMuxCfg = CY_BUTTON4_PIN_MUX,          .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
    { .enGroup = Button,   .pstcPort = CY_BUTTON5_PORT,          .u8Pin = CY_BUTTON5_PIN,          .enMuxCfg = CY_BUTTON5_PIN_MUX,          .u8DriveMode = CY_GPIO_DM_HIGHZ,  .u8GpioOutVal = 0 },
#endif
//============================================================================================================================================================================================
};

/*****************************************************************************
 ** \brief Button definition
 *****************************************************************************/
static const cy_button_stc_definition_t m_astcButtonDef[] =
{
#if (CY_USE_PSVP == 0)  // --> Button availability may differ depending on used target board, so check whether button is defined in BSP header
  #ifdef CY_USER_BUTTON_WAKE_PORT
    { .pstcPort = CY_USER_BUTTON_WAKE_PORT,     .u8Pin = CY_USER_BUTTON_WAKE_PIN,       .u8ButtonId = CY_GFXENV_BUTTON_ID_WAKE  },
  #endif
  #ifdef CY_USER_BUTTON_LEFT_PORT
    { .pstcPort = CY_USER_BUTTON_LEFT_PORT,     .u8Pin = CY_USER_BUTTON_LEFT_PIN,       .u8ButtonId = CY_GFXENV_BUTTON_ID_LEFT  },
  #endif
  #ifdef CY_USER_BUTTON_RIGHT_PORT
    { .pstcPort = CY_USER_BUTTON_RIGHT_PORT,    .u8Pin = CY_USER_BUTTON_RIGHT_PIN,      .u8ButtonId = CY_GFXENV_BUTTON_ID_RIGHT },
  #endif
  #ifdef CY_USER_BUTTON_UP_PORT
    { .pstcPort = CY_USER_BUTTON_UP_PORT,       .u8Pin = CY_USER_BUTTON_UP_PIN,         .u8ButtonId = CY_GFXENV_BUTTON_ID_UP    },
  #endif
  #ifdef CY_USER_BUTTON_DOWN_PORT
    { .pstcPort = CY_USER_BUTTON_DOWN_PORT,     .u8Pin = CY_USER_BUTTON_DOWN_PIN,       .u8ButtonId = CY_GFXENV_BUTTON_ID_DOWN  },
  #endif
#else
    { .pstcPort = CY_AB_BUTTON_PORT,            .u8Pin = CY_AB_BUTTON_PIN,              .u8ButtonId = CY_GFXENV_BUTTON_ID_AB_CB },
    { .pstcPort = CY_CEB_BUTTON_PORT,           .u8Pin = CY_CEB_BUTTON_PIN,             .u8ButtonId = CY_GFXENV_BUTTON_ID_CEB   },
    { .pstcPort = CY_BUTTON1_PORT,              .u8Pin = CY_BUTTON1_PIN,                .u8ButtonId = CY_GFXENV_BUTTON_ID_BB_1  },
    { .pstcPort = CY_BUTTON2_PORT,              .u8Pin = CY_BUTTON2_PIN,                .u8ButtonId = CY_GFXENV_BUTTON_ID_BB_2  },
    { .pstcPort = CY_BUTTON3_PORT,              .u8Pin = CY_BUTTON3_PIN,                .u8ButtonId = CY_GFXENV_BUTTON_ID_BB_3  },
    { .pstcPort = CY_BUTTON4_PORT,              .u8Pin = CY_BUTTON4_PIN,                .u8ButtonId = CY_GFXENV_BUTTON_ID_BB_4  },
    { .pstcPort = CY_BUTTON5_PORT,              .u8Pin = CY_BUTTON5_PIN,                .u8ButtonId = CY_GFXENV_BUTTON_ID_BB_5  },
#endif
};


/*****************************************************************************
 ** \brief Hold GfxEnv driver internal config/status of all SMIFs and SMIF_DEVICEs
 *****************************************************************************/
static stc_smif_cfg_internal_t m_astcSmifIntCfg[CY_GFXENV_NR_OF_SMIF];

/*****************************************************************************
 ** \brief SMIF DLL configuration for S27K
 *****************************************************************************/
static cy_stc_smif_dll_config_t m_stcSmifDllCfgS27K = 
{
    .pllFreq    = 0,                                                         // <- updated during runtime
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV2,                                  // <- updated during runtime
    .mdlTapSel  = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI400,  // <- updated during runtime
    .rxCapMode  = CY_SMIF_CAP_MODE_RWDS,
    .txSdrExtra = CY_SMIF_TX_ONE_PERIOD_AHEAD,
};

/*****************************************************************************
 ** \brief SMIF DLL configuration for S28H
 *****************************************************************************/
static cy_stc_smif_dll_config_t m_stcSmifDllCfgS28H = 
{
    .pllFreq    = 0,                                                        // <- updated during runtime
    .mdlOutDiv  = CY_SMIF_MDL_CLK_OUT_DIV2,                                 // <- updated during runtime
    .mdlTapSel  = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI400, // <- updated during runtime
    .rxCapMode  = CY_SMIF_CAP_MODE_SPI,                                     // <- updated during runtime
    .txSdrExtra = CY_SMIF_TX_ONE_PERIOD_AHEAD,
};

/*****************************************************************************
 ** \brief SMIF configuration for S27K
 *****************************************************************************/
static const cy_stc_smif_config_t m_stcSmifCfgS27K =
{
    // Other struct members not relevant for SMIF ver4

    .deselectDelay = CY_SMIF_MIN_DESELECT_2_CLK,     // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .setupDelay    = CY_SMIF_SETUP_1_CLK_PULUS_MIN,  // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,   // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .mode          = CY_SMIF_MEMORY,
    .blockEvent    = CY_SMIF_BUS_ERROR,
    .clkIfSrc      = CY_SMIF_CLKIF_SRC_CLK_PLL_NORMAL,
    .pDllCfg       = &m_stcSmifDllCfgS27K,
};

/*****************************************************************************
 ** \brief SMIF configuration for S28H
 *****************************************************************************/
static const cy_stc_smif_config_t m_stcSmifCfgS28H =
{
    // Other struct members not relevant for SMIF ver4

    .deselectDelay = CY_SMIF_MIN_DESELECT_2_CLK,     // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .setupDelay    = CY_SMIF_SETUP_1_CLK_PULUS_MIN,  // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,   // Should work at max. freq for any S26K/S26H/S27K device type (though it might not be optimal for slower frequencies)
    .mode          = CY_SMIF_NORMAL,                 // <- updated during runtime
    .blockEvent    = CY_SMIF_BUS_ERROR,
    .clkIfSrc      = CY_SMIF_CLKIF_SRC_CLK_PLL_NORMAL,
    .pDllCfg       = &m_stcSmifDllCfgS28H,
};

/*****************************************************************************
 ** \brief SMIF Device Configuration for S27K
 *****************************************************************************/
static cy_stc_device_hb_config_t m_stcSmifDevCfgS27K =
{
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_256_CYCLE,
    .totalTimeoutEnable = true,                            // HyperRAM has a total timeout constraint based on the maximum CS low time according its datasheet
    .totalTimeout       = 0,                               // <- updated during runtime for optimal value
    .addr               = 0,                               // <- updated during runtime as per user configuration!
    .size               = CY_SMIF_DEVICE_8M_BYTE,          // <- updated during runtime as per user configuration!
    .lc_hb              = GFXENV_S27K_INITIAL_LATENCY,     // <- updated during runtime for optimal value
    .hbDevType          = CY_SMIF_HB_SRAM,
};

/*****************************************************************************
 ** \brief SMIF Device Configuration for S28H
 *****************************************************************************/
static cy_stc_device_config_t m_stcSmifDevCfgS28H =
{
    .xipReadCmdId       = CY_SMIF_SEMP_RDID_8D8D8D_4ADR,
    .xipReadMode        = 00,                            // Don't care
    .writeEn            = true,
    .xipWriteCmdId      = CY_SMIF_SEMP_WTID_8D8D8D_4ADR,
    .xipWriteMode       = 0,                             // Don't care
    .mergeEnable        = true,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_4096_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 0,
    .addr               = 0,                             // <- updated during runtime as per user configuration!
    .size               = CY_SMIF_DEVICE_64M_BYTE,       // <- updated during runtime as per user configuration!
    .rdLatencyCode      = CY_SMIF_SEMP_RD_LATENCY9,      // <- updated during runtime for optimal value
    .wtLatencyCode      = CY_SMIF_SEMP_WT_LATENCY0,
};

/*****************************************************************************
 ** \brief Global MPU Control Bit Settings during SMIF initialization
 *****************************************************************************/
static cy_stc_mpu_global_ctrl_bits_t m_stcMpuGlobalCtrlBits =
{
    .mpuGlobalEnable = CY_MPU_GLOBAL_ENABLE,
    .privDefMapEn    = CY_MPU_USE_DEFAULT_MAP_AS_BG,
    .faultNmiEn      = CY_MPU_ENABLED_DURING_FAULT_NMI,
};
static cy_stc_mpu_global_ctrl_bits_t m_stcMpuGlobalCtrlBitsBackup;

/*****************************************************************************
 ** \brief MPU Region Settings during SMIF initialization
 *****************************************************************************/
static cy_stc_mpu_region_cfg_t m_stcSmifMpuRegion =
{
    .addr         = 0,               // Region will cover 0x60000000 - 0x9fffffff
    .size         = CY_MPU_SIZE_4GB, // Region will cover 0x60000000 - 0x9fffffff
    .srd          = 0xe7,            // Region will cover 0x60000000 - 0x9fffffff
    .permission   = CY_MPU_ACCESS_P_FULL_ACCESS,
    .attribute    = CY_MPU_ATTR_SHR_DEV,
    .execute      = CY_MPU_INST_ACCESS_EN,
    .enable       = CY_MPU_ENABLE,
};
static cy_stc_mpu_region_cfg_t m_stcSmifMpuRegionBackup;


/*****************************************************************************
* Local function prototypes ('static')
*****************************************************************************/
static void                  InitPortPins                   (const cy_gfxenv_stc_init_portpins_t * pstcPortInitCfg);
static void                  SetDriveStrengthTrimForPortPins(en_portpin_group_t enTargetGroup, uint8_t u8DriveStrengthTrimVal);
static cy_gfxenv_en_result_t ChangeSmifPllClock             (uint16_t u16SmifPllClkMhz);
static cy_gfxenv_en_result_t InitExtMem                     (const cy_gfxenv_stc_init_extmem_t * pstcExtMemCfg);
static cy_gfxenv_en_result_t InitSmifCoreGeneric            (stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg);
static cy_gfxenv_en_result_t InitSmifCoreAndDeviceForS27K   (stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg);
static cy_gfxenv_en_result_t InitSmifCoreAndDeviceForS28H   (stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg);
static cy_gfxenv_en_result_t ConfigureIoRegsInS27KXXXX2     (stc_smif_cfg_internal_t * pstcSmifIntCfg, cy_en_smif_hb_latency_code_t enLatency);
static cy_gfxenv_en_result_t ConfigureIoRegsInS28H          (stc_smif_cfg_internal_t * pstcSmifIntCfg, cy_en_smif_semp_read_latency_code_t enLatency);
static cy_gfxenv_en_result_t GetOptimalMemoryLatencyForS28H (bool b1V8Variant, cy_en_smif_trx_type_t enTransMode, uint8_t u8SmifExtClockMhz, cy_en_smif_semp_read_latency_code_t * penLatency);
static cy_gfxenv_en_result_t ResetS28HViaCsSignalingProtocol(uint8_t u8CoreNumber, uint8_t u8DeviceNumber);

/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/

/*******************************************************************************
* Function Name: Cy_GfxEnv_Init
****************************************************************************//**
*
* \brief  Initializes the GFX environment
*
* \param  pstcGfxEnvCfg : The pointer to the init configuration struct
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
cy_gfxenv_en_result_t Cy_GfxEnv_Init(const cy_gfxenv_stc_cfg_t *pstcGfxEnvCfg)
{
    // Initialize port pins if requested by user configuration
    if(pstcGfxEnvCfg->pstcInitPortPins != NULL)
    {
        InitPortPins(pstcGfxEnvCfg->pstcInitPortPins);
    }

    // Initialize software timer module if requested by user configuration
    if(pstcGfxEnvCfg->bInitSwTimer != false)
    {
        Cy_SwTmr_Init();
    }

    // Initialize semihosting (low level I/O) together with required clock and port pin setup if requested by user configuration
    if(pstcGfxEnvCfg->bInitSemihosting != false)
    {
        if(Cy_Semihosting_InitAll(CY_GFXENV_SEMIHOSTING_SCB, 115200UL, CY_GFXENV_SEMIHOSTING_WDG_HANDLER, false) != CY_SEMIHOSTING_SUCCESS)
        {
            return CY_GFXENV_ERROR;
        }
    }

    // Initialize button module if requested by user configuration
    if(pstcGfxEnvCfg->pstcInitButtons != NULL)
    {
        cy_gfxenv_stc_init_buttons_t * pstcCfg = pstcGfxEnvCfg->pstcInitButtons;

        if(Cy_Button_Init(&m_astcButtonDef[0], CY_GET_ARRAY_LEN(m_astcButtonDef), pstcCfg->pfnCallback, pstcCfg->u8CySwTimerId) != CY_BUTTON_SUCCESS)
        {
            return CY_GFXENV_ERROR;
        }
    }

    // Initialize external memories if requested by user configuration
    if(pstcGfxEnvCfg->pstcInitExtMem != NULL)
    {
        cy_gfxenv_en_result_t enExtMemInitRetVal;

        // Temporarily configure an MPU region to make SMIF XIP address space "Device Shareable"
        // which is needed for reliable access of I/O registers in HyperBus memories
        if(pstcGfxEnvCfg->pstcInitExtMem->bInitMpuTemporarily != false)
        {
            Cy_MPU_GetGlobalControlBits(&m_stcMpuGlobalCtrlBitsBackup);
            Cy_MPU_Disable();
            Cy_MPU_GetRegion(&m_stcSmifMpuRegionBackup, pstcGfxEnvCfg->pstcInitExtMem->u8MpuRegionTemp);
            Cy_MPU_SetRegion(&m_stcSmifMpuRegion,       pstcGfxEnvCfg->pstcInitExtMem->u8MpuRegionTemp);
            Cy_MPU_SetGlobalControlBits(&m_stcMpuGlobalCtrlBits);
        }

        enExtMemInitRetVal = InitExtMem(pstcGfxEnvCfg->pstcInitExtMem);

        // Restore MPU Settings
        if(pstcGfxEnvCfg->pstcInitExtMem->bInitMpuTemporarily != false)
        {
            Cy_MPU_Disable();
            Cy_MPU_SetRegion(&m_stcSmifMpuRegionBackup, pstcGfxEnvCfg->pstcInitExtMem->u8MpuRegionTemp);
            Cy_MPU_SetGlobalControlBits(&m_stcMpuGlobalCtrlBitsBackup);
        }

        if(enExtMemInitRetVal != CY_GFXENV_SUCCESS)
        {
            return enExtMemInitRetVal;
        }
    }

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: Cy_GfxEnv_FastInit
****************************************************************************//**
*
* \brief  Initializes the GFX environment with most commonly used settings.
*         Only a limited number of parameters can be configured, e.g.
*         whether SMIF instances and connected memories are initialized
*         because this would stuck execution if memory is not connected or in
*         case of signal problems.
*
* \warning u8Smif0ExtClockMhz and u8Smif1ExtClockMhz ratio needs to be 1, 2, 4 or 8 
*          (or 1, 0.5, 0.25, 0.125 respectively) at the moment, because by default
*          the SMIF HF clocks use the same source PLL and hence only the HF clock
*          dividers can be used
*
* \param  u8Smif0ExtClockMhz : SMIF0 external clock frequency,
*                              if value == 0, SMIF0 will not be initialized
* \param  u8Smif1ExtClockMhz : SMIF1 external clock frequency,
*                              if value == 0, SMIF1 will not be initialized
* \param  pfnButtonCallback  : Callback from button handler, can be NULL (then
*                              button driver and port pins are not initialized)
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
cy_gfxenv_en_result_t Cy_GfxEnv_FastInit(uint8_t u8Smif0ExtClockMhz, uint8_t u8Smif1ExtClockMhz, cy_button_callback_t pfnButtonCallback)
{
    cy_gfxenv_stc_cfg_t stcGfxEnvCfg =
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
                                    .bInitSmif0             = true,  // might be overriden by function parameter u8Smif0ExtClockMhz
                                    .bInitSmif1             = true,  // might be overriden by function parameter u8Smif1ExtClockMhz
                                    .bInitBacklightDisp0    = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                    .bInitBacklightDisp1    = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                    .bInitBacklightFpdLink0 = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                    .bInitBacklightFpdLink1 = false, // backlights are enabled by jumper setting on CY boards, would only be needed for PWM dimming
                                    .bInitButtonGpios       = true,
                                },
        .pstcInitExtMem       = &(cy_gfxenv_stc_init_extmem_t)
                                {
                                    .bInitMpuTemporarily = true,
                                    .u8MpuRegionTemp     = CY_GFXENV_SMIF_TEMP_MPU_REGION,
                                    .apstcInitSmif       = {
                                                                &(cy_gfxenv_stc_smif_cfg_t)
                                                                {
                                                                    .u8SmifExtClockMhz = 0, // updated by function parameter
                                                                    .apstcDevUsrCfg    = {
                                                                                             &(cy_gfxenv_stc_smif_device_cfg_t)
                                                                                             {
                                                                                                 .u32BaseAddress           = CY_GFXENV_FASTINIT_EXTMEM_FLASH_BASE,
                                                                                                 .u16SizeMegaBytes         = 64,
                                                                                                 .enExtMemType             = CY_GFXENV_EXTMEM_TYPE_S28H,
                                                                                             },
                                                                                             NULL,
                                                                                         },
                                                                }, 
                                                                &(cy_gfxenv_stc_smif_cfg_t)
                                                                {
                                                                    .u8SmifExtClockMhz = 0, // updated by function parameter
                                                                    .apstcDevUsrCfg    = {
                                                                                             NULL,
                                                                                             &(cy_gfxenv_stc_smif_device_cfg_t)
                                                                                             {
                                                                                                 .u32BaseAddress           = CY_GFXENV_FASTINIT_EXTMEM_RAM_BASE,
                                                                                                 .u16SizeMegaBytes         = 8,
                                                                                                 .enExtMemType             = CY_GFXENV_EXTMEM_TYPE_S27KXXXX2,
                                                                                             },
                                                                                         },
                                                                },
                                                            },
                                },
        .pstcInitButtons      = &(cy_gfxenv_stc_init_buttons_t)
                                {
                                    .u8CySwTimerId = (CY_SWTMR_MAX_TIMERS - 1), // use last timer
                                    .pfnCallback   = NULL,
                                },

    };

    // Disable SMIF clock, port and memory initialization if not requested by user, otherwise update the frequency with value from user
    if(u8Smif0ExtClockMhz == 0)
    {
        stcGfxEnvCfg.pstcInitPortPins->bInitSmif0 = false;
        stcGfxEnvCfg.pstcInitExtMem->apstcInitSmif[0] = NULL;
    }
    else
    {
        stcGfxEnvCfg.pstcInitExtMem->apstcInitSmif[0]->u8SmifExtClockMhz = u8Smif0ExtClockMhz;
    }
    if(u8Smif1ExtClockMhz == 0)
    {
        stcGfxEnvCfg.pstcInitPortPins->bInitSmif1 = false;
        stcGfxEnvCfg.pstcInitExtMem->apstcInitSmif[1] = NULL;
    }    
    else
    {
        stcGfxEnvCfg.pstcInitExtMem->apstcInitSmif[1]->u8SmifExtClockMhz = u8Smif1ExtClockMhz;
    }
    if((u8Smif0ExtClockMhz == 0) && (u8Smif1ExtClockMhz == 0))
    {
        stcGfxEnvCfg.pstcInitExtMem = NULL;
    }

    // Disable button driver and corresponding port pin initialization if user does not provide callback
    if(pfnButtonCallback == NULL)
    {
        stcGfxEnvCfg.pstcInitButtons = NULL;
    }
    else
    {
        stcGfxEnvCfg.pstcInitButtons->pfnCallback = pfnButtonCallback;
    }

    return Cy_GfxEnv_Init(&stcGfxEnvCfg);
}


/*******************************************************************************
* Function Name: Cy_GfxEnv_EnableTestImage
****************************************************************************//**
*
* \brief  Enables the output of test images on the 2 display links.
*         Prior initialization (e.g. Cy_GfxEnv_Init or Cy_GfxEnv_FastInit APIs)
*         may be necessary before calling this function.
*
* \param  enDisp0Type : The display type to be setup for display link #0
* \param  enDisp1Type : The display type to be setup for display link #1
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
cy_gfxenv_en_result_t Cy_GfxEnv_EnableTestImage(cy_gfxenv_en_disp_type_t enDisp0Type, cy_gfxenv_en_disp_type_t enDisp1Type)
{
    cy_gfxenv_en_result_t enRetVal = CY_GFXENV_SUCCESS;
    uint8_t u8Disp0ClkPllDiv;
    uint8_t u8Disp1ClkPllDiv;

  #ifdef CYREG_VIDEOSS0_VIDEOSSCFG_DSP0_CTL
    // Set global Enable bit for DSP0 domain
    VIDEOSS0_SUBSS0_VIDEOSSCFG->unDSP0_CTL.stcField.u1DSP0_ENABLED = 1;
  #endif
  #ifdef CYREG_VIDEOSS0_VIDEOSSCFG_DSP1_CTL
    // Set global Enable bit for DSP1 domain
    VIDEOSS0_SUBSS0_VIDEOSSCFG->unDSP1_CTL.stcField.u1DSP1_ENABLED = 1;
  #endif    


  #if (CY_USE_PSVP == 0)

    cy_en_sysclk_status_t enSysClkRetVal;
    uint32_t u32Disp0PixelClkHz;
    uint32_t u32Disp1PixelClkHz;
    uint32_t u32Disp0ClkPllHz;
    uint32_t u32Disp1ClkPllHz;
    uint32_t disp0PllInputFreq;
    uint32_t disp1PllInputFreq;

    if((enDisp0Type == CY_GFXENV_DISP_TYPE_NONE) && (enDisp1Type == CY_GFXENV_DISP_TYPE_NONE))
    {
        return CY_GFXENV_ERROR_BAD_PARAM; // at least one display must be selected
    }

    if(enDisp0Type != CY_GFXENV_DISP_TYPE_NONE)
    {

        u32Disp0PixelClkHz = (uint32_t) m_astcDispPresets[enDisp0Type].u16HorizontalTotal *
                             (uint32_t) m_astcDispPresets[enDisp0Type].u16VerticalTotal   *
                             (uint32_t) m_astcDispPresets[enDisp0Type].u8RefreshRateHz;

        // Search the necessary PLL frequency and divider
        for(u8Disp0ClkPllDiv = 1; u8Disp0ClkPllDiv <= 8; u8Disp0ClkPllDiv *= 2)
        {
            u32Disp0ClkPllHz = u32Disp0PixelClkHz * VIDEOSS_FIXED_DISP_CLK_DIV * u8Disp0ClkPllDiv;
            if((u32Disp0ClkPllHz >= DISP0_CLK_PLL_MIN_HZ) && (u32Disp0ClkPllHz <= DISP0_CLK_PLL_MAX_HZ))
            {
                break;
            }
        }

        if(u8Disp0ClkPllDiv > 8)
        {
            return CY_GFXENV_ERROR_BAD_PARAM; // suitable divider not found
        }

        enSysClkRetVal = Cy_SysClk_Pll400MDisable(CLK_PATH_DISP0_CLK);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
        enSysClkRetVal = Cy_SysClk_GetPllInputFrequency(CLK_PATH_DISP0_CLK - 1, &disp0PllInputFreq);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
        (void) enSysClkRetVal;

        cy_stc_pll_400M_config_t stcPllCfg =
        {
            .inputFreq    = disp0PllInputFreq,
            .outputFreq   = u32Disp0ClkPllHz,
            .outputMode   = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
            .fracEn       = true,
            .fracDitherEn = true,
            .sscgEn       = true,
            .sscgDitherEn = true,
            .sscgDepth    = CY_SYSCLK_SSCG_DEPTH_MINUS_2_0,
            .sscgRate     = CY_SYSCLK_SSCG_RATE_DIV_512,
        };
        enSysClkRetVal = Cy_SysClk_Pll400MConfigure(CLK_PATH_DISP0_CLK, &stcPllCfg);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
        enSysClkRetVal = Cy_SysClk_Pll400MEnable(CLK_PATH_DISP0_CLK, 10000);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
    }

    if(enDisp1Type != CY_GFXENV_DISP_TYPE_NONE)
    {

        u32Disp1PixelClkHz = (uint32_t) m_astcDispPresets[enDisp1Type].u16HorizontalTotal *
                             (uint32_t) m_astcDispPresets[enDisp1Type].u16VerticalTotal   *
                             (uint32_t) m_astcDispPresets[enDisp1Type].u8RefreshRateHz;

        // Search the necessary PLL frequency and divider
        for(u8Disp1ClkPllDiv = 1; u8Disp1ClkPllDiv <= 8; u8Disp1ClkPllDiv *= 2)
        {
            u32Disp1ClkPllHz = u32Disp1PixelClkHz * VIDEOSS_FIXED_DISP_CLK_DIV * u8Disp1ClkPllDiv;
            if((u32Disp1ClkPllHz >= DISP1_CLK_PLL_MIN_HZ) && (u32Disp1ClkPllHz <= DISP1_CLK_PLL_MAX_HZ))
            {
                break;
            }
        }

        if(u8Disp1ClkPllDiv > 8)
        {
            return CY_GFXENV_ERROR_BAD_PARAM; // suitable divider not found
        }

        enSysClkRetVal = Cy_SysClk_PllDisable(CLK_PATH_DISP1_CLK);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
        enSysClkRetVal = Cy_SysClk_GetPllInputFrequency(CLK_PATH_DISP1_CLK - 1, &disp1PllInputFreq);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);

        cy_stc_pll_config_t stcPllCfg =
        {
            .inputFreq    = disp1PllInputFreq,
            .outputFreq   = u32Disp1ClkPllHz,
            .outputMode   = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
            .lfMode       = false,
        };
        enSysClkRetVal = Cy_SysClk_PllConfigure(CLK_PATH_DISP1_CLK, &stcPllCfg);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
        enSysClkRetVal = Cy_SysClk_PllEnable(CLK_PATH_DISP1_CLK, 10000);
        CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
    }



  #else
    // PSVP

    if((m_astcDispPresets[enDisp0Type].bUseFpd != false) || (m_astcDispPresets[enDisp1Type].bUseFpd != false))
    {
        return CY_GFXENV_ERROR_BAD_PARAM; // no FPD Link on PSVP
    }
    u8Disp0ClkPllDiv = 1;
    u8Disp1ClkPllDiv = 1;

  #endif

    if(enDisp0Type != CY_GFXENV_DISP_TYPE_NONE)
    {
        if(m_astcDispPresets[enDisp0Type].bUseFpd != false)
        {
            uint8_t u8MapIndex = m_astcDispPresets[enDisp0Type].u8MapIndex;
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT3_0.u32Register   = m_aau32FpdMapping[0][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT7_4.u32Register   = m_aau32FpdMapping[1][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT11_8.u32Register  = m_aau32FpdMapping[2][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT15_12.u32Register = m_aau32FpdMapping[3][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT19_16.u32Register = m_aau32FpdMapping[4][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT23_20.u32Register = m_aau32FpdMapping[5][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unMAPBIT27_24.u32Register = m_aau32FpdMapping[6][u8MapIndex];

            VIDEOSS0_VIDEOIO0_DISENG00_DISENGCFG0->unCTL.stcField.u1DSPCLKDIVIDE = 1; // needed ???

            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unTCON_CTRL.u32Register = 0;
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unTCON_CTRL.stcField.u1BYPASS = 1;
            VIDEOSS0_VIDEOIO0_DISENG00_TCON0->unTCON_CTRL.stcField.u1ENLVDS = 1;

            if(Cy_Fpdlink_FastInit(Fpdlink0, (cy_fpdlink_en_pll_out_div_t) CLK_DIV_TO_REG_VAL(u8Disp0ClkPllDiv)) != CY_FPDLINK_SUCCESS)
            {
                enRetVal = CY_GFXENV_ERROR;
            }
        }

        VIDEOSS0_SUBSS0_VIDEOSSCFG->unCLKDSP0CFG.stcField.u2DIVVAL0       = CLK_DIV_TO_REG_VAL(u8Disp0ClkPllDiv);
        VIDEOSS0_SUBSS0_VIDEOSSCFG->unCLKDSP0CFG.stcField.u1TTLCLKINV0    = 0;

        VIDEOSS0_VIDEOIO0_DISENG00_FRAMEGEN0->unHTCFG1.stcField.u14HACT   = m_astcDispPresets[enDisp0Type].u16HorizontalActive;
        VIDEOSS0_VIDEOIO0_DISENG00_FRAMEGEN0->unHTCFG1.stcField.u14HTOTAL = m_astcDispPresets[enDisp0Type].u16HorizontalTotal;

        VIDEOSS0_VIDEOIO0_DISENG00_FRAMEGEN0->unVTCFG1.stcField.u14VACT   = m_astcDispPresets[enDisp0Type].u16VerticalActive;
        VIDEOSS0_VIDEOIO0_DISENG00_FRAMEGEN0->unVTCFG1.stcField.u14VTOTAL = m_astcDispPresets[enDisp0Type].u16VerticalTotal;

        VIDEOSS0_VIDEOIO0_DISENG00_FRAMEGEN0->unFGENABLE.stcField.u1FGEN  = 1;
    }

    if(enDisp1Type != CY_GFXENV_DISP_TYPE_NONE)
    {
        if(m_astcDispPresets[enDisp1Type].bUseFpd != false)
        {
          #ifdef VIDEOSS0_FPDLINK1          
            uint8_t u8MapIndex = m_astcDispPresets[enDisp1Type].u8MapIndex;
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT3_0.u32Register   = m_aau32FpdMapping[0][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT7_4.u32Register   = m_aau32FpdMapping[1][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT11_8.u32Register  = m_aau32FpdMapping[2][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT15_12.u32Register = m_aau32FpdMapping[3][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT19_16.u32Register = m_aau32FpdMapping[4][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT23_20.u32Register = m_aau32FpdMapping[5][u8MapIndex];
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unMAPBIT27_24.u32Register = m_aau32FpdMapping[6][u8MapIndex];

            VIDEOSS0_VIDEOIO0_DISENG10_DISENGCFG1->unCTL.stcField.u1DSPCLKDIVIDE = 1; // needed ???

            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unTCON_CTRL.u32Register = 0;
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unTCON_CTRL.stcField.u1BYPASS = 1;
            VIDEOSS0_VIDEOIO0_DISENG10_TCON1->unTCON_CTRL.stcField.u1ENLVDS = 1;

            if(Cy_Fpdlink_FastInit(Fpdlink1, (cy_fpdlink_en_pll_out_div_t) CLK_DIV_TO_REG_VAL(u8Disp1ClkPllDiv)) != CY_FPDLINK_SUCCESS)
            {
                enRetVal = CY_GFXENV_ERROR;
            }            
          #else              
            enRetVal = CY_GFXENV_ERROR_BAD_PARAM;            
          #endif
        }

        VIDEOSS0_SUBSS0_VIDEOSSCFG->unCLKDSP1CFG.stcField.u2DIVVAL1       = CLK_DIV_TO_REG_VAL(u8Disp1ClkPllDiv);
        VIDEOSS0_SUBSS0_VIDEOSSCFG->unCLKDSP1CFG.stcField.u1TTLCLKINV1    = 0;

        VIDEOSS0_VIDEOIO0_DISENG10_FRAMEGEN1->unHTCFG1.stcField.u14HACT   = m_astcDispPresets[enDisp1Type].u16HorizontalActive;
        VIDEOSS0_VIDEOIO0_DISENG10_FRAMEGEN1->unHTCFG1.stcField.u14HTOTAL = m_astcDispPresets[enDisp1Type].u16HorizontalTotal;

        VIDEOSS0_VIDEOIO0_DISENG10_FRAMEGEN1->unVTCFG1.stcField.u14VACT   = m_astcDispPresets[enDisp1Type].u16VerticalActive;
        VIDEOSS0_VIDEOIO0_DISENG10_FRAMEGEN1->unVTCFG1.stcField.u14VTOTAL = m_astcDispPresets[enDisp1Type].u16VerticalTotal;

        VIDEOSS0_VIDEOIO0_DISENG10_FRAMEGEN1->unFGENABLE.stcField.u1FGEN  = 1;
    }

    return enRetVal;
}


/*******************************************************************************
* Function Name: InitPortPins
****************************************************************************//**
*
* \brief  Initializes all port pins that shall be configured according to user
*         configuration.
*
* \param  pstcCfg : Pointer to to port pin config struct member of GFX Env
*                   config struct
*
*******************************************************************************/
static void InitPortPins(const cy_gfxenv_stc_init_portpins_t * pstcPortInitCfg)
{
    for(uint16_t u16Index = 0; u16Index < CY_GET_ARRAY_LEN(m_astcPortPinCfg); u16Index++)
    {
        const stc_portpin_cfg_t * pstcCurPortPin = &m_astcPortPinCfg[u16Index];

        // Skip this loop iteration if the current port pin cfg belongs to a group that shall not be initialized
        switch(pstcCurPortPin->enGroup)
        {
            case Display0:
                if(pstcPortInitCfg->bInitDisplay0Ttl == false)
                {
                    continue;
                }
                break;
            case Display1:
                if(pstcPortInitCfg->bInitDisplay1Ttl == false)
                {
                    continue;
                }
                break;
            case Capture0:
                if(pstcPortInitCfg->bInitCapture0Ttl == false)
                {
                    continue;
                }
                break;
            case Smif0:
                if(pstcPortInitCfg->bInitSmif0 == false)
                {
                    continue;
                }
                break;
            case Smif1:
                if(pstcPortInitCfg->bInitSmif1 == false)
                {
                    continue;
                }
                break;
            case BlDisp0:
                if(pstcPortInitCfg->bInitBacklightDisp0 == false)
                {
                    continue;
                }
                break;
            case BlDisp1:
                if(pstcPortInitCfg->bInitBacklightDisp1 == false)
                {
                    continue;
                }
                break;
            case BlFpd0:
                if(pstcPortInitCfg->bInitBacklightFpdLink0 == false)
                {
                    continue;
                }
                break;
            case BlFpd1:
              #ifdef VIDEOSS0_FPDLINK1
                if(pstcPortInitCfg->bInitBacklightFpdLink1 == false)
                {
                    continue;
                }
                break;
              #else
                continue;
              #endif
            case Button:
                if(pstcPortInitCfg->bInitButtonGpios == false)
                {
                    continue;
                }
                break;
            default:
                break;
        }

        // If more detailed port pin settings need to be made (e.g. input trip point, slew rate, drive strength)
        // the implementation must be changed to use Cy_GPIO_Pin_Init API
        Cy_GPIO_Pin_FastInit( pstcCurPortPin->pstcPort,
                              pstcCurPortPin->u8Pin,
                              pstcCurPortPin->u8DriveMode,
                              pstcCurPortPin->u8GpioOutVal,
                              pstcCurPortPin->enMuxCfg      );

    }
}


/*******************************************************************************
* Function Name: SetDriveStrengthTrimForPortPins
****************************************************************************//**
*
* \brief  Sets the drive strength trims for the target group of pins,
*         typically required for SMIF pins to meet datasheet conditions
*
* \param  enTargetGroup :          Target group of port pins for which drive
*                                  strength trim shall be changed
* \param  u8DriveStrengthTrimVal : Value for drive strength trim bitfield
*
*******************************************************************************/
static void SetDriveStrengthTrimForPortPins(en_portpin_group_t enTargetGroup, uint8_t u8DriveStrengthTrimVal)
{
    for(uint16_t u16Index = 0; u16Index < CY_GET_ARRAY_LEN(m_astcPortPinCfg); u16Index++)
    {
        const stc_portpin_cfg_t * pstcCurPortPin = &m_astcPortPinCfg[u16Index];

        if(pstcCurPortPin->enGroup == enTargetGroup)
        {
            Cy_GPIO_SetDriveSelTrim(pstcCurPortPin->pstcPort, pstcCurPortPin->u8Pin, u8DriveStrengthTrimVal);
        }
    }
}


/*******************************************************************************
* Function Name: ChangeSmifPllClock
****************************************************************************//**
*
* \brief  Changes the current SMIF PLL clock frequency
*
* \param  u16SmifPllClkMhz : SMIF PLL frequency, the resulting external frequency
*                            will then depend on current
*                            - HF clock divider (should be 1)
*                            - SMIF DLL divider
*******************************************************************************/
static cy_gfxenv_en_result_t ChangeSmifPllClock(uint16_t u16SmifPllClkMhz)
{
#if (CY_USE_PSVP == 0)
    uint32_t smifPllInputFreq;
    cy_en_sysclk_status_t enSysClkRetVal;

    enSysClkRetVal = Cy_SysClk_Pll400MDisable(CLK_PATH_SMIF_CLK);
    CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
    enSysClkRetVal = Cy_SysClk_GetPllInputFrequency(CLK_PATH_SMIF_CLK - 1, &smifPllInputFreq);
    CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);

    cy_stc_pll_400M_config_t stcPllCfg =
    {
        .inputFreq    = smifPllInputFreq,
        .outputFreq   = u16SmifPllClkMhz * 1000000UL,
        .outputMode   = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
        .fracEn       = false,
        .fracDitherEn = false,
        .sscgEn       = false,
        .sscgDitherEn = false,
        .sscgDepth    = CY_SYSCLK_SSCG_DEPTH_MINUS_2_0,
        .sscgRate     = CY_SYSCLK_SSCG_RATE_DIV_512,
    };
    enSysClkRetVal = Cy_SysClk_Pll400MConfigure(CLK_PATH_SMIF_CLK, &stcPllCfg);
    CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
    enSysClkRetVal = Cy_SysClk_Pll400MEnable(CLK_PATH_SMIF_CLK, 10000);
    CY_ASSERT(enSysClkRetVal == CY_SYSCLK_SUCCESS);
    (void) enSysClkRetVal;
#endif

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: InitExtMem
****************************************************************************//**
*
* \brief  Initializes all SMIF core instances and connected memories that shall be
*         configured according to user configuration.
*
* \param  pstcExtMemCfg : Pointer to to ExtMem config struct member of GFX Env
*                         config struct
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t InitExtMem(const cy_gfxenv_stc_init_extmem_t * pstcExtMemCfg)
{
    cy_gfxenv_en_result_t enRetVal = CY_GFXENV_SUCCESS;
    uint16_t u16SmifPllFreqMhz;
    uint8_t u8Smif;
    uint8_t u8MaxClkMhz = 0;

    //------------------------------------------------------------------------------------------------------------------------
    // Check clock constraints

    // 1) Find highest SMIF clock
    for(u8Smif = 0; u8Smif < CY_GFXENV_NR_OF_SMIF; u8Smif++)
    {
        if((pstcExtMemCfg->apstcInitSmif[u8Smif] != NULL) && (pstcExtMemCfg->apstcInitSmif[u8Smif]->u8SmifExtClockMhz > u8MaxClkMhz))
        {
            u8MaxClkMhz = pstcExtMemCfg->apstcInitSmif[u8Smif]->u8SmifExtClockMhz;
        }
    }

    for(u8Smif = 0; u8Smif < CY_GFXENV_NR_OF_SMIF; u8Smif++)
    {
        if(pstcExtMemCfg->apstcInitSmif[u8Smif] != NULL)
        {
            uint8_t u8SmifExtClockMhz = pstcExtMemCfg->apstcInitSmif[u8Smif]->u8SmifExtClockMhz;

            // 2) Check if all SMIF clocks can be derived from highest SMIF clock by applying a divider 1, 2, 4 or 8

            if((u8MaxClkMhz % u8SmifExtClockMhz) != 0)
            {
                return CY_GFXENV_ERROR_BAD_PARAM;
            }

            uint8_t u8ClkRatio = u8MaxClkMhz / u8SmifExtClockMhz;

            switch(u8ClkRatio)
            {
                case 1:
                case 2:
                case 4:
                case 8:
                    break;
                default:
                    return CY_GFXENV_ERROR_BAD_PARAM;
            }

            // 3) Check if the SMIF clock is inside the supported range and store required parameters for that range for later application

            m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider = 2; // Default value, may be updated below depending on actual frequency

            if((u8SmifExtClockMhz > 185) || (u8SmifExtClockMhz < 10)) // DLL lower limit is 160 MHz and max divider in SMIF is 16
            {
                return CY_GFXENV_ERROR_BAD_PARAM;
            }
            else if(u8SmifExtClockMhz > 166) // [185; 166[
            {
                m_astcSmifIntCfg[u8Smif].enMdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI400;
                m_astcSmifIntCfg[u8Smif].enSdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_SDL_TAP_XSPI400;
                m_astcSmifIntCfg[u8Smif].u8DriveStrengthTrimVal = CY_GPIO_DRIVE_STRENGTH_20OHM;
            }
            else if(u8SmifExtClockMhz > 133) // [166; 133[
            {
                m_astcSmifIntCfg[u8Smif].enMdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI333;
                m_astcSmifIntCfg[u8Smif].enSdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_SDL_TAP_XSPI333;
                m_astcSmifIntCfg[u8Smif].u8DriveStrengthTrimVal = CY_GPIO_DRIVE_STRENGTH_50OHM;
            }
            else if(u8SmifExtClockMhz > 100) // [133; 100[
            {
                m_astcSmifIntCfg[u8Smif].enMdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI266;
                m_astcSmifIntCfg[u8Smif].enSdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_SDL_TAP_XSPI266;
                m_astcSmifIntCfg[u8Smif].u8DriveStrengthTrimVal = CY_GPIO_DRIVE_STRENGTH_50OHM;
            }
            else                             // [100; 10]
            {
                m_astcSmifIntCfg[u8Smif].enMdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_MDL_TAP_XSPI200;
                m_astcSmifIntCfg[u8Smif].enSdlTap = (cy_en_cy_smif_ddl_tap_sel_t)GFXENV_SMIF_SDL_TAP_XSPI200;
                m_astcSmifIntCfg[u8Smif].u8DriveStrengthTrimVal = CY_GPIO_DRIVE_STRENGTH_50OHM;

                // Depending on the frequency two divider values might be possible to meet the DLL range of [160-400] MHz
                // The actual divider to be used depends on the clock ratio to the overall max clock across all SMIF cores to be configured
                if(u8SmifExtClockMhz >= 80)      // [100; 80]
                {
                    m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider = (u8ClkRatio > 1) ? 4 : 2;
                }
                else if(u8SmifExtClockMhz >= 40) // ]80; 40]
                {
                    m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider = (u8ClkRatio > 2) ? 8 : 4;
                }
                else if(u8SmifExtClockMhz >= 20) // ]40; 20]
                {
                    m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider = (u8ClkRatio > 4) ? 16 : 8;
                }
                else                              // ]20; 10]
                {
                    m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider = 16;
                }                
            }

            // Store value for PLL setup in next step. It is ok if it is overwritten by consecutive loop iterations because SMIF PLL frequencies need to match
            u16SmifPllFreqMhz = u8SmifExtClockMhz * m_astcSmifIntCfg[u8Smif].u8SmifDllOutputDivider;
        }
    }

    ChangeSmifPllClock(u16SmifPllFreqMhz);

    //------------------------------------------------------------------------------------------------------------------------
    // Initialization of all enabled SMIF cores

    for(u8Smif = 0; u8Smif < CY_GFXENV_NR_OF_SMIF; u8Smif++)
    {
        if(pstcExtMemCfg->apstcInitSmif[u8Smif] != NULL)
        {
            // Now that target frequency is known, the correct drive strength trim can be applied
            SetDriveStrengthTrimForPortPins((u8Smif == 0) ? Smif0 : Smif1, m_astcSmifIntCfg[u8Smif].u8DriveStrengthTrimVal);

            // Remember current SMIF core number and register base pointer
            m_astcSmifIntCfg[u8Smif].u8CoreNumber = u8Smif;
            m_astcSmifIntCfg[u8Smif].pstcSmifRegBase = (volatile cy_stc_smif_reg_t *) ((uint32_t) SMIF0_CORE0 + (u8Smif * SMIF_CORE_SECTION_SIZE));

            enRetVal = InitSmifCoreGeneric(&m_astcSmifIntCfg[u8Smif], pstcExtMemCfg->apstcInitSmif[u8Smif]);
            if(enRetVal != CY_GFXENV_SUCCESS)
            {
                return enRetVal;
            }
        }
    }

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: InitSmifCore
****************************************************************************//**
*
* \brief  Does some generic parameter setup for a SMIF core before calling the 
*         memory specific SMIF core init function
*
* \param  pstcSmifIntCfg : Pointer to internal SMIF config
* \param  pstcSmifUsrCfg : Pointer to user SMIF config
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t InitSmifCoreGeneric(stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg)
{
    cy_gfxenv_en_result_t enRetVal = CY_GFXENV_SUCCESS;

    CY_ASSERT(CY_GFXENV_NR_OF_DEVICES_PER_SMIF == 2);

    if((pstcSmifUsrCfg->apstcDevUsrCfg[0] != NULL) && (pstcSmifUsrCfg->apstcDevUsrCfg[1] != NULL))
    {
        // Currently only the setup of one DEVICE per SMIF_CORE is supported!
        return CY_GFXENV_ERROR_BAD_PARAM;
    }

    uint8_t u8DeviceNumber = (pstcSmifUsrCfg->apstcDevUsrCfg[0] != NULL) ? 0 : 1;

    // Remember the device number the memory is connected to, the pointer to the Device User config and the device register base pointers
    pstcSmifIntCfg->u8DeviceNumber    = u8DeviceNumber;
    pstcSmifIntCfg->pstcSmifDevUsrCfg = pstcSmifUsrCfg->apstcDevUsrCfg[u8DeviceNumber];    
    pstcSmifIntCfg->pstcDevRegBase    = Cy_SMIF_GetDeviceBasePtrFromSmifBasePtrAndDeviceNumber(pstcSmifIntCfg->pstcSmifRegBase, u8DeviceNumber);

    switch(pstcSmifIntCfg->pstcSmifDevUsrCfg->enExtMemType)
    {
        case CY_GFXENV_EXTMEM_TYPE_S27KXXXX2:
            enRetVal = InitSmifCoreAndDeviceForS27K(pstcSmifIntCfg, pstcSmifUsrCfg);
            break;
        case CY_GFXENV_EXTMEM_TYPE_S28H:
            enRetVal = InitSmifCoreAndDeviceForS28H(pstcSmifIntCfg, pstcSmifUsrCfg);
            break;
        default:
            return CY_GFXENV_ERROR_BAD_PARAM;
    }

    return enRetVal;
}


/*******************************************************************************
* Function Name: InitSmifCoreAndDeviceForS27K
****************************************************************************//**
*
* \brief  Initializes a SMIF core and device instance for accessing an S27K memory,
*         then configures the I/O registers in the memory before finalizing the SMIF
*         core and device initialization.
*
* \param  pstcSmifIntCfg : Pointer to internal SMIF config
* \param  pstcSmifUsrCfg : Pointer to user SMIF config
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t InitSmifCoreAndDeviceForS27K(stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg)
{
    cy_en_smif_hb_latency_code_t enOptimalLatency;
    cy_gfxenv_en_result_t        enRetVal = CY_GFXENV_SUCCESS;

    m_stcSmifDllCfgS27K.pllFreq   = pstcSmifUsrCfg->u8SmifExtClockMhz * pstcSmifIntCfg->u8SmifDllOutputDivider * 1000000UL;
    m_stcSmifDllCfgS27K.mdlOutDiv = GFXENV_SMIF_GET_DLL_ENUM_FROM_DIVIDER(pstcSmifIntCfg->u8SmifDllOutputDivider);

    pstcSmifIntCfg->stcSmifCtx = (cy_stc_smif_context_t) { 0 };
    if(Cy_SMIF_Init(pstcSmifIntCfg->pstcSmifRegBase, &m_stcSmifCfgS27K, GFXENV_SMIF_TIMEOUT, &pstcSmifIntCfg->stcSmifCtx) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR;
    }

    Cy_SMIF_Enable(pstcSmifIntCfg->pstcSmifRegBase, &pstcSmifIntCfg->stcSmifCtx);
    while(Cy_SMIF_IsDllLocked(pstcSmifIntCfg->pstcSmifRegBase) == false)
    {        
    }
    Cy_SMIF_ClearInterrupt(pstcSmifIntCfg->pstcSmifRegBase, SMIF_CORE_INTR_DLL_UNLOCK_Msk); // Ensure DLL Unlock flag is cleared (for diagnostic purposes later)

    // Cache and prefetching for the clk_slow port (CM0+, DMA, Crypto, ...) should be disabled during initialization (e.g. due to I/O reg access or command sequences)
    Cy_SMIF_CacheDisable   (pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);
    Cy_SMIF_CacheInvalidate(pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);

    // Capture Setup
    Cy_SMIF_DeviceSetRxCaptureDdr(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_RX_CAP_STYLE_DDR_HYPERBUS);
    Cy_SMIF_Set_DelayTapSel(pstcSmifIntCfg->pstcDevRegBase, pstcSmifIntCfg->enSdlTap);

    // The optimal access latency is frequency and device dependent, so get the optimal latency for the current device
    if(Cy_SMIF_HB_GetOptimalLatencyCode(pstcSmifUsrCfg->u8SmifExtClockMhz, CY_SMIF_HB_MEMTYPE_HYPERRAM_S27KXXXX2, &enOptimalLatency) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR_BAD_PARAM;
    }

    // Convert size to register bit field value (refer register TRM for definition of the SizeMask field)
    uint32_t u32SizeBytes = pstcSmifIntCfg->pstcSmifDevUsrCfg->u16SizeMegaBytes * 1024UL * 1024UL;
    uint32_t u32SizeMask = ~(u32SizeBytes - 1UL);

    // Check that region size is aligned to its base address
    if((pstcSmifIntCfg->pstcSmifDevUsrCfg->u32BaseAddress % u32SizeBytes) != 0)
    {
        return CY_GFXENV_ERROR_BAD_PARAM;
    }

    // Update the configuration struct with settings based on or derived from user configuration
    m_stcSmifDevCfgS27K.addr = pstcSmifIntCfg->pstcSmifDevUsrCfg->u32BaseAddress;
    m_stcSmifDevCfgS27K.size = (cy_en_device_size_t) u32SizeMask;

    uint32_t u32ClkMemFreqHz;        
    Cy_SysClk_GetClkMemFrequency(&u32ClkMemFreqHz);
    uint32_t u32TimeoutCycles = Cy_SMIF_HB_CalculateTotalTimeoutCyclesForHyperRAM(GFXENV_S27K_MAX_CS_LOW_NS, u32ClkMemFreqHz / 1000000UL, pstcSmifUsrCfg->u8SmifExtClockMhz);
    // Update the configuration struct with specific settings
    m_stcSmifDevCfgS27K.totalTimeout = u32TimeoutCycles;

    // Initial SMIF device (e.g. setup XIP read/write sequences, and other XIP related settings)
    if(Cy_SMIF_InitDeviceHyperBus(pstcSmifIntCfg->pstcDevRegBase, &m_stcSmifDevCfgS27K) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR;
    }

    // Ensure merging is disabled for now, because it can interfere with flash command sequences like writing to consecutive addresses in the write buffer. Can be re-enabled post-init.
    Cy_SMIF_DeviceTransferMergingDisable(pstcSmifIntCfg->pstcDevRegBase);

    // Access and configure the I/O registers in the actual memory device (e.g. optimize access latency, enable differential clock, ...)
    enRetVal = ConfigureIoRegsInS27KXXXX2(pstcSmifIntCfg, enOptimalLatency);
    if(enRetVal != CY_GFXENV_SUCCESS)
    {
        return enRetVal;
    }

    // Latency within SMIF register was already updated within ConfigureIoRegsInS27KXXXX2, also update the value in the struct, just to be consistent....
    m_stcSmifDevCfgS27K.lc_hb = enOptimalLatency;

    // As the last step, enable transfer merging if it had been enabled in the config for this device
    // (Note: merging was explicitly disabled during initialization to avoid potential problem with flash command sequences)
    if(m_stcSmifDevCfgS27K.mergeEnable == true)
    {
        Cy_SMIF_DeviceTransferMergingEnable(pstcSmifIntCfg->pstcDevRegBase, m_stcSmifDevCfgS27K.mergeTimeout);
    }

    // Enable cache and prefetching for the clk_slow port (CM0+, DMA, Crypto, ...)
    Cy_SMIF_CacheEnable           (pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);
    Cy_SMIF_CachePrefetchingEnable(pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: InitSmifCoreAndDeviceForS28H
****************************************************************************//**
*
* \brief  Initializes a SMIF core and device instance for accessing an S28H memory,
*         then configures the I/O registers in the memory before finalizing the SMIF
*         core and device initialization.
*
* \param  pstcSmifIntCfg : Pointer to internal SMIF config
* \param  pstcSmifUsrCfg : Pointer to user SMIF config
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t InitSmifCoreAndDeviceForS28H(stc_smif_cfg_internal_t * pstcSmifIntCfg, const cy_gfxenv_stc_smif_cfg_t * pstcSmifUsrCfg)
{
    cy_gfxenv_en_result_t enRetVal;

    // Reset S28H by toggling CS and DQ0 in a certain way as described in the datasheet.
    // Init code assumes a certain default register state after reset, refer ConfigureIoRegsInS28H() for details
    ResetS28HViaCsSignalingProtocol(pstcSmifIntCfg->u8CoreNumber, pstcSmifIntCfg->u8DeviceNumber);
    for(uint32_t u32Wait = 0; u32Wait < 100000; u32Wait++) // Some delay to ensure that device is out of reset (~600µs max)
    {        
    }

    m_stcSmifDllCfgS28H.pllFreq   = pstcSmifUsrCfg->u8SmifExtClockMhz * pstcSmifIntCfg->u8SmifDllOutputDivider * 1000000UL;
    m_stcSmifDllCfgS28H.mdlOutDiv = GFXENV_SMIF_GET_DLL_ENUM_FROM_DIVIDER(pstcSmifIntCfg->u8SmifDllOutputDivider);

    // Initialize context structs
    pstcSmifIntCfg->stcSmifCtx = (cy_stc_smif_context_t) { 0 };
    Cy_SMIF_InitExMemContext(CY_SMIF_SEMP, &pstcSmifIntCfg->stcExDevCtx);

    if(Cy_SMIF_Init(pstcSmifIntCfg->pstcSmifRegBase, &m_stcSmifCfgS28H, GFXENV_SMIF_TIMEOUT, &pstcSmifIntCfg->stcSmifCtx) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR;
    }

    Cy_SMIF_Enable(pstcSmifIntCfg->pstcSmifRegBase, &pstcSmifIntCfg->stcSmifCtx);
    while(Cy_SMIF_IsDllLocked(pstcSmifIntCfg->pstcSmifRegBase) == false)
    {        
    }
    Cy_SMIF_ClearInterrupt(pstcSmifIntCfg->pstcSmifRegBase, SMIF_CORE_INTR_DLL_UNLOCK_Msk); // Ensure DLL Unlock flag is cleared (for diagnostic purposes later)
    
    // Cache and prefetching for the clk_slow port (CM0+, DMA, Crypto, ...) should be disabled during initialization (e.g. due to I/O reg access or command sequences)
    Cy_SMIF_CacheDisable   (pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);
    Cy_SMIF_CacheInvalidate(pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);

    // Some SMIF device related settings, necessary before making our first accesses to S28H
    Cy_SMIF_DeviceSetRxCaptureSdr(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_RX_CAP_STYLE_SDR_POS);
    Cy_SMIF_DeviceSetRxCaptureDdr(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_RX_CAP_STYLE_DDR_OCTAL);
    Cy_SMIF_Set_DelayTapSel(pstcSmifIntCfg->pstcDevRegBase, 15); // Use late tap for capturing in SPI mode (based on own clock)
    
    // Optimal latency depends on frequency and device type (HS, HL). Code is hardcoded for HS support only and this is ensured by checking the IDs in ConfigureIoRegsInS28H()
    cy_en_smif_semp_read_latency_code_t enOptimalLatency;
    enRetVal = GetOptimalMemoryLatencyForS28H(true, CY_SPI_TRANSACTION_8D8D8D, pstcSmifUsrCfg->u8SmifExtClockMhz, &enOptimalLatency);
    if(enRetVal != CY_GFXENV_SUCCESS)
    {
        return enRetVal;
    }

    m_stcSmifDevCfgS28H.rdLatencyCode = enOptimalLatency;

    // Access and configure the I/O registers in the actual memory device (e.g. optimize access latency, change interface mode...)
    enRetVal = ConfigureIoRegsInS28H(pstcSmifIntCfg, enOptimalLatency);
    if(enRetVal != CY_GFXENV_SUCCESS)
    {
        return enRetVal;
    }

    //====================================================================================================================
    // Re-init SMIF in RWDS capture mode and then enable XIP mode


    while(Cy_SMIF_IsBusy(pstcSmifIntCfg->pstcSmifRegBase));
    Cy_SMIF_Disable(pstcSmifIntCfg->pstcSmifRegBase);

    // Change capture mode to capture by data strobe and re-init SMIF to apply the change
    m_stcSmifDllCfgS28H.rxCapMode = CY_SMIF_CAP_MODE_RWDS;

    if(Cy_SMIF_Init(pstcSmifIntCfg->pstcSmifRegBase, &m_stcSmifCfgS28H, GFXENV_SMIF_TIMEOUT, &pstcSmifIntCfg->stcSmifCtx) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR;
    }

    Cy_SMIF_Enable(pstcSmifIntCfg->pstcSmifRegBase, &pstcSmifIntCfg->stcSmifCtx);    
    while(Cy_SMIF_IsDllLocked(pstcSmifIntCfg->pstcSmifRegBase) == false)
    {        
    }
    Cy_SMIF_ClearInterrupt(pstcSmifIntCfg->pstcSmifRegBase, SMIF_CORE_INTR_DLL_UNLOCK_Msk); // Ensure DLL Unlock flag is cleared (for diagnostic purposes later)

    // Now change to the proper delay tap for capturing data by Data Strobe signal
    Cy_SMIF_Set_DelayTapSel(pstcSmifIntCfg->pstcDevRegBase, pstcSmifIntCfg->enSdlTap);

    // Convert size to register bit field value (refer register TRM for definition of the SizeMask field)
    uint32_t u32SizeBytes = pstcSmifIntCfg->pstcSmifDevUsrCfg->u16SizeMegaBytes * 1024UL * 1024UL;
    uint32_t u32SizeMask = ~(u32SizeBytes - 1UL);

    // Check that region size is aligned to its base address
    if((pstcSmifIntCfg->pstcSmifDevUsrCfg->u32BaseAddress % u32SizeBytes) != 0)
    {
        return CY_GFXENV_ERROR_BAD_PARAM;
    }

    // Update the configuration struct with settings based on or derived from user configuration
    m_stcSmifDevCfgS28H.addr = pstcSmifIntCfg->pstcSmifDevUsrCfg->u32BaseAddress;
    m_stcSmifDevCfgS28H.size = (cy_en_device_size_t) u32SizeMask;

    // Setup XIP command sequences
    if(Cy_SMIF_InitXIPModeForExMem(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_DEV_OSPI, &m_stcSmifDevCfgS28H, &pstcSmifIntCfg->stcExDevCtx) != CY_SMIF_SUCCESS)
    {
        return CY_GFXENV_ERROR;
    }

    cy_en_smif_status_t status;
    do
    {
        status = Cy_SMIF_SetMode(pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_MEMORY);
    } while(status != CY_SMIF_SUCCESS);

    // Enable cache and prefetching for the clk_slow port (CM0+, DMA, Crypto, ...)
    Cy_SMIF_CacheEnable           (pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);
    Cy_SMIF_CachePrefetchingEnable(pstcSmifIntCfg->pstcSmifRegBase, CY_SMIF_CACHE_SLOW);

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: ConfigureIoRegsInS27KXXXX2
****************************************************************************//**
*
* \brief  Configures the I/O registers in a connected S27K device
*
* \param  pstcSmifIntCfg : Pointer to internal SMIF config
* \param  enLatency      : Optimal latency as per the target frequency
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t ConfigureIoRegsInS27KXXXX2(stc_smif_cfg_internal_t * pstcSmifIntCfg, cy_en_smif_hb_latency_code_t enLatency)
{
    cy_un_h_ram_cfg0_reg_t unHyperRamVolatileCfgReg0;
    cy_un_h_ram_cfg0_reg_t unHyperRamVolatileCfgReg0ReadBack;
    cy_un_h_ram_cfg1_reg_t unHyperRamVolatileCfgReg1;
    cy_un_h_ram_cfg1_reg_t unHyperRamVolatileCfgReg1ReadBack;
    uint16_t               u16CfgReg1Mask;

    u16CfgReg1Mask                = CY_SMIF_CFG_REG1_MASK_WRITABLE_BITS_S27KXXXX2;
    unHyperRamVolatileCfgReg0.u16 = CY_SMIF_CFG_REG0_DEFAULT_S27KXXXX2;
    unHyperRamVolatileCfgReg1.u16 = CY_SMIF_CFG_REG1_DEFAULT_S27KXXXX2;

  #if (CY_USE_PSVP == 0) // PSVP does not support differential clock
    unHyperRamVolatileCfgReg1.fldUnion.s27kxxxx2.masterClkSel   = CY_SMIF_H_RAM_MASTER_CLK_DIFFERENTIAL; // Default: CY_SMIF_H_RAM_MASTER_CLK_SINGLE_ENDED
  #endif
    // --- Following settings are default, uncomment and change value if required ---
    // unHyperRamVolatileCfgReg1.fldUnion.s27kxxxx2.partialRefresh = CY_SMIF_H_RAM_REFRESH_ARRAY_FULL;
    // unHyperRamVolatileCfgReg1.fldUnion.s27kxxxx2.hybridSleep    = CY_SMIF_H_RAM_HYBRID_SLEEP_DISABLED;


    unHyperRamVolatileCfgReg0.fld.readLatency = enLatency;    
    // unHyperRamVolatileCfgReg0.fld.varLatency    = CY_SMIF_H_RAM_VAR_LATENCY; // Default: CY_SMIF_H_RAM_FIXED_LATENCY // TODO/FIX: Calibration may fail if variable latency is enabled

    // --- Following settings are default, uncomment and change value if required ---
    // unHyperRamVolatileCfgReg0.fld.driveStrength = CY_SMIF_H_RAM_DRV_34_OHMS;
    // unHyperRamVolatileCfgReg0.fld.burstLength   = CY_SMIF_H_RAM_BURST_LEN_32;
    // unHyperRamVolatileCfgReg0.fld.hyBurstEn     = CY_SMIF_H_RAM_HY_BURST_DISABLE;
    // unHyperRamVolatileCfgReg0.fld.deepPowerDown = CY_SMIF_H_RAM_NORMAL_OP;

    // Write config registers
    CY_SMIF_WriteHYPERRAM_REG(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_CFG_REG0_ADDR_S27K, unHyperRamVolatileCfgReg0.u16);
    CY_SMIF_WriteHYPERRAM_REG(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_CFG_REG1_ADDR_S27K, unHyperRamVolatileCfgReg1.u16);

    // Update SMIF device with new latency settings (otherwise read communication with Hyper Device may fail now)
    Cy_SMIF_HB_SetDummyCycles(pstcSmifIntCfg->pstcDevRegBase, CY_SMIF_HB_SRAM, enLatency);

    // Readback registers and compare
    CY_SMIF_ReadHYPERRAM_REG(pstcSmifIntCfg->pstcDevRegBase,  CY_SMIF_CFG_REG0_ADDR_S27K, &unHyperRamVolatileCfgReg0ReadBack.u16);
    CY_SMIF_ReadHYPERRAM_REG(pstcSmifIntCfg->pstcDevRegBase,  CY_SMIF_CFG_REG1_ADDR_S27K, &unHyperRamVolatileCfgReg1ReadBack.u16);

    if(  (unHyperRamVolatileCfgReg0ReadBack.u16                   !=  unHyperRamVolatileCfgReg0.u16)                   ||
        ((unHyperRamVolatileCfgReg1ReadBack.u16 & u16CfgReg1Mask) != (unHyperRamVolatileCfgReg1.u16 & u16CfgReg1Mask))    )
    {
        return CY_GFXENV_ERROR_EXTMEM_REG_READBACK;
    }

    return CY_GFXENV_SUCCESS;
}


/*******************************************************************************
* Function Name: ConfigureIoRegsInS28H
****************************************************************************//**
*
* \brief  Configures the I/O registers in a connected S28H device
* 
* \warning Function will be left with S28H configured in Octal DDR interface mode!
*
* \param  pstcSmifIntCfg : Pointer to internal SMIF config
* \param  enLatency      : Optimal latency as per the target frequency
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t ConfigureIoRegsInS28H(stc_smif_cfg_internal_t * pstcSmifIntCfg, cy_en_smif_semp_read_latency_code_t enLatency)
{
    cy_en_smif_slave_select_t enSlave = (pstcSmifIntCfg->u8DeviceNumber == 0) ? CY_SMIF_SLAVE_SELECT_0 : CY_SMIF_SLAVE_SELECT_1;

    un_cy_smif_semp_CFR2_t unCfr2 = {0};
    un_cy_smif_semp_CFR3_t unCfr3 = {0};
    un_cy_smif_semp_CFR5_t unCfr5 = {0};
    un_cy_smif_semp_CFR2_t unCfr2ReadBack = {0};
    un_cy_smif_semp_CFR3_t unCfr3ReadBack = {0};
    un_cy_smif_semp_CFR5_t unCfr5ReadBack = {0};

    // S28H has been reset before this function which means that its non-volatile config register values have been loaded into the corresponding volatile registers
    // Following code assumes that at least the interface mode as defined by CFR5.OPI-IT and CFR5.SDRDDR have their default value as per the datasheet (Legacy SPI: 1S-1S-1S)
    // If that is not the cause, a more complex detection method would have to be used (refer S28H in SMIF examples folder)

    //==============================================================================
    // CFR2: Ensure 4-byte addressing and optimal memory latency (for 8D-8D-8D mode)
    //------------------------------------------------------------------------------
    unCfr2.field.u4MEMLAT = enLatency;
    unCfr2.field.u1ADRBYT = CY_TRX_ADDR_4BYTE;
    // First try 3 byte addressing (if device is currently in 3 byte mode it will update the register, if it is in 4-byte mode the SPI transaction will end before the data is sent, so no chnage in device)
    Cy_SMIF_SEMP_WriteEnable(pstcSmifIntCfg->pstcSmifRegBase, enSlave, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_WriteAnyRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, unCfr2.u8, CY_SEMP_REG_ADDR_CFR2_V, CY_TRX_ADDR_3BYTE, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);

    // Now write again with 4-byte addressing (if device had originally been in 3 byte mode it was changed to 4 byte by previous write so we just write the same data again, if it was in 4-byte mode the register will be updated the first time)
    Cy_SMIF_SEMP_WriteEnable(pstcSmifIntCfg->pstcSmifRegBase, enSlave, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_WriteAnyRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, unCfr2.u8, CY_SEMP_REG_ADDR_CFR2_V, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);

    //======================================================================================================================================================
    // CFR3: Use maximum register read latency to work in all frequencies in 8D-8D-8D mode (for simplicity reasons we are not calculating the optimal value)
    //------------------------------------------------------------------------------------------------------------------------------------------------------
    unCfr3.field.u2VRGLAT = CY_SMIF_SEMP_REG_LATENCY3;
    Cy_SMIF_SEMP_WriteEnable(pstcSmifIntCfg->pstcSmifRegBase, enSlave, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_WriteAnyRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, unCfr3.u8, CY_SEMP_REG_ADDR_CFR3_V, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);

    //====================================================
    // CFR5: Change to Octal DDR interface mode (8D-8D-8D)
    //----------------------------------------------------
    unCfr5.field.u6RESRVD = 0x10; // keep default value of reserved bits
    unCfr5.field.u1SDRDDR = 1;
    unCfr5.field.u1OPI_IT = 1;
    Cy_SMIF_SEMP_WriteEnable(pstcSmifIntCfg->pstcSmifRegBase, enSlave, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_WriteAnyRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, unCfr5.u8, CY_SEMP_REG_ADDR_CFR5_V, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_1S1S1S, &pstcSmifIntCfg->stcSmifCtx);


    //====================================================================================
    // Check Device & Manufacturer ID and the crucial register values we had written above
    //------------------------------------------------------------------------------------
    const cy_stc_smif_semp_id_t stcExpectedDeviceManufacturerId = {0x34, 0x5B, 0x1A, 0x0F, 0x03, 0x90};
    cy_stc_smif_semp_id_t stcActualDeviceManufacturerId;
    Cy_SMIF_SEMP_ReadId(pstcSmifIntCfg->pstcSmifRegBase, enSlave, &stcActualDeviceManufacturerId, CY_SMIF_SEMP_REG_LATENCY3, CY_SPI_TRANSACTION_8D8D8D, sizeof(cy_stc_smif_semp_id_t), &pstcSmifIntCfg->stcSmifCtx);

    if(memcmp(&stcActualDeviceManufacturerId, &stcExpectedDeviceManufacturerId, sizeof(cy_stc_smif_semp_id_t)) != 0)
    {
        return CY_GFXENV_ERROR_EXTMEM_REG_READBACK;
    }

    Cy_SMIF_SEMP_ReadAnyVolatileRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, &unCfr2ReadBack.u8, CY_SEMP_REG_ADDR_CFR2_V, CY_SMIF_SEMP_REG_LATENCY3, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_8D8D8D, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_ReadAnyVolatileRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, &unCfr3ReadBack.u8, CY_SEMP_REG_ADDR_CFR3_V, CY_SMIF_SEMP_REG_LATENCY3, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_8D8D8D, &pstcSmifIntCfg->stcSmifCtx);
    Cy_SMIF_SEMP_ReadAnyVolatileRegister(pstcSmifIntCfg->pstcSmifRegBase, enSlave, &unCfr5ReadBack.u8, CY_SEMP_REG_ADDR_CFR5_V, CY_SMIF_SEMP_REG_LATENCY3, CY_TRX_ADDR_4BYTE, CY_SPI_TRANSACTION_8D8D8D, &pstcSmifIntCfg->stcSmifCtx);

    if((unCfr2ReadBack.field.u4MEMLAT != unCfr2.field.u4MEMLAT) ||
       (unCfr2ReadBack.field.u1ADRBYT != unCfr2.field.u1ADRBYT) ||
       (unCfr3ReadBack.field.u2VRGLAT != unCfr3.field.u2VRGLAT) ||
       (unCfr5ReadBack.field.u1SDRDDR != unCfr5.field.u1SDRDDR) ||
       (unCfr5ReadBack.field.u1OPI_IT != unCfr5.field.u1OPI_IT)    )
    {
        return CY_GFXENV_ERROR_EXTMEM_REG_READBACK;
    }

    return CY_GFXENV_SUCCESS;
}



/*******************************************************************************
* Function Name: GetOptimalMemoryLatencyForS28H
****************************************************************************//**
*
* \brief  Changes the HSIOM setting for the SMIF CLK_INV to the CLK_INV function
*
* \param  b1V8Variant       : Latency differs between 1.8V (S28HS) and 
*                             3.3V (S28HL) variant
* \param  enTransMode       : Transaction mode for which optimal latency cycles
*                             shall be calculated
* \param  u8SmifExtClockMhz : External SMIF frequency in MHz
* \param  penLatency        : Pointer where calculated latency will be stored,
*                             only valid if function returns CY_GFXENV_SUCCESS!
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t GetOptimalMemoryLatencyForS28H(bool b1V8Variant, cy_en_smif_trx_type_t enTransMode, uint8_t u8SmifExtClockMhz, cy_en_smif_semp_read_latency_code_t * penLatency)
{
    uint8_t au8FrequencyThresholdsOctalDdr[] = { 42, 57, 85, 107, 121, 135, 150, 164, 166 /*HL*/, 192, 200 };

    if(b1V8Variant)
    {
        au8FrequencyThresholdsOctalDdr[8] = 178; // HS
    }
    
    // Store maximum latency, in case caller does not evaluate return value...
    *penLatency = CY_SMIF_SEMP_RD_LATENCY15; 

    // Function currently only supports calculation for Octal DDR
    if(enTransMode != CY_SPI_TRANSACTION_8D8D8D)
    {        
        return CY_GFXENV_ERROR_BAD_PARAM;
    }

    for(uint8_t index = 0; index < CY_GET_ARRAY_LEN(au8FrequencyThresholdsOctalDdr); index++)
    {
        if(u8SmifExtClockMhz <= au8FrequencyThresholdsOctalDdr[index])
        {
            // Setting found, return the optimal latency code for the current frequency
            *penLatency = (cy_en_smif_semp_read_latency_code_t) index;
            return CY_GFXENV_SUCCESS;
        }
    }

    // Setting has not been found, return the maximum latency and an error
    return CY_GFXENV_ERROR_BAD_PARAM;
}



/*******************************************************************************
* Function Name: ResetS28HViaCsSignalingProtocol
****************************************************************************//**
*
* \brief  Resets an S28H through the CS reset signaling protocol
* 
* \warning After this function returns wait at least ~600µs before accessing the
*          device. (Refer datasheet for precise timings)
*
* \param  u8CoreNumber   : SMIF core number (0, 1)
* \param  u8DeviceNumber : Device number (0, 1) within #u8CoreNumber
*
* \return cy_gfxenv_en_result_t
*
*******************************************************************************/
static cy_gfxenv_en_result_t ResetS28HViaCsSignalingProtocol(uint8_t u8CoreNumber, uint8_t u8DeviceNumber)
{
    en_hsiom_sel_t csHsiomBackup;
    en_hsiom_sel_t dq0HsiomBackup;
    volatile stc_GPIO_PRT_t* csPort;
    volatile stc_GPIO_PRT_t* dq0Port;
    uint32_t csPin;
    uint32_t dq0Pin;

    switch(u8CoreNumber)
    {
        case 0:
            dq0Port = CY_SMIF0_DATA0_PORT;
            dq0Pin  = CY_SMIF0_DATA0_PIN;
            switch(u8DeviceNumber)
            {
                case 0:
                    csPort  = CY_SMIF0_SELECT0_PORT;
                    csPin   = CY_SMIF0_SELECT0_PIN;
                    break;
                case 1:
                    csPort  = CY_SMIF0_SELECT1_PORT;
                    csPin   = CY_SMIF0_SELECT1_PIN;
                    break;
                default:
                    return CY_GFXENV_ERROR_BAD_PARAM;                
            }
            break;
        case 1:
            dq0Port = CY_SMIF1_DATA0_PORT;
            dq0Pin  = CY_SMIF1_DATA0_PIN;
            switch(u8DeviceNumber)
            {
                case 0:
                    csPort  = CY_SMIF1_SELECT0_PORT;
                    csPin   = CY_SMIF1_SELECT0_PIN;
                    break;
                case 1:
                    csPort  = CY_SMIF1_SELECT1_PORT;
                    csPin   = CY_SMIF1_SELECT1_PIN;
                    break;
                default:
                    return CY_GFXENV_ERROR_BAD_PARAM;                
            }
            break;
        default:
            return CY_GFXENV_ERROR_BAD_PARAM;
    }

    // Backup current HSIO Mux state before changing to GPIO mode
    csHsiomBackup  = Cy_GPIO_GetHSIOM(csPort,  csPin);
    dq0HsiomBackup = Cy_GPIO_GetHSIOM(dq0Port, dq0Pin);
    Cy_GPIO_SetHSIOM(csPort,  csPin,  HSIOM_SEL_GPIO);
    Cy_GPIO_SetHSIOM(dq0Port, dq0Pin, HSIOM_SEL_GPIO);

    // Initial state (CS high, DQ0 low)
    Cy_GPIO_Set(csPort,  csPin);
    Cy_GPIO_Clr(dq0Port, dq0Pin);

    for(uint8_t u8Count = 0; u8Count < 4; u8Count++)
    {
        // Toggle CS
        Cy_GPIO_Inv(csPort,  csPin); // CS falling
        for(uint8_t u8Delay = 0; u8Delay < 100; u8Delay++); // should achieve >500ns confirm by measurement or change code to use proper time source
        Cy_GPIO_Inv(csPort,  csPin); // CS rising
        for(uint8_t u8Delay = 0; u8Delay < 100; u8Delay++); // should achieve >500ns confirm by measurement or change code to use proper time source
        // Toggle DQ0
        Cy_GPIO_Inv(dq0Port, dq0Pin); // Prepare new DQ0 value, 50ns setup and 50ns hold time with respect to rising CS are already ensured by above wait loops
    }

    // Restore original HSIO Mux state
    Cy_GPIO_SetHSIOM(csPort,  csPin,  csHsiomBackup);
    Cy_GPIO_SetHSIOM(dq0Port, dq0Pin, dq0HsiomBackup);

    return CY_GFXENV_SUCCESS;
}


#if defined(__cplusplus)
}
#endif


/* [] END OF FILE */
