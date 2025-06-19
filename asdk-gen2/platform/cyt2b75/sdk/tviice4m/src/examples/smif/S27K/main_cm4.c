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

#include "cy_project.h"
#include "cy_device_headers.h"
#include "SMIF_TestData.h"

#define SMIF_USED        CY_BSP_SMIF0_TYPE
#define SMIF_DEVICE_USED SMIF0_DEVICE1

#define SMIF_MPU_REG_NO    MPU_REG_SMIF0_DEV
#define SMIF_HF_CLOCK      CY_SYSCLK_HFCLK_5
#define PLL_200M_0_PATH_NO (3u)
#define PLL_200M_0_PLL_NO  (PLL_200M_0_PATH_NO-1u)
static void ChangePLLFrequency(uint32_t outputFreq)
{
#if (CY_USE_PSVP == 0u)
    uint32_t pllInputClockFreq = 0;
    CY_ASSERT(Cy_SysClk_GetPllInputFrequency(PLL_200M_0_PLL_NO, &pllInputClockFreq) == CY_SYSCLK_SUCCESS);

    cy_stc_pll_config_t pll200_1_Config = 
    {
        .inputFreq     = pllInputClockFreq,
        .outputFreq    = outputFreq,    // target PLL output
        .lfMode        = 0u,            // VCO frequency is [200MHz, 400MHz]
        .outputMode    = CY_SYSCLK_FLLPLL_OUTPUT_AUTO,
    };

    CY_ASSERT(Cy_SysClk_PllDisable(PLL_200M_0_PATH_NO) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllConfigure(PLL_200M_0_PATH_NO , &pll200_1_Config) == CY_SYSCLK_SUCCESS);
    CY_ASSERT(Cy_SysClk_PllEnable(PLL_200M_0_PATH_NO, 10000ul) == CY_SYSCLK_SUCCESS);
#endif
    return;
}

cy_stc_smif_config_t const smifConfig =
{
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_RWDS,
    .dlpAuto       = CY_SMIF_DLP_UPDATE_MANUAL,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .delaySel      = CY_SMIF_1_SEN_SEL_PER_TAP,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_3_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_1_CLK_PULUS_MIN,
    .mode          = CY_SMIF_MEMORY, // XIP mode
    .blockEvent    = CY_SMIF_BUS_ERROR,
};
cy_stc_smif_context_t smifContext;

#define HB_INITIAL_LC CY_SMIF_HB_LC4

cy_stc_device_hb_config_t smifDevHBSramCfg =
{
    .xipReadCmd         = CY_SMIF_HB_READ_CONTINUOUS_BURST,
    .xipWriteCmd        = CY_SMIF_HB_WRITE_CONTINUOUS_BURST,
    .mergeEnable        = false,
    .mergeTimeout       = CY_SMIF_MER_TIMEOUT_1_CYCLE,
    .totalTimeoutEnable = false,
    .totalTimeout       = 1000u,
    .addr               = 0,                       // will be updated in the application
    .size               = CY_SMIF_DEVICE_8M_BYTE,
    .lc_hb              = HB_INITIAL_LC,
    .hbDevType          = CY_SMIF_HB_SRAM,
};

typedef struct
{
    volatile stc_GPIO_PRT_t*  port;
    uint8_t                   pin;
    en_hsiom_sel_t            hsiom;
    uint32_t                  driveMode;
} cy_stc_smif_port_t;

cy_stc_smif_port_t smifPortCfg[] =
{
    {CY_BSP_SMIF0_CLK_PORT,     CY_BSP_SMIF0_CLK_PIN,     CY_BSP_SMIF0_CLK_PIN_MUX,     CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_RWDS_PORT,    CY_BSP_SMIF0_RWDS_PIN,    CY_BSP_SMIF0_RWDS_PIN_MUX,    CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_SELECT0_PORT, CY_BSP_SMIF0_SELECT0_PIN, CY_BSP_SMIF0_SELECT0_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_SELECT1_PORT, CY_BSP_SMIF0_SELECT1_PIN, CY_BSP_SMIF0_SELECT1_PIN_MUX, CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA0_PORT,   CY_BSP_SMIF0_DATA0_PIN,   CY_BSP_SMIF0_DATA0_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA1_PORT,   CY_BSP_SMIF0_DATA1_PIN,   CY_BSP_SMIF0_DATA1_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA2_PORT,   CY_BSP_SMIF0_DATA2_PIN,   CY_BSP_SMIF0_DATA2_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA3_PORT,   CY_BSP_SMIF0_DATA3_PIN,   CY_BSP_SMIF0_DATA3_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA4_PORT,   CY_BSP_SMIF0_DATA4_PIN,   CY_BSP_SMIF0_DATA4_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA5_PORT,   CY_BSP_SMIF0_DATA5_PIN,   CY_BSP_SMIF0_DATA5_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA6_PORT,   CY_BSP_SMIF0_DATA6_PIN,   CY_BSP_SMIF0_DATA6_PIN_MUX,   CY_GPIO_DM_STRONG},
    {CY_BSP_SMIF0_DATA7_PORT,   CY_BSP_SMIF0_DATA7_PIN,   CY_BSP_SMIF0_DATA7_PIN_MUX,   CY_GPIO_DM_STRONG},
};
#define SIZE_OF_PORT (sizeof(smifPortCfg)/sizeof(cy_stc_smif_port_t))

static void SmifPortInit(cy_stc_smif_port_t cfg[], uint8_t size)
{
    cy_stc_gpio_pin_config_t pinCfg = {0};
    for(uint32_t i = 0; i < size; i++)
    {
        pinCfg.driveMode = cfg[i].driveMode;
        pinCfg.hsiom     = cfg[i].hsiom;
        Cy_GPIO_Pin_Init(cfg[i].port, cfg[i].pin, &pinCfg);
    }
}

static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode);
bool SetSmifFrequencyWithDelayLineTapNumCalibrationForHRAM(uint32_t safeFreq, uint32_t freq, uint32_t usedAddr);
void CalibrateDelayLineSelect(uint32_t freq, bool isDDR);


int main(void)
{
    SystemInit();

    __enable_irq();

    Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
    ChangePLLFrequency(64000000); // SMIF out clock will be 32,000,000

    SmifPortInit(smifPortCfg ,SIZE_OF_PORT);

    /*************************************/
    /*       SMIF Initialization         */
    /*************************************/
    Cy_SMIF_Init(SMIF_USED, &smifConfig, 1000, &smifContext);
    Cy_SMIF_Set_DelayTapSel(SMIF_DEVICE_USED, (CY_SMIF_GetDelayTapsNumber(SMIF_USED) - 1));
    Cy_SMIF_Enable(SMIF_USED, &smifContext);
    Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH); // Only affects CM0+ side
    Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH); // this is required when checking status register of external device. Only affects CM0+ side

    volatile uint32_t* pHyperRamBaseAddr   = (uint32_t*)(CY_SMIF_GetXIP_Address(SMIF_USED) + 0x04000000ul);

    /*************************************/
    /*     SMIF DEVICE Initialization    */
    /*************************************/
    smifDevHBSramCfg.addr = (uint32_t)pHyperRamBaseAddr;
    Cy_SMIF_InitDeviceHyperBus(SMIF_DEVICE_USED, &smifDevHBSramCfg);

    /*************************************/
    /*      Hyper Bus Initialization     */
    /*************************************/
    /***** Set Configuration register 0 *****/
    // Load Configuration register 0
    cy_un_h_ram_cfg0_reg_t ramVCR  = { .u16 = CY_SMIF_CFG_REG0_DEFAULT_S27KXXXX1 };
    ramVCR.fld.readLatency         = smifDevHBSramCfg.lc_hb;
    CY_SMIF_WriteHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, ramVCR.u16);

    // Read Configuration register 0
    uint16_t readVCRValue0 = 0;
    CY_SMIF_ReadHYPERRAM_REG(SMIF_DEVICE_USED, CY_SMIF_CFG_REG0_ADDR_S27K, &readVCRValue0);

    // Verify Configuration register 0
    CY_ASSERT(readVCRValue0 == ramVCR.u16);

    /*************************************/
    /*  Calibration for Delay RWDS line  */
    /*************************************/

    /*********  Calibrate Delay Line Select   *******/
    // Please modify according to your HW condition.
    uint32_t targetSmifFreq = 100000000;
    uint32_t safeSmifFreq = 32000000;

    CalibrateDelayLineSelect(targetSmifFreq, true);
    SetSmifFrequencyWithDelayLineTapNumCalibrationForHRAM(safeSmifFreq, targetSmifFreq, 0x00000000);

    /*******************************************/
    /*       Hyper RAM Test In XIP mode        */
    /*******************************************/
    // From here on, We use XIP region as normal MEMORY to improve performance.

    /*********  SMIF cache enable   *******/
    Cy_SMIF_CacheEnable(SMIF_USED, CY_SMIF_CACHE_BOTH); // Only affects CM0+ side
    Cy_SMIF_CachePrefetchingEnable(SMIF_USED, CY_SMIF_CACHE_BOTH); // Only affects CM0+ side

    /*********  Marge mode enable   *******/
    Cy_SMIF_DeviceTransferMergingEnable(SMIF_DEVICE_USED,CY_SMIF_MER_TIMEOUT_256_CYCLE);
    while(Cy_SMIF_IsBusy(SMIF_USED));

    /*********  Write Data to Hyper RAM  *******/
    memcpy((void*)pHyperRamBaseAddr, (void*)programData, TEST_PROGRAM_SIZE);

    /*********  Read back and verify   *******/
    static uint8_t readBuf[TEST_PROGRAM_SIZE];
    memcpy((void*)readBuf, (void*)pHyperRamBaseAddr, TEST_PROGRAM_SIZE);

    // Verify data
    CY_ASSERT(memcmp((void*)readBuf, (void*)programData, TEST_PROGRAM_SIZE) == 0);

    /*********  Set 0 to Hyper RAM   *******/
    memset((void*)pHyperRamBaseAddr, 0, TEST_PROGRAM_SIZE);

    /********************************************/
    /*       Hyper RAM Test In MMIO mode        */
    /********************************************/
    /*********  SMIF cache disable   *******/
    Cy_SMIF_CacheInvalidate(SMIF_USED, CY_SMIF_CACHE_BOTH); // Only affects CM0+ side
    Cy_SMIF_CacheDisable(SMIF_USED, CY_SMIF_CACHE_BOTH); // this is required when checking status register of external device. Only affects CM0+ side

    SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

    /*********  Write Data to the Top Sector   *******/
    for(uint32_t i_WriteAddr = 0x00000000ul; i_WriteAddr < (TEST_PROGRAM_SIZE / 2);i_WriteAddr += 0x100)
    {
        Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                             CY_SMIF_SLAVE_SELECT_1,
                             CY_SMIF_HB_COUTINUOUS_BURST,
                             i_WriteAddr,             // address
                             0x100,                       // size in half word
                             (uint16_t*)&programData[i_WriteAddr*2],
                             CY_SMIF_HB_SRAM,
                             HB_INITIAL_LC,        // latency code, do not care
                             true,                    // Blocking mode
                             &smifContext
                             );
    }

    /*********  Read back and verify   *******/
    for(uint32_t i_WriteAddr = 0x00000000ul; i_WriteAddr < (TEST_PROGRAM_SIZE / 2);i_WriteAddr += 0x100)
    {
        uint16_t readBuffer[0x100];
        Cy_SMIF_HB_MMIO_Read(SMIF_USED,
                             CY_SMIF_SLAVE_SELECT_1,
                             CY_SMIF_HB_COUTINUOUS_BURST,
                             i_WriteAddr, // address
                             0x100,                      // size in half word
                             readBuffer,
                             HB_INITIAL_LC,
                             false,                      // Single initial latency
                             true,                       // Blocking mode
                             &smifContext
                             );
        CY_ASSERT(memcmp(readBuffer, &programData[i_WriteAddr*2], 0x200) == 0);
    }

    for(;;);
}

static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode)
{
    cy_en_smif_status_t status;
    do
    {
        status = Cy_SMIF_SetMode(base, mode);
    }while(status != CY_SMIF_SUCCESS);
}

bool SetSmifFrequencyWithDelayLineTapNumCalibrationForHRAM(uint32_t safeFreq, uint32_t freq, uint32_t usedAddr)
{
    uint8_t readTest[12]      = {0};
    uint8_t testData[12]      = {0x00, 0xFF, 0xFF, 0xFF, 0x00, 0xFF, 0x00, 0x00, 0x00, 0xFF, 0x55, 0xAA};
    bool    isMatch[32]       = {false};
    cy_en_smif_hb_latency_code_t lc;

    cy_en_smif_mode_t savedMode = Cy_SMIF_GetMode(SMIF_USED);

    SetModeWithBusyCheck(SMIF_USED, CY_SMIF_NORMAL);

    ChangePLLFrequency(safeFreq*2);

    lc = HB_INITIAL_LC;
    Cy_SMIF_HB_MMIO_Write(SMIF_USED,
                          CY_SMIF_SLAVE_SELECT_1,
                          CY_SMIF_HB_COUTINUOUS_BURST,
                          usedAddr,                // address
                          6,                       // size in half word
                          (uint16_t*)testData,
                          CY_SMIF_HB_SRAM,
                          lc,
                          true,                    // Blocking mode
                          &smifContext
                          );

    ChangePLLFrequency(freq*2);

    for(uint8_t i_tapNum = 0; i_tapNum < CY_SMIF_GetDelayTapsNumber(SMIF_USED); i_tapNum++)
    {
        while(Cy_SMIF_IsBusy(SMIF_USED));
        Cy_SMIF_Set_DelayTapSel(SMIF_DEVICE_USED, i_tapNum);
        Cy_SMIF_HB_MMIO_Read(SMIF_USED,
                         CY_SMIF_SLAVE_SELECT_1,
                         CY_SMIF_HB_COUTINUOUS_BURST,
                         usedAddr,                 // address
                         0x6,                      // size in half word
                         (uint16_t*)readTest,
                         lc,
                         false,                      // Single initial latency
                         true,                       // Blocking mode
                         &smifContext
                         );
        if(memcmp(&readTest, testData, 12) == 0)
        {
            isMatch[i_tapNum] = true;
        }
        else
        {
            isMatch[i_tapNum] = false;
        }
    }

    uint8_t bestTapNum = 0xFF;
    uint8_t consectiveMatchNum    = 0;
    uint8_t maxConsectiveMatchNum = 0;
    for(uint8_t i_tapNum = 0; i_tapNum < CY_SMIF_GetDelayTapsNumber(SMIF_USED); i_tapNum++)
    {
        if(isMatch[i_tapNum] == true)
        {
            consectiveMatchNum += 1;
        }
        else
        {
            if(maxConsectiveMatchNum < consectiveMatchNum)
            {
                maxConsectiveMatchNum = consectiveMatchNum;
                bestTapNum = i_tapNum - ((maxConsectiveMatchNum + 1) / 2);
                consectiveMatchNum = 0;
            }
        }
    }

    if(maxConsectiveMatchNum < consectiveMatchNum)
    {
        bestTapNum = CY_SMIF_GetDelayTapsNumber(SMIF_USED) - ((consectiveMatchNum + 1) / 2);
    }

    if(bestTapNum != 0xFF)
    {
        Cy_SMIF_Set_DelayTapSel(SMIF_DEVICE_USED, bestTapNum);
    }
    else
    {
        CY_ASSERT(false); // No right tap number
    }

    SetModeWithBusyCheck(SMIF_USED, savedMode);

    return true;
}


void CalibrateDelayLineSelect(uint32_t freq, bool isDDR)
{
    static const float minDelayPerTapInNs[4] = {0.0559, 0.1197, 0.1935, 0.3973};

    /*********  Calibrate Delay Line Select   *******/
    // disable delay line
    uint32_t tapLineChecked = 0;
    for(; tapLineChecked < 4; tapLineChecked++)
    {
        float maxTapNum;
        if(isDDR == true)
        {
            maxTapNum = (uint32_t)((1000000000.0 / (freq * 4 * minDelayPerTapInNs[tapLineChecked])) + 0.5);
        }
        else
        {
            maxTapNum = (uint32_t)((1000000000.0 / (freq * 2 * minDelayPerTapInNs[tapLineChecked])) + 0.5);
        }

        if(maxTapNum <= CY_SMIF_GetDelayTapsNumber(SMIF_USED))
        {
            break;
        }
    }

    if(tapLineChecked == 4)
    {
        tapLineChecked = 3;
    }

    while(Cy_SMIF_IsBusy(SMIF_USED));
    Cy_SMIF_Disable(SMIF_USED);
    Cy_SMIF_Set_DelayLineSel(SMIF_USED, (cy_en_smif_delay_line_t)tapLineChecked);
    Cy_SMIF_Enable(SMIF_USED, &smifContext);
}

/* [] END OF FILE */


