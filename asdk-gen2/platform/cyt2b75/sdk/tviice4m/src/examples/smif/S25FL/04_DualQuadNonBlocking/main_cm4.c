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
#include "SMIF_TestData.h"


/***** User Modifiable definitions *****/
#define DLP_VALUE                       (0x34u)
#define SMIF_CLK_FREQ_SETUP             (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_SDR_RD            (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_SDR_FST_RD        (50000000UL) // Please modify as per your HW
#define SMIF_CLK_FREQ_DDR_RD            (50000000UL) // Please modify as per your HW

/* Select the memory availabe on hardware to test */
#define CY_SMIF_USED                    SMIF0
#define CY_SMIF_QUAD_MEMORY             CY_SMIF_S25FXXXS

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
  #define TEST_SPI_READ_LC              CY_SMIF_S25FL_RD_LATENCY2
  #define DEVICE_SECTOR_ERASE           (0x00010000) // 64KB
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
  #define TEST_SPI_READ_LC              CY_SMIF_S25FL_RD_LATENCY8
  #define DEVICE_SECTOR_ERASE           (0x00001000) // 4KB
#endif
#define DEVICE_PROGRAM_BUFFER_SIZE (0x100)      // 256B
#define TEST_SPI_WRITE_LC               CY_SMIF_S25FL_WT_LATENCY0
#define TEST_PROGRAM_SECTOR             (0x01240000ul)
/**************************************/

#define TEST_DQUAD_XIP_PROGRAM_ADDR     (TEST_PROGRAM_SECTOR * 2ul)
#define DQUAD_PROGRAM_BUFFER_SIZE       (DEVICE_PROGRAM_BUFFER_SIZE * 2ul)
#define DQUAD_SECTOR_ERASE              (DEVICE_SECTOR_ERASE * 2ul)

#define D_QUAD_SLAVES (cy_en_smif_slave_select_t)((uint8_t)CY_SMIF_SLAVE_SELECT_0|(uint8_t)CY_SMIF_SLAVE_SELECT_1)

/**
    TEST_PROGRAM_SECTOR is the address recognized by External Quad Devices.
    TEST_DQUAD_XIP_PROGRAM_ADDR is the address recognized by TVII in XIP, Dual Quad mode.
**/

// #define DLP_SETTING_UPDATE_AUTO

#ifdef DLP_SETTING_UPDATE_AUTO
  #define SETTING_DELAY_SEL CY_SMIF_1_NEW_SEL_PER_TAP
  #define SETTING_DLP_AUTO  CY_SMIF_DLP_UPDATE_AUTO
#else
  #define SETTING_DELAY_SEL CY_SMIF_NO_DELAY_SEL
  #define SETTING_DLP_AUTO  CY_SMIF_DLP_UPDATE_MANUAL
#endif

#define USE_QUAD_MODE 1 // 0: not quad, 1: quad

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

cy_stc_smif_context_t smifContext;
uint8_t readDataBuffer[TEST_PROGRAM_SIZE] = {0};

cy_stc_smif_config_t smifConfig =
{
    .txClk         = CY_SMIF_INV_FOR_DDR,
    .rxClk         = CY_SMIF_INV_OUTPUT_CLK, // Note
    .delaySel      = SETTING_DELAY_SEL,
    .dlpAuto       = SETTING_DLP_AUTO,
    .capDelay      = CY_SMIF_CAPTURE_DELAY_0_CYCLE,
    .deselectDelay = CY_SMIF_MIN_DESELECT_1_CLK,
    .setupDelay    = CY_SMIF_SETUP_0_CLK_PULUS_MIN,
    .holdDelay     = CY_SMIF_HOLD_0_CLK_PULUS_MIN,
    .mode          = CY_SMIF_NORMAL, // MMIO mode
    .blockEvent    = CY_SMIF_BUS_ERROR,
};

cy_stc_sysint_irq_t smif_irq_cfg =
{
    .sysIntSrc  = smif_0_interrupt_IRQn,
    .intIdx     = CPUIntIdx3_IRQn,
    .isEnabled  = true,
};

static void TestFor_DQUAD_MMIO_WRITE_Then_MMIO_DDR_READ_NON_BLOCKING(uint16_t rd_cmdId);


static void SectorEraseProcedure(cy_en_smif_slave_select_t slave, uint32_t address);
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave);
static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode);


void SMIF_UserInterruptHandler(void)
{
    Cy_SMIF_Interrupt(CY_SMIF_USED, &smifContext);
}

cy_stc_ex_dev_context_t smifDeviceContext;


int main(void)
{
    SystemInit();

    __enable_irq();

    Cy_SysClk_HfClkEnable(SMIF_HF_CLOCK);
    //Cy_SysClk_HfClockSetDivider(SMIF_HF_CLOCK, CY_SYSCLK_HFCLK_DIVIDE_BY_4);

    // Please modify according to your HW condition.
    ChangePLLFrequency(SMIF_CLK_FREQ_SETUP*2);					// SMIF out clock will be 50,000,000

    SmifPortInit(smifPortCfg, SIZE_OF_PORT);

    CY_ASSERT(Cy_SMIF_InitExMemContext(CY_SMIF_S25FL, &smifDeviceContext) == CY_SMIF_SUCCESS);

    /* Interrupt settings */
    Cy_SysInt_InitIRQ(&smif_irq_cfg);
    Cy_SysInt_SetSystemIrqVector(smif_irq_cfg.sysIntSrc, SMIF_UserInterruptHandler);
    NVIC_SetPriority(smif_irq_cfg.intIdx, 0);
    NVIC_EnableIRQ(smif_irq_cfg.intIdx);

    /*************************/
    /* 1. Setting for SMIF 0 */
    /*************************/
    Cy_SMIF_DeInit(CY_SMIF_USED);
    Cy_SMIF_Init(CY_SMIF_USED, &smifConfig, 1000, &smifContext);
    Cy_SMIF_SetTxFifoTriggerLevel(CY_SMIF_USED, 7u);
    Cy_SMIF_SetRxFifoTriggerLevel(CY_SMIF_USED, 0u);

    /************************************/
    /* 2. Setting for SMIF 0 Device 0/1 */
    /************************************/
    Cy_SMIF_DeviceSetDataSelect(SMIF0_DEVICE0, CY_SMIF_DATA_SEL0);
    Cy_SMIF_DeviceWriteEnable(SMIF0_DEVICE0);
    Cy_SMIF_DeviceSetDataSelect(SMIF0_DEVICE1, CY_SMIF_DATA_SEL2);
    Cy_SMIF_DeviceWriteEnable(SMIF0_DEVICE1);

    /********************/
    /* 3. Enable SMIF 0 */
    /********************/
    Cy_SMIF_Enable(CY_SMIF_USED, &smifContext);

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
    /***********************************/
    /* 4. Setting for external devices */
    /***********************************/
    // 4.1 Send Rest command
    Cy_SMIF_S25FL_SoftwareReset(CY_SMIF_USED, D_QUAD_SLAVES, &smifContext);

    // wait at least 32us
    Cy_SysTick_DelayInUs(32ul);

    /*********************************************************/
    /********* Read Status and Configuration registers *******/
    /*********************************************************/
    // 4.2 Read Configuration register 1 and status register 1
    // Read current configuration register 1 and status register 1
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1 = { .u8 = 0u };

    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg1, &smifContext);
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevStatusReg0, &smifContext);
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevStatusReg1, &smifContext);

    /*********************************************************/
    /******* Modify Status and Configuration registers *******/
    /*********************************************************/
    // Modify QUAD and LC field to be set
    prevConfigReg0.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg0.field.u2LC   = TEST_SPI_READ_LC;
    prevConfigReg1.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1.field.u2LC   = TEST_SPI_READ_LC;
    prevStatusReg0.field.u3BP   = 0u; // no protection
    prevStatusReg1.field.u3BP   = 0u; // no protection

    /*********************************************************/
    /******* Write Status and Configuration registers ********/
    /*********************************************************/
    // Send Write Enable command
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, D_QUAD_SLAVES, &smifContext);

    // Write Registers
    stc_cy_smif_s25f_write_reg_t writeRegValue_0;
    writeRegValue_0.stsReg1 = prevStatusReg0.u8;
    writeRegValue_0.confgReg1 = prevConfigReg0.u8;

    stc_cy_smif_s25f_write_reg_t writeRegValue_1;
    writeRegValue_1.stsReg1 = prevStatusReg1.u8;
    writeRegValue_1.confgReg1 = prevConfigReg1.u8;

    // Write Registers
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &writeRegValue_0, 2, &smifContext);
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &writeRegValue_1, 2, &smifContext);

    // Wait writing registers complete
    CY_ASSERT_L1(WaitDeviceBecomesReady(D_QUAD_SLAVES) == 0u);

    /*********************************************************/
    /******* Verify Status and Configuration registers *******/
    /*********************************************************/
    // Check the config register have been writen properly
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg1, &smifContext);
    CY_ASSERT_L1(readConfigReg0.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg0.field.u2LC == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg1.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1.field.u2LC == TEST_SPI_READ_LC);
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
    // 4.1 Send Rest command
    // YOTS: Not implemented yet.

    /*********************************************************/
    /********* Read Status and Configuration registers *******/
    /*********************************************************/
    // 4.2 Read Configuration register 1 and status register 1
    // Read current configuration register 1 and status register 1
    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t prevConfigReg2_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t prevConfigReg3_0 = { .u8 = 0u };

    volatile un_cy_smif_s25fl128_SR1_t prevStatusReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t prevConfigReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t prevConfigReg2_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t prevConfigReg3_1 = { .u8 = 0u };

    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevStatusReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg2_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&prevConfigReg3_0, &smifContext);

    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevStatusReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg2_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&prevConfigReg3_1, &smifContext);

    /*********************************************************/
    /******* Modify Status and Configuration registers *******/
    /*********************************************************/
    prevStatusReg1_0.u8 = 0;
    prevConfigReg1_0.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1_0.field.u1CMP  = 0u;
    prevConfigReg2_0.field.u1QPI  = 0u; // Quad instruction disabled
    prevConfigReg3_0.field.u4RL   = TEST_SPI_READ_LC; // Latency code
    prevConfigReg3_0.field.u1WE   = 1u; // Wrap Disabled

    prevStatusReg1_1.u8 = 0;
    prevConfigReg1_1.field.u1QUAD = USE_QUAD_MODE;
    prevConfigReg1_1.field.u1CMP  = 0u;
    prevConfigReg2_1.field.u1QPI  = 0u; // Quad instruction disabled
    prevConfigReg3_1.field.u4RL   = TEST_SPI_READ_LC; // Latency code
    prevConfigReg3_1.field.u1WE   = 1u; // Wrap Disabled

    /*********************************************************/
    /******* Write Status and Configuration registers ********/
    /*********************************************************/
    // Send Write Enable command
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, D_QUAD_SLAVES, &smifContext);

    stc_cy_smif_s25f_write_reg_t writeRegValue_0;
    writeRegValue_0.stsReg1 = prevStatusReg1_0.u8;
    writeRegValue_0.confgReg1 = prevConfigReg1_0.u8;
    writeRegValue_0.confgReg2 = prevConfigReg2_0.u8;
    writeRegValue_0.confgReg3 = prevConfigReg3_0.u8;

    stc_cy_smif_s25f_write_reg_t writeRegValue_1;
    writeRegValue_1.stsReg1 = prevStatusReg1_1.u8;
    writeRegValue_1.confgReg1 = prevConfigReg1_1.u8;
    writeRegValue_1.confgReg2 = prevConfigReg2_1.u8;
    writeRegValue_1.confgReg3 = prevConfigReg3_1.u8;

    // Write Registers
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, &writeRegValue_0, 4, &smifContext);
    Cy_SMIF_S25FL_WriteStatusAndConfigRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, &writeRegValue_1, 4, &smifContext);

    // Wait writing registers complete
    CY_ASSERT_L1(WaitDeviceBecomesReady(D_QUAD_SLAVES) == 0u);

    /*********************************************************/
    /******* Verify Status and Configuration registers *******/
    /*********************************************************/
    // Check the config register have been writen properly
    volatile un_cy_smif_s25fl128_SR1_t readStatusReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t readConfigReg2_0 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t readConfigReg3_0 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readStatusReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg1_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg2_0, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_0, (uint8_t*)&readConfigReg3_0, &smifContext);

    CY_ASSERT_L1(readStatusReg1_0.u8 == 0u);
    CY_ASSERT_L1(readConfigReg1_0.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1_0.field.u1CMP == 0u);
    CY_ASSERT_L1(readConfigReg2_0.field.u1QPI == 0u);
    CY_ASSERT_L1(readConfigReg3_0.field.u4RL == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg3_0.field.u1WE == 1u);

    volatile un_cy_smif_s25fl128_SR1_t readStatusReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR1_t readConfigReg1_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR2_t readConfigReg2_1 = { .u8 = 0u };
    volatile un_cy_smif_s25fl128_CR3_t readConfigReg3_1 = { .u8 = 0u };
    Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readStatusReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg1_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister2(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg2_1, &smifContext);
    Cy_SMIF_S25FL_ReadConfigurationRegister3(CY_SMIF_USED, CY_SMIF_SLAVE_SELECT_1, (uint8_t*)&readConfigReg3_1, &smifContext);

    CY_ASSERT_L1(readStatusReg1_1.u8 == 0u);
    CY_ASSERT_L1(readConfigReg1_1.field.u1QUAD == USE_QUAD_MODE);
    CY_ASSERT_L1(readConfigReg1_1.field.u1CMP == 0u);
    CY_ASSERT_L1(readConfigReg2_1.field.u1QPI == 0u);
    CY_ASSERT_L1(readConfigReg3_1.field.u4RL == TEST_SPI_READ_LC);
    CY_ASSERT_L1(readConfigReg3_1.field.u1WE == 1u);
#endif

    // 4.4 Set DLP
    uint8_t dlp = DLP_VALUE;
    Cy_SMIF_S25FL_WriteVDLR(CY_SMIF_USED, D_QUAD_SLAVES, dlp, &smifContext);

    /********************************************************/
    /******* Write/Read Test (DDR Quad I/O Read) ************/
    /********************************************************/
    ChangePLLFrequency(SMIF_CLK_FREQ_DDR_RD*2);					// SMIF out clock will be 50,000,000
    
    smifConfig.rxClk = CY_SMIF_INV_OUTPUT_CLK;					// in case did not work, try on CY_SMIF_INV_FEEDBACK_CLK or CY_SMIF_INV_OUTPUT_CLK. !!!!
    Cy_SMIF_Init(CY_SMIF_USED, &smifConfig, 1000, &smifContext);

    TestFor_DQUAD_MMIO_WRITE_Then_MMIO_DDR_READ_NON_BLOCKING(CY_SMIF_S25FL_ID_4DDRQIOR);

    // Reaching here means this example worked properly.
    for(;;);
}


uint8_t swappedProgramData[TEST_PROGRAM_SIZE];

static void TestFor_DQUAD_MMIO_WRITE_Then_MMIO_DDR_READ_NON_BLOCKING(uint16_t rd_cmdId)
{
    SetModeWithBusyCheck(CY_SMIF_USED, CY_SMIF_NORMAL);
    for(uint32_t i_EraseSectorTopAddr = TEST_PROGRAM_SECTOR; i_EraseSectorTopAddr < (TEST_PROGRAM_SECTOR+TEST_PROGRAM_SIZE); i_EraseSectorTopAddr+= (DEVICE_SECTOR_ERASE*2))
    {
        SectorEraseProcedure(D_QUAD_SLAVES, ((i_EraseSectorTopAddr + TEST_PROGRAM_SECTOR) / 2));
    }

    /*  Program 64K byte data */
    // !!!!! User need to swap data to be written like below, when write in SDR Octal mode and read in DDR Octalmode !!!
    for(uint32_t i_swappingDataPos = 0; i_swappingDataPos < TEST_PROGRAM_SIZE; i_swappingDataPos += 2)
    {
        swappedProgramData[i_swappingDataPos]     = programData[i_swappingDataPos + 1];
        swappedProgramData[i_swappingDataPos + 1] = programData[i_swappingDataPos];
    }

    volatile uint32_t programAddress    = TEST_PROGRAM_SECTOR;
    volatile uint32_t programSizeInByte = TEST_PROGRAM_SIZE;

    for(uint32_t i_WriteOffsetAddrOfQuadDevice = 0; i_WriteOffsetAddrOfQuadDevice < (programSizeInByte / 2); i_WriteOffsetAddrOfQuadDevice+=DEVICE_PROGRAM_BUFFER_SIZE)
    {
        uint32_t writeOffsetAddrOfMCU = i_WriteOffsetAddrOfQuadDevice * 2ul;
        uint32_t writeStartAddress = i_WriteOffsetAddrOfQuadDevice + programAddress;

        // Write enable
        Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, D_QUAD_SLAVES, &smifContext);

        // Issue program
        Cy_SMIF_MMIO_Program_ExMem(CY_SMIF_USED, D_QUAD_SLAVES, CY_SMIF_S25FL_ID_4QPP, writeStartAddress, (DEVICE_PROGRAM_BUFFER_SIZE*2), (uint8_t*)&swappedProgramData[writeOffsetAddrOfMCU], CY_SMIF_NON_BLOCKING, TEST_SPI_WRITE_LC, 0, &smifDeviceContext, &smifContext);

        // Wait for the sending completes.
        while(Cy_SMIF_GetTxfrStatus(CY_SMIF_USED, &smifContext) == CY_SMIF_SEND_BUSY);
        // Check if the sending finished with success
        CY_ASSERT_L1(Cy_SMIF_GetTxfrStatus(CY_SMIF_USED, &smifContext) == CY_SMIF_SEND_CMPLT);

        // Wait for completion
        CY_ASSERT_L1(WaitDeviceBecomesReady(D_QUAD_SLAVES) == 0u);
    }

    /* Read 64K byte data */
    volatile uint32_t readAddress    = TEST_PROGRAM_SECTOR;
    volatile uint32_t readSizeInByte = TEST_PROGRAM_SIZE;

    // Read the programmed data
    Cy_SMIF_MMIO_Read_ExMem(CY_SMIF_USED, D_QUAD_SLAVES, rd_cmdId, readAddress, readSizeInByte, (uint8_t*)readDataBuffer, TEST_SPI_READ_LC, CY_SMIF_S25FL_RW_NON_CONTINUOUS, CY_SMIF_NON_BLOCKING, &smifDeviceContext, &smifContext);

    // Wait for the receiving completes.
    while(Cy_SMIF_GetTxfrStatus(CY_SMIF_USED, &smifContext) == CY_SMIF_REC_BUSY);
    // Check if the receiving finished with success
    CY_ASSERT_L1(Cy_SMIF_GetTxfrStatus(CY_SMIF_USED, &smifContext) == CY_SMIF_REC_CMPLT);

    // Check read data
    CY_ASSERT_L1((memcmp(readDataBuffer, programData, TEST_PROGRAM_SIZE) == 0));
}



static void SetModeWithBusyCheck(volatile cy_stc_smif_reg_t *base, cy_en_smif_mode_t mode)
{
    cy_en_smif_status_t status;
    do
    {
        status = Cy_SMIF_SetMode(base, mode);
    }while(status != CY_SMIF_SUCCESS);
}

// return 1, when one of selected device return error during erasing.
// return 0, when erasing completed without error.
static bool WaitDeviceBecomesReady(cy_en_smif_slave_select_t slave)
{
    for(uint8_t i_slaveNum = 0; i_slaveNum < CY_SMIF_GetDeviceNumber(CY_SMIF_USED); i_slaveNum++)
    {
        cy_en_smif_slave_select_t slaveToBeChecked = (cy_en_smif_slave_select_t)((uint8_t)slave & (0x01 << i_slaveNum));
        if((uint8_t)slaveToBeChecked == 0u)
        {
            // not selected slave
            continue;
        }

        // only selected slave will reach hare.
        while(1)
        {
            volatile un_cy_smif_s25fl128_SR1_t sr1 = { .u8 = 0u };
            Cy_SMIF_S25FL_ReadStatusRegister1(CY_SMIF_USED, slaveToBeChecked, (uint8_t*)&sr1, &smifContext);
            if(sr1.field.u1WIP == 0u)
            {
                break;
            }

#if (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXS)
            if((sr1.field.u1E_ERR == 1u) || (sr1.field.u1P_ERR == 1u))
            {
                return 1; // error
            }
#elif (CY_SMIF_QUAD_MEMORY == CY_SMIF_S25FXXXL)
            volatile un_cy_smif_s25fl128_SR2_t sr2 = { .u8 = 0u };
            Cy_SMIF_S25FL_ReadStatusRegister2(CY_SMIF_USED, slaveToBeChecked, (uint8_t*)&sr2, &smifContext);
            if((sr2.field.u1E_ERR == 1u) || (sr2.field.u1P_ERR == 1u))
            {
                 Cy_SMIF_S25FL_ClearStatusRegister(CY_SMIF_USED,
                                        slaveToBeChecked,
                                        &smifContext);

                return 1; // error
            }
#endif
        }
    }
    return 0;

}

static void SectorEraseProcedure(cy_en_smif_slave_select_t slave, uint32_t address)
{
    /*  Erase sector        */
    // Write enable
    Cy_SMIF_S25FL_WriteEnable(CY_SMIF_USED, slave, &smifContext);

    // Sector erase
    Cy_SMIF_S25FL_SectorErase(CY_SMIF_USED, slave, address, &smifContext);

    // Wait for completion
    CY_ASSERT_L1(WaitDeviceBecomesReady(D_QUAD_SLAVES) == 0);
}


/* [] END OF FILE */
