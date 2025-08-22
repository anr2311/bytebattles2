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
#include <stdint.h>
#include <stdbool.h>

void Cy_Cs42448_Init(void);
void Cy_Cs42448_DefaultConfig(void);
static void AudioSSInterruptHandlerTx(void);
static void AudioSSInterruptHandlerRx(void);
/*
    This example transmits .wav file to external audio DAC (Cs42448)
    via I2S protocol.
    Please note this example supports only 16 bit length PCM data.
    TVII is master, and the Cs42448 is slave. Sampling rate, 
    monaural/stereo will be determined by wav header.
    When internal clock is used, please connect I2S_MCLK to MCLK input
    port of the Cs42448. When external clock is used please connect
    external crystal output to I2S_IF of TVII and to MCLK input port
    of the Cs42448.
    - Operation Description
    I2S HW will trigger CPU interrupt and the CPU will then copy the 
    .wav sound data to the I2S FIFO in the IRQ.
*/

/* User setting for clock and  */
//------------------------------------------------------------------------------
#define I2S_INTERNAL_CLOCK          (0ul)
#define I2S_EXTERNAL_CLOCK          (1ul)
#define CY_I2S_USE_CLK              I2S_INTERNAL_CLOCK

#define CY_I2S_APP_WAV_FILE         (0)
#define CY_I2S_RX_TX_LOOP_BACK       (1)

#define CY_I2S_APP_FUNCTION         CY_I2S_RX_TX_LOOP_BACK

#if (CY_I2S_APP_FUNCTION == CY_I2S_APP_WAV_FILE)
    #include "wav.h"
#endif

#if (CY_USE_PSVP == 1)  
  #define SOURCE_CLOCK_FRQ        24000000ul
#else
    #define SOURCE_CLOCK_FRQ      196608000ul
#endif

#define I2S_SAMPLING_RATE (48000ul)
#define I2S_PCM_DATA_LEN  (16ul)
#define I2S_CHANNEL_COUNT (2ul)

#define I2S_CLK_FREQ      (I2S_SAMPLING_RATE*I2S_PCM_DATA_LEN*I2S_CHANNEL_COUNT)
#define I2S_BIT_CLOCK_DIV (SOURCE_CLOCK_FRQ/I2S_CLK_FREQ)

/* I2S-Tx and I2S-Rx interface configuration */
//------------------------------------------------------------------------------
#define CY_I2S_TX_FIFO_NUM (128)
#define I2S_TX_FIFO_TRIG   (20U)
#define I2S_RX_FIFO_TRIG   (60U)

cy_stc_i2s_config_t const config_I2S_TX =
{
    .txEnabled          = true,
    .txMasterMode       = CY_I2S_DEVICE_MASTER,
    .txFormat           = CY_I2S_LEFT_DELAYED,
    .txWordSize         = CY_I2S_LEN16,
    .txClkDiv           = I2S_BIT_CLOCK_DIV-1u,
#if (CY_I2S_USE_CLK ==    I2S_INTERNAL_CLOCK)
    .txClkSel           = CY_I2S_SEL_SRSS_CLK0,
#else
    .txClkSel           = CY_I2S_SEL_RX_MCLK_IN,
#endif
    .txSckPolarity      = 0,
    .txFsyncFormat      = CY_I2S_FSYNC_FMT_CHL_PERIOD,
    .txFsyncPolarity    = 1,
    .txChannelNum       = (I2S_CHANNEL_COUNT - 1),
    .txChannelEnable    = (1ul << I2S_CHANNEL_COUNT) - 1ul,
    .txChannelSize      = (I2S_PCM_DATA_LEN - 1),
    .txFifoTriggerLevel = I2S_TX_FIFO_TRIG,
};

cy_stc_i2s_config_t const config_I2S_RX =
{
    .rxEnabled          = true,
    .rxMasterMode       = CY_I2S_DEVICE_MASTER,
    .rxFormat           = CY_I2S_LEFT_DELAYED,
    .rxWordSize         = CY_I2S_LEN16,
    .rxClkDiv           = I2S_BIT_CLOCK_DIV-1u,
    .rxClkSel           = CY_I2S_SEL_SRSS_CLK0,
    .rxSckPolarity      = 0, 
    .rxFsyncFormat      = CY_I2S_FSYNC_FMT_CHL_PERIOD,
    .rxFsyncPolarity    = 1,
    .rxChannelNum       = (I2S_CHANNEL_COUNT - 1),
    .rxChannelEnable    = (1ul << I2S_CHANNEL_COUNT) - 1ul,
    .rxChannelSize      = (I2S_PCM_DATA_LEN - 1),
    .rxLateSampleDelay  = CY_I2S_LATE_SAMPLE_RISING,
    .rxLateCaptureDelay = CY_I2S_LATE_CAPT_EXTRA_DELAY_0,
    .rxWsExtension      = CY_I2S_WS_EXTN_SIGN,
    .rxFifoTriggerLevel = I2S_RX_FIFO_TRIG,
};

cy_stc_gpio_pin_prt_config_t g_i2s_pin_config[] =
{
//  {                       portReg,                       pinNum, outVal,                driveMode,                            hsiom, intEdge, intMask, vtrip, slewRate, driveSel },
    {   CY_AUDIOSS_TDM0_TX_SCK_PORT,   CY_AUDIOSS_TDM0_TX_SCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_TX_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    { CY_AUDIOSS_TDM0_TX_FSYNC_PORT, CY_AUDIOSS_TDM0_TX_FSYNC_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF, CY_AUDIOSS_TDM0_TX_FSYNC_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {    CY_AUDIOSS_TDM0_TX_SD_PORT,    CY_AUDIOSS_TDM0_TX_SD_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,    CY_AUDIOSS_TDM0_TX_SD_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {   CY_AUDIOSS_TDM0_RX_SCK_PORT,   CY_AUDIOSS_TDM0_RX_SCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_RX_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    { CY_AUDIOSS_TDM0_RX_FSYNC_PORT, CY_AUDIOSS_TDM0_RX_FSYNC_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF, CY_AUDIOSS_TDM0_RX_FSYNC_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {    CY_AUDIOSS_TDM0_RX_SD_PORT,    CY_AUDIOSS_TDM0_RX_SD_PIN,    0ul,         CY_GPIO_DM_HIGHZ,    CY_AUDIOSS_TDM0_RX_SD_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {   CY_AUDIOSS_TDM0_RX_MCK_PORT,   CY_AUDIOSS_TDM0_RX_MCK_PIN,    0ul,         CY_GPIO_DM_HIGHZ,   CY_AUDIOSS_TDM0_RX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#if (CY_I2S_USE_CLK == I2S_INTERNAL_CLOCK)
    {   CY_AUDIOSS_TDM0_TX_MCK_PORT,   CY_AUDIOSS_TDM0_TX_MCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_TX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#else
    {   CY_AUDIOSS_TDM0_TX_MCK_PORT,   CY_AUDIOSS_TDM0_TX_MCK_PIN,    0ul,         CY_GPIO_DM_HIGHZ,   CY_AUDIOSS_TDM0_TX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#endif
};
#define I2S_PORT_NUM (sizeof(g_i2s_pin_config)/sizeof(g_i2s_pin_config[0]))

static const cy_stc_sysint_irq_t irqCfgAudioSSTx =
{
    .sysIntSrc  = CY_AUDIOSS_TDM0_TX_IRQN,
    .intIdx     = CPUIntIdx4_IRQn,
    .isEnabled  = true,
};

static const cy_stc_sysint_irq_t irqCfgAudioSSRx =
{
    .sysIntSrc  = CY_AUDIOSS_TDM0_RX_IRQN,
    .intIdx     = CPUIntIdx5_IRQn,
    .isEnabled  = true,
};

#if CY_I2S_APP_FUNCTION == CY_I2S_RX_TX_LOOP_BACK
#define SOUND_BUF_SIZE (128ul)
uint16_t g_SoundBuffer[SOUND_BUF_SIZE] = { 0ul };
#endif

uint32_t WavCounter = 0;
int main(void)
{
    SystemInit();

    __enable_irq();

    /*******************************************************/
    /*    Initialize Cirrus logic  Audio IC (CS42448)      */
    /*******************************************************/
    /* Initialize control path for audio codec */
    Cy_Cs42448_Init();

    /* Initialize & default setup for audio codec */
    Cy_Cs42448_DefaultConfig();

    /**************************************/
    /* Port setting for I2S communication */
    /**************************************/
    Cy_GPIO_Multi_Pin_Init(g_i2s_pin_config, I2S_PORT_NUM);

    /*******************************************/
    /* Interrupt setting for I2S communication */
    /*******************************************/
    Cy_SysInt_InitIRQ(&irqCfgAudioSSTx);
    Cy_SysInt_SetSystemIrqVector(irqCfgAudioSSTx.sysIntSrc, AudioSSInterruptHandlerTx);
    Cy_SysInt_InitIRQ(&irqCfgAudioSSRx);
    Cy_SysInt_SetSystemIrqVector(irqCfgAudioSSRx.sysIntSrc, AudioSSInterruptHandlerRx);
    NVIC_EnableIRQ(irqCfgAudioSSTx.intIdx);
    NVIC_EnableIRQ(irqCfgAudioSSRx.intIdx);

    /********************************************/
    /*          initialization for I2S          */
    /********************************************/
    // De-Initialize I2S
    Cy_I2S_DeInit(CY_AUDIOSS_TDM_TYPE);

    // Flush FIFO of I2S 
    //Cy_I2S_ClearTxFifo(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);
    //Cy_I2S_ClearRxFifo(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

    // Initialize I2S Tx settings
    Cy_I2S_Tx_Init(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, &config_I2S_TX);

    // Initialize I2S Interrupt settings
    Cy_I2S_SetTxTriggerInterruptMask(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

    /* Start I2S Tx */
    Cy_I2S_EnableTx(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

#if CY_I2S_APP_FUNCTION == CY_I2S_RX_TX_LOOP_BACK

    Cy_I2S_Rx_Init(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT, &config_I2S_RX);

    Cy_I2S_SetRxTriggerInterruptMask(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

    Cy_I2S_EnableRx(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

#endif

    for(;;);
}

/* AudioSS i2s tx interrupt handler */
//------------------------------------------------------------------------------
static void AudioSSInterruptHandlerTx(void)
{
    uint32_t masked = Cy_I2S_GetTxInterruptStatusMasked(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

    if((masked & CY_I2S_INTR_TX_FIFO_TRIGGER) != 0ul)
    {
        #if CY_I2S_APP_FUNCTION == CY_I2S_APP_WAV_FILE
            for(uint8_t data_i = 0; data_i < I2S_RX_FIFO_TRIG; data_i++ )
            {
                Cy_I2S_WriteTxData(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, (WAV_DATA[data_i + WavCounter] - 0x7FFE));
            }
            WavCounter += I2S_RX_FIFO_TRIG;
            if (WavCounter >= sizeof(WAV_DATA)/sizeof(WAV_DATA[0]))
            {
                WavCounter = 0;
            }

        #elif CY_I2S_APP_FUNCTION == CY_I2S_RX_TX_LOOP_BACK
            for(uint8_t data_i = 0; data_i < I2S_RX_FIFO_TRIG; data_i++ )
            {
                Cy_I2S_WriteTxData(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, g_SoundBuffer[data_i]);
            }
        #endif
    }

    if((masked & CY_I2S_INTR_TX_FIFO_UNDERFLOW) != 0ul)
    {
        CY_ASSERT(false);
    }

    if((masked & CY_I2S_INTR_TX_FIFO_OVERFLOW) != 0ul)
    {
        CY_ASSERT(false);
    }

    Cy_I2S_ClearTxInterrupt(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, masked);
}

/* AudioSS i2s rx interrupt handler */
//------------------------------------------------------------------------------
static void AudioSSInterruptHandlerRx(void)
{
    volatile uint32_t rxData;
    uint32_t masked = Cy_I2S_GetRxInterruptStatusMasked(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

    if((masked & CY_I2S_INTR_RX_FIFO_TRIGGER) != 0ul)
    {
        for(uint8_t data_i = 0; data_i <I2S_RX_FIFO_TRIG; data_i++)
        {
            rxData = Cy_I2S_ReadRxData(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);
            #if CY_I2S_APP_FUNCTION == CY_I2S_RX_TX_LOOP_BACK
                g_SoundBuffer[data_i] = rxData;
            #endif
        }
    }

    if((masked & CY_I2S_INTR_RX_FIFO_UNDERFLOW) != 0ul)
    {
        CY_ASSERT(false);
    }

    if((masked & CY_I2S_INTR_RX_FIFO_OVERFLOW) != 0ul)
    {
        CY_ASSERT(false);
    }
    Cy_I2S_ClearRxInterrupt(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT, masked);
}


/*******************************************************************************
* Function Name: Cy_Cs42448_DefaultConfig
****************************************************************************//**
*
* \brief
* Function to initialize default settings for audio codec CS42448.
*
* \param
* none
*
* \return 
* none
*
* \note
* SCB3 is fixed for the audio codec control path.
* 
*******************************************************************************/
void Cy_Cs42448_DefaultConfig(void)
{
    uint8_t readRegData = 0;

    cy_un_cs42448_reg_powerCtl_t        powerCtl        = {.u8  = 0u};
    powerCtl.u8 = 0; // set all power down function disable
    Cy_Cs42448_WriteCtrReg(CY_CS42448_POWER_CTRL_REGISTER, powerCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_POWER_CTRL_REGISTER);
    CY_ASSERT(powerCtl.u8 == readRegData);

    cy_un_cs42448_reg_funcMode_t        funcMode        = {.u8  = 0u};
    funcMode.bitField.DACFM = 3; // Slave mode: Auto-detect sample rates
    funcMode.bitField.ADCFM = 3; // Slave mode: Auto-detect sample rates
    funcMode.bitField.MFreq = 0; // 2.0480MHz to 25.6000MHz
    Cy_Cs42448_WriteCtrReg(CY_CS42448_FUNCTIONAL_MODE_REGISTER, funcMode.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_FUNCTIONAL_MODE_REGISTER);
    CY_ASSERT(funcMode.u8 == readRegData);

    cy_un_cs42448_reg_interfaceFmt_t    interfaceFmt    = {.u8  = 0u};
    interfaceFmt.bitField.Freeze = 0;
    interfaceFmt.bitField.AUXDIF = 1; // I2S (does not matter)
    interfaceFmt.bitField.DACDIF = 1; // I2S, up to 24-bit data
    interfaceFmt.bitField.ADCDIF = 1; // I2S, up to 24-bit data
    Cy_Cs42448_WriteCtrReg(CY_CS42448_INTERFACE_CTRL_REGISTER, interfaceFmt.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_INTERFACE_CTRL_REGISTER);
    CY_ASSERT(interfaceFmt.u8 == readRegData);

    sv_un_cs42448_reg_adcDacCtl_t       adcDacCtl       = {.u8  = 0u};
    adcDacCtl.bitField.DACDEM = 0; // De-Emphasis Enabled (Auto-Detect Fs)
    Cy_Cs42448_WriteCtrReg(CY_CS42448_ADC_CTRL_REGISTER, adcDacCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_ADC_CTRL_REGISTER);
    CY_ASSERT(adcDacCtl.u8 == readRegData);

    cy_un_cs42448_reg_transCtl_t        transitionCtl   = {.u8  = 0u};
    transitionCtl.bitField.DACSZC = 1;
    transitionCtl.bitField.ADCSZC = 1;
    Cy_Cs42448_WriteCtrReg(CY_CS42448_TRANSITION_CTRL_REGISTER, transitionCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_TRANSITION_CTRL_REGISTER);
    CY_ASSERT(transitionCtl.u8 == readRegData);

    cy_un_cs42448_reg_dacMuteCtl_t      dacMuteCtl      = {.u8  = 0u};
    dacMuteCtl.bitField.aoutmute = 0; // all DAC auto mute disable
    Cy_Cs42448_WriteCtrReg(CY_CS42448_DAC_MUTE_CTRL_REGISTER, dacMuteCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_DAC_MUTE_CTRL_REGISTER);
    CY_ASSERT(dacMuteCtl.u8 == readRegData);

    cy_un_cs42448_reg_ain1VolCtl_t      ain1VolCtl      = {.u8  = 0u};
    ain1VolCtl.bitField.ainvol = 0; // 0 dB
    Cy_Cs42448_WriteCtrReg(CY_CS42448_VOLUME_CTRL_AIN_1_REGISTER, ain1VolCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_VOLUME_CTRL_AIN_1_REGISTER);
    CY_ASSERT(ain1VolCtl.u8 == readRegData);

    cy_un_cs42448_reg_aout1VolCtl_t     aout1VolCtl     = {.u8  = 0u};
    aout1VolCtl.bitField.aoutvol = 0; // 0dB
    Cy_Cs42448_WriteCtrReg(CY_CS42448_VOLUME_CTRL_AOUT_1_REGISTER, aout1VolCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_VOLUME_CTRL_AOUT_1_REGISTER);
    CY_ASSERT(aout1VolCtl.u8 == readRegData);
}
/* [] END OF FILE */
