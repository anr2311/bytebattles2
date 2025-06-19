/***************************************************************************//**
* \file main_cm0plus.c
*
* \version 1.0
*
* \brief Main example file for CM0+
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
#include "wav.h"
/*
     CY_TDM_WAV_FILE mode 		: It will output 1K sine wave to left and right.
     						  		Please check the audio by connecting a headphone
                                    on J53.

     CY_TDM_RX_TX_LOOP_BACK mode 	: Connect the Audio Aux cable to a audio source
                                   	  say, Laptop or mobile phone and play an audio.
                                   	  Connect the other end of Aux cable to TRRS board, and connect 
                                   	  the TIP pin to R727.2 and SLEEVE pin to GND.
                                      You can listen to the music played over phone 
                                      on J53.
*/

/* User setting value */
//------------------------------------------------------------------------------
#define CY_TDM_INTERNAL_CLOCK   (0)

#define CY_TDM_RX_TX_LOOP_BACK  (0)
#define CY_TDM_WAV_FILE       (1)

/* Application Functionality Selection */
#define CY_TDM_USE_CLK          CY_TDM_INTERNAL_CLOCK
#define CY_TDM_APP_FUNCTION     CY_TDM_RX_TX_LOOP_BACK

/* End of Application Functionality Selection */

/* Clock Selection Macros */
//------------------------------------------------------------------------------
#define TDM_IP_FIFO_SIZE_MAX    (128)
#define TDM_TX_FIFO_THRESHOLD   (20U)
#define TDM_RX_FIFO_THRESHOLD   (60U)

#if (CY_USE_PSVP == 1)  
  #define SOURCE_CLOCK_FRQ      24000000ul
#else
  #define SOURCE_CLOCK_FRQ      196608000ul
#endif

/* Smaple rate and clock setup */
#define TDM_SAMPLING_RATE       (48000ul)
#define TDM_SLOT_WIDTH          (32ul)
#define TDM_CHANNEL_COUNT       (8ul)

#define TDM_CLK_FREQ            (TDM_SAMPLING_RATE*TDM_SLOT_WIDTH*TDM_CHANNEL_COUNT)
#define TDM_BIT_CLOCK_DIV       (SOURCE_CLOCK_FRQ/TDM_CLK_FREQ)

#define TDM_ACTIVE_CHANNELS     (2U)
/* PCM data to be transmitted over tdm */
//------------------------------------------------------------------------------

#define SOUND_BUF_SIZE (128ul)
uint16_t g_SoundBuffer[SOUND_BUF_SIZE] = { 0ul };


/* Port pin config for TDM */
//------------------------------------------------------------------------------
cy_stc_gpio_pin_prt_config_t g_tdm_pin_config[] =
{
//  {                       portReg,                       pinNum, outVal,                driveMode,                            hsiom, intEdge, intMask, vtrip, slewRate, driveSel },
    {   CY_AUDIOSS_TDM0_TX_SCK_PORT,   CY_AUDIOSS_TDM0_TX_SCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_TX_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    { CY_AUDIOSS_TDM0_TX_FSYNC_PORT, CY_AUDIOSS_TDM0_TX_FSYNC_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF, CY_AUDIOSS_TDM0_TX_FSYNC_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {    CY_AUDIOSS_TDM0_TX_SD_PORT,    CY_AUDIOSS_TDM0_TX_SD_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,    CY_AUDIOSS_TDM0_TX_SD_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {   CY_AUDIOSS_TDM0_RX_SCK_PORT,   CY_AUDIOSS_TDM0_RX_SCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_RX_SCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    { CY_AUDIOSS_TDM0_RX_FSYNC_PORT, CY_AUDIOSS_TDM0_RX_FSYNC_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF, CY_AUDIOSS_TDM0_RX_FSYNC_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {    CY_AUDIOSS_TDM0_RX_SD_PORT,    CY_AUDIOSS_TDM0_RX_SD_PIN,    0ul,         CY_GPIO_DM_HIGHZ,    CY_AUDIOSS_TDM0_RX_SD_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
    {   CY_AUDIOSS_TDM0_RX_MCK_PORT,   CY_AUDIOSS_TDM0_RX_MCK_PIN,    0ul,         CY_GPIO_DM_HIGHZ,   CY_AUDIOSS_TDM0_RX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#if (CY_TDM_USE_CLK == CY_TDM_INTERNAL_CLOCK)
    {   CY_AUDIOSS_TDM0_TX_MCK_PORT,   CY_AUDIOSS_TDM0_TX_MCK_PIN,    0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_TX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#else
    {   CY_AUDIOSS_TDM0_TX_MCK_PORT,   CY_AUDIOSS_TDM0_TX_MCK_PIN,    0ul,         CY_GPIO_DM_HIGHZ,   CY_AUDIOSS_TDM0_TX_MCK_PIN_MUX,     0ul,     0ul,   0ul,      0ul,      0ul },
#endif
    {   CY_AUDIOSS_TDM0_RST_PORT,      CY_AUDIOSS_TDM0_RST_PIN,       0ul, CY_GPIO_DM_STRONG_IN_OFF,   CY_AUDIOSS_TDM0_RST_PIN_MUX,        0ul,     0ul,   0ul,      0ul,      0ul },
};
#define TDM_PORT_NUM (sizeof(g_tdm_pin_config)/sizeof(g_tdm_pin_config[0]))

/* TDM-Tx and TDM-Rx interface configuration */
//------------------------------------------------------------------------------
cy_stc_tdm_config_t config_tdm = 
{
    .tx_config =
    {
        .txEnable          = true,
        .txMasterMode       = CY_TDM_DEVICE_MASTER,
        .txFormat           = CY_TDM_LEFT_DELATED,
        .txWordSize         = CY_TDM_SIZE_16,
        .txClkDiv           = TDM_BIT_CLOCK_DIV,
    #if (CY_TDM_USE_CLK == CY_TDM_INTERNAL_CLOCK)
        .txClkSel           = CY_TDM_SEL_SRSS_CLK0,
    #else
        .txClkSel           = CY_TDM_SEL_TX_MCLK_IN,
    #endif
        .txSckPolarity      = CY_TDM_CLK,
        .txFsyncFormat      = CY_TDM_CH_PERIOD,
        .txFsyncPolarity    = CY_TDM_SIGN,
        .txChannelNUM       = TDM_CHANNEL_COUNT,
        .txChEN             = (1 << TDM_ACTIVE_CHANNELS) - 1,
        .txChannelSize      = TDM_SLOT_WIDTH,
        .txFIFOTriggerLevel = TDM_TX_FIFO_THRESHOLD,
    },
    .rx_config =
    {
        .rxEnable           = true,
        .rxMasterMode       = CY_TDM_DEVICE_MASTER,
        .rxFormat           = CY_TDM_LEFT_DELATED,
        .rxWordSize         = CY_TDM_SIZE_16,
        .rxClkDiv           = TDM_BIT_CLOCK_DIV,
    #if (CY_TDM_USE_CLK == CY_TDM_INTERNAL_CLOCK)
        .rxClkSel           = CY_TDM_SEL_SRSS_CLK0,
    #else
        .rxClkSel           = CY_TDM_SEL_TX_MCLK_IN,
    #endif
        .rxSckPolarity      = CY_TDM_CLK,
        .rxFsyncFormat      = CY_TDM_CH_PERIOD,
        .rxFsyncPolarity    = CY_TDM_SIGN,
        .rxChannelNUM       = TDM_CHANNEL_COUNT,
        .rxChEN             = (1 << TDM_ACTIVE_CHANNELS) - 1,
        .rxChannelSize      = TDM_SLOT_WIDTH,
        .rxLateSample       = false,
        .rxTestMode         = false,
        .rxFIFOTriggerLevel = TDM_RX_FIFO_THRESHOLD,
    },
};

/* TDM irq configuration */
//------------------------------------------------------------------------------
static cy_stc_sysint_irq_t irqCfgAudioSSTx =
{
    .sysIntSrc  = CY_AUDIOSS_TDM0_TX_IRQN,
    .intIdx     = CPUIntIdx4_IRQn,
    .isEnabled  = true,
};

static cy_stc_sysint_irq_t irqCfgAudioSSRx =
{
    .sysIntSrc  = CY_AUDIOSS_TDM0_RX_IRQN,
    .intIdx     = CPUIntIdx4_IRQn,
    .isEnabled  = true,
};

/* Global function declaration */
//------------------------------------------------------------------------------
void ReceiveTdmData(void);
void AudioSSInterruptHandlerRx(void);
void AudioSSInterruptHandlerTx(void);
void Cy_Cs42448_DefaultConfig(void);
uint32_t WavCounter = 0;
/* TDM data receive handler */
//------------------------------------------------------------------------------
void ReceiveTdmData(void)
{
    volatile uint32_t rxData;
    for(uint8_t data_i = 0; data_i < TDM_RX_FIFO_THRESHOLD; data_i++)
    {
        rxData = Cy_AudioTDM_ReadRxData(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

#if CY_TDM_APP_FUNCTION == CY_TDM_RX_TX_LOOP_BACK

        g_SoundBuffer[data_i] = rxData;

#endif
    }
}

/* AudioSS tdm interrupt handler */
//------------------------------------------------------------------------------
void AudioSSInterruptHandlerTx(void)
{
    uint32_t TX_maskedIntrStatus;
    TX_maskedIntrStatus = Cy_AudioTDM_GetTxInterruptStatusMasked(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

        #if CY_TDM_APP_FUNCTION == CY_TDM_WAV_FILE
            for(uint8_t data_i = 0; data_i < TDM_RX_FIFO_THRESHOLD; data_i++ )
            {
                Cy_AudioTDM_WriteTxData(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, (WAV_DATA[data_i + WavCounter] - 0x7FFE));
            }
            WavCounter += TDM_RX_FIFO_THRESHOLD;
            if (WavCounter >= sizeof(WAV_DATA)/sizeof(WAV_DATA[0]))
            {
                WavCounter = 0;
            }

        #elif CY_TDM_APP_FUNCTION == CY_TDM_RX_TX_LOOP_BACK
            for(uint8_t data_i = 0; data_i < TDM_RX_FIFO_THRESHOLD; data_i++ )
            {
                Cy_AudioTDM_WriteTxData(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, g_SoundBuffer[data_i]);
            }
        #endif

    Cy_AudioTDM_ClearTxInterrupt(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT, TX_maskedIntrStatus);
}

void AudioSSInterruptHandlerRx(void)
{
    uint32_t RX_maskedIntrStatus;
    RX_maskedIntrStatus = Cy_AudioTDM_GetRxInterruptStatusMasked(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

    // only FIFO TRIGGER is acceptable
    CY_ASSERT((Cy_AudioTDM_GetRxInterrupt(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT) & ~CY_TDM_INTR_RX_FIFO_TRIGGER) == 0);

    ReceiveTdmData();

    Cy_AudioTDM_ClearRxInterrupt(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT, RX_maskedIntrStatus);
}

/* Function does defult codec configuration for TDM */
//------------------------------------------------------------------------------
void Cy_Cs42448_DefaultConfig(void)
{

    Cy_GPIO_Clr(CY_AUDIOSS_TDM0_RST_PORT, CY_AUDIOSS_TDM0_RST_PIN);
    Cy_SysTick_DelayInUs(50000);
    Cy_GPIO_Set(CY_AUDIOSS_TDM0_RST_PORT, CY_AUDIOSS_TDM0_RST_PIN);

    uint8_t readRegData = 0;

    cy_un_cs42448_reg_powerCtl_t        powerCtl        = {.u8  = 0u};
    powerCtl.u8 = 0; // set all power down function disable
    Cy_Cs42448_WriteCtrReg(CY_CS42448_POWER_CTRL_REGISTER, powerCtl.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_POWER_CTRL_REGISTER);
    CY_ASSERT(powerCtl.u8 == readRegData);

    cy_un_cs42448_reg_funcMode_t        funcMode        = {.u8  = 0u};
    funcMode.bitField.DACFM = 3; // Slave mode: Auto-detect sample rates
    funcMode.bitField.ADCFM = 3; // Slave mode: Auto-detect sample rates
    funcMode.bitField.MFreq = 0; // 1.0290MHz to 12.288MHz
    Cy_Cs42448_WriteCtrReg(CY_CS42448_FUNCTIONAL_MODE_REGISTER, funcMode.u8);
    readRegData = Cy_Cs42448_ReadCtrReg(CY_CS42448_FUNCTIONAL_MODE_REGISTER);
    CY_ASSERT(funcMode.u8 == readRegData);

    cy_un_cs42448_reg_interfaceFmt_t    interfaceFmt    = {.u8  = 0u};
    interfaceFmt.bitField.Freeze = 0;
    interfaceFmt.bitField.AUXDIF = 0; // TDM (does not matter)
    interfaceFmt.bitField.DACDIF = 6; // TDM, up to 24-bit data (slave only)
    interfaceFmt.bitField.ADCDIF = 6; // TDM, up to 24-bit data (slave only)
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

/* Main application function */
//------------------------------------------------------------------------------
int main(void)
{
    SystemInit();

    __enable_irq();

    /* Initialize control path for audio codec */
    Cy_Cs42448_Init();

    /* Initialize & default setup for audio codec */
    Cy_Cs42448_DefaultConfig();

    /* Port Setting for TDM communication */
    Cy_GPIO_Multi_Pin_Init(g_tdm_pin_config, TDM_PORT_NUM);

    /* Interrupt setting for TDM communication */
    Cy_SysInt_InitIRQ(&irqCfgAudioSSTx);
    Cy_SysInt_SetSystemIrqVector(irqCfgAudioSSTx.sysIntSrc, AudioSSInterruptHandlerTx);
    Cy_SysInt_InitIRQ(&irqCfgAudioSSRx);
    Cy_SysInt_SetSystemIrqVector(irqCfgAudioSSRx.sysIntSrc, AudioSSInterruptHandlerRx);
    NVIC_EnableIRQ(irqCfgAudioSSTx.intIdx);

    /* Init TX interface */
    Cy_AudioTDM_DeInit(CY_AUDIOSS_TDM_TYPE);
    Cy_AudioTDM_Init(CY_AUDIOSS_TDM_TYPE, &config_tdm);
    Cy_AudioTDM_SetTxTriggerInterruptMask(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

    /* Enable TX interface */
    Cy_AudioTDM_StartTx(&CY_AUDIOSS_TDM_TYPE->TDM_TX_STRUCT);

    /* Enable RX interface */
#if CY_TDM_APP_FUNCTION == CY_TDM_RX_TX_LOOP_BACK
    /* Init RX interface */
    Cy_AudioTDM_SetRxTriggerInterruptMask(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);

    /* Enable RX interface */
    Cy_AudioTDM_StartRx(&CY_AUDIOSS_TDM_TYPE->TDM_RX_STRUCT);
#endif
    for(;;);
}

/* [] END OF FILE */
