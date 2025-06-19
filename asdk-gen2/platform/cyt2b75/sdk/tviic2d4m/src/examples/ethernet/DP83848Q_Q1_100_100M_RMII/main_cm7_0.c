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
#include "DP83848Q_Q1_Reg.h"

/*
    This example receives and transmits ethernet packets with external machine such as a PC.
    First, this code transmits a packet containing an original message.
    After that, if this TVII received a packet, it would send back the packet adding an extra message
    to the received message.
    This example sets up time stump unit as well. The TSU interrupt the CPU every 1s. And the CPU
    toggle a LED level in the interrupt handler.
*/ 

#define EMAC_MII           0 // Not supported
#define EMAC_RMII          1
#define EMAC_GMII          2 // Not supported
#define EMAC_RGMII         3 // Not supported
#define ETH_LINKSPEED_10   10
#define ETH_LINKSPEED_100  100
#define ETH_LINKSPEED_1000 1000 // Not supported

#define ETH_FRAME_SIZE_FCS (4)

/** User Parameters      */
#define EMAC_INTERFACE     EMAC_RMII
#define EMAC_LINKSPEED     ETH_LINKSPEED_100
#define ETH_INTR_INDEX     (CPUIntIdx3_IRQn)

#define PHY_ADDR     (1)       // DP83848Q_Q1's PHYAD0 pin has internal week pull up register
#define PHY_OUI_MSB  (0x2000u)
#define PHY_OUI_LSB  (0x17u)
#define PHY_VNDR_MDL (9u)
#define PHY_MDL_REV  (0u)

#if EMAC_LINKSPEED == ETH_LINKSPEED_10
 #define SPEED_SELECTION_BIT_VALUE (0u)
#elif EMAC_LINKSPEED == ETH_LINKSPEED_100
 #define SPEED_SELECTION_BIT_VALUE (1u)
#else
 #error "Non-supported speed"
#endif

#if EMAC_INTERFACE != EMAC_RMII
 #error "Non-supported ether mode"
#endif

/* Device depending code */
#define HF_CLK_IDX_TO_TSU (CY_SYSCLK_HFCLK_5)

/********************************************************/
#define ETH_REG_BASE         CY_GIG_ETH_TYPE

#define ETH_INTR_SRC         (CY_GIG_ETH_IRQN0)
#define ETH_INTR_SRC_Q1      (CY_GIG_ETH_IRQN1)
#define ETH_INTR_SRC_Q2      (CY_GIG_ETH_IRQN2)

#define ETHx_TD0_PORT        CY_GIG_ETH_TD0_PORT
#define ETHx_TD0_PIN         CY_GIG_ETH_TD0_PIN
#define ETHx_TD0_PIN_MUX     CY_GIG_ETH_TD0_PIN_MUX

#define ETHx_TD1_PORT        CY_GIG_ETH_TD1_PORT
#define ETHx_TD1_PIN         CY_GIG_ETH_TD1_PIN
#define ETHx_TD1_PIN_MUX     CY_GIG_ETH_TD1_PIN_MUX

#define ETHx_TX_CTL_PORT     CY_GIG_ETH_TX_CTL_PORT
#define ETHx_TX_CTL_PIN      CY_GIG_ETH_TX_CTL_PIN
#define ETHx_TX_CTL_PIN_MUX  CY_GIG_ETH_TX_CTL_PIN_MUX

#define ETHx_RD0_PORT        CY_GIG_ETH_RD0_PORT
#define ETHx_RD0_PIN         CY_GIG_ETH_RD0_PIN
#define ETHx_RD0_PIN_MUX     CY_GIG_ETH_RD0_PIN_MUX

#define ETHx_RD1_PORT        CY_GIG_ETH_RD1_PORT
#define ETHx_RD1_PIN         CY_GIG_ETH_RD1_PIN
#define ETHx_RD1_PIN_MUX     CY_GIG_ETH_RD1_PIN_MUX

#define ETHx_RX_CTL_PORT     CY_GIG_ETH_RX_CTL_PORT
#define ETHx_RX_CTL_PIN      CY_GIG_ETH_RX_CTL_PIN
#define ETHx_RX_CTL_PIN_MUX  CY_GIG_ETH_RX_CTL_PIN_MUX

#define ETHx_RX_ER_PORT      CY_GIG_ETH_RX_ER_PORT
#define ETHx_RX_ER_PIN       CY_GIG_ETH_RX_ER_PIN
#define ETHx_RX_ER_PIN_MUX   CY_GIG_ETH_RX_ER_PIN_MUX

#define ETHx_REF_CLK_PORT    CY_GIG_ETH_REF_CLK_PORT
#define ETHx_REF_CLK_PIN     CY_GIG_ETH_REF_CLK_PIN
#define ETHx_REF_CLK_PIN_MUX CY_GIG_ETH_REF_CLK_PIN_MUX

#define ETHx_MDC_PORT        CY_GIG_ETH_MDC_PORT
#define ETHx_MDC_PIN         CY_GIG_ETH_MDC_PIN
#define ETHx_MDC_PIN_MUX     CY_GIG_ETH_MDC_PIN_MUX

#define ETHx_MDIO_PORT       CY_GIG_ETH_MDIO_PORT
#define ETHx_MDIO_PIN        CY_GIG_ETH_MDIO_PIN
#define ETHx_MDIO_PIN_MUX    CY_GIG_ETH_MDIO_PIN_MUX

#define USER_LED_PORT    CY_USER_LED2_PORT
#define USER_LED_PIN     CY_USER_LED2_PIN
#define USER_LED_PIN_MUX CY_USER_LED2_PIN_MUX

/** Declaration of private functions */
static void Cy_Ethx_InterruptHandler (void);
static void Ethx_RxFrameCB( cy_pstc_eth_type_t pstcEth, uint8_t * u8RxBuffer, uint32_t u32Length );
static void Ethx_TxFrameSuccessful( cy_pstc_eth_type_t pstcEth, uint8_t u8QueueIndex , uint32_t bufIdx);
static void Ethx_TSUIncrement(cy_pstc_eth_type_t pstcEth);
static void InitPHY_DP83848Q_Q1(void);
static bool Check_DP83848Q_Q1_LinkStatus(void);
static void CalcTSUIncValue(uint32_t sourceFreq, CEDI_TimerIncrement* incValue);

/********************************************************/
const cy_stc_gpio_pin_config_t gLedPortGfg =
{
    .outVal    = 0ul,
    .driveMode = CY_GPIO_DM_STRONG_IN_OFF,
    .hsiom     = USER_LED_PIN_MUX,
    .intEdge   = 0ul,
    .intMask   = 0ul,
    .vtrip     = 0ul,
    .slewRate  = 0ul,
    .driveSel  = 0ul,
};

const cy_stc_gpio_pin_prt_config_t gEtherPorts[] =
{
//  {              port,              pin, outVal,                driveMode,                hsiom, intEdge, intMask, vtrip, slewRate, driveSel, },
    {     ETHx_TD0_PORT,     ETHx_TD0_PIN,      0, CY_GPIO_DM_STRONG_IN_OFF,     ETHx_TD0_PIN_MUX,       0,       0,     1,        0,        0, },
    {     ETHx_TD1_PORT,     ETHx_TD1_PIN,      0, CY_GPIO_DM_STRONG_IN_OFF,     ETHx_TD1_PIN_MUX,       0,       0,     1,        0,        0, },
    {  ETHx_TX_CTL_PORT,  ETHx_TX_CTL_PIN,      0, CY_GPIO_DM_STRONG_IN_OFF,  ETHx_TX_CTL_PIN_MUX,       0,       0,     1,        0,        0, },
    {     ETHx_RD0_PORT,     ETHx_RD0_PIN,      0,         CY_GPIO_DM_HIGHZ,     ETHx_RD0_PIN_MUX,       0,       0,     1,        0,        0, },
    {     ETHx_RD1_PORT,     ETHx_RD1_PIN,      0,         CY_GPIO_DM_HIGHZ,     ETHx_RD1_PIN_MUX,       0,       0,     1,        0,        0, },
    {  ETHx_RX_CTL_PORT,  ETHx_RX_CTL_PIN,      0,         CY_GPIO_DM_HIGHZ,  ETHx_RX_CTL_PIN_MUX,       0,       0,     1,        0,        0, },
    {   ETHx_RX_ER_PORT,   ETHx_RX_ER_PIN,      0,         CY_GPIO_DM_HIGHZ,   ETHx_RX_ER_PIN_MUX,       0,       0,     1,        0,        0, },
    { ETHx_REF_CLK_PORT, ETHx_REF_CLK_PIN,      0,         CY_GPIO_DM_HIGHZ, ETHx_REF_CLK_PIN_MUX,       0,       0,     1,        0,        0, },
    {     ETHx_MDC_PORT,     ETHx_MDC_PIN,      0, CY_GPIO_DM_STRONG_IN_OFF,     ETHx_MDC_PIN_MUX,       0,       0,     1,        0,        0, },
    {    ETHx_MDIO_PORT,    ETHx_MDIO_PIN,      0,        CY_GPIO_DM_STRONG,    ETHx_MDIO_PIN_MUX,       0,       0,     1,        0,        0, },
};

/** Wrapper configuration   */
const cy_str_ethif_wrapper_config stcWrapperConfig = {
#if ((EMAC_INTERFACE == EMAC_RMII) && (EMAC_LINKSPEED == ETH_LINKSPEED_10))
    .stcInterfaceSel = CY_ETHIF_CTL_RMII_10,      /** 10 Mbps RMII */
#elif ((EMAC_INTERFACE == EMAC_RMII) && (EMAC_LINKSPEED == ETH_LINKSPEED_100))
    .stcInterfaceSel = CY_ETHIF_CTL_RMII_100,     /** 100 Mbps RMII */
#else
    #error "Error in configuration"
    .stcInterfaceSel = CY_ETHIF_CTL_MII_10,
#endif
    .bRefClockSource = CY_ETHIF_EXTERNAL_HSIO,  /** assigning Ref_Clk to HSIO Clock, it is recommended to use external clock coming from HSIO  */
    .u8RefClkDiv = 1u,                          /** RefClk: 50MHz, Divide Refclock by 2 to have 25 MHz tx clock  */
};

/** Default Timer register values   */
const CEDI_1588TimerVal stc1588TimerValue = {
    .secsUpper = 0u,
    .secsLower = 0ul,
    .nanosecs  = 0ul,
};

/** TSU configuration   */
CEDI_TimerIncrement stc1588TimerInc; // Will be updated in runtime
const cy_str_ethif_TSU_Config stcTSUConfig = {
    .pstcTimerValue         = (CEDI_1588TimerVal*)&stc1588TimerValue,    /** start value for the counter     */
    .pstcTimerIncValue      = &stc1588TimerInc,
    .bOneStepTxSyncEnable   = false,                 /** useful when PTP protocol is in place    */
    .enTxDescStoreTimeStamp = CEDI_TX_TS_DISABLED,
    .enRxDescStoreTimeStamp = CEDI_RX_TS_DISABLED,
    .bStoreNSinRxDesc       = false,
};

/** General Ethernet configuration  */
const cy_stc_ethif_configuration_t stcENETConfig = {
    .bintrEnable         = true,                      /** Interrupt enable  */
    .dmaDataBurstLen     = CEDI_DMA_DBUR_LEN_4,
    .u8dmaCfgFlags       = CEDI_CFG_DMA_FRCE_TX_BRST,
    .mdcPclkDiv          = CEDI_MDC_DIV_BY_48,        /** source clock is 80 MHz and MDC must be less than 2.5MHz   */
    .u8rxLenErrDisc      = 0u,                        /** Length error frame not discarded  */
    .u8disCopyPause      = 0u,
    .u8chkSumOffEn       = 0u,                        /** Checksum for both Tx and Rx disabled    */
    .u8rx1536ByteEn      = 1u,                        /** Enable receive frame up to 1536    */
    .u8rxJumboFrEn       = 0u,
    .u8enRxBadPreamble   = 1u,
    .u8ignoreIpgRxEr     = 0u,
    .u8storeUdpTcpOffset = 0u,
    .u8aw2wMaxPipeline   = 2u,                        /** Value must be > 0   */
    .u8ar2rMaxPipeline   = 2u,                        /** Value must be > 0   */
    .u8pfcMultiQuantum   = 0u,
    .pstcWrapperConfig   = (cy_str_ethif_wrapper_config*)&stcWrapperConfig,
    .pstcTSUConfig       = (cy_str_ethif_TSU_Config*)&stcTSUConfig, /** TSU settings    */
    .btxq0enable         = true,                      /** Tx Q0 Enabled   */
    .btxq1enable         = false,                     /** Tx Q1 Disabled  */
    .btxq2enable         = false,                     /** Tx Q2 Disabled  */
    .brxq0enable         = true,                      /** Rx Q0 Enabled   */
    .brxq1enable         = false,                     /** Rx Q1 Disabled  */
    .brxq2enable         = false,                     /** Rx Q2 Disabled  */
};

/** Interrupt configurations    */
const cy_stc_ethif_interruptconfig_t stcInterruptConfig = {
    .btsu_time_match        = false, /** Time stamp unit time match event */
    .bwol_rx                = false, /** Wake on LAN event received */
    .blpi_ch_rx             = false, /** LPI indication status bit change received */
    .btsu_sec_inc           = true,  /** TSU seconds register increment */
    .bptp_tx_pdly_rsp       = false, /** PTP pdelay_resp frame transmitted */
    .bptp_tx_pdly_req       = false, /** PTP pdelay_req frame transmitted */
    .bptp_rx_pdly_rsp       = false, /** PTP pdelay_resp frame received */
    .bptp_rx_pdly_req       = false, /** PTP pdelay_req frame received */
    .bptp_tx_sync           = false, /** PTP sync frame transmitted */
    .bptp_tx_dly_req        = false, /** PTP delay_req frame transmitted */
    .bptp_rx_sync           = false, /** PTP sync frame received */
    .bptp_rx_dly_req        = false, /** PTP delay_req frame received */
    .bext_intr              = false, /** External input interrupt detected */
    .bpause_frame_tx        = false, /** Pause frame transmitted */
    .bpause_time_zero       = false, /** Pause time reaches zero or zeroq pause frame received */
    .bpause_nz_qu_rx        = false, /** Pause frame with non-zero quantum received */
    .bhresp_not_ok          = false, /** DMA hresp not OK */
    .brx_overrun            = false, /** Rx overrun error */
    .bpcs_link_change_det   = false, /** Link status change detected by PCS */
    .btx_complete           = true,  /** Frame has been transmitted successfully */
    .btx_fr_corrupt         = false, /** Tx frame corruption */
    .btx_retry_ex_late_coll = false, /** Retry limit exceeded or late collision */
    .btx_underrun           = false, /** Tx underrun */
    .btx_used_read          = false, /** Used bit set has been read in Tx descriptor list */
    .brx_used_read          = false, /** Used bit set has been read in Rx descriptor list */
    .brx_complete           = true,  /** Frame received successfully and stored */
    .bman_frame             = false, /** Management Frame Sent */

    /** call back functions  */
    .rxframecb      = Ethx_RxFrameCB,
    .txerrorcb      = NULL,
    .txcompletecb   = Ethx_TxFrameSuccessful, /** Set it to NULL, if do not wish to have callback */
    .tsuSecondInccb = Ethx_TSUIncrement,
    .tsuCompMatchcb = NULL,
};

/** Enable Ethernet Interrupts  */
const cy_stc_sysint_irq_t irq_cfg_ethx_q0 = {.sysIntSrc  = ETH_INTR_SRC,    .intIdx  = ETH_INTR_INDEX, .isEnabled  = true};
const cy_stc_sysint_irq_t irq_cfg_ethx_q1 = {.sysIntSrc  = ETH_INTR_SRC_Q1, .intIdx  = ETH_INTR_INDEX, .isEnabled  = true};
const cy_stc_sysint_irq_t irq_cfg_ethx_q2 = {.sysIntSrc  = ETH_INTR_SRC_Q2, .intIdx  = ETH_INTR_INDEX, .isEnabled  = true};

const uint8_t gMAC_SRC_ADDR[6] = {0xAA,0xAA,0xAA,0xAA,0xAA,0xAA};
const uint8_t gMAC_DST_ADDR[6] = {0xBB,0xBB,0xBB,0xBB,0xBB,0xBB};

#pragma pack(1)
typedef struct
{
    uint8_t  destMacAddr[6];
    uint8_t  srcMacAddr[6];
    uint16_t frameType;
} stc_eth_header_t;
#pragma pack()

int main(void)
{
    SystemInit();
    __enable_irq();

    /** Configure Ethernet Port pins    */
    Cy_GPIO_Pin_Init(USER_LED_PORT, USER_LED_PIN, &gLedPortGfg);
    Cy_GPIO_Multi_Pin_Init(gEtherPorts, (sizeof(gEtherPorts)/sizeof(gEtherPorts[0])));

    /** Init ethernet interrupts    */
    Cy_SysInt_InitIRQ(&irq_cfg_ethx_q0);
    Cy_SysInt_SetSystemIrqVector(irq_cfg_ethx_q0.sysIntSrc, Cy_Ethx_InterruptHandler);

    Cy_SysInt_InitIRQ(&irq_cfg_ethx_q1);
    Cy_SysInt_SetSystemIrqVector(irq_cfg_ethx_q1.sysIntSrc, Cy_Ethx_InterruptHandler);

    Cy_SysInt_InitIRQ(&irq_cfg_ethx_q2);
    Cy_SysInt_SetSystemIrqVector(irq_cfg_ethx_q2.sysIntSrc, Cy_Ethx_InterruptHandler);

    uint32_t priority = 0ul;
    NVIC_SetPriority(ETH_INTR_INDEX, priority);
    NVIC_EnableIRQ(ETH_INTR_INDEX);

    /** Initialize ENET MAC */
    uint32_t ClockFreqOfTSU;
    Cy_SysClk_GetHfClkFrequency(HF_CLK_IDX_TO_TSU, &ClockFreqOfTSU);
    CalcTSUIncValue(ClockFreqOfTSU, &stc1588TimerInc);
    CY_ASSERT(Cy_EthIf_Init(ETH_REG_BASE, (cy_stc_ethif_configuration_t*)&stcENETConfig, (cy_stc_ethif_interruptconfig_t*)&stcInterruptConfig) == CY_ETHIF_SUCCESS);
    Cy_EthIf_SetPhysAddr(ETH_REG_BASE, (uint8_t*)gMAC_SRC_ADDR);

    /** PHY initialization  */
    InitPHY_DP83848Q_Q1();
    while(Check_DP83848Q_Q1_LinkStatus() == false);

    /** Send a message   */
    /* Get a pointer to the tx buffer which has been allocated the driver */
    uint8_t bufferIdx;
    uint8_t* pTxBuffer;
    CY_ASSERT(Cy_EthIf_ProvideTxBuffer(&bufferIdx, &pTxBuffer) == CY_ETHIF_SUCCESS);

    /* Copy an original message to the top of the tx buffer */
    int8_t message[] = "Hello, This is TraveoII Ethernet Test!";
    memcpy(pTxBuffer, message, sizeof(message));

    /* Start transmission */
    uint16_t frameType = 0xCCDD; // dummy
    uint8_t qIdx = 0;
    CY_ASSERT(Cy_EthIf_TransmitTrigger(ETH_REG_BASE, bufferIdx, sizeof(message), qIdx, frameType, (uint8_t*)gMAC_DST_ADDR) == CY_ETHIF_SUCCESS);

    for(;;);
}

/** Interrupt Handlers for Ethernet 1     */
static void Cy_Ethx_InterruptHandler (void)
{
    Cy_EthIf00_DecodeEvent();
}

// EMAC *****************
static void Ethx_RxFrameCB(cy_pstc_eth_type_t pstcEth, uint8_t * u8RxBuffer, uint32_t u32Length)
{
    /* Get a pointer to the tx buffer which has been allocated the driver */
    uint8_t bufferIdx;
    uint8_t* pTxBuffer;
    CY_ASSERT(Cy_EthIf_ProvideTxBuffer(&bufferIdx, &pTxBuffer) == CY_ETHIF_SUCCESS);

    /* Copy an original message to the top of the tx buffer */
    int8_t message[] = "Hello, TVII sent back a received message: ";
    uint32_t orglMsgLen = sizeof(message);
    memcpy(pTxBuffer, message, orglMsgLen);

    /* Copy the received payload to the tx buffer */
    uint32_t payloadLen = u32Length - sizeof(stc_eth_header_t);
    stc_eth_header_t* pRxPacket = (stc_eth_header_t*)u8RxBuffer;
    uint8_t* pPayLoad = (uint8_t*)((uint32_t)u8RxBuffer + sizeof(stc_eth_header_t));
    uint32_t maxLength = CY_ETH_SIZE_MAX_FRAME - sizeof(stc_eth_header_t) - ETH_FRAME_SIZE_FCS - sizeof(orglMsgLen);
    uint32_t copyLen = (payloadLen < maxLength) ? payloadLen : maxLength;
    memcpy(&pTxBuffer[orglMsgLen], pPayLoad, copyLen);

    /* Start transmission */
    uint8_t qIdx = 0;
    CY_ASSERT(Cy_EthIf_TransmitTrigger(ETH_REG_BASE, bufferIdx, (orglMsgLen + copyLen), qIdx, pRxPacket->frameType, pRxPacket->srcMacAddr) == CY_ETHIF_SUCCESS);
}

static void Ethx_TxFrameSuccessful( cy_pstc_eth_type_t pstcEth, uint8_t u8QueueIndex, uint32_t bufIdx)
{
    // Do nothing
}

static void Ethx_TSUIncrement(cy_pstc_eth_type_t pstcEth)
{
    Cy_GPIO_Inv(USER_LED_PORT, USER_LED_PIN);
}


/*******************************************************************************
* Function Name: InitPHY_DP83848Q_Q1
****************************************************************************//**
*
* \brief Dedicated to initialize PHY DP83848Q_Q1
*
* \Note:
* If hardware consist of any other PHY than DP83848Q_Q1, dedicated function shall be written to configure relevant 
* registers in side PHY
*******************************************************************************/
static void InitPHY_DP83848Q_Q1(void)
{
    volatile uint32_t    u32ReadData = 0;

    /** Reg PHYIDR1/2 :Read PHY Identifier to check connection */
    un_dp83848q_q1_phyidr1_t r_phyidr1 = {0};
    r_phyidr1.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_PHYIDR1, PHY_ADDR);
    CY_ASSERT(r_phyidr1.f.u16OUI_MSB == PHY_OUI_MSB);

    un_dp83848q_q1_phyidr2_t r_phyidr2 = {0};
    r_phyidr2.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_PHYIDR2, PHY_ADDR);
    CY_ASSERT(r_phyidr2.f.u6OUI_LSB  == PHY_OUI_LSB);
    CY_ASSERT(r_phyidr2.f.u6VNDR_MDL == PHY_VNDR_MDL);
    CY_ASSERT(r_phyidr2.f.u4MDL_REV  == PHY_MDL_REV);

    /** Reg BMCR: Perform a full reset, including All registers   */
    un_dp83848q_q1_bmcr_t wr_bmcr = {0};
    wr_bmcr.f.u1RESET             = 1;
    Cy_EthIf_PhyRegWrite(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_BMCR, wr_bmcr.u16, PHY_ADDR);
    while(1)
    {
        wr_bmcr.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_BMCR, PHY_ADDR);
        if(wr_bmcr.f.u1RESET == 0)
        {
            break;
        }
    }

    /** Reg BMCR: 10M/100M, Full Duplex and Auto Negotiation OFF  */
    wr_bmcr.u16                         = 0u;                        // clear temporary variable
    wr_bmcr.f.u1SPEED_SELECTION         = SPEED_SELECTION_BIT_VALUE; // speed
    wr_bmcr.f.u1AUTO_NEGOTIATION_ENABLE = 0u;                        // Auto-negotiation off
    wr_bmcr.f.u1DUPLEX_MODE             = 1u;                        // Full-Duplex mode
    Cy_EthIf_PhyRegWrite(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_BMCR, wr_bmcr.u16, PHY_ADDR);

    wr_bmcr.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_BMCR, PHY_ADDR);
    CY_ASSERT(wr_bmcr.f.u1DUPLEX_MODE     == 1u);
    CY_ASSERT(wr_bmcr.f.u1SPEED_SELECTION == SPEED_SELECTION_BIT_VALUE);

    /** Reg RBR: RMII and Bypass Register  */
    un_dp83848q_q1_rbr_t wr_rbr = {0};
    wr_rbr.f.u1RMII_MODE        = 1u;
    Cy_EthIf_PhyRegWrite(ETH_REG_BASE, DP83848Q_Q1_REG_EXTENDED_RBR, wr_rbr.u16, PHY_ADDR);

    wr_rbr.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_EXTENDED_RBR, PHY_ADDR);
    CY_ASSERT(wr_rbr.f.u1RMII_MODE == 1u);
}

/*******************************************************************************
* Function Name: Check_DP83848Q_Q1_LinkStatus
****************************************************************************//**
*
* \brief Function reads specific register from DP83867IR to learn Link status
*
* \param 
* \return true Link up
* \return false Link Down 
*
*******************************************************************************/
static bool Check_DP83848Q_Q1_LinkStatus(void)
{
    un_dp83848q_q1_bmsr_t r_bmsr = {0};
    r_bmsr.u16 = Cy_EthIf_PhyRegRead(ETH_REG_BASE, DP83848Q_Q1_REG_NORMAL_BMSR, PHY_ADDR);
    if(r_bmsr.f.u1LINK_STATUS == 1u)
    {
        return true;  /** Link up */
    }
    else
    {
        return false; /** Link down   */
    }
}

/*******************************************************************************
* Function Name: CalcTSUIncValue
****************************************************************************//**
*
* \brief Calculate TSU Inc Value from source clock frequency of the TSU
*
* \param sourceFreq frequency of source clock of the TSU
* \param incValue pointer to the result 
*
*******************************************************************************/
static void CalcTSUIncValue(uint32_t sourceFreq, CEDI_TimerIncrement* incValue)
{
    uint64_t secInNamo = 1000000000ull;
    uint64_t fractionBitCount = 24; // 24 bit fraction

    uint64_t incValueWithfixedPoint = (secInNamo << fractionBitCount)  / (uint64_t)sourceFreq;
    incValue->nanoSecsInc = (uint32_t)((incValueWithfixedPoint & 0xFFFFFFFFF000000ull) >> fractionBitCount);
    incValue->subNsInc    = (uint16_t)((incValueWithfixedPoint & 0x000000000FFFF00ull) >> 8ull);
    incValue->lsbSubNsInc = (uint8_t)(incValueWithfixedPoint & 0x0000000000000FFull);
    incValue->altIncCount = 0; // Don't care for now
    incValue->altNanoSInc = 0; // Don't care for now
}

/* [] END OF FILE */
