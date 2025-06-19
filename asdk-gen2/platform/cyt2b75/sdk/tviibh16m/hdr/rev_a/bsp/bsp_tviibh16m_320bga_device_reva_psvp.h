
/***************************************************************************//**
* \file :
* bsp_tviibh16m_320bga_device_reva0_psvp.h
* 
* \brief
* Project specific header
* 
* \note
* !!! Auto generated file and should not be modified !!!
* Generator revision: 1.9.4
* 
********************************************************************************
* \copyright
* Copyright 2016-2021, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef BSP_TVIIBH16M_320BGA_DEVICE_REVA_PSVP_H
#define BSP_TVIIBH16M_320BGA_DEVICE_REVA_PSVP_H

#ifdef __cplusplus
extern C {
#endif

/******************************************************************************/
/******************************************************************************/


#if (defined(CYT6BJCDHE)) && (CY_USE_PSVP == 1)

#define CY_CANFD0_CH1_AVAILABLE
#define CY_LIN0_CH0_AVAILABLE
#define CY_SMIF0_AVAILABLE
#define CY_ETH1_AVAILABLE

/******************************************************************************/
/* GPIO                                                                       */
/******************************************************************************/

/**** LED ****/

#define CY_AB_LED_PORT                 	GPIO_PRT5
#define CY_AB_LED_PIN                   4
#define CY_AB_LED_PIN_MUX               P5_4_GPIO

#define CY_LED0_PORT   			        GPIO_PRT4
#define CY_LED0_PIN    			        0
#define CY_LED0_PIN_MUX			        P4_0_GPIO
#define CY_LED0_IRQN   			        ioss_interrupts_gpio_dpslp_4_IRQn

#define CY_LED1_PORT   			        GPIO_PRT29
#define CY_LED1_PIN    			        2
#define CY_LED1_PIN_MUX			        P29_2_GPIO
#define CY_LED1_IRQN   			        ioss_interrupts_gpio_dpslp_29_IRQn

#define CY_LED2_PORT   			        GPIO_PRT29
#define CY_LED2_PIN    			        0
#define CY_LED2_PIN_MUX			        P29_0_GPIO
#define CY_LED2_IRQN   			        ioss_interrupts_gpio_dpslp_29_IRQn

#define CY_LED3_PORT   			        GPIO_PRT3
#define CY_LED3_PIN    			        2
#define CY_LED3_PIN_MUX			        P3_2_GPIO
#define CY_LED3_IRQN   			        ioss_interrupts_gpio_dpslp_3_IRQn

#define CY_LED4_PORT   			        GPIO_PRT3
#define CY_LED4_PIN    			        0
#define CY_LED4_PIN_MUX			        P3_0_GPIO
#define CY_LED4_IRQN   			        ioss_interrupts_gpio_dpslp_3_IRQn

#define CY_LED7_PORT   		            GPIO_PRT0
#define CY_LED7_PIN    		            0
#define CY_LED7_PIN_MUX		            P0_0_GPIO
#define CY_LED7_IRQN   		            ioss_interrupts_gpio_dpslp_0_IRQn

#define CY_LED8_PORT   			        GPIO_PRT28
#define CY_LED8_PIN    			        7
#define CY_LED8_PIN_MUX			        P28_7_GPIO
#define CY_LED8_IRQN   			        ioss_interrupts_gpio_dpslp_28_IRQn

#define CY_LED9_PORT   			        GPIO_PRT6
#define CY_LED9_PIN    			        1
#define CY_LED9_PIN_MUX			        P6_1_GPIO
#define CY_LED9_IRQN   			        ioss_interrupts_gpio_dpslp_6_IRQn


/**** BUTTON ****/
#define CY_BUTTON5_PORT   		        GPIO_PRT4
#define CY_BUTTON5_PIN    		        1
#define CY_BUTTON5_PIN_MUX		        P4_1_GPIO
#define CY_BUTTON5_IRQN   		        ioss_interrupts_gpio_dpslp_4_IRQn

#define CY_BUTTON4_PORT   		        GPIO_PRT4
#define CY_BUTTON4_PIN    		        5
#define CY_BUTTON4_PIN_MUX		        P4_5_GPIO
#define CY_BUTTON4_IRQN   		        ioss_interrupts_gpio_dpslp_4_IRQn

#define CY_BUTTON3_PORT   		        GPIO_PRT30
#define CY_BUTTON3_PIN    		        1
#define CY_BUTTON3_PIN_MUX		        P30_1_GPIO
#define CY_BUTTON3_IRQN   		        ioss_interrupts_gpio_dpslp_30_IRQn

#define CY_BUTTON2_PORT   		        GPIO_PRT5
#define CY_BUTTON2_PIN    		        0
#define CY_BUTTON2_PIN_MUX		        P5_0_GPIO
#define CY_BUTTON2_IRQN   		        ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BUTTON1_PORT   		        GPIO_PRT5
#define CY_BUTTON1_PIN    		        3
#define CY_BUTTON1_PIN_MUX		        P5_3_GPIO
#define CY_BUTTON1_IRQN   		        ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_AB_BUTTON_PORT              	GPIO_PRT5
#define CY_AB_BUTTON_PIN                1
#define CY_AB_BUTTON_PIN_MUX            P5_1_GPIO
#define CY_AB_BUTTON_IRQN		        ioss_interrupts_gpio_dpslp_5_IRQn

/******************************************************************************/
/* CAN                                                                        */
/******************************************************************************/

#define CY_CANFD0_TYPE   				CANFD0_CH1

#define CY_CANFD0_TX_PORT   			GPIO_PRT0
#define CY_CANFD0_TX_PIN    			2
#define CY_CANFD0_TX_PIN_MUX			P0_2_CANFD0_TTCAN_TX1

#define CY_CANFD0_RX_PORT   			GPIO_PRT0
#define CY_CANFD0_RX_PIN    			3
#define CY_CANFD0_RX_PIN_MUX			P0_3_CANFD0_TTCAN_RX1

#define CY_CANFD0_PCLK   				PCLK_CANFD0_CLOCK_CAN1
#define CY_CANFD0_IRQN   				canfd_0_interrupts0_1_IRQn

/******************************************************************************/
/* LIN                                                                        */
/******************************************************************************/

#define CY_LINCH0_TYPE   				LIN0_CH0

#define CY_LINCH0_TX_PORT   			GPIO_PRT2
#define CY_LINCH0_TX_PIN    			1
#define CY_LINCH0_TX_PIN_MUX			P2_1_LIN0_LIN_TX0

#define CY_LINCH0_RX_PORT   			GPIO_PRT21
#define CY_LINCH0_RX_PIN    			5
#define CY_LINCH0_RX_PIN_MUX			P21_5_LIN0_LIN_RX0

#define CY_LINCH0_EN_PORT   			GPIO_PRT2
#define CY_LINCH0_EN_PIN    			2
#define CY_LINCH0_EN_PIN_MUX			P2_2_LIN0_LIN_EN0

#define CY_LINCH0_PCLK   				PCLK_LIN0_CLOCK_CH_EN0
#define CY_LINCH0_IRQN   				lin_0_interrupts_0_IRQn

/******************************************************************************/
/* SAR                                                                        */
/******************************************************************************/

#define CY_ADC_POT_TYPE   	            PASS0_SAR0
#define CY_ADC_POT_CHN    	            0
#define CY_ADC_POT_PORT   				GPIO_PRT6
#define CY_ADC_POT_PIN    				0
#define CY_ADC_POT_PIN_MUX				P6_0_GPIO
#define CY_ADC_POT_PCLK   				PCLK_PASS0_CLOCK_SAR0
#define CY_ADC_POT_IRQN   				pass_0_interrupts_sar_0_IRQn


/******************************************************************************/
/* AUDIOSS                                                                    */
/******************************************************************************/

#define CY_AUDIOSS_TX_SCK_PORT   		GPIO_PRT11
#define CY_AUDIOSS_TX_SCK_PIN    		1
#define CY_AUDIOSS_TX_SCK_PIN_MUX		P11_1_AUDIOSS0_TX_SCK

#define CY_AUDIOSS_TX_WS_PORT   		GPIO_PRT11
#define CY_AUDIOSS_TX_WS_PIN    		2
#define CY_AUDIOSS_TX_WS_PIN_MUX		P11_2_AUDIOSS0_TX_WS

#define CY_AUDIOSS_TX_SDO_PORT   		GPIO_PRT12
#define CY_AUDIOSS_TX_SDO_PIN    		0
#define CY_AUDIOSS_TX_SDO_PIN_MUX		P12_0_AUDIOSS0_TX_SDO

#define CY_AUDIOSS_RX_SCK_PORT   		GPIO_PRT12
#define CY_AUDIOSS_RX_SCK_PIN    		2
#define CY_AUDIOSS_RX_SCK_PIN_MUX		P12_2_AUDIOSS0_RX_SCK

#define CY_AUDIOSS_RX_WS_PORT   		GPIO_PRT12
#define CY_AUDIOSS_RX_WS_PIN    		3
#define CY_AUDIOSS_RX_WS_PIN_MUX		P12_3_AUDIOSS0_RX_WS

#define CY_AUDIOSS_RX_SDI_PORT   		GPIO_PRT12
#define CY_AUDIOSS_RX_SDI_PIN    		4
#define CY_AUDIOSS_RX_SDI_PIN_MUX		P12_4_AUDIOSS0_RX_SDI

#define CY_AUDIOSS_IRQN   				audioss_0_interrupt_i2s_IRQn

#define CY_AUDIOSS_SPI_TYPE             SCB1

#define CY_AUDIOSS_SPI_MISO_PORT        GPIO_PRT20
#define CY_AUDIOSS_SPI_MISO_PIN         3
#define CY_AUDIOSS_SPI_MISO_PIN_MUX     P20_3_SCB1_SPI_MISO

#define CY_AUDIOSS_SPI_MOSI_PORT        GPIO_PRT20
#define CY_AUDIOSS_SPI_MOSI_PIN         4
#define CY_AUDIOSS_SPI_MOSI_PIN_MUX     P20_4_SCB1_SPI_MOSI

#define CY_AUDIOSS_SPI_CLK_PORT         GPIO_PRT20
#define CY_AUDIOSS_SPI_CLK_PIN          5
#define CY_AUDIOSS_SPI_CLK_PIN_MUX      P20_5_SCB1_SPI_CLK

#define CY_AUDIOSS_SPI_SS_PORT          GPIO_PRT20
#define CY_AUDIOSS_SPI_SS_PIN           6
#define CY_AUDIOSS_SPI_SS_PIN_MUX       P20_6_SCB1_SPI_SELECT0

#define CY_AUDIOSS_SPI_PCLK             PCLK_SCB1_CLOCK
#define CY_AUDIOSS_SPI_IRQN             scb_1_interrupt_IRQn

#define CY_AUDIOSS_SPI_SS               0

/******************************************************************************/
/* ETH                                                                        */
/******************************************************************************/

#define CY_AUTO_ETH_TYPE   				ETH1

#define CY_AUTO_ETH_REF_CLK_PORT   		GPIO_PRT26
#define CY_AUTO_ETH_REF_CLK_PIN    		0
#define CY_AUTO_ETH_REF_CLK_PIN_MUX		P26_0_ETH1_REF_CLK

#define CY_AUTO_ETH_TX_CTL_PORT   		GPIO_PRT26
#define CY_AUTO_ETH_TX_CTL_PIN    		1
#define CY_AUTO_ETH_TX_CTL_PIN_MUX		P26_1_ETH1_TX_CTL

#define CY_AUTO_ETH_TX_CLK_PORT   		GPIO_PRT26
#define CY_AUTO_ETH_TX_CLK_PIN    		2
#define CY_AUTO_ETH_TX_CLK_PIN_MUX		P26_2_ETH1_TX_CLK

#define CY_AUTO_ETH_TD0_PORT   			GPIO_PRT26
#define CY_AUTO_ETH_TD0_PIN    			3
#define CY_AUTO_ETH_TD0_PIN_MUX			P26_3_ETH1_TXD0

#define CY_AUTO_ETH_TD1_PORT   			GPIO_PRT26
#define CY_AUTO_ETH_TD1_PIN    			4
#define CY_AUTO_ETH_TD1_PIN_MUX			P26_4_ETH1_TXD1

#define CY_AUTO_ETH_TD2_PORT   			GPIO_PRT26
#define CY_AUTO_ETH_TD2_PIN    			5
#define CY_AUTO_ETH_TD2_PIN_MUX			P26_5_ETH1_TXD2

#define CY_AUTO_ETH_TD3_PORT   			GPIO_PRT26
#define CY_AUTO_ETH_TD3_PIN    			6
#define CY_AUTO_ETH_TD3_PIN_MUX			P26_6_ETH1_TXD3

#define CY_AUTO_ETH_RD0_PORT   			GPIO_PRT26
#define CY_AUTO_ETH_RD0_PIN    			7
#define CY_AUTO_ETH_RD0_PIN_MUX			P26_7_ETH1_RXD0

#define CY_AUTO_ETH_RD1_PORT   			GPIO_PRT27
#define CY_AUTO_ETH_RD1_PIN    			0
#define CY_AUTO_ETH_RD1_PIN_MUX			P27_0_ETH1_RXD1

#define CY_AUTO_ETH_RD2_PORT   			GPIO_PRT27
#define CY_AUTO_ETH_RD2_PIN    			1
#define CY_AUTO_ETH_RD2_PIN_MUX			P27_1_ETH1_RXD2

#define CY_AUTO_ETH_RD3_PORT   			GPIO_PRT27
#define CY_AUTO_ETH_RD3_PIN    			2
#define CY_AUTO_ETH_RD3_PIN_MUX			P27_2_ETH1_RXD3

#define CY_AUTO_ETH_RX_CTL_PORT   		GPIO_PRT27
#define CY_AUTO_ETH_RX_CTL_PIN    		3
#define CY_AUTO_ETH_RX_CTL_PIN_MUX		P27_3_ETH1_RX_CTL

#define CY_AUTO_ETH_RX_CLK_PORT   		GPIO_PRT27
#define CY_AUTO_ETH_RX_CLK_PIN    		4
#define CY_AUTO_ETH_RX_CLK_PIN_MUX		P27_4_ETH1_RX_CLK

#define CY_AUTO_ETH_MDC_PORT   			GPIO_PRT27
#define CY_AUTO_ETH_MDC_PIN    			6
#define CY_AUTO_ETH_MDC_PIN_MUX			P27_6_ETH1_MDC

#define CY_AUTO_ETH_MDIO_PORT   		GPIO_PRT27           
#define CY_AUTO_ETH_MDIO_PIN    		5                    
#define CY_AUTO_ETH_MDIO_PIN_MUX		P27_5_ETH1_MDIO      

#define CY_AUTO_ETH_TXER_PORT   		GPIO_PRT33
#define CY_AUTO_ETH_TXER_PIN    		1
#define CY_AUTO_ETH_TXER_PIN_MUX		P33_1_ETH1_TX_ER

#define CY_AUTO_ETH_RX_ER_PORT   		GPIO_PRT34
#define CY_AUTO_ETH_RX_ER_PIN    		3
#define CY_AUTO_ETH_RX_ER_PIN_MUX		P34_3_ETH1_RX_ER

#define CY_AUTO_ETH_IRQN0   			eth_1_interrupt_eth_0_IRQn
#define CY_AUTO_ETH_IRQN1   			eth_1_interrupt_eth_1_IRQn
#define CY_AUTO_ETH_IRQN2   			eth_1_interrupt_eth_2_IRQn


/******************************************************************************/
/* SDHC                                                                       */
/******************************************************************************/

#define CY_SDHC_TYPE   					SDHC0

#define CY_SDHC_WP_PORT   				GPIO_PRT6
#define CY_SDHC_WP_PIN    				2
#define CY_SDHC_WP_PIN_MUX				P6_2_SDHC0_CARD_MECH_WRITE_PROT

#define CY_SDHC_CARD_DETECT_N_PORT   	GPIO_PRT24
#define CY_SDHC_CARD_DETECT_N_PIN    	0
#define CY_SDHC_CARD_DETECT_N_PIN_MUX	P24_0_SDHC0_CARD_DETECT_N

#define CY_SDHC_CLK_CARD_PORT   		GPIO_PRT24
#define CY_SDHC_CLK_CARD_PIN    		2
#define CY_SDHC_CLK_CARD_PIN_MUX		P24_2_SDHC0_CLK_CARD

#define CY_SDHC_CARD_CMD_PORT   		GPIO_PRT24
#define CY_SDHC_CARD_CMD_PIN    		3
#define CY_SDHC_CARD_CMD_PIN_MUX		P24_3_SDHC0_CARD_CMD

#define CY_SDHC_CARD_DAT_3TO00_PORT   	GPIO_PRT25
#define CY_SDHC_CARD_DAT_3TO00_PIN    	0
#define CY_SDHC_CARD_DAT_3TO00_PIN_MUX	P25_0_SDHC0_CARD_DAT_3TO00

#define CY_SDHC_CARD_DAT_3TO01_PORT   	GPIO_PRT25
#define CY_SDHC_CARD_DAT_3TO01_PIN    	1
#define CY_SDHC_CARD_DAT_3TO01_PIN_MUX	P25_1_SDHC0_CARD_DAT_3TO01

#define CY_SDHC_CARD_DAT_3TO02_PORT   	GPIO_PRT25
#define CY_SDHC_CARD_DAT_3TO02_PIN    	2
#define CY_SDHC_CARD_DAT_3TO02_PIN_MUX	P25_2_SDHC0_CARD_DAT_3TO02

#define CY_SDHC_CARD_DAT_3TO03_PORT   	GPIO_PRT25
#define CY_SDHC_CARD_DAT_3TO03_PIN    	3
#define CY_SDHC_CARD_DAT_3TO03_PIN_MUX	P25_3_SDHC0_CARD_DAT_3TO03

#define CY_SDHC_GENERAL_IRQN   			sdhc_0_interrupt_general_IRQn

/******************************************************************************/
/* SMIF                                                                       */
/******************************************************************************/

#define CY_SMIF0_TYPE   				SMIF0

#define CY_SMIF0_CLK_PORT   			GPIO_PRT6
#define CY_SMIF0_CLK_PIN    			3
#define CY_SMIF0_CLK_PIN_MUX			P6_3_SMIF0_SPIHB_CLK

#define CY_SMIF0_RWDS_PORT   			GPIO_PRT6
#define CY_SMIF0_RWDS_PIN    			4
#define CY_SMIF0_RWDS_PIN_MUX			P6_4_SMIF0_SPIHB_RWDS

#define CY_SMIF0_SELECT0_PORT   		GPIO_PRT6
#define CY_SMIF0_SELECT0_PIN    		5
#define CY_SMIF0_SELECT0_PIN_MUX		P6_5_SMIF0_SPIHB_SELECT0

#define CY_SMIF0_SELECT1_PORT   		GPIO_PRT7
#define CY_SMIF0_SELECT1_PIN    		0
#define CY_SMIF0_SELECT1_PIN_MUX		P7_0_SMIF0_SPIHB_SELECT1

#define CY_SMIF0_DATA0_PORT   			GPIO_PRT7
#define CY_SMIF0_DATA0_PIN    			1
#define CY_SMIF0_DATA0_PIN_MUX			P7_1_SMIF0_SPIHB_DATA0

#define CY_SMIF0_DATA1_PORT   			GPIO_PRT7
#define CY_SMIF0_DATA1_PIN    			2
#define CY_SMIF0_DATA1_PIN_MUX			P7_2_SMIF0_SPIHB_DATA1

#define CY_SMIF0_DATA2_PORT   			GPIO_PRT7
#define CY_SMIF0_DATA2_PIN    			3
#define CY_SMIF0_DATA2_PIN_MUX			P7_3_SMIF0_SPIHB_DATA2

#define CY_SMIF0_DATA3_PORT   			GPIO_PRT7
#define CY_SMIF0_DATA3_PIN    			4
#define CY_SMIF0_DATA3_PIN_MUX			P7_4_SMIF0_SPIHB_DATA3

#define CY_SMIF0_DATA4_PORT   			GPIO_PRT7
#define CY_SMIF0_DATA4_PIN    			5
#define CY_SMIF0_DATA4_PIN_MUX			P7_5_SMIF0_SPIHB_DATA4

#define CY_SMIF0_DATA5_PORT   			GPIO_PRT8
#define CY_SMIF0_DATA5_PIN    			0
#define CY_SMIF0_DATA5_PIN_MUX			P8_0_SMIF0_SPIHB_DATA5

#define CY_SMIF0_DATA6_PORT   			GPIO_PRT8
#define CY_SMIF0_DATA6_PIN    			1
#define CY_SMIF0_DATA6_PIN_MUX			P8_1_SMIF0_SPIHB_DATA6

#define CY_SMIF0_DATA7_PORT   			GPIO_PRT8
#define CY_SMIF0_DATA7_PIN    			2
#define CY_SMIF0_DATA7_PIN_MUX			P8_2_SMIF0_SPIHB_DATA7

#define CY_SMIF0_IRQN   				smif_0_interrupt_IRQn

/******************************************************************************/
/* FLEXRAY                                                                    */
/******************************************************************************/

#define CY_FRA_TYPE   					FLEXRAY0_CH

#define CY_FRA_RXD_PORT   				GPIO_PRT10
#define CY_FRA_RXD_PIN    				2
#define CY_FRA_RXD_PIN_MUX				P10_2_FLEXRAY0_RXDA

#define CY_FRA_TXD_PORT   				GPIO_PRT10
#define CY_FRA_TXD_PIN    				3
#define CY_FRA_TXD_PIN_MUX				P10_3_FLEXRAY0_TXDA

#define CY_FRA_TXEN_PORT   				GPIO_PRT10
#define CY_FRA_TXEN_PIN    				4
#define CY_FRA_TXEN_PIN_MUX				P10_4_FLEXRAY0_TXENA_N

#define CY_FRA_PCLK   					PCLK_FLEXRAY0_CLK_FLEXRAY
#define CY_FRA_IRQN   					flexray_0_interrupt_IRQn


/******************************************************************************/
/******************************************************************************/

#else 	/* No Device or PSVP Selected */

    #error Choose correct device from the device specific Datasheet. 

#endif 	/* #if(defined(CYT6BJCCJS)) */


#ifdef __cplusplus
}
#endif


#endif 	/* BSP_TVIIBH16M_320BGA_DEVICE_REVA0_PSVP_H */


/* [] END OF FILE */
