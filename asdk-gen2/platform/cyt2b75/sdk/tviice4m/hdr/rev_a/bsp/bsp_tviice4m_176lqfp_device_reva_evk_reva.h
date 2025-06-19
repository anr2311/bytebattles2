
/***************************************************************************//**
* \file :
* bsp_tviice4m_176lqfp_device_reva_evk_reva.h
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

#ifndef BSP_TVIICE4M_176LQFP_DEVICE_REVA_EVK_REVA_H
#define BSP_TVIICE4M_176LQFP_DEVICE_REVA_EVK_REVA_H

#ifdef __cplusplus
extern C {
#endif

/******************************************************************************/
/******************************************************************************/


#if (defined(CYT2CL8BAS)) && (CY_USE_PSVP == 0)

#define CY_BSP_SMIF0_AVAILABLE
#define CY_BSP_TDM0_TDM_STRUCT0_AVAILABLE
#define CY_BSP_CANFD0_CH0_AVAILABLE
#define CY_BSP_LIN0_CH0_AVAILABLE
#define CY_BSP_CXPI0_CH0_AVAILABLE
#define CY_BSP_CANFD1_CH0_AVAILABLE
#define CY_BSP_CANFD1_CH1_AVAILABLE
#define CY_BSP_LIN0_CH1_AVAILABLE

/******************************************************************************/
/* GPIO                                                                       */
/******************************************************************************/


/**** LED ****/
#define CY_BSP_BB_USER_LED0_PORT   					GPIO_PRT10
#define CY_BSP_BB_USER_LED0_PIN    					2
#define CY_BSP_BB_USER_LED0_PIN_MUX					P10_2_GPIO
#define CY_BSP_BB_USER_LED0_IRQN   					ioss_interrupts_gpio_dpslp_10_IRQn

#define CY_BSP_BB_USER_LED1_PORT   					GPIO_PRT19
#define CY_BSP_BB_USER_LED1_PIN    					4
#define CY_BSP_BB_USER_LED1_PIN_MUX					P19_4_GPIO
#define CY_BSP_BB_USER_LED1_IRQN   					ioss_interrupts_gpio_dpslp_19_IRQn

#define CY_BSP_USER_LED1_PORT   					GPIO_PRT19
#define CY_BSP_USER_LED1_PIN    					5
#define CY_BSP_USER_LED1_PIN_MUX					P19_5_GPIO
#define CY_BSP_USER_LED1_IRQN   					ioss_interrupts_gpio_dpslp_19_IRQn

#define CY_BSP_USER_LED2_PORT   					GPIO_PRT5
#define CY_BSP_USER_LED2_PIN    					6
#define CY_BSP_USER_LED2_PIN_MUX					P5_6_GPIO
#define CY_BSP_USER_LED2_IRQN   					ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BSP_BB_USER_LED2_PORT   					GPIO_PRT5
#define CY_BSP_BB_USER_LED2_PIN    					7
#define CY_BSP_BB_USER_LED2_PIN_MUX					P5_7_GPIO
#define CY_BSP_BB_USER_LED2_IRQN   					ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BSP_BB_USER_LED3_PORT   					GPIO_PRT6
#define CY_BSP_BB_USER_LED3_PIN    					4
#define CY_BSP_BB_USER_LED3_PIN_MUX					P6_4_GPIO
#define CY_BSP_BB_USER_LED3_IRQN   					ioss_interrupts_gpio_dpslp_6_IRQn

#define CY_BSP_BB_USER_LED4_PORT   					GPIO_PRT6
#define CY_BSP_BB_USER_LED4_PIN    					5
#define CY_BSP_BB_USER_LED4_PIN_MUX					P6_5_GPIO
#define CY_BSP_BB_USER_LED4_IRQN   					ioss_interrupts_gpio_dpslp_6_IRQn

#define CY_BSP_BB_USER_LED5_PORT   					GPIO_PRT7
#define CY_BSP_BB_USER_LED5_PIN    					0
#define CY_BSP_BB_USER_LED5_PIN_MUX					P7_0_GPIO
#define CY_BSP_BB_USER_LED5_IRQN   					ioss_interrupts_gpio_dpslp_7_IRQn

#define CY_BSP_BB_USER_LED6_PORT   					GPIO_PRT7
#define CY_BSP_BB_USER_LED6_PIN    					3
#define CY_BSP_BB_USER_LED6_PIN_MUX					P7_3_GPIO
#define CY_BSP_BB_USER_LED6_IRQN   					ioss_interrupts_gpio_dpslp_7_IRQn

#define CY_BSP_BB_USER_LED7_PORT   					GPIO_PRT7
#define CY_BSP_BB_USER_LED7_PIN    					4
#define CY_BSP_BB_USER_LED7_PIN_MUX					P7_4_GPIO
#define CY_BSP_BB_USER_LED7_IRQN   					ioss_interrupts_gpio_dpslp_7_IRQn


/**** BUTTON ****/
#define CY_BSP_USER_BUTTON1_PORT   					GPIO_PRT19
#define CY_BSP_USER_BUTTON1_PIN    					6
#define CY_BSP_USER_BUTTON1_PIN_MUX					P19_6_GPIO
#define CY_BSP_USER_BUTTON1_IRQN   					ioss_interrupts_gpio_dpslp_19_IRQn

#define CY_BSP_HIB_WAKE_BUTTON5_PORT   				GPIO_PRT2
#define CY_BSP_HIB_WAKE_BUTTON5_PIN    				5
#define CY_BSP_HIB_WAKE_BUTTON5_PIN_MUX				P2_5_GPIO
#define CY_BSP_HIB_WAKE_BUTTON5_IRQN   				ioss_interrupts_gpio_dpslp_2_IRQn

#define CY_BSP_USER_BUTTON2_PORT   					GPIO_PRT4
#define CY_BSP_USER_BUTTON2_PIN    					7
#define CY_BSP_USER_BUTTON2_PIN_MUX					P4_7_GPIO
#define CY_BSP_USER_BUTTON2_IRQN   					ioss_interrupts_gpio_dpslp_4_IRQn

#define CY_BSP_BB_USER_BUTTON0_PORT   				GPIO_PRT5
#define CY_BSP_BB_USER_BUTTON0_PIN    				1
#define CY_BSP_BB_USER_BUTTON0_PIN_MUX				P5_1_GPIO
#define CY_BSP_BB_USER_BUTTON0_IRQN   				ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BSP_USER_BUTTON3_PORT   					GPIO_PRT5
#define CY_BSP_USER_BUTTON3_PIN    					5
#define CY_BSP_USER_BUTTON3_PIN_MUX					P5_5_GPIO
#define CY_BSP_USER_BUTTON3_IRQN   					ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BSP_USER_BUTTON4_PORT   					GPIO_PRT6
#define CY_BSP_USER_BUTTON4_PIN    					0
#define CY_BSP_USER_BUTTON4_PIN_MUX					P6_0_GPIO
#define CY_BSP_USER_BUTTON4_IRQN   					ioss_interrupts_gpio_dpslp_6_IRQn

#define CY_BSP_BB_USER_BUTTON1_PORT   				GPIO_PRT7
#define CY_BSP_BB_USER_BUTTON1_PIN    				7
#define CY_BSP_BB_USER_BUTTON1_PIN_MUX				P7_7_GPIO
#define CY_BSP_BB_USER_BUTTON1_IRQN   				ioss_interrupts_gpio_dpslp_7_IRQn


/******************************************************************************/
/* TCPWM                                                                      */
/******************************************************************************/

#define CY_BSP_BB_CXPI0_CLK_TYPE   					TCPWM0_GRP0_CNT15
#define CY_BSP_BB_CXPI0_CLK_PORT   					GPIO_PRT18
#define CY_BSP_BB_CXPI0_CLK_PIN    					6
#define CY_BSP_BB_CXPI0_CLK_PIN_MUX					P18_6_TCPWM0_LINE15
#define CY_BSP_BB_CXPI0_CLK_PCLK   					PCLK_TCPWM0_CLOCKS15
#define CY_BSP_BB_CXPI0_CLK_IRQN   					tcpwm_0_interrupts_15_IRQn


/******************************************************************************/
/* SCB                                                                        */
/******************************************************************************/

#define CY_BSP_AUDIOSS_I2C_TYPE   					SCB2
#define CY_BSP_AUDIOSS_I2C_SDA_PORT   				GPIO_PRT10
#define CY_BSP_AUDIOSS_I2C_SDA_PIN    				0
#define CY_BSP_AUDIOSS_I2C_SDA_PIN_MUX				P10_0_SCB2_I2C_SDA

#define CY_BSP_AUDIOSS_I2C_SCL_PORT   				GPIO_PRT10
#define CY_BSP_AUDIOSS_I2C_SCL_PIN    				1
#define CY_BSP_AUDIOSS_I2C_SCL_PIN_MUX				P10_1_SCB2_I2C_SCL
#define CY_BSP_AUDIOSS_I2C_PCLK   					PCLK_SCB2_CLOCK
#define CY_BSP_AUDIOSS_I2C_IRQN   					scb_2_interrupt_IRQn

#define CY_BSP_USB_SCB_UART_TYPE   					SCB1
#define CY_BSP_USB_SCB_UART_RX_PORT   				GPIO_PRT11
#define CY_BSP_USB_SCB_UART_RX_PIN    				0
#define CY_BSP_USB_SCB_UART_RX_PIN_MUX				P11_0_SCB1_UART_RX

#define CY_BSP_USB_SCB_UART_TX_PORT   				GPIO_PRT11
#define CY_BSP_USB_SCB_UART_TX_PIN    				1
#define CY_BSP_USB_SCB_UART_TX_PIN_MUX				P11_1_SCB1_UART_TX
#define CY_BSP_USB_SCB_UART_PCLK   					PCLK_SCB1_CLOCK
#define CY_BSP_USB_SCB_UART_IRQN   					scb_1_interrupt_IRQn


/******************************************************************************/
/* CAN                                                                        */
/******************************************************************************/

#define CY_BSP_CANFD0_TYPE   						CANFD0_CH0
#define CY_BSP_CANFD0_TX_PORT   					GPIO_PRT18
#define CY_BSP_CANFD0_TX_PIN    					2
#define CY_BSP_CANFD0_TX_PIN_MUX					P18_2_CANFD0_TTCAN_TX0

#define CY_BSP_CANFD0_RX_PORT   					GPIO_PRT18
#define CY_BSP_CANFD0_RX_PIN    					3
#define CY_BSP_CANFD0_RX_PIN_MUX					P18_3_CANFD0_TTCAN_RX0
#define CY_BSP_CANFD0_PCLK   						PCLK_CANFD0_CLOCK_CAN0
#define CY_BSP_CANFD0_IRQN   						canfd_0_interrupts0_0_IRQn

#define CY_BSP_BB_CANFD0_TYPE   					CANFD1_CH0
#define CY_BSP_BB_CANFD0_TX_PORT   					GPIO_PRT4
#define CY_BSP_BB_CANFD0_TX_PIN    					0
#define CY_BSP_BB_CANFD0_TX_PIN_MUX					P4_0_CANFD1_TTCAN_TX0

#define CY_BSP_BB_CANFD0_RX_PORT   					GPIO_PRT4
#define CY_BSP_BB_CANFD0_RX_PIN    					1
#define CY_BSP_BB_CANFD0_RX_PIN_MUX					P4_1_CANFD1_TTCAN_RX0
#define CY_BSP_BB_CANFD0_PCLK   					PCLK_CANFD1_CLOCK_CAN0
#define CY_BSP_BB_CANFD0_IRQN   					canfd_1_interrupts0_0_IRQn

#define CY_BSP_BB_CANFD1_TYPE   					CANFD1_CH1
#define CY_BSP_BB_CANFD1_TX_PORT   					GPIO_PRT4
#define CY_BSP_BB_CANFD1_TX_PIN    					2
#define CY_BSP_BB_CANFD1_TX_PIN_MUX					P4_2_CANFD1_TTCAN_TX1

#define CY_BSP_BB_CANFD1_RX_PORT   					GPIO_PRT4
#define CY_BSP_BB_CANFD1_RX_PIN    					3
#define CY_BSP_BB_CANFD1_RX_PIN_MUX					P4_3_CANFD1_TTCAN_RX1
#define CY_BSP_BB_CANFD1_PCLK   					PCLK_CANFD1_CLOCK_CAN1
#define CY_BSP_BB_CANFD1_IRQN   					canfd_1_interrupts0_1_IRQn


/******************************************************************************/
/* LIN                                                                        */
/******************************************************************************/

#define CY_BSP_LINCH0_TYPE   						LIN0_CH0
#define CY_BSP_LINCH0_TX_PORT   					GPIO_PRT19
#define CY_BSP_LINCH0_TX_PIN    					0
#define CY_BSP_LINCH0_TX_PIN_MUX					P19_0_LIN0_LIN_TX0

#define CY_BSP_LINCH0_RX_PORT   					GPIO_PRT19
#define CY_BSP_LINCH0_RX_PIN    					1
#define CY_BSP_LINCH0_RX_PIN_MUX					P19_1_LIN0_LIN_RX0
#define CY_BSP_LINCH0_PCLK   						PCLK_LIN0_CLOCK_CH_EN0
#define CY_BSP_LINCH0_IRQN   						lin_0_interrupts_0_IRQn

#define CY_BSP_BB_LINCH1_TYPE   					LIN0_CH1
#define CY_BSP_BB_LINCH1_TX_PORT   					GPIO_PRT4
#define CY_BSP_BB_LINCH1_TX_PIN    					4
#define CY_BSP_BB_LINCH1_TX_PIN_MUX					P4_4_LIN0_LIN_TX1

#define CY_BSP_BB_LINCH1_RX_PORT   					GPIO_PRT4
#define CY_BSP_BB_LINCH1_RX_PIN    					5
#define CY_BSP_BB_LINCH1_RX_PIN_MUX					P4_5_LIN0_LIN_RX1
#define CY_BSP_BB_LINCH1_PCLK   					PCLK_LIN0_CLOCK_CH_EN1
#define CY_BSP_BB_LINCH1_IRQN   					lin_0_interrupts_1_IRQn


/******************************************************************************/
/* CXPI                                                                       */
/******************************************************************************/

#define CY_BSP_BB_CXPI0_TYPE   						CXPI0_CH0
#define CY_BSP_BB_CXPI0_TX_PORT   					GPIO_PRT2
#define CY_BSP_BB_CXPI0_TX_PIN    					2
#define CY_BSP_BB_CXPI0_TX_PIN_MUX					P2_2_CXPI0_CXPI_TX0

#define CY_BSP_BB_CXPI0_RX_PORT   					GPIO_PRT2
#define CY_BSP_BB_CXPI0_RX_PIN    					3
#define CY_BSP_BB_CXPI0_RX_PIN_MUX					P2_3_CXPI0_CXPI_RX0

#define CY_BSP_BB_CXPI0_EN_PORT   					GPIO_PRT2
#define CY_BSP_BB_CXPI0_EN_PIN    					4
#define CY_BSP_BB_CXPI0_EN_PIN_MUX					P2_4_CXPI0_CXPI_EN0
#define CY_BSP_BB_CXPI0_PCLK   						PCLK_CXPI0_CLOCK_CH_EN0
#define CY_BSP_BB_CXPI0_IRQN   						cxpi_0_interrupts_0_IRQn


/******************************************************************************/
/* SAR                                                                        */
/******************************************************************************/

#define CY_BSP_ADC_POT_TYPE   	PASS0_SAR1
#define CY_BSP_ADC_POT_CHN    	0
#define CY_BSP_ADC_POT_PORT   						GPIO_PRT5
#define CY_BSP_ADC_POT_PIN    						0
#define CY_BSP_ADC_POT_PIN_MUX						P5_0_GPIO
#define CY_BSP_ADC_POT_PCLK   						PCLK_PASS0_CLOCK_SAR1
#define CY_BSP_ADC_POT_IRQN   						pass_0_interrupts_sar_32_IRQn

#define CY_BSP_BB_ADC_POT_TYPE   	PASS0_SAR1
#define CY_BSP_BB_ADC_POT_CHN    	3
#define CY_BSP_BB_ADC_POT_PORT   					GPIO_PRT5
#define CY_BSP_BB_ADC_POT_PIN    					3
#define CY_BSP_BB_ADC_POT_PIN_MUX					P5_3_GPIO
#define CY_BSP_BB_ADC_POT_PCLK   					PCLK_PASS0_CLOCK_SAR1
#define CY_BSP_BB_ADC_POT_IRQN   					pass_0_interrupts_sar_35_IRQn


/******************************************************************************/
/* SMIF                                                                       */
/******************************************************************************/

#define CY_BSP_SMIF0_TYPE   						SMIF0
#define CY_BSP_SMIF0_SELECT1_PORT   				GPIO_PRT8
#define CY_BSP_SMIF0_SELECT1_PIN    				0
#define CY_BSP_SMIF0_SELECT1_PIN_MUX				P8_0_SMIF0_SPIHB_SELECT1

#define CY_BSP_SMIF0_SELECT0_PORT   				GPIO_PRT8
#define CY_BSP_SMIF0_SELECT0_PIN    				1
#define CY_BSP_SMIF0_SELECT0_PIN_MUX				P8_1_SMIF0_SPIHB_SELECT0

#define CY_BSP_SMIF0_DATA7_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA7_PIN    					2
#define CY_BSP_SMIF0_DATA7_PIN_MUX					P8_2_SMIF0_SPIHB_DATA7

#define CY_BSP_SMIF0_DATA6_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA6_PIN    					3
#define CY_BSP_SMIF0_DATA6_PIN_MUX					P8_3_SMIF0_SPIHB_DATA6

#define CY_BSP_SMIF0_DATA5_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA5_PIN    					4
#define CY_BSP_SMIF0_DATA5_PIN_MUX					P8_4_SMIF0_SPIHB_DATA5

#define CY_BSP_SMIF0_DATA4_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA4_PIN    					5
#define CY_BSP_SMIF0_DATA4_PIN_MUX					P8_5_SMIF0_SPIHB_DATA4

#define CY_BSP_SMIF0_DATA3_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA3_PIN    					6
#define CY_BSP_SMIF0_DATA3_PIN_MUX					P8_6_SMIF0_SPIHB_DATA3

#define CY_BSP_SMIF0_DATA2_PORT   					GPIO_PRT8
#define CY_BSP_SMIF0_DATA2_PIN    					7
#define CY_BSP_SMIF0_DATA2_PIN_MUX					P8_7_SMIF0_SPIHB_DATA2

#define CY_BSP_SMIF0_DATA1_PORT   					GPIO_PRT9
#define CY_BSP_SMIF0_DATA1_PIN    					0
#define CY_BSP_SMIF0_DATA1_PIN_MUX					P9_0_SMIF0_SPIHB_DATA1

#define CY_BSP_SMIF0_DATA0_PORT   					GPIO_PRT9
#define CY_BSP_SMIF0_DATA0_PIN    					1
#define CY_BSP_SMIF0_DATA0_PIN_MUX					P9_1_SMIF0_SPIHB_DATA0

#define CY_BSP_SMIF0_CLK_PORT   					GPIO_PRT9
#define CY_BSP_SMIF0_CLK_PIN    					2
#define CY_BSP_SMIF0_CLK_PIN_MUX					P9_2_SMIF0_SPIHB_CLK

#define CY_BSP_SMIF0_RWDS_PORT   					GPIO_PRT9
#define CY_BSP_SMIF0_RWDS_PIN    					3
#define CY_BSP_SMIF0_RWDS_PIN_MUX					P9_3_SMIF0_SPIHB_RWDS
#define CY_BSP_SMIF0_IRQN   						smif_0_interrupt_IRQn


/******************************************************************************/
/* TDM                                                                        */
/******************************************************************************/

#define CY_BSP_AUDIOSS_TDM0_TYPE   					TDM0_TDM_STRUCT0
#define CY_BSP_AUDIOSS_TDM0_TX_MCK_PORT   			GPIO_PRT11
#define CY_BSP_AUDIOSS_TDM0_TX_MCK_PIN    			2
#define CY_BSP_AUDIOSS_TDM0_TX_MCK_PIN_MUX			P11_2_TDM0_TDM_TX_MCK0

#define CY_BSP_AUDIOSS_TDM0_TX_SCK_PORT   			GPIO_PRT11
#define CY_BSP_AUDIOSS_TDM0_TX_SCK_PIN    			3
#define CY_BSP_AUDIOSS_TDM0_TX_SCK_PIN_MUX			P11_3_TDM0_TDM_TX_SCK0

#define CY_BSP_AUDIOSS_TDM0_TX_FSYNC_PORT   		GPIO_PRT11
#define CY_BSP_AUDIOSS_TDM0_TX_FSYNC_PIN    		4
#define CY_BSP_AUDIOSS_TDM0_TX_FSYNC_PIN_MUX		P11_4_TDM0_TDM_TX_FSYNC0

#define CY_BSP_AUDIOSS_TDM0_TX_SD_PORT   			GPIO_PRT11
#define CY_BSP_AUDIOSS_TDM0_TX_SD_PIN    			5
#define CY_BSP_AUDIOSS_TDM0_TX_SD_PIN_MUX			P11_5_TDM0_TDM_TX_SD0
#define CY_BSP_AUDIOSS_TDM0_TX_IRQN   				tdm_0_interrupts_tx_0_IRQn

#define CY_BSP_AUDIOSS_TDM0_RX_MCK_PORT   			GPIO_PRT12
#define CY_BSP_AUDIOSS_TDM0_RX_MCK_PIN    			3
#define CY_BSP_AUDIOSS_TDM0_RX_MCK_PIN_MUX			P12_3_TDM0_TDM_RX_MCK0

#define CY_BSP_AUDIOSS_TDM0_RX_SCK_PORT   			GPIO_PRT12
#define CY_BSP_AUDIOSS_TDM0_RX_SCK_PIN    			4
#define CY_BSP_AUDIOSS_TDM0_RX_SCK_PIN_MUX			P12_4_TDM0_TDM_RX_SCK0

#define CY_BSP_AUDIOSS_TDM0_RX_FSYNC_PORT   		GPIO_PRT12
#define CY_BSP_AUDIOSS_TDM0_RX_FSYNC_PIN    		5
#define CY_BSP_AUDIOSS_TDM0_RX_FSYNC_PIN_MUX		P12_5_TDM0_TDM_RX_FSYNC0

#define CY_BSP_AUDIOSS_TDM0_RX_SD_PORT   			GPIO_PRT12
#define CY_BSP_AUDIOSS_TDM0_RX_SD_PIN    			6
#define CY_BSP_AUDIOSS_TDM0_RX_SD_PIN_MUX			P12_6_TDM0_TDM_RX_SD0
#define CY_BSP_AUDIOSS_TDM0_RX_IRQN   				tdm_0_interrupts_rx_0_IRQn


/******************************************************************************/
/* SG                                                                         */
/******************************************************************************/

#define CY_BSP_AUDIOSS_SG0_TYPE   					SG0_SG_STRUCT0
#define CY_BSP_AUDIOSS_SG0_TONE_PORT   				GPIO_PRT6
#define CY_BSP_AUDIOSS_SG0_TONE_PIN    				1
#define CY_BSP_AUDIOSS_SG0_TONE_PIN_MUX				P6_1_SG0_SG_TONE0

#define CY_BSP_AUDIOSS_SG0_AMPL_PORT   				GPIO_PRT6
#define CY_BSP_AUDIOSS_SG0_AMPL_PIN    				2
#define CY_BSP_AUDIOSS_SG0_AMPL_PIN_MUX				P6_2_SG0_SG_AMPL0
#define CY_BSP_AUDIOSS_SG0_IRQN   					sg_0_interrupts_0_IRQn


/******************************************************************************/
/* PWM                                                                        */
/******************************************************************************/

#define CY_BSP_AUDIOSS_PWM0_TYPE   					PWM0_TX0
#define CY_BSP_AUDIOSS_PWM0_L1P0_PORT   			GPIO_PRT13
#define CY_BSP_AUDIOSS_PWM0_L1P0_PIN    			0
#define CY_BSP_AUDIOSS_PWM0_L1P0_PIN_MUX			P13_0_PWM0_PWM_LINE1_P0

#define CY_BSP_AUDIOSS_PWM0_L1N0_PORT   			GPIO_PRT13
#define CY_BSP_AUDIOSS_PWM0_L1N0_PIN    			1
#define CY_BSP_AUDIOSS_PWM0_L1N0_PIN_MUX			P13_1_PWM0_PWM_LINE1_N0

#define CY_BSP_AUDIOSS_PWM0_L2P0_PORT   			GPIO_PRT13
#define CY_BSP_AUDIOSS_PWM0_L2P0_PIN    			2
#define CY_BSP_AUDIOSS_PWM0_L2P0_PIN_MUX			P13_2_PWM0_PWM_LINE2_P0

#define CY_BSP_AUDIOSS_PWM0_L2N0_PORT   			GPIO_PRT13
#define CY_BSP_AUDIOSS_PWM0_L2N0_PIN    			3
#define CY_BSP_AUDIOSS_PWM0_L2N0_PIN_MUX			P13_3_PWM0_PWM_LINE2_N0
#define CY_BSP_AUDIOSS_PWM0_IRQN   					pwm_0_interrupts_0_IRQn


/******************************************************************************/
/* MISC                                                                       */
/******************************************************************************/

#define CY_BSP_LCD_SEG_0_TYPE   					LCD0_BASE
#define CY_BSP_LCD_SEG_0_PORT   					GPIO_PRT13
#define CY_BSP_LCD_SEG_0_PIN    					0
#define CY_BSP_LCD_SEG_0_PIN_MUX					P13_0_LCD_SEG0

#define CY_BSP_LCD_SEG_1_PORT   					GPIO_PRT13
#define CY_BSP_LCD_SEG_1_PIN    					1
#define CY_BSP_LCD_SEG_1_PIN_MUX					P13_1_LCD_SEG1

#define CY_BSP_LCD_SEG_2_PORT   					GPIO_PRT13
#define CY_BSP_LCD_SEG_2_PIN    					2
#define CY_BSP_LCD_SEG_2_PIN_MUX					P13_2_LCD_SEG2

#define CY_BSP_LCD_SEG_3_PORT   					GPIO_PRT13
#define CY_BSP_LCD_SEG_3_PIN    					3
#define CY_BSP_LCD_SEG_3_PIN_MUX					P13_3_LCD_SEG3

#define CY_BSP_LCD_SEG_4_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_4_PIN    					0
#define CY_BSP_LCD_SEG_4_PIN_MUX					P14_0_LCD_SEG4

#define CY_BSP_LCD_SEG_5_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_5_PIN    					1
#define CY_BSP_LCD_SEG_5_PIN_MUX					P14_1_LCD_SEG5

#define CY_BSP_LCD_SEG_6_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_6_PIN    					2
#define CY_BSP_LCD_SEG_6_PIN_MUX					P14_2_LCD_SEG6

#define CY_BSP_LCD_SEG_7_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_7_PIN    					3
#define CY_BSP_LCD_SEG_7_PIN_MUX					P14_3_LCD_SEG7

#define CY_BSP_LCD_SEG_8_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_8_PIN    					4
#define CY_BSP_LCD_SEG_8_PIN_MUX					P14_4_LCD_SEG8

#define CY_BSP_LCD_SEG_9_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_9_PIN    					5
#define CY_BSP_LCD_SEG_9_PIN_MUX					P14_5_LCD_SEG9

#define CY_BSP_LCD_SEG_10_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_10_PIN    					6
#define CY_BSP_LCD_SEG_10_PIN_MUX					P14_6_LCD_SEG10

#define CY_BSP_LCD_SEG_11_PORT   					GPIO_PRT14
#define CY_BSP_LCD_SEG_11_PIN    					7
#define CY_BSP_LCD_SEG_11_PIN_MUX					P14_7_LCD_SEG11

#define CY_BSP_LCD_SEG_12_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_12_PIN    					0
#define CY_BSP_LCD_SEG_12_PIN_MUX					P15_0_LCD_SEG12

#define CY_BSP_LCD_SEG_13_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_13_PIN    					1
#define CY_BSP_LCD_SEG_13_PIN_MUX					P15_1_LCD_SEG13

#define CY_BSP_LCD_SEG_14_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_14_PIN    					2
#define CY_BSP_LCD_SEG_14_PIN_MUX					P15_2_LCD_SEG14

#define CY_BSP_LCD_SEG_15_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_15_PIN    					3
#define CY_BSP_LCD_SEG_15_PIN_MUX					P15_3_LCD_SEG15

#define CY_BSP_LCD_SEG_16_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_16_PIN    					4
#define CY_BSP_LCD_SEG_16_PIN_MUX					P15_4_LCD_SEG16

#define CY_BSP_LCD_SEG_17_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_17_PIN    					5
#define CY_BSP_LCD_SEG_17_PIN_MUX					P15_5_LCD_SEG17

#define CY_BSP_LCD_SEG_18_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_18_PIN    					6
#define CY_BSP_LCD_SEG_18_PIN_MUX					P15_6_LCD_SEG18

#define CY_BSP_LCD_SEG_19_PORT   					GPIO_PRT15
#define CY_BSP_LCD_SEG_19_PIN    					7
#define CY_BSP_LCD_SEG_19_PIN_MUX					P15_7_LCD_SEG19

#define CY_BSP_LCD_SEG_20_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_20_PIN    					0
#define CY_BSP_LCD_SEG_20_PIN_MUX					P16_0_LCD_SEG20

#define CY_BSP_LCD_SEG_21_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_21_PIN    					1
#define CY_BSP_LCD_SEG_21_PIN_MUX					P16_1_LCD_SEG21

#define CY_BSP_LCD_SEG_22_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_22_PIN    					2
#define CY_BSP_LCD_SEG_22_PIN_MUX					P16_2_LCD_SEG22

#define CY_BSP_LCD_SEG_23_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_23_PIN    					3
#define CY_BSP_LCD_SEG_23_PIN_MUX					P16_3_LCD_SEG23

#define CY_BSP_LCD_SEG_24_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_24_PIN    					4
#define CY_BSP_LCD_SEG_24_PIN_MUX					P16_4_LCD_SEG24

#define CY_BSP_LCD_SEG_25_PORT   					GPIO_PRT16
#define CY_BSP_LCD_SEG_25_PIN    					5
#define CY_BSP_LCD_SEG_25_PIN_MUX					P16_5_LCD_SEG25

#define CY_BSP_LCD_SEG_26_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_26_PIN    					0
#define CY_BSP_LCD_SEG_26_PIN_MUX					P17_0_LCD_SEG26

#define CY_BSP_LCD_SEG_27_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_27_PIN    					1
#define CY_BSP_LCD_SEG_27_PIN_MUX					P17_1_LCD_SEG27

#define CY_BSP_LCD_SEG_28_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_28_PIN    					2
#define CY_BSP_LCD_SEG_28_PIN_MUX					P17_2_LCD_SEG28

#define CY_BSP_LCD_SEG_29_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_29_PIN    					3
#define CY_BSP_LCD_SEG_29_PIN_MUX					P17_3_LCD_SEG29

#define CY_BSP_LCD_SEG_30_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_30_PIN    					4
#define CY_BSP_LCD_SEG_30_PIN_MUX					P17_4_LCD_SEG30

#define CY_BSP_LCD_SEG_31_PORT   					GPIO_PRT17
#define CY_BSP_LCD_SEG_31_PIN    					5
#define CY_BSP_LCD_SEG_31_PIN_MUX					P17_5_LCD_SEG31

#define CY_BSP_LCD_COM_0_PORT   					GPIO_PRT17
#define CY_BSP_LCD_COM_0_PIN    					6
#define CY_BSP_LCD_COM_0_PIN_MUX					P17_6_LCD_COM32

#define CY_BSP_LCD_COM_1_PORT   					GPIO_PRT17
#define CY_BSP_LCD_COM_1_PIN    					7
#define CY_BSP_LCD_COM_1_PIN_MUX					P17_7_LCD_COM33

#define CY_BSP_LCD_COM_2_PORT   					GPIO_PRT18
#define CY_BSP_LCD_COM_2_PIN    					0
#define CY_BSP_LCD_COM_2_PIN_MUX					P18_0_LCD_COM34

#define CY_BSP_LCD_COM_3_PORT   					GPIO_PRT18
#define CY_BSP_LCD_COM_3_PIN    					1
#define CY_BSP_LCD_COM_3_PIN_MUX					P18_1_LCD_COM35


/******************************************************************************/
/******************************************************************************/

#else 	/* No Device or PSVP Selected */

    #error Choose correct device from the device specific Datasheet. 

#endif 	/* #if(defined(CYT2CL8BAS)) */


#ifdef __cplusplus
}
#endif


#endif 	/* BSP_TVIICE4M_176LQFP_DEVICE_REVA_EVK_REVA_H */


/* [] END OF FILE */
