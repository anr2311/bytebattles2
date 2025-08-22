
/***************************************************************************//**
* \file :
* bsp_tviic2d6mddr_500bga_device_reva_psvp.h
* 
* \brief
* Project specific header
* 
* \note
* !!! Auto generated file and should not be modified !!!
* Generator revision: 1.9.2
* 
********************************************************************************
* \copyright
* Copyright 2016-2021, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef BSP_TVIIC2D6MDDR_500BGA_DEVICE_REVA_PSVP_H
#define BSP_TVIIC2D6MDDR_500BGA_DEVICE_REVA_PSVP_H

#ifdef __cplusplus
extern C {
#endif

/******************************************************************************/
/******************************************************************************/


#if (defined(CYT4ENDBAS)) && (CY_USE_PSVP == 1)

#define CY_BSP_CXPI0_CH0_AVAILABLE
#define CY_BSP_LIN0_CH1_AVAILABLE
#define CY_BSP_CANFD1_CH0_AVAILABLE
#define CY_BSP_SMIF0_AVAILABLE
#define CY_BSP_SMIF1_AVAILABLE
#define CY_BSP_TTL_DSP1_AVAILABLE

/******************************************************************************/
/* GPIO                                                                       */
/******************************************************************************/


/**** LED ****/
#define CY_BSP_BB_USER_LED2_PORT   				GPIO_PRT9
#define CY_BSP_BB_USER_LED2_PIN    				0
#define CY_BSP_BB_USER_LED2_PIN_MUX				P9_0_GPIO
#define CY_BSP_BB_USER_LED2_IRQN   				ioss_interrupts_gpio_act_9_IRQn

#define CY_BSP_BB_USER_LED3_PORT   				GPIO_PRT9
#define CY_BSP_BB_USER_LED3_PIN    				1
#define CY_BSP_BB_USER_LED3_PIN_MUX				P9_1_GPIO
#define CY_BSP_BB_USER_LED3_IRQN   				ioss_interrupts_gpio_act_9_IRQn

#define CY_BSP_BB_USER_LED0_PORT   				GPIO_PRT9
#define CY_BSP_BB_USER_LED0_PIN    				2
#define CY_BSP_BB_USER_LED0_PIN_MUX				P9_2_GPIO
#define CY_BSP_BB_USER_LED0_IRQN   				ioss_interrupts_gpio_act_9_IRQn

#define CY_BSP_BB_USER_LED1_PORT   				GPIO_PRT9
#define CY_BSP_BB_USER_LED1_PIN    				3
#define CY_BSP_BB_USER_LED1_PIN_MUX				P9_3_GPIO
#define CY_BSP_BB_USER_LED1_IRQN   				ioss_interrupts_gpio_act_9_IRQn


/**** BUTTON ****/
#define CY_BSP_BB_USER_BUTTON_1_PORT   			GPIO_PRT1
#define CY_BSP_BB_USER_BUTTON_1_PIN    			5
#define CY_BSP_BB_USER_BUTTON_1_PIN_MUX			P1_5_GPIO
#define CY_BSP_BB_USER_BUTTON_1_IRQN   			ioss_interrupts_gpio_dpslp_1_IRQn

#define CY_BSP_BB_USER_BUTTON_4_PORT   			GPIO_PRT2
#define CY_BSP_BB_USER_BUTTON_4_PIN    			1
#define CY_BSP_BB_USER_BUTTON_4_PIN_MUX			P2_1_GPIO
#define CY_BSP_BB_USER_BUTTON_4_IRQN   			ioss_interrupts_gpio_dpslp_2_IRQn

#define CY_BSP_BB_USER_BUTTON_5_PORT   			GPIO_PRT7
#define CY_BSP_BB_USER_BUTTON_5_PIN    			0
#define CY_BSP_BB_USER_BUTTON_5_PIN_MUX			P7_0_GPIO
#define CY_BSP_BB_USER_BUTTON_5_IRQN   			ioss_interrupts_gpio_dpslp_7_IRQn

#define CY_BSP_BB_USER_BUTTON_3_PORT   			GPIO_PRT7
#define CY_BSP_BB_USER_BUTTON_3_PIN    			5
#define CY_BSP_BB_USER_BUTTON_3_PIN_MUX			P7_5_GPIO
#define CY_BSP_BB_USER_BUTTON_3_IRQN   			ioss_interrupts_gpio_dpslp_7_IRQn

#define CY_BSP_BB_USER_BUTTON_2_PORT   			GPIO_PRT8
#define CY_BSP_BB_USER_BUTTON_2_PIN    			1
#define CY_BSP_BB_USER_BUTTON_2_PIN_MUX			P8_1_GPIO
#define CY_BSP_BB_USER_BUTTON_2_IRQN   			ioss_interrupts_gpio_dpslp_8_IRQn


/**** MISC_GPIO ****/
#define CY_BL_DISP1_PWM_PORT   				GPIO_PRT5
#define CY_BL_DISP1_PWM_PIN    				0
#define CY_BL_DISP1_PWM_PIN_MUX				P5_0_GPIO
#define CY_BL_DISP1_PWM_IRQN   				ioss_interrupts_gpio_dpslp_5_IRQn

#define CY_BSP_BB_CXPI_SEL_PORT   				GPIO_PRT7
#define CY_BSP_BB_CXPI_SEL_PIN    				6
#define CY_BSP_BB_CXPI_SEL_PIN_MUX				P7_6_GPIO
#define CY_BSP_BB_CXPI_SEL_IRQN   				ioss_interrupts_gpio_dpslp_7_IRQn


/******************************************************************************/
/* TCPWM                                                                      */
/******************************************************************************/

#define CY_BSP_BB_CXPI_CLK_PWM_TYPE   			TCPWM0_GRP0_CNT24
#define CY_BSP_BB_CXPI_CLK_PWM_PORT   			GPIO_PRT9
#define CY_BSP_BB_CXPI_CLK_PWM_PIN    			5
#define CY_BSP_BB_CXPI_CLK_PWM_PIN_MUX			P9_5_TCPWM0_LINE24
#define CY_BSP_BB_CXPI_CLK_PWM_PCLK   			PCLK_TCPWM0_CLOCKS24
#define CY_BSP_BB_CXPI_CLK_PWM_IRQN   			tcpwm_0_interrupts_24_IRQn


/******************************************************************************/
/* SCB                                                                        */
/******************************************************************************/

#define CY_MIPI_CSI_I2C_TYPE   				SCB0
#define CY_MIPI_CSI_I2C_SDA_PORT   			GPIO_PRT3
#define CY_MIPI_CSI_I2C_SDA_PIN    			0
#define CY_MIPI_CSI_I2C_SDA_PIN_MUX			P3_0_SCB0_I2C_SDA

#define CY_MIPI_CSI_I2C_SCL_PORT   			GPIO_PRT3
#define CY_MIPI_CSI_I2C_SCL_PIN    			1
#define CY_MIPI_CSI_I2C_SCL_PIN_MUX			P3_1_SCB0_I2C_SCL
#define CY_MIPI_CSI_I2C_PCLK   				PCLK_SCB0_CLOCK
#define CY_MIPI_CSI_I2C_IRQN   				scb_0_interrupt_IRQn

#define CY_USB_SCB_UART_TYPE   					SCB6
#define CY_USB_SCB_UART_RX_PORT   				GPIO_PRT1
#define CY_USB_SCB_UART_RX_PIN    				0
#define CY_USB_SCB_UART_RX_PIN_MUX				P1_0_SCB6_UART_RX

#define CY_USB_SCB_UART_TX_PORT   				GPIO_PRT1
#define CY_USB_SCB_UART_TX_PIN    				1
#define CY_USB_SCB_UART_TX_PIN_MUX				P1_1_SCB6_UART_TX
#define CY_USB_SCB_UART_PCLK   					PCLK_SCB6_CLOCK
#define CY_USB_SCB_UART_IRQN   					scb_6_interrupt_IRQn


/******************************************************************************/
/* CAN                                                                        */
/******************************************************************************/

#define CY_BSP_BB_CAN0_TYPE   					CANFD1_CH0
#define CY_BSP_BB_CAN0_TX_PORT   				GPIO_PRT4
#define CY_BSP_BB_CAN0_TX_PIN    				6
#define CY_BSP_BB_CAN0_TX_PIN_MUX				P4_6_CANFD1_TTCAN_TX0

#define CY_BSP_BB_CAN0_RX_PORT   				GPIO_PRT4
#define CY_BSP_BB_CAN0_RX_PIN    				7
#define CY_BSP_BB_CAN0_RX_PIN_MUX				P4_7_CANFD1_TTCAN_RX0
#define CY_BSP_BB_CAN0_PCLK   					PCLK_CANFD1_CLOCK_CAN0
#define CY_BSP_BB_CAN0_IRQN   					canfd_1_interrupts0_0_IRQn


/******************************************************************************/
/* LIN                                                                        */
/******************************************************************************/

#define CY_BSP_BB_LIN0_TYPE   					LIN0_CH1
#define CY_BSP_BB_LIN0_RX_PORT   				GPIO_PRT5
#define CY_BSP_BB_LIN0_RX_PIN    				1
#define CY_BSP_BB_LIN0_RX_PIN_MUX				P5_1_LIN0_LIN_TX1

#define CY_BSP_BB_LIN0_TX_PORT   				GPIO_PRT5
#define CY_BSP_BB_LIN0_TX_PIN    				2
#define CY_BSP_BB_LIN0_TX_PIN_MUX				P5_2_LIN0_LIN_RX1

#define CY_BSP_BB_LIN0_EN_PORT   				GPIO_PRT5
#define CY_BSP_BB_LIN0_EN_PIN    				3
#define CY_BSP_BB_LIN0_EN_PIN_MUX				P5_3_LIN0_LIN_EN1
#define CY_BSP_BB_LIN0_PCLK   					PCLK_LIN0_CLOCK_CH_EN1
#define CY_BSP_BB_LIN0_IRQN   					lin_0_interrupts_1_IRQn


/******************************************************************************/
/* CXPI                                                                       */
/******************************************************************************/

#define CY_BSP_BB_CXPI_TYPE   					CXPI0_CH0
#define CY_BSP_BB_CXPI_TX_PORT   				GPIO_PRT1
#define CY_BSP_BB_CXPI_TX_PIN    				6
#define CY_BSP_BB_CXPI_TX_PIN_MUX				P1_6_CXPI0_CXPI_TX0

#define CY_BSP_BB_CXPI_RX_PORT   				GPIO_PRT1
#define CY_BSP_BB_CXPI_RX_PIN    				7
#define CY_BSP_BB_CXPI_RX_PIN_MUX				P1_7_CXPI0_CXPI_RX0

#define CY_BSP_BB_CXPI_EN_PORT   				GPIO_PRT2
#define CY_BSP_BB_CXPI_EN_PIN    				0
#define CY_BSP_BB_CXPI_EN_PIN_MUX				P2_0_CXPI0_CXPI_EN0
#define CY_BSP_BB_CXPI_PCLK   					PCLK_CXPI0_CLOCK_CH_EN0
#define CY_BSP_BB_CXPI_IRQN   					cxpi_0_interrupts_0_IRQn


/******************************************************************************/
/* SAR                                                                        */
/******************************************************************************/

#define CY_BSP_BB_ADC_POT_TYPE   	            PASS0_SAR1
#define CY_BSP_BB_ADC_POT_CHN    	            10
#define CY_BSP_BB_ADC_POT_PORT   				GPIO_PRT8
#define CY_BSP_BB_ADC_POT_PIN    				2
#define CY_BSP_BB_ADC_POT_PIN_MUX				P8_2_GPIO
#define CY_BSP_BB_ADC_POT_PCLK   				PCLK_PASS0_CLOCK_SAR1
#define CY_BSP_BB_ADC_POT_IRQN   				pass_0_interrupts_sar_42_IRQn


/******************************************************************************/
/* SMIF                                                                       */
/******************************************************************************/

#define CY_SMIF0_TYPE   					SMIF0

#define CY_SMIF0_DATA4_PORT   				GPIO_PRT14
#define CY_SMIF0_DATA4_PIN    				0
#define CY_SMIF0_DATA4_PIN_MUX				P14_0_SMIF0_SMIF0_SPIHB_DATA4

#define CY_SMIF0_DATA2_PORT   				GPIO_PRT14
#define CY_SMIF0_DATA2_PIN    				1
#define CY_SMIF0_DATA2_PIN_MUX				P14_1_SMIF0_SMIF0_SPIHB_DATA2

#define CY_SMIF0_DATA3_PORT   				GPIO_PRT14
#define CY_SMIF0_DATA3_PIN    				2
#define CY_SMIF0_DATA3_PIN_MUX				P14_2_SMIF0_SMIF0_SPIHB_DATA3

#define CY_SMIF0_DATA5_PORT   				GPIO_PRT14
#define CY_SMIF0_DATA5_PIN    				3
#define CY_SMIF0_DATA5_PIN_MUX				P14_3_SMIF0_SMIF0_SPIHB_DATA5

#define CY_SMIF0_RWDS_PORT   				GPIO_PRT14
#define CY_SMIF0_RWDS_PIN    				4
#define CY_SMIF0_RWDS_PIN_MUX				P14_4_SMIF0_SMIF0_SPIHB_RWDS

#define CY_SMIF0_DATA0_PORT   				GPIO_PRT16
#define CY_SMIF0_DATA0_PIN    				0
#define CY_SMIF0_DATA0_PIN_MUX				P16_0_SMIF0_SMIF0_SPIHB_DATA0

#define CY_SMIF0_DATA6_PORT   				GPIO_PRT16
#define CY_SMIF0_DATA6_PIN    				1
#define CY_SMIF0_DATA6_PIN_MUX				P16_1_SMIF0_SMIF0_SPIHB_DATA6

#define CY_SMIF0_SELECT0_PORT   			GPIO_PRT16
#define CY_SMIF0_SELECT0_PIN    			2
#define CY_SMIF0_SELECT0_PIN_MUX			P16_2_SMIF0_SMIF0_SPIHB_SELECT0

#define CY_SMIF0_DATA1_PORT   				GPIO_PRT16
#define CY_SMIF0_DATA1_PIN    				3
#define CY_SMIF0_DATA1_PIN_MUX				P16_3_SMIF0_SMIF0_SPIHB_DATA1

#define CY_SMIF0_DATA7_PORT   				GPIO_PRT16
#define CY_SMIF0_DATA7_PIN    				4
#define CY_SMIF0_DATA7_PIN_MUX				P16_4_SMIF0_SMIF0_SPIHB_DATA7

#define CY_SMIF0_SELECT1_PORT   			GPIO_PRT16
#define CY_SMIF0_SELECT1_PIN    			5
#define CY_SMIF0_SELECT1_PIN_MUX			P16_5_SMIF0_SMIF0_SPIHB_SELECT1

#define CY_SMIF0_CLK_INV_PORT   			GPIO_PRT15
#define CY_SMIF0_CLK_INV_PIN    			0
#define CY_SMIF0_CLK_INV_PIN_MUX			P15_0_SMIF0_SMIF0_SPIHB_CLK

#define CY_SMIF0_CLK_PORT   				GPIO_PRT15
#define CY_SMIF0_CLK_PIN    				1
#define CY_SMIF0_CLK_PIN_MUX				P15_1_SMIF0_SMIF0_SPIHB_CLK
#define CY_SMIF0_IRQN   					smif_0_interrupt_IRQn

#define CY_SMIF1_TYPE   					SMIF1
#define CY_SMIF1_DATA4_PORT   				GPIO_PRT17
#define CY_SMIF1_DATA4_PIN    				0
#define CY_SMIF1_DATA4_PIN_MUX				P17_0_SMIF0_SMIF1_SPIHB_DATA4

#define CY_SMIF1_DATA2_PORT   				GPIO_PRT17
#define CY_SMIF1_DATA2_PIN    				1
#define CY_SMIF1_DATA2_PIN_MUX				P17_1_SMIF0_SMIF1_SPIHB_DATA2

#define CY_SMIF1_DATA3_PORT   				GPIO_PRT17
#define CY_SMIF1_DATA3_PIN    				2
#define CY_SMIF1_DATA3_PIN_MUX				P17_2_SMIF0_SMIF1_SPIHB_DATA3

#define CY_SMIF1_DATA5_PORT   				GPIO_PRT17
#define CY_SMIF1_DATA5_PIN    				3
#define CY_SMIF1_DATA5_PIN_MUX				P17_3_SMIF0_SMIF1_SPIHB_DATA5

#define CY_SMIF1_RWDS_PORT   				GPIO_PRT17
#define CY_SMIF1_RWDS_PIN    				4
#define CY_SMIF1_RWDS_PIN_MUX				P17_4_SMIF0_SMIF1_SPIHB_RWDS

#define CY_SMIF1_DATA0_PORT   				GPIO_PRT19
#define CY_SMIF1_DATA0_PIN    				0
#define CY_SMIF1_DATA0_PIN_MUX				P19_0_SMIF0_SMIF1_SPIHB_DATA0

#define CY_SMIF1_DATA6_PORT   				GPIO_PRT19
#define CY_SMIF1_DATA6_PIN    				1
#define CY_SMIF1_DATA6_PIN_MUX				P19_1_SMIF0_SMIF1_SPIHB_DATA6

#define CY_SMIF1_SELECT0_PORT   			GPIO_PRT19
#define CY_SMIF1_SELECT0_PIN    			2
#define CY_SMIF1_SELECT0_PIN_MUX			P19_2_SMIF0_SMIF1_SPIHB_SELECT0

#define CY_SMIF1_DATA1_PORT   				GPIO_PRT19
#define CY_SMIF1_DATA1_PIN    				3
#define CY_SMIF1_DATA1_PIN_MUX				P19_3_SMIF0_SMIF1_SPIHB_DATA1

#define CY_SMIF1_DATA7_PORT   				GPIO_PRT19
#define CY_SMIF1_DATA7_PIN    				4
#define CY_SMIF1_DATA7_PIN_MUX				P19_4_SMIF0_SMIF1_SPIHB_DATA7

#define CY_SMIF1_SELECT1_PORT   			GPIO_PRT19
#define CY_SMIF1_SELECT1_PIN    			5
#define CY_SMIF1_SELECT1_PIN_MUX			P19_5_SMIF0_SMIF1_SPIHB_SELECT1

#define CY_SMIF1_CLK_INV_PORT   			GPIO_PRT18
#define CY_SMIF1_CLK_INV_PIN    			0
#define CY_SMIF1_CLK_INV_PIN_MUX			P18_0_SMIF0_SMIF1_SPIHB_CLK

#define CY_SMIF1_CLK_PORT   				GPIO_PRT18
#define CY_SMIF1_CLK_PIN    				1
#define CY_SMIF1_CLK_PIN_MUX				P18_1_SMIF0_SMIF1_SPIHB_CLK
#define CY_SMIF1_IRQN   					smif_1_interrupt_IRQn


/******************************************************************************/
/* DSP                                                                        */
/******************************************************************************/

#define CY_DISP1_DATA_A1_11_PORT            	GPIO_PRT9
#define CY_DISP1_DATA_A1_11_PIN             	6
#define CY_DISP1_DATA_A1_11_PIN_MUX         	P9_6_VIDEOSS0_TTL_DSP1_DATA_A111

#define CY_DISP1_DATA_A0_11_PORT   	            GPIO_PRT9
#define CY_DISP1_DATA_A0_11_PIN    	            7
#define CY_DISP1_DATA_A0_11_PIN_MUX	            P9_7_VIDEOSS0_TTL_DSP1_DATA_A011

#define CY_DISP1_DATA_A1_10_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A1_10_PIN    	            0
#define CY_DISP1_DATA_A1_10_PIN_MUX	            P10_0_VIDEOSS0_TTL_DSP1_DATA_A110

#define CY_DISP1_DATA_A0_10_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A0_10_PIN    	            1
#define CY_DISP1_DATA_A0_10_PIN_MUX	            P10_1_VIDEOSS0_TTL_DSP1_DATA_A010

#define CY_DISP1_DATA_A1_9_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A1_9_PIN    	            2
#define CY_DISP1_DATA_A1_9_PIN_MUX	            P10_2_VIDEOSS0_TTL_DSP1_DATA_A19

#define CY_DISP1_DATA_A0_9_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A0_9_PIN    	            3
#define CY_DISP1_DATA_A0_9_PIN_MUX	            P10_3_VIDEOSS0_TTL_DSP1_DATA_A09

#define CY_DISP1_DATA_A1_8_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A1_8_PIN    	            4
#define CY_DISP1_DATA_A1_8_PIN_MUX	            P10_4_VIDEOSS0_TTL_DSP1_DATA_A18

#define CY_DISP1_DATA_A0_8_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A0_8_PIN    	            5
#define CY_DISP1_DATA_A0_8_PIN_MUX	            P10_5_VIDEOSS0_TTL_DSP1_DATA_A08

#define CY_DISP1_DATA_A1_7_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A1_7_PIN    	            6
#define CY_DISP1_DATA_A1_7_PIN_MUX	            P10_6_VIDEOSS0_TTL_DSP1_DATA_A17

#define CY_DISP1_DATA_A0_7_PORT   	            GPIO_PRT10
#define CY_DISP1_DATA_A0_7_PIN    	            7
#define CY_DISP1_DATA_A0_7_PIN_MUX	            P10_7_VIDEOSS0_TTL_DSP1_DATA_A07

#define CY_DISP1_DATA_A1_6_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A1_6_PIN    	            0
#define CY_DISP1_DATA_A1_6_PIN_MUX	            P11_0_VIDEOSS0_TTL_DSP1_DATA_A16

#define CY_DISP1_DATA_A0_6_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A0_6_PIN    	            1
#define CY_DISP1_DATA_A0_6_PIN_MUX	            P11_1_VIDEOSS0_TTL_DSP1_DATA_A06

#define CY_DISP1_DATA_A1_5_PORT   	            GPIO_PRT11 
#define CY_DISP1_DATA_A1_5_PIN    	            2
#define CY_DISP1_DATA_A1_5_PIN_MUX	            P11_2_VIDEOSS0_TTL_DSP1_DATA_A15

#define CY_DISP1_DATA_A0_5_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A0_5_PIN    	            3
#define CY_DISP1_DATA_A0_5_PIN_MUX	            P11_3_VIDEOSS0_TTL_DSP1_DATA_A05

#define CY_DISP1_DATA_A1_4_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A1_4_PIN    	            4
#define CY_DISP1_DATA_A1_4_PIN_MUX	            P11_4_VIDEOSS0_TTL_DSP1_DATA_A14

#define CY_DISP1_DATA_A0_4_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A0_4_PIN    	            5
#define CY_DISP1_DATA_A0_4_PIN_MUX	            P11_5_VIDEOSS0_TTL_DSP1_DATA_A04

#define CY_DISP1_DATA_A1_3_PORT   	            GPIO_PRT11
#define CY_DISP1_DATA_A1_3_PIN    	            6
#define CY_DISP1_DATA_A1_3_PIN_MUX	            P11_6_VIDEOSS0_TTL_DSP1_DATA_A13

#define CY_DISP1_DATA_A0_3_PORT   		        GPIO_PRT11
#define CY_DISP1_DATA_A0_3_PIN    		        7
#define CY_DISP1_DATA_A0_3_PIN_MUX		        P11_7_VIDEOSS0_TTL_DSP1_DATA_A03

#define CY_DISP1_DATA_A1_2_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A1_2_PIN    		        0
#define CY_DISP1_DATA_A1_2_PIN_MUX		        P12_0_VIDEOSS0_TTL_DSP1_DATA_A12

#define CY_DISP1_DATA_A0_2_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A0_2_PIN    		        1
#define CY_DISP1_DATA_A0_2_PIN_MUX		        P12_1_VIDEOSS0_TTL_DSP1_DATA_A02

#define CY_DISP1_DATA_A1_1_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A1_1_PIN    		        2
#define CY_DISP1_DATA_A1_1_PIN_MUX		        P12_2_VIDEOSS0_TTL_DSP1_DATA_A11

#define CY_DISP1_DATA_A0_1_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A0_1_PIN    		        3
#define CY_DISP1_DATA_A0_1_PIN_MUX		        P12_3_VIDEOSS0_TTL_DSP1_DATA_A01

#define CY_DISP1_DATA_A1_0_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A1_0_PIN    		        4
#define CY_DISP1_DATA_A1_0_PIN_MUX		        P12_4_VIDEOSS0_TTL_DSP1_DATA_A10

#define CY_DISP1_DATA_A0_0_PORT   		        GPIO_PRT12
#define CY_DISP1_DATA_A0_0_PIN    		        5
#define CY_DISP1_DATA_A0_0_PIN_MUX		        P12_5_VIDEOSS0_TTL_DSP1_DATA_A00

#define CY_DISP1_CLK_PORT   				    GPIO_PRT12
#define CY_DISP1_CLK_PIN    				    6
#define CY_DISP1_CLK_PIN_MUX				    P12_6_VIDEOSS0_TTL_DSP1_CLOCK

#define CY_DISP1_CTRL0_PORT   		            GPIO_PRT12
#define CY_DISP1_CTRL0_PIN    		            7
#define CY_DISP1_CTRL0_PIN_MUX		            P12_7_VIDEOSS0_TTL_DSP1_CONTROL0

#define CY_DISP1_CTRL1_PORT   		            GPIO_PRT13
#define CY_DISP1_CTRL1_PIN    		            0
#define CY_DISP1_CTRL1_PIN_MUX		            P13_0_VIDEOSS0_TTL_DSP1_CONTROL1

#define CY_DISP1_CTRL2_PORT   			        GPIO_PRT13
#define CY_DISP1_CTRL2_PIN    			        1
#define CY_DISP1_CTRL2_PIN_MUX			        P13_1_VIDEOSS0_TTL_DSP1_CONTROL2

/* AUDIOSS TDM I2C */
#define CY_AUDIOSS_I2C_TYPE   					    SCB6
#define CY_AUDIOSS_I2C_SDA_PORT   				    GPIO_PRT1
#define CY_AUDIOSS_I2C_SDA_PIN    				    0
#define CY_AUDIOSS_I2C_SDA_PIN_MUX				    P1_0_SCB6_I2C_SDA

#define CY_AUDIOSS_I2C_SCL_PORT   				    GPIO_PRT1
#define CY_AUDIOSS_I2C_SCL_PIN    				    1
#define CY_AUDIOSS_I2C_SCL_PIN_MUX				    P1_1_SCB6_I2C_SCL
#define CY_AUDIOSS_I2C_PCLK   					    PCLK_SCB6_CLOCK
#define CY_AUDIOSS_I2C_IRQN   					    scb_6_interrupt_IRQn

/******************************************************************************/
/******************************************************************************/

#else 	/* No Device or PSVP Selected */

    #error Choose correct device from the device specific Datasheet. 

#endif 	/* #if(defined(CYT4ENDBAS)) */


#ifdef __cplusplus
}
#endif


#endif 	/* BSP_TVIIC2D6MDDR_500BGA_DEVICE_REVA_PSVP_H */


/* [] END OF FILE */
