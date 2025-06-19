/***************************************************************************//**
* \file cyreg_flashc1.h
*
* \brief
* FLASHC1 register definition header
*
* \note
* Generator version: 1.6.0.1423
* Database revision: TVIIBH16M_A0_390780
*
********************************************************************************
* \copyright
* Copyright 2016-2023, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _CYREG_FLASHC1_H_
#define _CYREG_FLASHC1_H_

#include "cyip_flashc1_v2.h"

/**
  * \brief Flash Macro Registers (FLASHC1_FM_CTL_ECT0)
  */
#define CYREG_FLASHC1_FM_CTL            ((volatile un_FLASHC1_FM_CTL_t*) 0x4025F000UL)
#define CYREG_FLASHC1_FM_CODE_MARGIN    ((volatile un_FLASHC1_FM_CODE_MARGIN_t*) 0x4025F004UL)
#define CYREG_FLASHC1_FM_ADDR           ((volatile un_FLASHC1_FM_ADDR_t*) 0x4025F008UL)
#define CYREG_FLASHC1_INTR              ((volatile un_FLASHC1_INTR_t*) 0x4025F020UL)
#define CYREG_FLASHC1_INTR_SET          ((volatile un_FLASHC1_INTR_SET_t*) 0x4025F024UL)
#define CYREG_FLASHC1_INTR_MASK         ((volatile un_FLASHC1_INTR_MASK_t*) 0x4025F028UL)
#define CYREG_FLASHC1_INTR_MASKED       ((volatile un_FLASHC1_INTR_MASKED_t*) 0x4025F02CUL)
#define CYREG_FLASHC1_ECC_OVERRIDE      ((volatile un_FLASHC1_ECC_OVERRIDE_t*) 0x4025F030UL)
#define CYREG_FLASHC1_FM_DATA           ((volatile un_FLASHC1_FM_DATA_t*) 0x4025F040UL)
#define CYREG_FLASHC1_BOOKMARK          ((volatile un_FLASHC1_BOOKMARK_t*) 0x4025F064UL)
#define CYREG_FLASHC1_MAIN_FLASH_SAFETY ((volatile un_FLASHC1_MAIN_FLASH_SAFETY_t*) 0x4025F400UL)
#define CYREG_FLASHC1_STATUS            ((volatile un_FLASHC1_STATUS_t*) 0x4025F404UL)
#define CYREG_FLASHC1_WORK_FLASH_SAFETY ((volatile un_FLASHC1_WORK_FLASH_SAFETY_t*) 0x4025F500UL)

/**
  * \brief Flash Macro Registers (FLASHC10)
  */
#define CYREG_FLASHC1_FLASH_CTL         ((volatile un_FLASHC1_FLASH_CTL_t*) 0x40250000UL)
#define CYREG_FLASHC1_FLASH_PWR_CTL     ((volatile un_FLASHC1_FLASH_PWR_CTL_t*) 0x40250004UL)
#define CYREG_FLASHC1_FLASH_CMD         ((volatile un_FLASHC1_FLASH_CMD_t*) 0x40250008UL)
#define CYREG_FLASHC1_ECC_CTL           ((volatile un_FLASHC1_ECC_CTL_t*) 0x402502A0UL)
#define CYREG_FLASHC1_FM_SRAM_ECC_CTL0  ((volatile un_FLASHC1_FM_SRAM_ECC_CTL0_t*) 0x402502B0UL)
#define CYREG_FLASHC1_FM_SRAM_ECC_CTL1  ((volatile un_FLASHC1_FM_SRAM_ECC_CTL1_t*) 0x402502B4UL)
#define CYREG_FLASHC1_FM_SRAM_ECC_CTL2  ((volatile un_FLASHC1_FM_SRAM_ECC_CTL2_t*) 0x402502B8UL)
#define CYREG_FLASHC1_FM_SRAM_ECC_CTL3  ((volatile un_FLASHC1_FM_SRAM_ECC_CTL3_t*) 0x402502BCUL)
#define CYREG_FLASHC1_CM0_CA_CTL0       ((volatile un_FLASHC1_CM0_CA_CTL0_t*) 0x40250400UL)
#define CYREG_FLASHC1_CM0_CA_CTL1       ((volatile un_FLASHC1_CM0_CA_CTL1_t*) 0x40250404UL)
#define CYREG_FLASHC1_CM0_CA_CTL2       ((volatile un_FLASHC1_CM0_CA_CTL2_t*) 0x40250408UL)
#define CYREG_FLASHC1_CM0_CA_STATUS0    ((volatile un_FLASHC1_CM0_CA_STATUS0_t*) 0x40250440UL)
#define CYREG_FLASHC1_CM0_CA_STATUS1    ((volatile un_FLASHC1_CM0_CA_STATUS1_t*) 0x40250444UL)
#define CYREG_FLASHC1_CM0_CA_STATUS2    ((volatile un_FLASHC1_CM0_CA_STATUS2_t*) 0x40250448UL)
#define CYREG_FLASHC1_CM0_STATUS        ((volatile un_FLASHC1_CM0_STATUS_t*) 0x40250460UL)
#define CYREG_FLASHC1_CM7_0_STATUS      ((volatile un_FLASHC1_CM7_0_STATUS_t*) 0x402504E0UL)
#define CYREG_FLASHC1_CM7_1_STATUS      ((volatile un_FLASHC1_CM7_1_STATUS_t*) 0x40250560UL)
#define CYREG_FLASHC1_CM7_2_STATUS      ((volatile un_FLASHC1_CM7_2_STATUS_t*) 0x40250564UL)
#define CYREG_FLASHC1_CM7_3_STATUS      ((volatile un_FLASHC1_CM7_3_STATUS_t*) 0x40250568UL)
#define CYREG_FLASHC1_CRYPTO_BUFF_CTL   ((volatile un_FLASHC1_CRYPTO_BUFF_CTL_t*) 0x40250580UL)
#define CYREG_FLASHC1_DW0_BUFF_CTL      ((volatile un_FLASHC1_DW0_BUFF_CTL_t*) 0x40250600UL)
#define CYREG_FLASHC1_DW1_BUFF_CTL      ((volatile un_FLASHC1_DW1_BUFF_CTL_t*) 0x40250680UL)
#define CYREG_FLASHC1_DMAC_BUFF_CTL     ((volatile un_FLASHC1_DMAC_BUFF_CTL_t*) 0x40250700UL)
#define CYREG_FLASHC1_SLOW0_MS_BUFF_CTL ((volatile un_FLASHC1_SLOW0_MS_BUFF_CTL_t*) 0x40250780UL)

#endif /* _CYREG_FLASHC1_H_ */


/* [] END OF FILE */
