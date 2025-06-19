/***************************************************************************//**
* \file bb_bsp_tvic2d6mddr.h
*
* \brief
* Project specific header
*
* \note
* It contains references to all generated header files and should
* not be modified.
*
********************************************************************************
* \copyright
* Copyright 2018-2021, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef BB_BSP_TVIIC2D6MDDR_H
#define BB_BSP_TVIIC2D6MDDR_H

#ifdef __cplusplus
extern "C" {
#endif
  
/******************************************************************************/
/******************************************************************************/

// PSVP supports only 500-BGA Package
#if (CY_USE_PSVP == 1) && defined(CYT4ENDBAS) && defined(CY_MCU_rev_a)

    #include "bsp\bsp_tviic2d6mddr_500bga_device_reva_psvp.h"
	
// 500-BGA, Rev_A Silicon sample with Rev_A of CPU Board
#elif (CY_USE_PSVP == 0) && \
      (defined(CYT4ENDBAS) || defined(CYT4ENDBCS) || defined(CYT4ENDBES) || defined(CYT4ENDBGS) || \
       defined(CYT4ENDBJS) || defined(CYT4ENDBLS) || defined(CYT4ENDBNS) || defined(CYT4ENDBQS)) \
      && defined(CY_MCU_rev_a) && defined(CY_500BGA_EVK_rev_a)
	
    #include "bsp\bsp_tviic2d6mddr_500bga_device_reva_evk_reva.h"
       
#else // No Device or PSVP Selected
  
    #error "Choose either a Device based on TVII-C-2D-6M-DDR Datasheet or CY_USE_PSVP"
    
#endif // #if (CY_USE_PSVP == 1)
  
/******************************************************************************/
/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* BB_BSP_TVIIC2D6MDDR_H */

/* [] END OF FILE */
