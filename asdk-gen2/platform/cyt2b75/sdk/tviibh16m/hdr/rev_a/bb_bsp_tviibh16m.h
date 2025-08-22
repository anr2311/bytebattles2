/***************************************************************************//**
* \file bb_bsp_tviibh16m.h
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
* Copyright 2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#ifndef _BB_BSP_TVIIBH16M_H_
#define _BB_BSP_TVIIBH16M_H_

#ifdef __cplusplus
extern "C" {
#endif
  
/******************************************************************************/
/******************************************************************************/

// PSVP supports only 320-BGA Package
#if (CY_USE_PSVP == 1) && (defined(CYT6BJCDHE) || defined(CYT6BJCDHS)) && defined(CY_MCU_rev_a)

    #include "bsp\bsp_tviibh16m_320bga_device_reva_psvp.h"
	
// 320-BGA, Rev_A Silicon sample with Rev_A of CPU Board
#elif (CY_USE_PSVP == 0) && (defined(CYT6BJCDHE) || defined(CYT6BJCDHS)) && defined(CY_MCU_rev_a) && defined(CY_320BGA_EVK_rev_a)
	
    #include "bsp\bsp_tviibh16m_320bga_device_reva_evk_reva.h"
  
// 272-BGA, Rev_A Silicon sample with Rev_A of CPU Board
#elif (CY_USE_PSVP == 0) && (defined(CYT6BJBDHE) || defined(CYT6BJBDHS)) && defined(CY_MCU_rev_a) && defined(CY_272BGA_EVK_rev_a)
	
    #include "bsp\bsp_tviibh16m_272bga_device_reva_evk_reva.h"
  
// 176-TEQFP, Rev_A Silicon sample with Rev_A of CPU Board
#elif (CY_USE_PSVP == 0) && (defined(CYT6BJ8DDE) || defined(CYT6BJ8DDS)) && defined(CY_MCU_rev_a) && defined(CY_176TEQFP_EVK_rev_a)
	
    #include "bsp\bsp_tviibh16m_176teqfp_device_reva_evk_reva.h"
       
#else // No Device or PSVP Selected
  
    #error "Choose either a Device based on TVII-B-H-16M Datasheet or use CY_USE_PSVP"
    
#endif // #if (CY_USE_PSVP == 1)
  
/******************************************************************************/
/******************************************************************************/

#ifdef __cplusplus
}
#endif

#endif /* _BB_BSP_TVIIBH16M_H_ */

/* [] END OF FILE */
