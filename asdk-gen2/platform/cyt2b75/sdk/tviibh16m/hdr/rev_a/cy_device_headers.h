/***************************************************************************//**
* \file cy_device_headers.h
*
* \brief
* Common header file to be included by the drivers.
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

#ifndef _CY_DEVICE_HEADERS_H_
#define _CY_DEVICE_HEADERS_H_

#if defined (CYT6BJ8DDS)
    #include "cyt6bj8dds.h"
#elif defined (CYT6BJ8DDE)
    #include "cyt6bj8dde.h"
#elif defined (CYT6BJBDHS)
    #include "cyt6bjbdhs.h"
#elif defined (CYT6BJBDHE)
    #include "cyt6bjbdhe.h"
#elif defined (CYT6BJCDHS)
    #include "cyt6bjcdhs.h"
#elif defined (CYT6BJCDHE)
    #include "cyt6bjcdhe.h"
#else
    #error Undefined part number
#endif

#endif /* _CY_DEVICE_HEADERS_H_ */


/* [] END OF FILE */
