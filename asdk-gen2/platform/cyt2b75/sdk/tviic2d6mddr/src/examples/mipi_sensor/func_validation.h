/***************************************************************************//**
* \file func_validation.h
* \version 1.0
*
* Contains some basic testcase for testing the LPDDR4 functionality
*
********************************************************************************
* \copyright
* Copyright 2020-2021, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/******************************************************************************
 * 
 *  This file contains function used only for functional validation and is not part 
 *  of the official driver . These functions are not intended to be delivered 
 *  with the SDL 
 *  
 ******************************************************************************/



#if !defined(__FUNC_VALIDATION_H__)
#define __FUNC_VALIDATION_H__

//#include <math.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>
#include <ctype.h> 
#include "drivers/lpddr4/cy_lpddr4.h"
#include <string.h>

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */


/************************************************************************
 * 
 *   ___ ___  ___  __   ___   _    ___ ___   _ _____ ___ ___  _  _ 
 *  | __/ _ \| _ \ \ \ / /_\ | |  |_ _|   \ /_|_   _|_ _/ _ \| \| |
 *  | _| (_) |   /  \ V / _ \| |__ | || |) / _ \| |  | | (_) | .` |
 *  |_| \___/|_|_\   \_/_/ \_|____|___|___/_/ \_|_| |___\___/|_|\_|
 *        ___ _   _ ___ ___  ___  ___ ___    ___  _  _ _ __   __
 *       | _ | | | | _ | _ \/ _ \/ __| __|  / _ \| \| | |\ \ / /
 *       |  _| |_| |   |  _| (_) \__ | _|  | (_) | .` | |_\ V / 
 *       |_|  \___/|_|_|_|  \___/|___|___|  \___/|_|\_|____|_|  
 * 
 * Functions used during validation only. Normally lpddr4 configuration structure 
 * is used  to initialize the controller. After Initialization there is normally 
 * no need to change :
 *     Addressmapping ,
 *     Route Config,
 *     
 *     .....tbc.
 * 
 * For some settings it´s even not not possible to change them once the Controller IP is enabled 
 * for example QoS.During Validation it might be needed to change some of
 * these values the functions in the following section can be used for 
 * this purpose 
 ****************************************************************************/
/*******************************************************************************
                       Start of LPDDR4 Adddress Mapping Stuff
*******************************************************************************/

/*******************************************************************************
 * Function Name: Cy_Lpddr_SetNewAddrMapByStr
 ***************************************************************************//**
 *
 * \brief Function to set new Address Mapping Scheme in the controller
 * 
 * Function programs a new address mapping schem to the controller. The new address
 * mapping scheme is passed to the function in string format e.g: "1L_3B_15R_10C_0"
 * This function is for testing purpose only !!!!!!!
 * 
 * 
 * \param   base Pointer to the LPDDR4 Base Structure from type 
 *          \ref stc_LPDDR4_t
 * 
 * \param   char *  string pointer to the address mapping string 
 *          in format like "1L_3B_15R_10C_0"
 *
 *  \return \ref  cy_en_lpddr4_retval_t<br>
 *  CY_LPDDR4_SUCCESS if everything was ok<br>
 *  CY_LPDDR4_BAD_PARAM if a wrong parameter was passed to the function<br>
 *  CY_LPDDR4_ERROR invalid address mapping string or out of memory
 * 
 *  * \note 
 * Before programming a new address mapping scheme the controller must be stopped.
 * The address mapping string must follow some rules (sample string "1L_3B_15R_10C_0" )
 *      L=channel   1bit 
 *      B=bank      3bits
 *      R=row       15bits
 *      C=column    10bits
 *      0=unused
 * So 1L_3B_15R_10C_0 = 0 AXI Bit Zero is unused, next 10 AXI bits are used 
 * for column address followed by 15bits for row, 3bits for bank and 1 bit for 
 * channel. 
 *  - no unsed bits in the middle of the string "1L_3B_1_15R_10C_0" --> NOK
 *  - user must take care to provide only valid address mapping strings 
 *  - 
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetNewAddrMapByStr(volatile stc_LPDDR4_t * base,char *  string)
{
    if(NULL == string )
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    char* addressMappingString = (char *)malloc(strlen(string)*sizeof(char));
    if(addressMappingString == NULL)
    {
        return CY_LPDDR4_ERROR;
    }
    /*flag used for detecting if addressline has a alphabetical char or is only number */
    bool isNum = 1;
    
    /* count for row AXI Addressbits */
    uint32_t rowBitCount=0;
    
    /* count for  coloumn AXI Addressbits */
    uint32_t colBitCount=0;

    /* count for bank AXI Addressbits */
    uint32_t bankBitCount=0;

    /* count for channel AXI Addressbits */
    uint32_t channelBitCount=0;

    int i;
    /* this is the current AXI Bit index*/
    uint32_t addressBitIdx=0;
    
    int x=0;
   
    /* this is the digit part of a address string part wwL_xxB_yyC_zzR_ --> ww,xx,yy,zz*/
    char addressLineNumberCoding[2];
   
   
    /* local variable to store the char used in the addressMappingString which gives
    the information of the type off address line L= Channel,C=Column,R=Row,B=Bank */
    char addressLineCharCoding;
   
    /*this will hold the maximal index of addressSchemePartNr
    as extracted from the addressMappingString */
    int addresSchemePartNrIdx=0;
   
    /* array which will hold the substrings after cutting the address mapping string in pieces*/
    char *addresscheme[31];

    /* is a subpart of the addressstring  */
    char *addressSchemePartNr;

    /* this is the separator used in the addressMappinString between different address line areas */
    char separator[] = "_";

    memset(addressMappingString, '\0', sizeof(addressMappingString));
    strcpy(addressMappingString, string);

    //cut the string into pieces
    addresscheme[addresSchemePartNrIdx]= strtok(addressMappingString, separator);
    /*cut the string in parts */
    while(addresscheme[addresSchemePartNrIdx] != NULL)
    {
        addresSchemePartNrIdx++;
        addresscheme[addresSchemePartNrIdx]= strtok(NULL, separator);
    }

    while(addresSchemePartNrIdx!=0)
    {
        addresSchemePartNrIdx--;
        addressSchemePartNr = addresscheme[addresSchemePartNrIdx];
        isNum = 1;
        for(i=0;  i< strlen(addressSchemePartNr) && isNum == 1; i++ )
        {
            //check if the current char is number or alphabetical
            if(isdigit(addressSchemePartNr[i]))
            {
                isNum = 1;
            }else
            {
                isNum = 0;
                addressLineCharCoding = addressSchemePartNr[i];
            }
        }
        if(isNum!=1)
        {
            strncpy(addressLineNumberCoding,addressSchemePartNr,(strlen(addressSchemePartNr)));
        }
        if (isNum)
        {
        if(addressBitIdx==0)
            {
                addressBitIdx=(atoi(addresscheme[addresSchemePartNrIdx]))+1;
            }else
            {
                addressBitIdx+=atoi(addresscheme[addresSchemePartNrIdx]);
            }
        }
        else
        {
            /* if set coloumn address bits when address coding is "C" */
            if((addressLineCharCoding=='C')||(addressLineCharCoding=='c'))
            {
                for( x=0; x<atoi(addressLineNumberCoding);x++)
                {
                    switch(colBitCount)
                    {
                        case 0: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B0 = addressBitIdx;
                                break;
                        case 1: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B1 = addressBitIdx;
                                break;
                        case 2: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B2 = addressBitIdx;
                                break;
                        case 3: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B3 = addressBitIdx;
                                break;
                        case 4: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B4 = addressBitIdx;
                                break;
                        case 5: base->LPDDR4_CORE.unADDR0.stcField.u5COL_ADDR_MAP_B5 = addressBitIdx;
                                break;
                        case 6: base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B6 = addressBitIdx;
                                break;
                        case 7: base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B7 = addressBitIdx;
                                break;
                        case 8: base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B8 = addressBitIdx;
                                break;
                        case 9: base->LPDDR4_CORE.unADDR1.stcField.u5COL_ADDR_MAP_B9 = addressBitIdx;
                                break;
                    }
                    colBitCount++;
                    addressBitIdx++;
                }
            /* if set row address bits  when address coding is "R" */
            }else if((addressLineCharCoding=='R')||(addressLineCharCoding=='r'))
            {
                for( x=0; x<atoi(addressLineNumberCoding);x++)
                {
                    switch(rowBitCount)
                    {
                        case 0: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B0 = addressBitIdx;
                                break;
                        case 1: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B1 = addressBitIdx;
                                break;
                        case 2: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B2 = addressBitIdx;
                                break;
                        case 3: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B3 = addressBitIdx;
                                break;
                        case 4: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B4 = addressBitIdx;
                                break;
                        case 5: base->LPDDR4_CORE.unADDR2.stcField.u5ROW_ADDR_MAP_B5 = addressBitIdx;
                                break;
                        case 6: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B6 = addressBitIdx;
                                break;
                        case 7: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B7 = addressBitIdx;
                                break;
                        case 8: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B8 = addressBitIdx;
                                break;
                        case 9: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B9 = addressBitIdx;
                                break;
                        case 10: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B10 = addressBitIdx;
                                break;
                        case 11: base->LPDDR4_CORE.unADDR3.stcField.u5ROW_ADDR_MAP_B11 = addressBitIdx;
                                break;
                        case 12: base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B12 = addressBitIdx;
                                break;
                        case 13: base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B13 = addressBitIdx;
                                break;
                        case 14: base->LPDDR4_CORE.unADDR4.stcField.u5ROW_ADDR_MAP_B14 = addressBitIdx;
                                break;
                    }
                    rowBitCount++;
                    addressBitIdx++;
                }
            /* if set bank address bits  when address coding is "B" */
            }else if((addressLineCharCoding=='B')||(addressLineCharCoding=='b'))
            {
                for( x=0; x<atoi(addressLineNumberCoding);x++)
                {
        
                    switch(bankBitCount)
                    {
                        case 0: base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B0 = addressBitIdx;
                                break;
                        case 1: base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B1 = addressBitIdx;
                                break;
                        case 2: base->LPDDR4_CORE.unADDR5.stcField.u5BANK_ADDR_MAP_B2 = addressBitIdx;
                                break;
                    }
                    bankBitCount++;
                    addressBitIdx++;
                }
            /* if set channel address bits when address coding is "L" */
            }else if((addressLineCharCoding=='L')||(addressLineCharCoding=='l'))
            {
                for( x=0; x<atoi(addressLineNumberCoding);x++)
                {
                    switch(channelBitCount)
                    {
                        case 0: base->LPDDR4_CORE.unADDR5.stcField.u5CHAN_ADDR_MAP_B0 = addressBitIdx;
                                break;
                    }
                    channelBitCount++;
                    addressBitIdx++;
                }
            }
        }
    }
    if((channelBitCount!=1) &&
       (bankBitCount!=3)&&
       (rowBitCount!=15)&&
       (colBitCount!=10))
       {
           free(addressMappingString);
           return CY_LPDDR4_ERROR;
       }
    free(addressMappingString);
    return CY_LPDDR4_SUCCESS;
}




/*******************************************************************************
*                        Start of LPDDR4 Adddress Mapping Stuff
*******************************************************************************/
/*******************************************************************************
*                        Start of LPDDR4 QoS
*******************************************************************************/

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetCPUSSQoSMasterIdx
 ***************************************************************************//**
 *  \brief Function to set the Priority of a CPUSS Master with Idx
 *  
 *  \note The QoS registers must be configured when IP is Disabled(CTL.ENABLED =0). 
 *  Any attempt to modify the register after IP Enable (CTL.ENABLED =1) 
 *  would get  AHB error response.For running the controller with different QoS
 *  settings Controller must be deacivated ( Cy_Lpddr_ControllerDeinit() 
 *  and initilaized with new settings Cy_Lpddr_ConrollerInit()).
 *  User can call Cy_Lpddr_ConrollerInit() with different config structure or
 *  call Cy_Lpddr_SetCPUSSQoSMasterIdx() to change the settings in a given 
 *  config structure which can then be passed to Cy_Lpddr_ConrollerInit().
 *  
 * 
 *  
 * \param Pointer to a lpddr4 config structure \ref cy_stc_lpddr4_config_t
 * 
 * \param masterIdx is the Id of the Master which QoS priority shall be set 
 *        maximum allowed master idx is CY_MAX_QOS_MASTERID
 * 
 * \param prio from type \ref cy_en_lpddr4_qos_priority_t is the priority which 
 *        will be set for the the selected master.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetCPUSSQoSMasterIdx(cy_stc_lpddr4_config_t * config,uint32_t masterIdx,cy_en_lpddr4_qos_priority_t prio)
{
    /*local copy */
    un_LPDDR4_LPDDR4_WRAPPER_QOS_CPUSS_t unCPUSS_QoS;
    if(NULL==config)
    { 
        return CY_LPDDR4_BAD_PARAM;
    }
    if(masterIdx > CY_LPDDR_MAX_QOS_MASTERID)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*get a local copy of what is set in the lpddr4 configuration structure*/
    unCPUSS_QoS.u32Register = config->unQOS_CPUSS.u32Register;
    /* clear the bits for the selected Master*/
    unCPUSS_QoS.u32Register &= ~(CY_LPDDR_MAX_QOS_PRIO_MASK <<(masterIdx*2));
    /* now set the new priority setting*/
    unCPUSS_QoS.u32Register |=  ((int) prio)<< (masterIdx*2);
    config->unQOS_CPUSS.u32Register = unCPUSS_QoS.u32Register; 
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetVIDEOSS_WR_QoSMasterIdx
 ***************************************************************************//**
 *  \brief Function to set the Priority of a VIDEOSS_WR Master with Idx
 *  
 *  \note The QoS registers must be configured when IP is Disabled(CTL.ENABLED =0). 
 *  Any attempt to modify the register after IP Enable (CTL.ENABLED =1) 
 *  would get  AHB error response.For running the controller with different QoS
 *  settings Controller must be deacivated ( Cy_Lpddr_ControllerDeinit() 
 *  and initilaized with new settings Cy_Lpddr_ConrollerInit()).
 *  User can call Cy_Lpddr_ConrollerInit() with different config structure or
 *  call Cy_Lpddr_SetVIDEOSS_WR_QoSMasterIdx() to change the settings in a given 
 *  config structure which can then be passed to Cy_Lpddr_ConrollerInit().
 *  
 * 
 *  
 * \param Pointer to a lpddr4 config structure \ref cy_stc_lpddr4_config_t
 * 
 * \param masterIdx is the Id of the Master which QoS priority shall be set 
 *        maximum allowed master idx is CY_MAX_QOS_MASTERID
 * 
 * \param prio from type \ref cy_en_lpddr4_qos_priority_t is the priority which 
 *        will be set for the the selected master.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetVIDEOSS_WR_QoSMasterIdx(cy_stc_lpddr4_config_t * config,uint32_t masterIdx,cy_en_lpddr4_qos_priority_t prio)
{
    /*local copy */
    un_LPDDR4_LPDDR4_WRAPPER_QOS_VIDEOSS_WR_t unQOS_VIDEOSS_WR;
    if(NULL==config)
    { 
        return CY_LPDDR4_BAD_PARAM;
    }
    if(masterIdx > CY_LPDDR_MAX_QOS_MASTERID)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*get a local copy of what is set in the lpddr4 configuration structure*/
    unQOS_VIDEOSS_WR.u32Register = config->unQOS_VIDEOSS_WR.u32Register;
    /* clear the bits for the selected Master*/
    unQOS_VIDEOSS_WR.u32Register &= ~(CY_LPDDR_MAX_QOS_PRIO_MASK<<(masterIdx*2));
    /* now set the new priority setting*/
    unQOS_VIDEOSS_WR.u32Register |=  ((int) prio)<< (masterIdx*2);
    config->unQOS_VIDEOSS_WR.u32Register = unQOS_VIDEOSS_WR.u32Register; 
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetVIDEOSS_RD_QoSMasterIdx
 ***************************************************************************//**
 *  \brief Function to set the Priority of a VIDEOSS_WR Master with Idx
 *  
 *  \note The QoS registers must be configured when IP is Disabled(CTL.ENABLED =0). 
 *  Any attempt to modify the register after IP Enable (CTL.ENABLED =1) 
 *  would get  AHB error response.For running the controller with different QoS
 *  settings Controller must be deacivated ( Cy_Lpddr_ControllerDeinit() 
 *  and initilaized with new settings Cy_Lpddr_ConrollerInit()).
 *  User can call Cy_Lpddr_ConrollerInit() with different config structure or
 *  call Cy_Lpddr_SetVIDEOSS_RD_QoSMasterIdx() to change the settings in a given 
 *  config structure which can then be passed to Cy_Lpddr_ConrollerInit().
 *  
 * 
 *  
 * \param Pointer to a lpddr4 config structure \ref cy_stc_lpddr4_config_t
 * 
 * \param masterIdx is the Id of the Master which QoS priority shall be set 
 *        maximum allowed master idx is CY_MAX_QOS_MASTERID
 * 
 * \param prio from type \ref cy_en_lpddr4_qos_priority_t is the priority which 
 *        will be set for the the selected master.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetVIDEOSS_RD_QoSMasterIdx(cy_stc_lpddr4_config_t * config,uint32_t masterIdx,cy_en_lpddr4_qos_priority_t prio)
{
    /*local copy */
    un_LPDDR4_LPDDR4_WRAPPER_QOS_VIDEOSS_RD_t unQOS_VIDEOSS_RD;
    if(NULL==config)
    { 
        return CY_LPDDR4_BAD_PARAM;
    }
    if(masterIdx > CY_LPDDR_MAX_QOS_MASTERID)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*get a local copy of what is set in the lpddr4 configuration structure*/
    unQOS_VIDEOSS_RD.u32Register = config->unQOS_VIDEOSS_RD.u32Register;
    /* clear the bits for the selected Master*/
    unQOS_VIDEOSS_RD.u32Register &= ~(CY_LPDDR_MAX_QOS_PRIO_MASK<<(masterIdx*2));
    /* now set the new priority setting*/
    unQOS_VIDEOSS_RD.u32Register |=  ((int) prio)<< (masterIdx*2);
    config->unQOS_VIDEOSS_RD.u32Register = unQOS_VIDEOSS_RD.u32Register; 
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
*                        End of LPDDR4 QoS
*******************************************************************************/


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetRealtimePrioRtIdx
 ***************************************************************************//**
 *  \brief Function to enable the realtime priority for a route selected by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param rIdx. The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 *
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetRealtimePrioRtIdx( volatile stc_LPDDR4_t * base,uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL==base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ACQ_REALTIME_EN = 0x1;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ACQ_REALTIME_EN = 0x1;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ACQ_REALTIME_EN = 0x1;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ACQ_REALTIME_EN = 0x1;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrRealtimePrioRtIdx
 ***************************************************************************//**
 *  \brief Function to disable the realtime priority for a route selected by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ClrRealtimePrioRtIdx( volatile stc_LPDDR4_t * base,uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL==base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ACQ_REALTIME_EN = 0x0;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ACQ_REALTIME_EN = 0x0;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ACQ_REALTIME_EN = 0x0;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ACQ_REALTIME_EN = 0x0;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetARQ_OOO_RtIdx
 ***************************************************************************//**
 *  \brief Function to set the out of order level for read accesses on route 
 *         specified by rIdx
 * 
 * 
 * 
 * 
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param level - out of order level from type \ref cy_en_lpddr4_route_ooo_state_t
 
 * RTCFGx_RTx_arq_ooo_en    | RTCFGx_RTx_arq_lahead_en  |Descripton
 * ----------------------------------------------------------------------------------
 *              0           |               x           |Out of Order Disable
 *              1           |               0           |Enable Level 1, Disable Level 2
 *              1           |               1           |Enable Level 1 and Level 2
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetARQ_OOO_RtIdx( volatile stc_LPDDR4_t * base,
                                                cy_en_lpddr4_route_ooo_state_t level,
                                                uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_OOO_EN = 0x0;    
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_OOO_EN = 0x1;    
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_LAHEAD_EN = 0x0;    
                    /*base->LPDDR4_CORE.unRTCFG0_RT0.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_OOO_EN = 0x1;
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT0.u32Register |= (1<<24);*/
                }
                break;
        case 1: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_OOO_EN = 0x0;
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_OOO_EN = 0x1;
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_LAHEAD_EN = 0x0;
                    /*base->LPDDR4_CORE.unRTCFG0_RT1.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_OOO_EN = 0x1;
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT1.u32Register |= (1<<24);*/
                }
                break;
        case 2: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_OOO_EN = 0x0;    
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_OOO_EN = 0x1;    
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_LAHEAD_EN = 0x0;    
                    /*base->LPDDR4_CORE.unRTCFG0_RT2.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_OOO_EN = 0x1;  
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_LAHEAD_EN = 0x1;    
                    /*base->LPDDR4_CORE.unRTCFG0_RT2.u32Register |= (1<<24);*/
                }
                break;
        case 3:if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_OOO_EN = 0x0;    
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_OOO_EN = 0x1;    
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_LAHEAD_EN = 0x0;    
                    /*base->LPDDR4_CORE.unRTCFG0_RT3.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_OOO_EN = 0x1;  
                    /*TODO change as soon as the new header file is available */
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_LAHEAD_EN = 0x1;    
                    /*base->LPDDR4_CORE.unRTCFG0_RT3.u32Register |= (1<<24);*/
                } 
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetAWQStarvThresRouteIdx
 ***************************************************************************//**
 *  \brief Function to set the AWQ Starvation Threshold of the route 
 *         specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param stvThresh - AWQ starvation threshold
 * 
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetAWQStarvThresRouteIdx( volatile stc_LPDDR4_t * base,
                                                uint32_t rIdx,
                                                uint32_t stvThresh)
{
   /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    if(255 < stvThresh)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG1_RT0.stcField.u8AWQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 1: 
                base->LPDDR4_CORE.unRTCFG1_RT1.stcField.u8AWQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 2: 
                base->LPDDR4_CORE.unRTCFG1_RT2.stcField.u8AWQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 3:
                base->LPDDR4_CORE.unRTCFG1_RT3.stcField.u8AWQ_STARV_TH = (uint8_t) stvThresh;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetARQStarvThresRouteIdx
 ***************************************************************************//**
 *  \brief Function to set the ARQ Starvation Threshold of the route 
 *         specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param stvThresh - ARQ starvation threshold
 * 
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetARQStarvThresRouteIdx( volatile stc_LPDDR4_t * base,
                                                uint32_t stvThresh,
                                                uint32_t rIdx)
{
   /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    if(255 < stvThresh)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG1_RT0.stcField.u8ARQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 1: 
                base->LPDDR4_CORE.unRTCFG1_RT1.stcField.u8ARQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 2: 
                base->LPDDR4_CORE.unRTCFG1_RT2.stcField.u8ARQ_STARV_TH = (uint8_t) stvThresh;
                break;
        case 3:
                base->LPDDR4_CORE.unRTCFG1_RT3.stcField.u8ARQ_STARV_TH = (uint8_t) stvThresh;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetARQLatBarrierStatusRtIdx
 ***************************************************************************//**
 *  \brief Function to Enable the ARQ Lat Barrier of the route specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param latBarrierStatus  \ref cy_en_lpddr4_onoff_t
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetARQLatBarrierStatusRtIdx( volatile stc_LPDDR4_t * base,
                                                        cy_en_lpddr4_onoff_t latBarrierStatus,
                                                        uint32_t rIdx)
{
   /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1ARQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1ARQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1ARQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1ARQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetAWQLatBarrierStatusRtIdx
 ***************************************************************************//**
 *  \brief Function to Enable the AWQ Lat Barrier of the route specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param latBarrierStatus  \ref cy_en_lpddr4_onoff_t
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetAWQLatBarrierStatusRtIdx( volatile stc_LPDDR4_t * base,
                                                        cy_en_lpddr4_onoff_t latBarrierStatus,
                                                        uint32_t rIdx)
{
   /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_LAT_BARRIER_EN = (uint32_t) latBarrierStatus;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetAWQ_OOO_RtIdx
 ***************************************************************************//**
 *  \brief Function to set the out of order level for write accesses on route 
 *         specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 * 
 * \param level - out of order level from type \ref cy_en_lpddr4_route_ooo_state_t
 * 
 * RTCFGx_RTx_awq_ooo_en    | RTCFGx_RTx_awq_lahead_en  |Descripton
 * ----------------------------------------------------------------------------------
 *              0           |               x           |Out of Order Disable
 *              1           |               0           |Enable Level 1, Disable Level 2
 *              1           |               1           |Enable Level 1 and Level 2
 * 
 * \param rIdx . The controller has 4 routes with index 0..3. 
 * RT with index = 0 is the route for CPUSS the other routes with index 1,2,3
 * are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetAWQ_OOO_RtIdx( volatile stc_LPDDR4_t * base,
                                                cy_en_lpddr4_route_ooo_state_t level,
                                                uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_OOO_EN = 0x0;
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_LAHEAD_EN = 0x0;
                    /*base->LPDDR4_CORE.unRTCFG0_RT0.u32Register &= ~(1<<25);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1AWQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT0.u32Register |= (1<<25);*/
                }
                break;
        case 1: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_OOO_EN = 0x0;
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_LAHEAD_EN = 0x0;
                    /*base->LPDDR4_CORE.unRTCFG0_RT1.u32Register &= ~(1<<25);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1AWQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT1.u32Register |= (1<<25);*/
                }
                break;
        case 2: if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_OOO_EN = 0x0;
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_LAHEAD_EN = 0x0;
                    /*base->LPDDR4_CORE.unRTCFG0_RT2.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1AWQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT2.u32Register |= (1<<24);*/
                }
                break;
        case 3:if( level == CY_LPDDR4_OUT_OF_ORDER_DISABLED)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_OOO_EN = 0x0;
                }else if (level == CY_LPDDR4_OUT_OF_ORDER_LEVEL_1)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_LAHEAD_EN = 0x0;
                    /*base->LPDDR4_CORE.unRTCFG0_RT3.u32Register &= ~(1<<24);*/
                }else if(level == CY_LPDDR4_OUT_OF_ORDER_LEVEL1_AND_LEVEL2)
                {
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_OOO_EN = 0x1;
                    base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1AWQ_LAHEAD_EN = 0x1;
                    /*base->LPDDR4_CORE.unRTCFG0_RT3.u32Register |= (1<<24);*/
                } 
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetExtPriRouteIdx
 ***************************************************************************//**
 *  \brief Function to enable the external priority setting for controller route 
 *         specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetExtPriRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1EXT_PRI = 0x1;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1EXT_PRI = 0x1;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1EXT_PRI = 0x1;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1EXT_PRI = 0x1;
                break;

    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrExtPriRouteIdx
 ***************************************************************************//**
 *  \brief Function to disble the external priority setting for controller route 
 *         specified by rIdx
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ClrExtPriRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1EXT_PRI = 0x0;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1EXT_PRI = 0x0;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1EXT_PRI = 0x0;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1EXT_PRI = 0x0;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetMaxPriRouteIdx
 ***************************************************************************//**
 *  \brief Function to enable max priority setting for controller route 
 *         specified by rIdx
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetMaxPriRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1MAX_PRI = 0x1;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1MAX_PRI = 0x1;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1MAX_PRI = 0x1;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1MAX_PRI = 0x1;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_ClrMaxPriRouteIdx
 ***************************************************************************//**
 *  \brief Function to disable max priority setting for controller route 
 *         specified by rIdx
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_ClrMaxPriRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u1MAX_PRI = 0x0;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u1MAX_PRI = 0x0;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u1MAX_PRI = 0x0;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u1MAX_PRI = 0x0;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDynARQPrioThresRouteIdx
 ***************************************************************************//**
 *  \brief Function to set the read high/low queue priority threshold for the 
 *         route selected by rIdx.
 * 
 *  \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \param arqLvlHi read high queue threshold
 * 
 * \param arqLvlLo read low queue threshold
 * 
 *  \note
 *  Internal  dynamic priority is automatically enabled when external 
 *  priority configuration is disabled.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDynARQPrioThresRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx, 
                                                  uint32_t arqLvlHi,
                                                  uint32_t arqLvlLo)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*max threshold value is CY_LPDDR4_ARQ_AWQ_PRIORITY_THRESHOLD*/
    if(arqLvlHi > CY_LPDDR4_ARQ_AWQ_PRIORITY_THRESHOLD)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*low threshold must be less than high thresh.*/
    if(arqLvlHi <= arqLvlLo)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u4ARQ_LVL_HI = arqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u4ARQ_LVL_LO = arqLvlLo;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u4ARQ_LVL_HI = arqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u4ARQ_LVL_LO = arqLvlLo;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u4ARQ_LVL_HI = arqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u4ARQ_LVL_LO = arqLvlLo;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u4ARQ_LVL_HI = arqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u4ARQ_LVL_LO = arqLvlLo;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDynAWQPrioThresRouteIdx
 ***************************************************************************//**
 *  \brief Function to set the write high/low queue priority threshold for the 
 *         route selected by rIdx.
 *
 *
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param rIdx . The controller has 4 routes with index 0..3. 
 *        RT with index = 0 is the route for CPUSS the other routes with 
 *        index 1,2,3 are for VIDEOSS.
 * 
 * \param awqLvlHi read high queue threshold
 * 
 * \param awqLvlLo read low queue threshold
 * 
 *  \note
 *  Internal  dynamic priority is automatically enabled when external 
 *  priority configuration is disabled.
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDynAWQPrioThresRouteIdx( volatile stc_LPDDR4_t * base,
                                                  uint32_t rIdx,
                                                  uint32_t awqLvlHi,
                                                  uint32_t awqLvlLo)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*check if a valid route (route 0..3) was selected */
    if(CY_LPDDR_MAX_ROUTEIDX < rIdx)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    if(awqLvlHi > CY_LPDDR4_ARQ_AWQ_PRIORITY_THRESHOLD)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /**/
    if(awqLvlHi <= awqLvlLo)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    switch(rIdx)
    {
        case 0: base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u4AWQ_LVL_HI = awqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT0.stcField.u4AWQ_LVL_LO = awqLvlLo;
                break;
        case 1: base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u4AWQ_LVL_HI = awqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT1.stcField.u4AWQ_LVL_LO = awqLvlLo;
                break;
        case 2: base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u4AWQ_LVL_HI = awqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT2.stcField.u4AWQ_LVL_LO = awqLvlLo;
                break;
        case 3: base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u4AWQ_LVL_HI = awqLvlHi;
                base->LPDDR4_CORE.unRTCFG0_RT3.stcField.u4AWQ_LVL_LO = awqLvlLo;
                break;
    }
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDMCTLRdReqMin
 ***************************************************************************//**
 *  \brief Function to set the minimum available read requests which will be 
 *         served before switching to write DMCTL[20:13]
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDMCTLRdReqMin(volatile stc_LPDDR4_t * base,uint32_t rdReqMin)
{
    /*check for valid pointer*/
    if((NULL == base)||(rdReqMin > 255))
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    //Todo change when new header is available
    // uint32_t temp = base->LPDDR4_CORE.unDMCTL.u32Register;
    // temp &= 0xFFE01FFF;
    // temp |= (rdReqMin << 13);
    // base->LPDDR4_CORE.unDMCTL.u32Register = temp;
    base->LPDDR4_CORE.unDMCTL.stcField.u8RD_REQ_MIN = rdReqMin;
    return CY_LPDDR4_SUCCESS;   
}





/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDMCTLWrReqMin
 ***************************************************************************//**
 *  \brief Function to set the minimum available write requests which will be 
 *         served before switching to write DMCTL[28:21]
 * 
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDMCTLWrReqMin(volatile stc_LPDDR4_t * base,uint32_t wrReqMin)
{
    /*check for valid pointer*/
    if((NULL == base)||(wrReqMin>255))
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    //Todo change when new header is available
    // uint32_t temp = base->LPDDR4_CORE.unDMCTL.u32Register;
    // temp &= 0xE01ffFFF;
    // temp |=(wrReqMin << 21);
    // base->LPDDR4_CORE.unDMCTL.u32Register = temp;
    base->LPDDR4_CORE.unDMCTL.stcField.u8WR_REQ_MIN = wrReqMin;
    return CY_LPDDR4_SUCCESS;   
}




/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDMCTLOpenPagePolicy
 ***************************************************************************//**
 *  \brief Function to set the bank policy in DMCTL Register to Open Page Policy
 *         DMCTL[8] will be set 1
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDMCTLOpenPagePolicy(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    base->LPDDR4_CORE.unDMCTL.stcField.u1BANK_POLICY = 0x1;
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDMCTLClosedPagePolicy
 ***************************************************************************//**
 *  \brief Function to set the bank policy in DMCTL Register to Open Page Policy
 *         DMCTL[8] will be cleared 0
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDMCTLClosedPolicy(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    base->LPDDR4_CORE.unDMCTL.stcField.u1BANK_POLICY = 0x0;
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDMCTLBankPolicy
 ***************************************************************************//**
 *  \brief 
 * Function to set the bank policy in DMCTL Register to based on the
 * parameter policy from type \ref cy_en_lpddr4_dmctl_bank_policy_t
 * Depending on policy the DMCTL[8] will be set or cleared
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param policy Bank Policy to set 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDMCTLBankPolicy(volatile stc_LPDDR4_t * base,cy_en_lpddr4_dmctl_bank_policy_t policy)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    base->LPDDR4_CORE.unDMCTL.stcField.u1BANK_POLICY = policy;
    return CY_LPDDR4_SUCCESS;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_EnableDMCTLChUnlock
 ***************************************************************************//**
 *  \brief Function to enable the channel unlcok feature in DMCTL[29] Register 
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_EnableDMCTLChUnlock(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCTL.u32Register |= (1<<29);
    base->LPDDR4_CORE.unDMCTL.stcField.u1CHAN_UNLOCK = 0x1;
    return CY_LPDDR4_SUCCESS;   
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_DisableDMCTLChUnlock
 ***************************************************************************//**
 *  \brief Function to disable the channel unlcok feature in DMCTL[29] Register 
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_DisableDMCTLChUnlock(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCTL.u32Register &= ~(1<<29);
    base->LPDDR4_CORE.unDMCTL.stcField.u1CHAN_UNLOCK = 0x0;
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_DisableDMCTLHiPrioImServ
 ***************************************************************************//**
 *  \brief Function to disable the High Priority Immediately Serving Feature in
 *         DMCTL[30] Register 
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_DisableDMCTLHiPrioImServ(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCTL.u32Register &= ~(1<<30);
    base->LPDDR4_CORE.unDMCTL.stcField.u1HI_PRI_IMM = 0x0;
    return CY_LPDDR4_SUCCESS;
}

/*******************************************************************************
 *  Function Name: Cy_Lpddr_EnableDMCTLHiPrioImServ
 ***************************************************************************//**
 *  \brief Function to enable the High Priority Immediately Serving Feature in
 *         DMCTL[30] Register 
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_EnableDMCTLHiPrioImServ(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCTL.u32Register |= ~(1<<30);
    base->LPDDR4_CORE.unDMCTL.stcField.u1HI_PRI_IMM = 0x1;
    return CY_LPDDR4_SUCCESS;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_EnableInlineECC
 ***************************************************************************//**
 *  \brief Function to enable the Inline ECC Functionality in the 
 *         DMCFG Register . DMCFG[8]
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_EnableInlineECC(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCFG.u32Register |= (1 << LPDDR4_LPDDR4_CORE_DMCFG_INT_ECC_EN_Msk);
    base->LPDDR4_CORE.unDMCFG.stcField.u1INLINE_ECC_EN = 0x1;
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_DisableInlineECC
 ***************************************************************************//**
 *  \brief Function to disable the Inline ECC Functionality in the 
 *         DMCFG Register . DMCFG[8]
 *         
 * 
 * \param Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_DisableInlineECC(volatile stc_LPDDR4_t * base)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*TODO as soon as new header files are available this needs some attention */
    //base->LPDDR4_CORE.unDMCFG.u32Register &= ~(1 << LPDDR4_LPDDR4_CORE_DMCFG_INT_ECC_EN_Msk);
    base->LPDDR4_CORE.unDMCFG.stcField.u1INLINE_ECC_EN = 0x0;
    return CY_LPDDR4_SUCCESS;
}




/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchReadLow
 ***************************************************************************//**
 *  \brief Function to program the READ_LOW in TREG6[9:0] 
 * READ_LOW is the Maximum number of cycles that a low priority read can wait before 
 * initiating the direction switch
 * 
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param readLow the readLow value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchTmrReadLow(volatile stc_LPDDR4_t * base,uint32_t readLow)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*READ_LOW must be always > READ_HIGH*/
    if(readLow > base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_HIGH)
    {
        base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_LOW = readLow;
        return CY_LPDDR4_SUCCESS;
    }
    return CY_LPDDR4_BAD_PARAM;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchReadHigh
 ***************************************************************************//**
 *  \brief Function to program the READ_HIGH in TREG6[19:10] 
 * READ_HIGH is the Maximum number of cycles that a high priority read can 
 * wait before  initiating the direction switch
 * 
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param readHigh the readHIgh value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchReadHigh(volatile stc_LPDDR4_t * base,uint32_t readHigh)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*READ_LOW must be always > READ_HIGH*/
    if(readHigh < base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_LOW)
    {
        base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_HIGH = readHigh;
        return CY_LPDDR4_SUCCESS;
    }
    return CY_LPDDR4_BAD_PARAM;
}




/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchWriteLow
 ***************************************************************************//**
 *  \brief Function to program the WRITE_LOW in TREG6[9:0] 
 * WRITE_LOW is the Maximum number of cycles that a low priority read can wait before 
 * initiating the direction switch
 * 
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param writeLow the readLow value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchWriteLow(volatile stc_LPDDR4_t * base,uint32_t writeLow)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*WRITE_LOW must be always > WRITE_HIGH*/
    if(writeLow > base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_HIGH)
    {
        base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_LOW = writeLow;
        return CY_LPDDR4_SUCCESS;
    }
    return CY_LPDDR4_BAD_PARAM;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchWriteHigh
 ***************************************************************************//**
 *  \brief Function to program the WRITE_HIGH in TREG6[19:10] 
 * WRITE_HIGH is the Maximum number of cycles that a high priority write can 
 * wait before  initiating the direction switch
 * 
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param writeHigh the writeHigh value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchWriteHigh(volatile stc_LPDDR4_t * base,uint32_t writeHigh)
{
    /*check for valid pointer*/
    if(NULL == base)
    {
        return CY_LPDDR4_BAD_PARAM;
    }
    /*WRITE_LOW must be always > WRITE_HIGH*/
    if(writeHigh < base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_LOW)
    {
        base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_HIGH = writeHigh;
        return CY_LPDDR4_SUCCESS;
    }
    return CY_LPDDR4_BAD_PARAM;
}



/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchWriteThlds
 ***************************************************************************//**
 *  \brief Function to program the WRITE direction threholds
 * Function to program the thresholds for direction switching in write mode 
 * WRITE_LOW  = TREG8[9:0] Maximum number of cycles that a low priority write can 
 * wait before  initiating the direction switch
 * 
 * WRITE_HIGH = TREG8[19:10]  Maximum number of cycles that a high priority 
 * write can wait before initiating the direction switch
 * 
 * \note WRITE_LOW must be programmed > WRITE_HIGH
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param writeHigh the writeHigh value which shall be programmed 
 * 
 * \param writeLow the writeLow value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchWriteThlds(volatile stc_LPDDR4_t * base,uint32_t writeHigh,uint32_t writeLow)
{
    /*check for valid pointer*/
    if((NULL == base)||(writeLow < writeHigh)||(writeLow > 1024)||(writeHigh > 1024))
    {
        return CY_LPDDR4_BAD_PARAM;

    }
    base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_LOW = writeLow;
    base->LPDDR4_CORE.unTREG8.stcField.u10T_WRITE_HIGH = writeHigh;
    return CY_LPDDR4_SUCCESS;
}


/*******************************************************************************
 *  Function Name: Cy_Lpddr_SetDirSwtchReadThlds
 ***************************************************************************//**
 *  \brief Function to program the READ direction threholds
 * Function to program the thresholds for direction switching in read mode 
 * READ_LOW  = TREG6[9:0] Maximum number of cycles that a low priority read can 
 * wait before initiating the direction switch
 * 
 * READ_HIGH = TREG6[19:10] Maximum number of cycles that a high priority read 
 * can wait before initiating the direction switch
 * 
 * \note READ_LOW must be programmed > READ_HIGH
 * 
 * \param base Pointer to the LPDDR4 Base Structure from type \ref stc_LPDDR4_t
 *  
 * \param readHigh the readHigh value which shall be programmed 
 * 
 * \param readLow the readLow value which shall be programmed 
 * 
 * \return \ref cy_en_lpddr4_retval_t
 * CY_LPDDR4_BAD_PARAM if a wrong parameter was passed
 * CY_LPDDR4_SUCCESS if everythink was ok
 ******************************************************************************/
cy_en_lpddr4_retval_t Cy_Lpddr_SetDirSwtchReadThlds(volatile stc_LPDDR4_t * base,uint32_t readHigh,uint32_t readLow)
{
    /*check for valid pointer*/
    if((NULL == base)||(readLow < readHigh)||(readLow > 1024)||(readHigh > 1024))
    {
        return CY_LPDDR4_BAD_PARAM;

    }
    base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_LOW = readLow;
    base->LPDDR4_CORE.unTREG6.stcField.u10T_READ_HIGH = readHigh;
    return CY_LPDDR4_SUCCESS;
}












/*******************************************************************************
 * 
 * ___         _    ___  ___  __   __    _ _    _      _   _          
 *| __|_ _  __| |  / _ \| __| \ \ / __ _| (_)__| |__ _| |_(_)___ _ _  
 *| _|| ' \/ _` | | (_) | _|   \ V / _` | | / _` / _` |  _| / _ | ' \ 
 *|___|_||_\__,_|  \___/|_|     \_/\__,_|_|_\__,_\__,_|\__|_\___|_||_|
 *                             ___ _         __  __ 
 *                           / __| |_ _  _ / _|/ _|
 *                           \__ |  _| || |  _|  _|
 *                           |___/\__|\_,_|_| |_|  
 ********************************************************************************/

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* __FUNC_VALIDATION_H__ */

/** \} group_lpddr */

/* [] END OF FILE */
