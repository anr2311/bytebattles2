/***************************************************************************//**
* \file cy_dp83848q_q1.c
*
* \brief Driver for Ethernet PHY DP83848Q_Q1
* 
*
********************************************************************************
* \copyright
* Copyright 2016-2022, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*****************************************************************************
* Include files
*****************************************************************************/
#include "cy_project.h"
#include "cy_device_headers.h"
#include "cy_dp83848q_q1.h"

/*****************************************************************************
* Local pre-processor symbols/macros ('define')
*****************************************************************************/

/*****************************************************************************
* Global variable definitions (declared in header file with 'extern')
*****************************************************************************/


/*****************************************************************************
* Local type definitions ('typedef')
*****************************************************************************/


/*****************************************************************************
* Local function prototypes ('static')                                                                            
*****************************************************************************/


/*****************************************************************************
* Local variable definitions ('static')
*****************************************************************************/


/*****************************************************************************
* Function implementation - global ('extern') and local ('static')
*****************************************************************************/

/*******************************************************************************
* Function Name: Cy_Dp83848q_Q1_Init
****************************************************************************//**
*
* \brief Initialize an object which accesses DP83848Q_Q1 ethernet PHY.
*
* \param pEth
* Pointer to top of ethernet IP register area.
*
* \param phyAddr
* Address of the ether PHY
*
* \param handle
* The accessing object to be initialized.
*
*******************************************************************************/
void Cy_Dp83848q_Q1_Init(volatile stc_ETH_t* pEth, uint8_t phyAddr, cy_stc_dp83848q_q1_t* handle)
{
    CY_ASSERT(pEth != NULL);

    handle->pEth    = pEth;
    handle->phyAddr = phyAddr;
}

/*******************************************************************************
* Function Name: Cy_Dp83848q_Q1_ReadReg
****************************************************************************//**
*
* \brief Read normal register of the DP83848Q_Q1 ether PHY.
*
* \param reg
* The register information \ref cy_en_dp83848q_q1_reg_info_t
*
* \param handle
* The accessing object
*
* \return
* read value
*
*******************************************************************************/
uint16_t Cy_Dp83848q_Q1_ReadReg(cy_en_dp83848q_q1_reg_info_t reg, cy_stc_dp83848q_q1_t* handle)
{
    CY_ASSERT(handle->pEth != NULL);

    uint16_t addr = reg;
    return Cy_EthIf_PhyRegRead(handle->pEth, addr, handle->phyAddr);
}

/*******************************************************************************
* Function Name: Cy_Dp83848q_Q1_WriteReg
****************************************************************************//**
*
* \brief Write normal register of the DP83848Q_Q1 ether PHY.
*
* \param reg
* The register information \ref cy_en_dp83848q_q1_reg_info_t
*
* \param data
* The value to be written to the register.
*
* \param handle
* The accessing object.
*
*******************************************************************************/
void Cy_Dp83848q_Q1_WriteReg(cy_en_dp83848q_q1_reg_info_t reg, uint16_t data, cy_stc_dp83848q_q1_t* handle)
{
    CY_ASSERT(handle->pEth != NULL);

    uint16_t addr = reg;
    Cy_EthIf_PhyRegWrite(handle->pEth, addr, data, handle->phyAddr);
}


/*******************************************************************************
* Function Name: Cy_Dp83848q_Q1_ConfigurePhy
****************************************************************************//**
*
* \brief Resets and configures the PHY with the provided settings.
* 
* Does not wait for link up, use Cy_Dp83848q_Q1_IsLinkUp to check or poll for it 
* after calling this function
*
* \param phyCfg
* PHY configuration parameters
*
* \param handle
* The accessing object.
*
*******************************************************************************/
void Cy_Dp83848q_Q1_ConfigurePhy(cy_stc_dp83848q_q1_phy_cfg_t * phyCfg,  cy_stc_dp83848q_q1_t* handle)
{
    cy_un_dp83848q_q1_bmcr_t   bmcr;
    cy_un_dp83848q_q1_bmsr_t   bmsr;
    cy_un_dp83848q_q1_anar_t   anar;
    cy_un_dp83848q_q1_rbr_t    rbr;

    // Reset the PHY via its BMCR reg and wait for completion
    bmcr.u16 = 0;
    bmcr.f.u1RESET = 1;
    Cy_Dp83848q_Q1_WriteReg(CY_DP83848Q_Q1_REG_NORMAL_BMCR, bmcr.u16, handle);
    do {
        bmcr.u16 = Cy_Dp83848q_Q1_ReadReg(CY_DP83848Q_Q1_REG_NORMAL_BMCR, handle);
    } while(bmcr.f.u1RESET == 1);

    // Select between MII <-> RMII mode
    rbr.u16 = 0;
    rbr.f.u2ELAST_BUF = 1; // keep default
    if(phyCfg->ifMode == CY_DP83848Q_Q1_INTERFACE_RMII)
    {
        // Enable RMII mode
        rbr.f.u1RMII_MODE = 1;
    }
    else
    {
        // Disable RMII mode (enables MII mode)
        rbr.f.u1RMII_MODE = 0;
    }
    Cy_Dp83848q_Q1_WriteReg(CY_DP83848Q_Q1_REG_NORMAL_RBR, rbr.u16, handle);


    if(phyCfg->enableAutoNeg)
    {
        // Configure advertisement of capabilities based on user configuration 
        // (Only speed! Full duplex is mandatory because MAC does not support half-duplex)
        anar.u16 = 0; // Clear all capabilities    
        anar.f.u5SELECTOR = 1; // default, required

        // Advertise symmetic pause mode capabilities (MAC accepts and can send pause frames)
        anar.f.u1PAUSE = 1;
        //anar.f.u1ASM_DIR = 1;

        if(phyCfg->speedMode == CY_DP83848Q_Q1_SPEED_10)
        {
            anar.f.u1_10_FD = 1;
        }
        else if(phyCfg->speedMode == CY_DP83848Q_Q1_SPEED_100)
        {
            anar.f.u1TX_FD = 1;
        }

        Cy_Dp83848q_Q1_WriteReg(CY_DP83848Q_Q1_REG_NORMAL_ANAR, anar.u16, handle);

        // Restart auto-negotiation and wait until it has been initiated
        bmcr.u16 = 0;
        bmcr.f.u1AUTO_NEGOTIATION_ENABLE  = 1;
        bmcr.f.u1RESTART_AUTO_NEGOTIATION = 1;
        Cy_Dp83848q_Q1_WriteReg(CY_DP83848Q_Q1_REG_NORMAL_BMCR, bmcr.u16, handle);
        do {
            bmcr.u16 = Cy_Dp83848q_Q1_ReadReg(CY_DP83848Q_Q1_REG_NORMAL_BMCR, handle);
        } while(bmcr.f.u1RESTART_AUTO_NEGOTIATION == 1);

        // Wait until auto-negotiation is complete
        do {
            bmsr.u16 = Cy_Dp83848q_Q1_ReadReg(CY_DP83848Q_Q1_REG_NORMAL_BMSR, handle);        
        } while(bmsr.f.u1AUTO_NEGOTIATION_COMPLETE == 0);
    }
    else
    {
        bmcr.u16 = 0;
        bmcr.f.u1DUPLEX_MODE = 1ul;
        if(phyCfg->speedMode == CY_DP83848Q_Q1_SPEED_10)
        {
            bmcr.f.u1SPEED_SELECTION = 0;
        }
        else if(phyCfg->speedMode == CY_DP83848Q_Q1_SPEED_100)
        {
            bmcr.f.u1SPEED_SELECTION = 1;
        }

        Cy_Dp83848q_Q1_WriteReg(CY_DP83848Q_Q1_REG_NORMAL_BMCR, bmcr.u16, handle);
    }
}


/*******************************************************************************
* Function Name: Cy_Dp83848q_Q1_IsLinkUp
****************************************************************************//**
*
* \brief Function checks and returns the link status from a DP83848Q_Q1 register
*
* \param handle
* The accessing object.
* 
* \return true Link up
* \return false Link Down 
*
*******************************************************************************/
bool Cy_Dp83848q_Q1_IsLinkUp(cy_stc_dp83848q_q1_t* handle)
{
    cy_un_dp83848q_q1_bmsr_t bmsr;
    bmsr.u16 = Cy_Dp83848q_Q1_ReadReg(CY_DP83848Q_Q1_REG_NORMAL_BMSR, handle);
    if (bmsr.f.u1LINK_STATUS == 1)
    {
        /** Link up */
        return true;
    }
    else
    {
        /** Link down   */
        return false;
    }
}


/* [] END OF FILE */
