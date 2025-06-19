/***************************************************************************//**
* \file cy_seglcd.c
* \version 1.0.2
*
* \brief
* Provides an API implementation of the SegLCD driver
*
********************************************************************************
* \copyright
* Copyright 2018-2021, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

#include "cy_seglcd.h"

#if defined(CY_IP_MXLCD)

#define CY_SEGLCD_ENABLE_Msk                            (LCD_CONTROL_LS_EN_Msk | LCD_CONTROL_HS_EN_Msk)
#define CY_SEGLCD_NIBBLE                                (4UL)

#define CY_SEGLCD_DIV_MAX                               (0x0000FFFFUL) /* The max. divider values */
#define CY_SEGLCD_DIV_MAX_LS_VER1                       (0x000000FFUL) /* The max. divider values for Low Speed mode for MXLCD_ver1 */

#define CY_SEGLCD_SYM_BIT_IDX_Pos                       (0U)
#define CY_SEGLCD_SYM_BIT_IDX_Msk                       ((uint32_t) 0x00000007UL << CY_SEGLCD_SYM_BIT_IDX_Pos)
#define CY_SEGLCD_SYM_BYTE_IDX_Pos                      (3U)
#define CY_SEGLCD_SYM_BYTE_IDX_Msk                      ((uint32_t) 0x00001FFFUL << CY_SEGLCD_SYM_BYTE_IDX_Pos)

#define CY_SEGLCD_PIX_Pos(pixel)                        ((CY_SEGLCD_EXTR_NBL(pixel) * CY_SEGLCD_NIBBLE) + \
                                                         (CY_SEGLCD_EXTR_COM(pixel) % CY_SEGLCD_NIBBLE))
#define CY_SEGLCD_PIX_Msk(pixel)                        ((uint32_t)(1UL << CY_SEGLCD_PIX_Pos(pixel)))

#define CY_SEGLCD_REVISION(base)                        (_FLD2VAL(LCD_ID_REVISION, base->unID.u32Register))
#define CY_SEGLCD_REV_1(base)                           (1UL == CY_SEGLCD_REVISION(base))
#define CY_SEGLCD_IS_ENABLED(base)                      (0UL != (base->unCONTROL.u32Register & CY_SEGLCD_ENABLE_Msk))
#define CY_SEGLCD_IS_MSK_VALID(pixel)                   (0UL == ((pixel) & ~(CY_SEGLCD_COM_Msk | CY_SEGLCD_OCT_Msk | CY_SEGLCD_NBL_Msk)))
#define CY_SEGLCD_IS_COM_VALID(pixel)                   (LCD_CHIP_TOP_COM_NR > CY_SEGLCD_EXTR_COM(pixel))
#define CY_SEGLCD_IS_PIX_VALID(pixel)                   (CY_SEGLCD_IS_MSK_VALID(pixel) && CY_SEGLCD_IS_COM_VALID(pixel))


#define CY_SEGLCD_COM_NUM_MIN                           (2U)
#define CY_SEGLCD_COM_NUM_MAX                           (LCD_CHIP_TOP_COM_NR)
#define CY_SEGLCD_IS_COM_NUM_VALID(num)                 ((CY_SEGLCD_COM_NUM_MIN <= (num)) && ((num) <= CY_SEGLCD_COM_NUM_MAX))

#define CY_SEGLCD_IS_BASIC(type)                        (((uint16_t)CY_SEGLCD_7SEG  == (type)) || \
                                                         ((uint16_t)CY_SEGLCD_14SEG == (type)) || \
                                                         ((uint16_t)CY_SEGLCD_16SEG == (type)) || \
                                                         ((uint16_t)CY_SEGLCD_5X8DM == (type)))

#define CY_SEGLCD_IS_PIXMAP(disp)                       ((NULL != (disp)) && \
                                                         (NULL != (disp)->pixMap))

#define CY_SEGLCD_IS_SYMBOLIC(disp, pos)                (CY_SEGLCD_IS_PIXMAP(disp) && \
                                                         (NULL != (disp)->font) && \
                                                         (NULL != (disp)->font->fontMap) && \
                                                         ((uint32_t)CY_SEGLCD_BAR < (disp)->type) && \
                                                         (((pos) < (disp)->symNum)))

#define CY_SEGLCD_IS_ASCII(disp, pos)                   (CY_SEGLCD_IS_SYMBOLIC(disp, pos) && \
                                                         ((disp)->font->ascii))

#define CY_SEGLCD_IS_BARGRAPH(disp, pos)                (CY_SEGLCD_IS_PIXMAP(disp) && \
                                                         ((uint32_t)CY_SEGLCD_BAR == (disp)->type) && \
                                                         (((pos) < (disp)->symNum)))

#define CY_SEGLCD_IS_SPEED_VALID(speed)                 ((CY_SEGLCD_SPEED_LOW  == (speed)) || \
                                                         (CY_SEGLCD_SPEED_HIGH == (speed)))

#define CY_SEGLCD_IS_WAVE_VALID(wave)                   ((CY_SEGLCD_TYPE_A == (wave)) || \
                                                         (CY_SEGLCD_TYPE_B == (wave)))

#define CY_SEGLCD_IS_DRIVE_VALID(drive)                 ((CY_SEGLCD_PWM         == (drive)) || \
                                                         (CY_SEGLCD_CORRELATION == (drive)))

#define CY_SEGLCD_IS_BIAS_4_5_SUPPORTED(base,speed)     ((CY_SEGLCD_REV_1(base)) ? (CY_SEGLCD_SPEED_HIGH == (speed)) : true)

#define CY_SEGLCD_IS_BIAS_VALID(base,bias,speed)        ((CY_SEGLCD_BIAS_HALF   == (bias))  || \
                                                         (CY_SEGLCD_BIAS_THIRD  == (bias))  || \
         (CY_SEGLCD_IS_BIAS_4_5_SUPPORTED(base,speed) && (CY_SEGLCD_BIAS_FOURTH == (bias))) || \
         (CY_SEGLCD_IS_BIAS_4_5_SUPPORTED(base,speed) && (CY_SEGLCD_BIAS_FIFTH  == (bias))))

#define CY_SEGLCD_IS_LSCLK_VALID(lsClk)                 ((CY_SEGLCD_LSCLK_LF == (lsClk)) || \
                                                         (CY_SEGLCD_LSCLK_MF == (lsClk)))

#define CY_SEGLCD_FR_RATE_MIN                           (30U)
#define CY_SEGLCD_FR_RATE_MAX                           (150U)
#define CY_SEGLCD_IS_RATE_VALID(frRate)                 (((frRate) >= CY_SEGLCD_FR_RATE_MIN) && ((frRate) <= CY_SEGLCD_FR_RATE_MAX))

#define CY_SEGLCD_CNTR_MAX                              (100UL)
#define CY_SEGLCD_IS_CNTR_VALID(contrast)               (((uint32_t)(contrast)) <= CY_SEGLCD_CNTR_MAX)

#define CY_SEGLCD_FREQ_MIN                              (10000UL)
#define CY_SEGLCD_FREQ_MAX                              (100000000UL)
#define CY_SEGLCD_IS_FREQ_VALID(freq)                   (((freq) >= CY_SEGLCD_FREQ_MIN) && ((freq) <= CY_SEGLCD_FREQ_MAX))

#define CY_SEGLCD_SPACE(disp)                           (((disp)->font->ascii) ? ' ' : CY_SEGLCD_NUM_BLANK)

/* The timeout value for Low Speed mode enable/disable procedures, two ILO clock periods, in uSeconds */
#define CY_SEGLCD_TIMEOUT                               ((2U * 1000000UL) / 32768UL)


/* Local function declaration */
static char_t NumToChar(uint32_t value);
static void InvClrData(stc_LCD_t * base, bool inv);
static volatile uint32_t * GetDataRegPtr(stc_LCD_t * base, uint32_t pixel);


/*******************************************************************************
* Function Name: GetDataRegPtr
****************************************************************************//**
*
* Returns a pointer to the data frame register for a specified pixel.
* Used by \ref Cy_SegLCD_WritePixel and \ref Cy_SegLCD_ReadPixel functions.
*
* \param base  The base pointer to the LCD instance registers.
* \param pixel The predefined packed number that points to the pixel location
*              in the frame buffer.
*
*******************************************************************************/
static volatile uint32_t * GetDataRegPtr(stc_LCD_t * base, uint32_t pixel)
{
    uint32_t * locDataPtr[] = {&base->unDATA0[0].u32Register, &base->unDATA1[0].u32Register, &base->unDATA2[0].u32Register, &base->unDATA3[0].u32Register};

    return (&(locDataPtr[CY_SEGLCD_EXTR_COM(pixel) / CY_SEGLCD_NIBBLE][CY_SEGLCD_EXTR_OCT(pixel)]));
}

/*******************************************************************************
* Function Name: InvClrData
****************************************************************************//**
*
* Inverts or clears the Frame Data buffer.
* Used by \ref Cy_SegLCD_Deinit,
*         \ref Cy_SegLCD_ClrFrame and
*         \ref Cy_SegLCD_InvFrame functions.
*
* \param base  The base pointer to the LCD instance registers.
* \param inv:
*    - true - inverts the data buffer,
*    - false - clears the data buffer.
*
*******************************************************************************/
static void InvClrData(stc_LCD_t * base, bool inv)
{
    uint32_t i;

    // rmkn: TBC LCD_OCTET_NUM->CY_SEGLCD_OCTET
    for (i = 0UL; i < CY_SEGLCD_OCTET; i++)
    {
        base->unDATA0[i].u32Register = (inv) ? (~base->unDATA0[i].u32Register) : 0UL;

        if (0UL != LCD_OCTET_NUM_8)
        {
            base->unDATA1[i].u32Register = (inv) ? (~base->unDATA1[i].u32Register) : 0UL;

            if (0UL != LCD_OCTET_NUM_16)
            {
                base->unDATA2[i].u32Register = (inv) ? (~base->unDATA2[i].u32Register) : 0UL;
                base->unDATA3[i].u32Register = (inv) ? (~base->unDATA3[i].u32Register) : 0UL;
            }
        }
    }
}

/*******************************************************************************
* Function Name: NumToChar
****************************************************************************//**
*
* Converts an one-digit hexadecimal numbers 0x0...0xF
* into the ASCII character codes.
* Used by the \ref Cy_SegLCD_WriteNumber function.
*
* \param value The value to be converted.
* \return The ASCII code of the 'value' character.
*
*******************************************************************************/
static char_t NumToChar(uint32_t value)
{
    char_t character;

    if (value < 10UL) /* For dec numbers 0...9 */
    {
        character = (char_t)((uint32_t)'0' + value);
    }
    else if (value < 0x10UL) /* For hex numbers A...F */
    {
        character = (char_t)(((uint32_t)'A' - 0xAUL) + value);
    }
    else
    {
        character = ' '; /* The blank/space character */
    }

    return (character);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_Init
****************************************************************************//**
*
* Initializes/restores the default Segment LCD block configuration.
*
* \param base   The base pointer to the LCD instance registers.
* \param config The pointer to a configuration structure.
* \return \ref cy_en_seglcd_status_t.
*
* Side Effects: The block is disabled to change the settings.
*
* \funcusage 
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Config
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Init
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_Init(stc_LCD_t * base, cy_stc_seglcd_config_t const * config)
{
    cy_en_seglcd_status_t retVal = CY_SEGLCD_BAD_PARAM;
    
    if (NULL != base)
    {
        un_LCD_CONTROL_t tLCD_CONTROL = {0};
        tLCD_CONTROL.stcField.u1LCD_MODE = config->speed;
        tLCD_CONTROL.stcField.u1TYPE = config->wave;
        tLCD_CONTROL.stcField.u1OP_MODE = config->drive;
        tLCD_CONTROL.stcField.u2BIAS = config->bias;
        tLCD_CONTROL.stcField.u4COM_NUM = config->comNum - 2UL;
        base->unCONTROL.u32Register = tLCD_CONTROL.u32Register;

        /* Calculate the sub-frame and dead-time dividers */        
        un_LCD_DIVIDER_t tLCD_DIVIDER = {0};
        tLCD_DIVIDER.stcField.u16SUBFR_DIV = CY_SYSLIB_DIV_ROUND((CY_SYSLIB_DIV_ROUND(config->clkFreq, config->frRate * 4UL * config->comNum) - 1UL) * config->contrast, 100UL);
        tLCD_DIVIDER.stcField.u16DEAD_DIV = CY_SYSLIB_DIV_ROUND(CY_SYSLIB_DIV_ROUND(config->clkFreq * ((uint32_t)(100UL - (uint32_t)config->contrast)), (uint32_t)config->frRate), 100UL);
        base->unDIVIDER.u32Register = tLCD_DIVIDER.u32Register;
        retVal = CY_SEGLCD_SUCCESS;
    }
    
    return (retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_Contrast
****************************************************************************//**
*
* Sets a specified contrast.
*
* \param base   The base pointer to the LCD instance registers.
* \param contrast The contrast value to be set.
* \param config The pointer to a configuration structure.
* \return \ref cy_en_seglcd_status_t.
*
* Side Effects: The configuration structure contrast value is also updated.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_ContrastInit
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Contrast
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_Contrast(stc_LCD_t * base, uint32_t contrast, cy_stc_seglcd_config_t * config)
{
    cy_en_seglcd_status_t retVal = CY_SEGLCD_BAD_PARAM;

    if (CY_SEGLCD_IS_CNTR_VALID (contrast))
    {
        bool enabled = CY_SEGLCD_IS_ENABLED(base); /* Store the block state */

        config->contrast = (uint8_t)contrast;

        retVal = Cy_SegLCD_Init(base, config);

        if ((CY_SEGLCD_SUCCESS == retVal) && enabled)
        {
            Cy_SegLCD_Enable(base); /* Restore the block state */
        }
    }

    return (retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_Deinit
****************************************************************************//**
*
* De-initializes the LCD block (resets the block registers to default state).
*
* \param base   The base pointer to the LCD instance registers.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Deinit
*
*******************************************************************************/
void Cy_SegLCD_Deinit(stc_LCD_t * base)
{
    Cy_SegLCD_Disable(base);

    base->unCONTROL.u32Register = 0UL;
    base->unDIVIDER.u32Register = 0UL;
    
    InvClrData(base, false); /* Clear the entire frame buffer to all zeroes */
}

/*******************************************************************************
* Function Name: Cy_SegLCD_Enable
****************************************************************************//**
*
* Enables the Segment LCD block.
*
* \param base The base pointer to the LCD instance registers.
*
* \funcusage 
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Config
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Init
*
*******************************************************************************/
void Cy_SegLCD_Enable(stc_LCD_t * base)
{
    uint32_t ctrlStatus = base->unCONTROL.u32Register;
    if (_FLD2BOOL(LCD_CONTROL_LCD_MODE, ctrlStatus))
    {
        base->unCONTROL.u32Register |= LCD_CONTROL_HS_EN_Msk;
    }
    else /* CY_SEGLCD_SPEED_HIGH */
    {
        uint32_t timeout;
        base->unCONTROL.u32Register |= LCD_CONTROL_LS_EN_Msk;
        ctrlStatus = base->unCONTROL.u32Register;

        for (timeout = CY_SEGLCD_TIMEOUT; (_FLD2BOOL(LCD_CONTROL_LS_EN_STAT, ctrlStatus)) && (0UL != timeout); timeout--)
        {
            /* Waiting for enabling takes effect in the ILO clock domain.
             * The timeout status does not affect anything,
             * the timeout is to avoid a deadlock here.
             */
            Cy_SysLib_DelayUs(1U);
        }
    }
}

/*******************************************************************************
* Function Name: Cy_SegLCD_Disable
****************************************************************************//**
*
* Disables the Segment LCD block.
*
* \param base The base pointer to the LCD instance registers.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Disable
*
*******************************************************************************/
void Cy_SegLCD_Disable(stc_LCD_t * base)
{
    uint32_t ctrlStatus = base->unCONTROL.u32Register;
    if (_FLD2BOOL(LCD_CONTROL_LCD_MODE, ctrlStatus)) /* CY_SEGLCD_SPEED_HIGH */
    {
        base->unCONTROL.u32Register &= ~LCD_CONTROL_HS_EN_Msk;
    }
    else /* CY_SEGLCD_SPEED_LOW */
    {
        if (_FLD2BOOL(LCD_CONTROL_LS_EN, ctrlStatus))
        {
            uint32_t timeout;

            for (timeout = CY_SEGLCD_TIMEOUT; (!_FLD2BOOL(LCD_CONTROL_LS_EN_STAT, ctrlStatus)) && (0UL != timeout); timeout--)
            {
                 Cy_SysLib_DelayUs(1U); /* Waiting for enabling takes effect in the ILO clock domain. */
            }

            base->unCONTROL.u32Register &= ~LCD_CONTROL_LS_EN_Msk;
            ctrlStatus = base->unCONTROL.u32Register;

            for (timeout = CY_SEGLCD_TIMEOUT; (_FLD2BOOL(LCD_CONTROL_LS_EN_STAT, ctrlStatus)) && (0UL != timeout); timeout--)
            {
                Cy_SysLib_DelayUs(1U); /* Waiting for disabling takes effect in the ILO clock domain. */
            }

            /* The timeout status does not affect anything, the timeout is to avoid a deadlock */
        }
    }
}

/*******************************************************************************
* Function Name: Cy_SegLCD_ClrFrame
****************************************************************************//**
*
* Clears the frame buffer and initiates the common lines.
* In general case it is recommended to be called after \ref Cy_SegLCD_Init 
*                                           and before \ref Cy_SegLCD_Enable.
*
* \param base    The base pointer to the LCD instance registers.
* \param commons The pointer to array of common lines.
*                The array size is specified by \ref cy_stc_seglcd_config_t.comNum.
*                Each common line value should be made using
*                \ref CY_SEGLCD_COMMON macro.
*
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage 
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Config
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Init
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_ClrFrame(stc_LCD_t * base, uint32_t const * commons)
{
    uint32_t retVal = (uint32_t)CY_SEGLCD_BAD_PARAM;

    if (NULL != commons)
    {
        uint32_t i;

        InvClrData(base, false); /* Clear the entire frame buffer to all zeroes */

        retVal = (uint32_t)CY_SEGLCD_SUCCESS;

        /* Re-initialize the commons */
        for (i = 0UL; i < CY_SEGLCD_COM_NUM(base); i++)
        {
            retVal |= (uint32_t)Cy_SegLCD_WritePixel(base, commons[i], true);
        }
    }

    return((cy_en_seglcd_status_t)retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_InvFrame
****************************************************************************//**
*
* Inverts the frame buffer (all the connected pixel states are inverted).
*
* \param base    The base pointer to the LCD instance registers.
* \param commons The pointer to an array of common lines.
*                The array size is specified by \ref cy_stc_seglcd_config_t.comNum.
*                Each common line value should be made using
*                \ref CY_SEGLCD_COMMON macro.
*
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_InvFrame
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_InvFrame(stc_LCD_t * base, uint32_t const * commons)
{
    uint32_t retVal = (uint32_t)CY_SEGLCD_BAD_PARAM;
    
    if (NULL != commons)
    {
        uint32_t i;

        InvClrData(base, true); /* Invert the entire frame buffer. */

        retVal = (uint32_t)CY_SEGLCD_SUCCESS;

        /* Re-initialize the common lines */
        for (i = 0UL; i < CY_SEGLCD_COM_NUM(base); i++)
        {
            uint32_t locCom = commons[i] & ~CY_SEGLCD_COM_Msk;
            uint32_t j = 0UL;

            /* Clear common lines data after frame buffer inverting */
            for (j = 0UL; j < CY_SEGLCD_COM_NUM(base); j++)
            {
                retVal |= (uint32_t)Cy_SegLCD_WritePixel(base, locCom | ((uint32_t)(j << CY_SEGLCD_COM_Pos)), false);
            }

            /* Set data for the common lines */
            retVal |= (uint32_t)Cy_SegLCD_WritePixel(base, commons[i], true);
        }
    }
    
    return((cy_en_seglcd_status_t)retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_WritePixel
****************************************************************************//**
*
*  Sets or clears a specified pixel.
*
* \param base The base pointer to the LCD instance registers.
* \param pixel The predefined packed number that points to the pixel location
*              in the frame buffer.
* \param value Specifies the pixel on/off state.
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.h snippet_Cy_SegLCD_DefPixel
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_ActPixel
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_WritePixel(stc_LCD_t * base, uint32_t pixel, bool value)
{
    cy_en_seglcd_status_t retVal = CY_SEGLCD_BAD_PIXEL;
    
    if (CY_SEGLCD_IS_PIX_VALID(pixel))
    {
        /* Extract the pixel location. */
        uint32_t loc_Pos = CY_SEGLCD_PIX_Pos(pixel);
        uint32_t loc_Msk = CY_SEGLCD_PIX_Msk(pixel);

        /* Write a new pixel value into the frame buffer. */
        CY_REG32_CLR_SET(*GetDataRegPtr(base, pixel), loc, value ? 1UL : 0UL);

        retVal = CY_SEGLCD_SUCCESS;
    }

    return(retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_ReadPixel
****************************************************************************//**
*
* Gets the state of a specified pixel.
*
* \param base The base pointer to the LCD instance registers.
* \param pixel The predefined packed number that points to the pixel location
*              in the frame buffer. Each pixel value should be made using
*              \ref CY_SEGLCD_PIXEL macro.
* \return Boolean pixel state.
*         If pixel value is invalid - the 'false' is returned.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.h snippet_Cy_SegLCD_DefPixel
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_ActPixel
*
*******************************************************************************/
bool Cy_SegLCD_ReadPixel(stc_LCD_t * base, uint32_t pixel)
{
    bool retVal = false;
    
    if (CY_SEGLCD_IS_PIX_VALID(pixel))
    {
        /* Get the pixel value from the frame buffer */
        retVal = (0UL != (*GetDataRegPtr(base, pixel) & CY_SEGLCD_PIX_Msk(pixel)));
    }
    else
    {
        CY_ASSERT_L1(false);
    }

    return (retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_WriteChar
****************************************************************************//**
*
* Writes a specified character onto a specified display.
* Supports all display types except \ref CY_SEGLCD_BAR.
*
* \param base The base pointer to the LCD instance registers.
* \param character The code of the character to display.
*                  Should be within the font symbol codes range specified by
*           \ref cy_stc_seglcd_font_t.first and \ref cy_stc_seglcd_font_t.last
* \param position The position of the character/digit on display. 
*                 Zero is the most left character/digit of the specified
*                 display \ref cy_stc_seglcd_disp_t.
* \param display The pointer to the display structure \ref cy_stc_seglcd_disp_t.
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_Char
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_WriteChar(stc_LCD_t * base,
                                            char_t   character,
                                          uint32_t   position,
                        cy_stc_seglcd_disp_t const * display)
{
    uint32_t retVal = (uint32_t)CY_SEGLCD_BAD_PARAM;

    if (CY_SEGLCD_IS_SYMBOLIC(display, position)) /* If the display is symbolic/numeric */
    {
        uint32_t i;
        retVal = CY_SEGLCD_IS_BASIC(display->type) ? (uint32_t)CY_SEGLCD_SUCCESS : (uint32_t)CY_SEGLCD_CUSTOM;

        if ((character < display->font->first) || (character > display->font->last)) /* if out of the font range */
        {
            retVal |= (uint32_t)CY_SEGLCD_BAD_CHAR;
            character = CY_SEGLCD_SPACE(display); /* The blank (space) character */
        }

        character = (char_t)(character - display->font->first); /* Shift to the font char map starting index */

        for (i = 0UL; i < (uint32_t)display->type; i++)
        {
            uint32_t locPix = *(display->pixMap + (position * (uint32_t)display->type) + i); /* Current display pixel */
            uint32_t locIdx = _FLD2VAL(CY_SEGLCD_SYM_BYTE_IDX, i) + ((uint32_t)character * CY_SYSLIB_DIV_ROUNDUP((uint32_t)display->type, CY_SEGLCD_OCTET)); /* current symbol byte index */
            bool     locVal = 0U != (display->font->fontMap[locIdx] & (uint8_t)(1U << _FLD2VAL(CY_SEGLCD_SYM_BIT_IDX, i))); /* current pixel value */

            retVal |= (uint32_t)Cy_SegLCD_WritePixel(base, locPix, display->invert != locVal);
        }
    }

    return((cy_en_seglcd_status_t)retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_WriteNumber
****************************************************************************//**
*
* Displays an integer value onto a specified display.
* Supports all display types except \ref CY_SEGLCD_BAR.
*
* \note The sign conversion, sign display, decimal points, and other special segments
*       outside the display symbols themselves should be handled on upper layer code.
*
* \param base The base pointer to the LCD instance registers.
* \param value The unsigned integer number to be displayed.
* \param position The position of the least significant digit of the number
*        as counted left to right starting at 0 - the first symbol on the left
*        of the specified display.
*        If the specified display contains fewer symbols than the number
*        requires to be displayed, the extra (more significant) digit(s) is(are) not
*        displayed and the \ref CY_SEGLCD_EXCEED value is returned.
* \param display The pointer to the display structure \ref cy_stc_seglcd_disp_t.
* \param zeroes:
*    - true - all the unused digits on the left of the displayed value are zeroes.
*    - false - all the above mentioned digits are blank.
* \param hex:
*    - true - the value is displayed in the hexadecimal format.
*    - false - the value is displayed in the decimal format.
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_DecNum
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_HexNum
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_WriteNumber(stc_LCD_t * base,
                                            uint32_t   value,
                                            uint32_t   position,
                          cy_stc_seglcd_disp_t const * display,
                                                bool   zeroes,
                                                bool   hex)
{
    uint32_t retVal = (uint32_t)CY_SEGLCD_BAD_PARAM;

    if (CY_SEGLCD_IS_SYMBOLIC(display, position)) /* If the display is symbolic/numeric */
    {
        uint32_t locNum;
        uint32_t i = position + 1UL;
        uint32_t locDiv = (hex) ? 0x10UL : 10UL; /* The digit weight for hex/dec counting systems */
        retVal = (uint32_t)CY_SEGLCD_SUCCESS;

        do
        {
            i--;

            if ((0UL == value) && (i != position) && (!zeroes))
            {
                /* For 'not zeroes' mode each extra digit is cleared. */
                locNum = (uint32_t)CY_SEGLCD_NUM_BLANK;
            }
            else
            {
                /* Display the subsequent digit or '0' for 'zeroes' mode */
                locNum = value % locDiv;
            }

            /* Calculate the next digit value */
            value = value / locDiv;

            retVal |= (uint32_t)Cy_SegLCD_WriteChar(base, (display->font->ascii) ? NumToChar(locNum) : (char_t)locNum, i, display);
        }
        while (i != 0UL); /* While the current digit position is in the range of the display, keep processing the output */

        if (0UL != value)
        {
            retVal |= (uint32_t)CY_SEGLCD_EXCEED;
        }
    }

    return((cy_en_seglcd_status_t)retVal);
}

/*******************************************************************************
* Function Name: Cy_SegLCD_WriteString
****************************************************************************//**
*
* Writes a specified zero-terminated char string onto a specified display.
* Supports all display types except \ref CY_SEGLCD_BAR.
*
* \param base The base pointer to the LCD instance registers.
* \param string The pointer to the ASCII-coded null-terminated character string.
* \param position The position of the first string character at the display
*        as counted left to right starting at 0 - the first symbol on the left
*        of the specified display.
*        If the specified display contains fewer symbols than the string
*        requires to be displayed, the extra character(s) is(are) not displayed
*        and the \ref CY_SEGLCD_EXCEED value is returned.
* \param display The pointer to the display structure \ref cy_stc_seglcd_disp_t.
* \return \ref cy_en_seglcd_status_t.
*
* \funcusage
* \snippet seglcd/snippet/SegLCD_Snpt.c snippet_Cy_SegLCD_String
*
*******************************************************************************/
cy_en_seglcd_status_t Cy_SegLCD_WriteString(stc_LCD_t * base,
                                        char_t const * string,
                                            uint32_t   position,
                          cy_stc_seglcd_disp_t const * display)
{
    uint32_t retVal = (uint32_t)CY_SEGLCD_BAD_PARAM;

    if (CY_SEGLCD_IS_ASCII(display, position)) /* If the display is symbolic/numeric and the font is ASCII-coded */
    {
        uint32_t i;
        retVal = (uint32_t)CY_SEGLCD_SUCCESS;

        for (i = 0UL; (i + position) < display->symNum; i++)
        {
            retVal |= (uint32_t)Cy_SegLCD_WriteChar(base, ('\0' != string[i]) ? string[i] : CY_SEGLCD_SPACE(display), i + position, display);
        }

        if ('\0' != string[i])
        {
            retVal |= (uint32_t)CY_SEGLCD_EXCEED;
        }
    }

    return((cy_en_seglcd_status_t)retVal);
}

#endif /* CY_IP_MXLCD */

/* [] END OF FILE */
