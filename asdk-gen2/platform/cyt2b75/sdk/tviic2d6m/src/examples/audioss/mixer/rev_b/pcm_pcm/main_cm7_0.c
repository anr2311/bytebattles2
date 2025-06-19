/***************************************************************************//**
* \file main_cm7_0.c
*
* \version 1.0
*
* \brief Main example file for CM7_0
*
********************************************************************************
* \copyright
* Copyright 2016-2020, Cypress Semiconductor Corporation. All rights reserved.
* You may use this file only in accordance with the license, terms, conditions,
* disclaimers, and limitations in the end user license agreement accompanying
* the software package with which this file was provided.
*******************************************************************************/

/*
    This example pushes data defined by "gPcmData" into the Mixer SRC FIFO.
    Then, pops the data from the Mixer DST FIFO.
    It is expected that "gDestPcmData" will get value of amount all SRC FIFO data.
    E.g. gDestPcmData[x] = gPcmData[0][x] + gPcmData[1][x] + ...
*/

#include "cy_project.h"
#include "cy_device_headers.h"

#define MIXER_SRC_FIFO_NUM        (2)
#define MIXER_SRC_FIFO_ENTRY_MAX  (128)
#define MIXER_SRC_FIFO_ENTRY_HALF (MIXER_SRC_FIFO_ENTRY_MAX/2)
#define PCM_DATA_NUM              (96)
#define PCM_DATA_PAIR_NUM         (PCM_DATA_NUM/2)             // The data contains channel 0 and channel 1

/* The Source PCM data  */
/* Even-numbered data is channel0 data. Odd-numbered data is channel1 data. */
static const int16_t gPcmData[MIXER_SRC_FIFO_NUM][PCM_DATA_NUM] =
{
    /* Data for SRC FIFO 1 */
    {
        11187, -31119, -29677,  24435,  21770,   9866, -30159,  -2849,  13359, -28725, -32328, -13796,  27914, -23414,  23085, -23885,
        26751,  11532, -19217, -13448, -19282, -21215,   8117, -26012, -20089,  19770,  21580,  32101, -22752,  15594,   9523,  22554,
        16641,  10152,  13562, -22329,   9687,   3419,  20902,  15752,  28517,  27117, -10279,  19641,    724,  27148, -18705,   1244,
        16258,   2695,  30940, -17649,   8655,  -1022, -15612,  14424,  29183, -14149,  24869,  22559,  -2865, -12703, -25135, -23918,
         3437, -23023,   3322,  22033,   -194, -28730,  25199, -11961, -11653, -24216,  26530, -31423,   2415,   2860,  -8461,  -7583,
        16013, -20624,  26587, -29355, -15063,    777,  23289,  -6018, -32313,  22569, -18350, -27603, -11595,    278, -32767,  32767,
    },
    /* Data for SRC FIFO 2 */
    {
        -18553, -12972,   9844, -17636,  -9865,   8045,  19701, -11150, -15004,  17440,  23417,  23083, -22182, -17045,  10023,  -3233,
        -19002,  21848, -27784,  25701, -21925, -19521,  21082,   7116,  -9910,  -5766,  -5239,     75,  24112,  30473,  19150,  24469,
          9227, -30862, -27907, -27012,   4480, -12054, -20745,  23354, -16516,   9872, -23433, -25351,  23061, -26301,  -7733,  11484,
        -15864,   9513, -30382,  30612, -21482,  -6783, -12300,  -4147,  -2411,  27774,   8114,  31686, -13673,  -3759,  25013, -20266,
        -26480,   3633,  13443,  28663, -16721, -15366,  18459,  -2813,  31780, -23721, -27820, -24603,   -259, -29376, -12081,   2736,
         17621,  26873,  -2364,  -9803, -30667,  -7039,  16548,  29196,   3969, -32365, -31483,  22763, -10706,  31871, -32767,  32767,
    },
};

/* The buffer for result PCM data  */
static int16_t gDestPcmData[PCM_DATA_NUM] = {0};

/* Declaration of local functions  */
static int32_t GetSrcPcmPairData(uint32_t fifoIdx, uint32_t entryIdx);
static int16_t SumOfSrcData(uint32_t entryIdx);

/* Setting variable for the Mixer SRC FIFOs */
const cy_stc_mixer_src_config_t gMixerSrcConfig =
{
    .fsRatio  = CY_MIXER_SRC_FREQ_NO_RESAMPLE, /* No re-sampling */
    .ch0sel   = CY_MIXER_SRC_DATA_LOW,         /* The lower 16bit of the FIFO represents channel 0 PCM data */
    .ch1sel   = CY_MIXER_SRC_DATA_HIGH,        /* The upper 16bit of the FIFO represents channel 1 PCM data */
    .fadeCode = CY_MIXER_CODE_SAME,            /* No fade effect */
    .gainCode = CY_MIXER_CODE_SAME,            /* No gain */
    .trgLevel = 0,                             /* Set SRC FIFO threshold (Not used in this example) */
};

/* Setting variable for the Mixer DST FIFO */
const cy_stc_mixer_dst_config_t gMixerDestConfig =
{
    .fadeCode = CY_MIXER_CODE_SAME, /* No fade effect */
    .gainCode = CY_MIXER_CODE_SAME, /* No gain */
    .trgLevel = 0,                  /* Set DST FIFO threshold (Not used in this example) */
};

/* Main application function */
//------------------------------------------------------------------------------
int main(void)
{
    SystemInit();

    __enable_irq();

    /**************************/
    /* Settings for SRC FIFOs */
    /**************************/
    for(uint32_t srcFifoIdx = 0; srcFifoIdx < MIXER_SRC_FIFO_NUM; srcFifoIdx += 1)
    {
        /* Apply the FIFO settings */
        Cy_Mixer_SourceInit(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx], &gMixerSrcConfig);

        /* Enable the SRC mixer */
        Cy_Mixer_Src_Enable(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx]);

        /* Activate */
        Cy_Mixer_Src_Activate(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx]);
    }

    /**************************/
    /* Settings for DST FIFO  */
    /**************************/
    /* Apply the FIFO settings */
    Cy_Mixer_DestinationInit(&MIXER0->MIXER_DST_STRUCT, &gMixerDestConfig);

    /* Enable the DST mixer */
    Cy_Mixer_Dst_Enable(MIXER0_MIXER_DST_STRUCT);

    /* Activate */
    Cy_Mixer_Dst_Activate(MIXER0_MIXER_DST_STRUCT);

    /**************************/
    /* Pushing Dummy Data     */
    /**************************/
    /* First 31 entry won't be used directly by mixer because of 63 tap filter.
       Therefore, input dummy data (0). */
    uint32_t halfTapNum = 31;
    for(uint32_t idx = 0; idx < halfTapNum; idx += 1)
    {
        for(uint32_t srcFifoIdx = 0; srcFifoIdx < MIXER_SRC_FIFO_NUM; srcFifoIdx += 1)
        {
            Cy_Mixer_Src_FifoWriteData(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx], /* dummy data */ 0);
        }
    }

    /**********************************/
    /* Fill Half of the SRC FIFO      */
    /**********************************/
    /* Push data until half of source FIFO size data will be stored, 
       because mixing starts when source FIFO has more than half(0x3E) entries,
       since there are an internal FIR filter with 63 taps. */
    uint32_t entryIdx = 0;
    for(; entryIdx < (MIXER_SRC_FIFO_ENTRY_HALF - halfTapNum); entryIdx += 1)
    {
        for(uint32_t srcFifoIdx = 0; srcFifoIdx < MIXER_SRC_FIFO_NUM; srcFifoIdx += 1)
        {
            Cy_Mixer_Src_FifoWriteData(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx], GetSrcPcmPairData(srcFifoIdx, entryIdx));
        }
    }

    /*****************************************/
    /* Push into SRC FIFO, Pop from DST FIFO */
    /*****************************************/
    uint32_t entryOffset = entryIdx;
    for(uint32_t dataIdx = 0; dataIdx < PCM_DATA_PAIR_NUM; dataIdx += 1)
    {
        for(uint32_t srcFifoIdx = 0; srcFifoIdx < MIXER_SRC_FIFO_NUM; srcFifoIdx += 1)
        {
            Cy_Mixer_Src_FifoWriteData(&MIXER0->MIXER_SRC_STRUCT[srcFifoIdx],GetSrcPcmPairData(srcFifoIdx, entryOffset+dataIdx));
        }

        /* Wait until the destination buffer get at least one entry */
        while(Cy_Mixer_Dst_FIFO_Used(MIXER0_MIXER_DST_STRUCT) == 0);

        /* Get the mixer result data 32bit by 32bit */
        int32_t* pDstBuffer = (int32_t*)&gDestPcmData[dataIdx*2];
        *pDstBuffer = Cy_Mixer_Dst_FifoReadData(MIXER0_MIXER_DST_STRUCT);
    }

    /********************/
    /* Check the result */
    /********************/
    for(uint32_t dataIdx = 0; dataIdx < PCM_DATA_NUM; dataIdx+=1)
    {
        CY_ASSERT(SumOfSrcData(dataIdx) == gDestPcmData[dataIdx]);
    }

    for(;;);
}

/* This function returns sum of source PCM data at input index */
static int16_t SumOfSrcData(uint32_t entryIdx)
{
    const int32_t MAX_VALUE_OF_INT16 = 0x00007FFF;
    const int32_t MIN_VALUE_OF_INT16 = 0xFFFF8000;

    /* Calculate sum of source data */
    int32_t sum = 0;
    for(uint32_t srcFifoIdx = 0; srcFifoIdx < MIXER_SRC_FIFO_NUM; srcFifoIdx += 1)
    {
        sum += gPcmData[srcFifoIdx][entryIdx];
    }

    /* Saturate the sum into int16_t value */
    if(sum > MAX_VALUE_OF_INT16)
    {
        sum = MAX_VALUE_OF_INT16;
    }
    else if (sum < MIN_VALUE_OF_INT16)
    {
        sum = MIN_VALUE_OF_INT16;
    }

    return (int16_t)sum;
}

/* This function returns channel 0 and channel 1 source PCM data at a once as a 32bit data.
   If input value of "entryIdx" is larger than "PCM_DATA_PAIR_NUM", it outputs dummy value (0) */
static int32_t GetSrcPcmPairData(uint32_t fifoIdx, uint32_t entryIdx)
{
    if(entryIdx < PCM_DATA_PAIR_NUM)
    {
        int32_t* pPcmData = (int32_t*)gPcmData[fifoIdx];
        return pPcmData[entryIdx];
    }
    else
    {
        /* Return dummy value */
        return 0;
    }
}

/* [] END OF FILE */
