
#include "drivers/lpddr4/cy_lpddr4.h"
#include "func_validation.h"
#include "basic_tests.h"

#define INIT_WITH_TRAINING          0
#define INIT_WITHOUT_TRAINING       1

#define INIT_TYPE   INIT_WITH_TRAINING

#define MR4_DQS2DQ_TIMER_ID 0

#include "Config_02_RD_LLLH_WR_LLLM_1_AC2.h"

/** Interrupt Configuration for catching LPDDR4 Faults*/
cy_stc_sysint_irq_t irq_cfg_fault = 
{
    .sysIntSrc  = cpuss_interrupts_fault_0_IRQn,
    .intIdx     = CPUIntIdx2_IRQn,
    .isEnabled  = true
};

/*******************************************************************************
 * Fault Handler for FAULT_STRUCT0 in which the LPDDR4 faults are mapped
 * **************************************************************************//*
 *
 *  LPDDR4 faults are mapped to Fault Structure 0
 *  In this Fault Handler the actions in case of fault can be handled
 * 
 ******************************************************************************/
void irqFaultReport0Handler(void)
{
    cy_en_sysflt_source_t status;
    
    uint32_t fltData = 0;
    status = Cy_SysFlt_GetErrorSource(FAULT_STRUCT0);

    if(status != CY_SYSFLT_NO_FAULT)
    {
        /* LPDDR4 FATAL Fault */
        if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_FATAL_FAULT) 
        {
            //add some usefull code for fatal handling 
            fltData = Cy_SysFlt_GetData0(FAULT_STRUCT0);
            switch(fltData)
            {
                /* DATA0[0]: 0: non-correctable ECC fault */
                case 0: 
                        /* accord. SAS Reset at least the LPDDR4 system. 
                           Reset at least the LPDDR4 system. */ 
                        __NOP();
                        // halt System
                        CY_ASSERT(0);
                        break;
            
                /*  DATA0[0]: 1: PLL unlock fault */
                case 1:
                        /*Reset at least the LPDDR4 system. */ 
                        __NOP();
                        // halt System
                        CY_ASSERT(0);
                        break;
            }
        /* LPDDR4 none fatal GSM ( Global Statemachine fault */
        }else if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_GSM_FAULT) 
        {
            Cy_Lpddr_SyncGSMStateOnFault();
             /*add something usefull here*/

            /* clear the interrupt flag before issuing next user command*/
             Cy_Lpddr_ClearGCFSM_DMCFG();
        }else if(status == CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_ECC_SEC_FAULT) 
        {   
            //add some usefull code for none fatal fault handling 
            /* correctable ECC Fault */
            __NOP();
        }
      Cy_SysFlt_ClearStatus(FAULT_STRUCT0);
    }
    Cy_SysFlt_ClearInterrupt(FAULT_STRUCT0);
}


void LPDDR4_Init (void)
{
    /**
    * specifies the system temperature gradient of your system
    * used to calculate the cycle time for reading MR4 and updating 
    * the refresh rate [°C/s] */
    uint32_t temperatureGradient = 30; 
    /**
     * system response delay of your system
     * used to calculate the cycle time for reading MR4 and updating 
     * the refresh rate [ms] */
    uint32_t systemResponseDelay = 1;

    /* *local variable for the MR4 Read Interval */
    uint32_t mr4ReadInterval=0;

    /* Enable the LPDDR4 power sequencing */
    #ifdef CY_SYS_LPDDR_POWER_EN
        Cy_SystemLpdd4PowerUp(CY_SYS_LPDDR4_PG_POLL, CY_SYS_LPDDR4_POWER_DELAY_MS);
    #endif  /* CY_SYS_LPDDR_POWER_EN */

    /***************************************************************************
                            Fault report settings
    ***************************************************************************/
    /* clear status (typically this process is done by boot code)*/
    Cy_SysFlt_ClearStatus(FAULT_STRUCT0); 
    /* set the Mask bit for LPDDR fatal Fault DEC ECC Fault / PLL unlock  */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_FATAL_FAULT);
    /* set the Mask bit for LPDDR none fatal Global State Machine Fault */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_GSM_FAULT);
    /* set the Mask bit for LPDDR SEC ECCC none fatal fault */
    Cy_SysFlt_SetMaskByIdx(FAULT_STRUCT0, CY_SYSFLT_LPDDR4_0_LPDDR4_NONFATAL_ECC_SEC_FAULT);
    Cy_SysFlt_SetInterruptMask(FAULT_STRUCT0);

    /***************************************************************************
                               Interrupt settings
    ***************************************************************************/
    Cy_SysInt_InitIRQ(&irq_cfg_fault);
    Cy_SysInt_SetSystemIrqVector(irq_cfg_fault.sysIntSrc, irqFaultReport0Handler);
    NVIC_SetPriority(irq_cfg_fault.intIdx, 0);
    NVIC_EnableIRQ(irq_cfg_fault.intIdx);

    LPDDR40->LPDDR4_WRAPPER.unPLL800_CLOCK_CTL.stcField.u1ENABLE = 0;

    /* Initialize LPDDR4 wit either Reload or Training */
    #if ( INIT_TYPE == INIT_WITH_TRAINING )
        /* real training only possible on real silicon */
        CY_ASSERT(CY_LPDDR4_SUCCESS == Cy_Lpddr_ControllerInit(LPDDR40,&lpddr_config,CY_LPDDR4_INIT_WITH_TRAINING));
    #elif ( INIT_TYPE == INIT_WITHOUT_TRAINING ) 
        CY_ASSERT(CY_LPDDR4_SUCCESS == Cy_Lpddr_ControllerInit(LPDDR40,&lpddr_config,CY_LPDDR4_INIT_WITH_RELOAD));
    #else
        #error Init Type not set
    #endif
    // CheckMemoryBlockVal(0x80000000,0xffffff,0x0);
    // CheckMemoryBlockVal(0x80000000,0xffff,0xFFFFFFFF);
    // CheckMemoryBlock(0x80000000,0xffff);

    CY_ASSERT(0==CheckMemoryBlockVal(0x80000000,0xffff,0x0));
    CY_ASSERT(0==CheckMemoryBlockVal(0x80000000,0xffff,0xFFFFFFFF));
    CY_ASSERT(0==CheckMemoryBlock(0x80000000,0xffff));

    // Initializes the HW timer used as SW Timer source (SysTick)
    Cy_SwTmr_Init();

    /* calculate the the interval for reading the Mode Register MR4 as specified in JEDEC209-4B */
    mr4ReadInterval = Cy_Lpddr_CalcMR4ReadInterval(systemResponseDelay,temperatureGradient);
    /* Register LPDDR4 Memory MOnitoring as callback function */
    Cy_SwTmr_StartHighPrio(MR4_DQS2DQ_TIMER_ID,mr4ReadInterval, true,Cy_Lpddr_MemoryMonitoring);
      


}



