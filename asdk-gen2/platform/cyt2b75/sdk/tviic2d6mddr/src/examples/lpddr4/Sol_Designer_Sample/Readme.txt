In order to use this example copy all files from this folder to the /src- folder. Please set the configuration type to be used to Soluton Designer Configuration. This can be achieved by setting the macro define in /src/drivers/lpddr4/cy_dramconfsel.h

/*please specify if you are using a Solution Designer Config or Macro Type Config*/
#define MANUAL_CONFIG           	1	/*  to use either Config_02_RD_LLLH_WR_LLLM_1_AC2.h, Config_02_RD_LLHH_WR_LLLM_1_AC2.h or ...*/
#define SOL_DESIGNER_CONFIG     	2	/* to use a configuration prepared with Solution Designer */
#define RELASE_BUILD_DUMMY_CONFIG   	3	/* to avoid build errors in none LPDDR4 projects where no LPDDR4 config is available */

#define CONFIG_TYPE     SOL_DESIGNER_CONFIG     //<------ set config type here

All other settings as DRAM/JEDEC Timing , PSVP Frequency, PLL settings will be done  in Solution Designer. 
All settings done with Solution Deisgner are exported to a C header file lpddr4_cfg.h. 
File soldsgncfg. contains a lpddr4 configuration structure used to init the lpddr4 controller. The structure 
uses the macro defines from lpddr4_cfg.h to init the config structure.




Initialization:
----------------------

User can select how to Init the controller by setting the INIT_TYPE

        /******************************************************************************/
        /*                              INIT TYPE Definintion                         */
        /******************************************************************************/
        #define INIT_WITH_TRAINING          0   
        #define INIT_WITHOUT_TRAINING       1
        /* 
            macro to set the LPDDR4 Controller initialization
            for PSVP: 
                    #define INIT_TYPE       INIT_WITH_TRAINING 

            for FCV or SOC both versions are possible. 
            When set to #define INIT_TYPE   INIT_WITHOUT_TRAINING
            the PTSRxx register in the lpddr_config structure must be set with training 
            data matching  user hardware as these values are used to load known 
            training values to the phy for initialization
        */
        #define INIT_TYPE   INIT_WITH_TRAINING
        //#define INIT_TYPE   INIT_WITHOUT_TRAINING

    - when set to INIT_WITH_TRAINING the controller will be initialized running
      all necessary trainings, CBT, WRLVL,RDLVL,GateTarining.....

    - when set to INIT_WITHOUT_TRAINING the controller is initialize by reloading
      valid data from PTSRxx registers. Once the controller was successfully trained
      the user might store the data in some nonvolatile memory as it can be used 
      during next initialization for reloading valid data back to the controller
      and re-init without the need of training. Save also time when simulating the 
      code. The content of the PTSR Registers in the config files are 
      values from a successfull training on FCV. When using real silicon we need
      run a real training and note down the trained values from the PTSRxx registers 
      and include them in the lpddr_*.h config to reload valid data to the controller.

