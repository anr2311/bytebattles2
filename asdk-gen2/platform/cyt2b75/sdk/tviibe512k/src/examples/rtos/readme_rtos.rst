RTOS: Sample use case 
=====================
Description
^^^^^^^^^^^
- This example demonstrates RTOS use case with semaphores to trigger tasks using FreeRTOS. 
- LEDs will blink in sequence based on the way semaphores are activated.
- Three tasks are configured and used with.

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2Bx devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board/CYTVII-B-E-1M-100-CPU Board
- CYTVII-B-E-BB Rev.A Board

Dependency
^^^^^^^^^^
- CM4 example to be copied to the main “src”

Expectation
^^^^^^^^^^^
- After running the example, user should be able to see the LEDs blinking in sequence on the base board connected.

- Things to take care in IAR:
   - Select “rtos” configuration
   - Build/compile cm4_mc workspace
   - Open cm0plus_cm4 workspace with latest configuration, go to options, 
      - Debugger->Multicore->Change session configuration XML to rtos.xml ($WS_DIR$\cm0plus_cm4_setup_rev_x.xml to $WS_DIR$\cm0plus_cm4_setup_rev_x_rtos.xml)
      - Debugger->Images->Path, change “cm4_mc\rev_x” to “cm4_mc\rtos”
   - Build cm0plus_cm4 workspace, debug and run
- GHS: RTOS is included for the application core, no other configuration is needed.
- Note: ‘x’ stands for the latest revision of the device supported in SDL.
