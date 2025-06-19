SYSPM – Hibernate – Wake-up from RTC 
====================================
Description
^^^^^^^^^^^
- This example demonstrates how to enter Hibernate mode and exit it through an RTC alarm

Target Device
^^^^^^^^^^^^^
- Traveo-II CYT2B7x devices

CPU Board
^^^^^^^^^
- CYTVII-B-E-1M-176-CPU Rev. C Board

Dependency
^^^^^^^^^^
- It might be recommended to disconnect the debugger after programming to prevent and debugger errors caused by the connection loss when entering Hibernate mode

Expectation
^^^^^^^^^^^
- One LED on the base board is toggled for some seconds to indicate that the execution goes through the startup
- Then the RTC alarm is configured and Hibernate mode is entered
- After some seconds the MCU wakes up and the LED toggling can be seen
- After the toggling stops, a second LED will be set by the RTC Alarm IRQ handler to indicate that there was a pending RTC alarm