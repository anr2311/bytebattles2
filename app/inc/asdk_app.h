#ifndef ASDK_APP_H
#define ASDK_APP_H

#include "systick/cy_systick.h"

#define STATE_STARTUP           0x00u
#define STATE_MOVE_NORMAL       0x01u
#define STATE_MOVE_LEFT         0x02u
#define STATE_MOVE_RIGHT        0x03u
#define STATE_HILL_HOLD         0x04u
#define STATE_SIDESTAND_KILL    0x05u
#define STATE_MOVE_REVERSE      0x06u
#define STATE_POT_HOLES_HUMPS   0x07u

#define ASDK_DELAY(DELAY_MS) (Cy_SysTick_DelayInUs(DELAY_MS * 1000U));

void asdk_app_init();

void set_startup_conditions(bool status);

extern void set_ecu_mode(uint8_t mode);

#endif
