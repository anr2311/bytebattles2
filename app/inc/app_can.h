#ifndef APP_CAN_H
#define APP_CAN_H

#include "asdk_platform.h"
#include "asdk_can_service.h"

#define VEHICLE_CAN ASDK_CAN_MODULE_CAN_CH_1

#define CAN_306_BYTE0_RIDING_MODE_INDEX      0u
#define CAN_306_BYTE0_RIDING_MODE_NEUTRAL    0u
#define CAN_306_BYTE0_RIDING_MODE_FW         1u
#define CAN_306_BYTE0_RIDING_MODE_BW         2u
#define CAN_306_BYTE0_RIDING_MODE_HOLD_UP    3u
#define CAN_306_BYTE0_RIDING_MODE_HOLD_DOWN  4u

#define CAN_306_BYTE1_VEH_SPEED_INDEX    1u
#define CAN_306_BYTE1_VEH_SPEED_0KMPH    0u
#define CAN_306_BYTE1_VEH_SPEED_1KMPH    1u
#define CAN_306_BYTE1_VEH_SPEED_2KMPH    2u
#define CAN_306_BYTE1_VEH_SPEED_3KMPH    3u
#define CAN_306_BYTE1_VEH_SPEED_4KMPH    3u
#define CAN_306_BYTE1_VEH_SPEED_5KMPH    3u

#define CAN_305_BYTE0_CMD_INDEX       0u
#define CAN_305_BYTE0_DATA_INDEX      1u

#define CAN_305_BYTE0_HEADLIGHT_CMD_DATA        2u
#define CAN_305_HEADLIGHT_OFF         0u
#define CAN_305_HEADLIGHT_ON        1u

#define CAN_305_BYTE1_HORN_CMD_DATA_ENABLE       1u
#define CAN_305_BYTE1_HORN_CMD_DATA_DISABLE       0u

#define CAN_305_BYTE0_INDICATOR_CMD_DATA        4u
#define CAN_305_BYTE1_INDICATOR_DATA_DISABLE      0u
#define CAN_305_BYTE1_INDICATOR_DATA_LEFT_ON      1u
#define CAN_305_BYTE1_INDICATOR_DATA_RIGHT_ON      2u
#define CAN_305_BYTE1_INDICATOR_DATA_LEFT_OFF      3u
#define CAN_305_BYTE1_INDICATOR_DATA_RIGHT_OFF      4u

#define CAN_305_BYTE0_CMD_TYPE_INDEX            0u
#define CAN_305_BYTE0_CMD_DATA_INDEX            1u
#define CAN_305_CMD_TYPE_TAIL_LAMP              3u
#define CAN_305_CMD_DATA_CLR_TAIL_LAMP          0u
#define CAN_305_CMD_DATA_SET_TAIL_LAMP          1u

#define CAN_305_CMD_TYPE_MANUAL_BREAK           6u
#define CAN_305_CMD_DATA_CLR_MANUAL_BREAK       0u
#define CAN_305_CMD_DATA_SET_MANUAL_BREAK       1u

#define CAN_305_HEADLIGHT_OFF         0u
#define CAN_305_HEADLIGHT_ON        1u

#define CAN_305_CMD_TYPE_DEGREE         0x05

void app_can_init();
void app_can_deinit();
void app_can_iteration();
void app_can_iteration_300ms();
void app_can_send(uint32_t can_id, uint8_t *data, uint8_t data_length);

void app_can_move_forward(uint8_t speed);
void app_can_move_backward(uint8_t speed);

void app_can_blink_right_twice();
void app_can_blink_left_twice();

void app_can_0deg();
void app_can_neg_30();
void app_can_pos_30();
void app_can_pos_60();
void app_can_neg_60();

bool return_reverse_button();

extern void HillHold(void);
extern void PotHolesHumps();

uint8_t app_can_blink_right_continuous(void);
uint8_t app_can_blink_left_continuous(void);

#endif // APP_CAN_H
