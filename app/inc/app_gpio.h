#ifndef APP_GPIO_H
#define APP_GPIO_H

/* ASDK includes */
#include "asdk_gpio.h"
#include "asdk_mcu_pins.h"

#include "stdbool.h"

extern asdk_gpio_config_t gpio_output_config[];
extern asdk_gpio_config_t gpio_input_config[];

extern const uint8_t gpio_output_config_size;
extern const uint8_t gpio_input_config_size;

typedef enum
{
    DIRECTION_STRAIGHT = 0,
    DIRECTION_SLIGHT_RIGHT = 1,
    DIRECTION_SLIGHT_LEFT = 2,
    DIRECTION_RIGHT = 3,
    DIRECTION_LEFT = 4,
    CAUTION = 5,
    PARK = 6
} asdk_gpio_direction_type_t;

/* Application specific APIs */
void app_gpio_init();
void app_gpio_iteration();
void app_gpio_toggle(asdk_mcu_pin_t pin);
void app_gpio_set_pin_state(asdk_mcu_pin_t pin, bool state);
bool app_gpio_get_pin_state(asdk_mcu_pin_t pin);
uint8_t get_ir_middle_status();
uint8_t set_ir_right_status();
uint8_t get_ir_left_status();
uint8_t app_gpio_IR_direction();
bool get_rain_status();

#endif
