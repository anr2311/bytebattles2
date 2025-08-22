/* Platform dependent includes */
#include "asdk_platform.h"

/* ASDK includes */
#include "asdk_error.h"

/* Application specific includes */
#include <stdbool.h>
#include "app_gpio.h"
#include "gpio_cfg.h"
#include "app_can.h"

/* Debug Print includes */
#include "debug_print.h"

volatile bool button_pressed = false;

static uint8_t IR_detect_left_status = 0;
static uint8_t IR_detect_right_status = 0;
static uint8_t IR_detect_middle_status = 0;
static bool Rain_detected_status;
static bool temp = false; //Temporary variable to store rain sensor state

static uint8_t rain_detected_debounce_counter = 0x00u;

static void ir_sensor_iteration(void);
static void rain_sensor_iteration(void);

/* Interrupt callback funtion for GPIO */
static void gpio_callback(asdk_mcu_pin_t mcu_pin, uint32_t pin_state)
{
    switch (mcu_pin)
    {
    case MCU_PIN_29:
        button_pressed = true;
        break;

    default:
        break;
    }
}

/* Will be called from asdk_app_init() */
void app_gpio_init()
{
    asdk_errorcode_t status = ASDK_GPIO_SUCCESS;

    /* Initialize output pins */

    for (uint8_t i=0; i<gpio_output_config_size; i++)
    {
        status = asdk_gpio_init(&gpio_output_config[i]);
        ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);
    }

    /* Initialize input pins */

    for (uint8_t i=0; i<gpio_input_config_size; i++)
    {
        status = asdk_gpio_init(&gpio_input_config[i]);
        ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);
    }

    /* Initialize gpio peripheral ISR callback */

    status = asdk_gpio_install_callback(gpio_callback);
    ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);
}

/* Will be called from asdk_app_loop() */
void app_gpio_iteration()
{
    // DEBUG_PRINTF("Iterating GPIO\r\n" );
    asdk_gpio_output_toggle(USER_LED_1);

    if (button_pressed)
    {
        // DEBUG_PRINTF("Button pressed\r\n" );

        button_pressed = false;

        asdk_gpio_output_toggle(USER_LED_2);
    }    

    ir_sensor_iteration();

    rain_sensor_iteration();
}

bool app_gpio_get_pin_state(asdk_mcu_pin_t pin)
{
    asdk_errorcode_t status = ASDK_GPIO_SUCCESS;
    asdk_gpio_state_t pin_state = ASDK_GPIO_STATE_INVALID;

    status = asdk_gpio_get_input_state(pin, &pin_state);
    ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);

    if (pin_state == ASDK_GPIO_STATE_HIGH)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void app_gpio_set_pin_state(asdk_mcu_pin_t pin, bool state)
{
    asdk_errorcode_t status = ASDK_GPIO_SUCCESS;

    if (state)
    {
        status = asdk_gpio_output_set(pin);
    }
    else
    {
        status = asdk_gpio_output_clear(pin);
    }

    ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);
}

void app_gpio_toggle(asdk_mcu_pin_t pin)
{
    asdk_errorcode_t status = asdk_gpio_output_toggle(pin);
    ASDK_DEV_ERROR_ASSERT(status, ASDK_GPIO_SUCCESS);
}

static void ir_sensor_iteration(void)
{
    /* IR Sensing Left */
    if (app_gpio_get_pin_state(IR_SENSE_LEFT) == false) {
        /* IR detected */
        IR_detect_left_status = 1;
    } else {
        /* IR not detected */
       IR_detect_left_status = 0;
    }

    /* IR Sensing Right */
    if (app_gpio_get_pin_state(IR_SENSE_RIGHT) == false) {
        /* IR detected */
        IR_detect_right_status = 1;
    } else {
        /* IR not detected */
       IR_detect_right_status = 0;
    }

    /* IR Sensing Middle */
    if ((app_gpio_get_pin_state(IR_SENSE_MIDDLE) == false) || (app_gpio_get_pin_state(IR_SENSE_MIDDLE_1) == false)) 
    {
        /* IR detected */
        IR_detect_middle_status = 1;   //Inverted logic because following the white line
    } else {
        /* IR not detected */
       IR_detect_middle_status = 0;    //Inverted logic because following the white line
    }
}

static void rain_sensor_iteration(void)
{
    /* Rain Sensing */
    
    if (app_gpio_get_pin_state(RAIN1_SENSE) == true) {
        rain_detected_debounce_counter = rain_detected_debounce_counter + 2;
        if(rain_detected_debounce_counter >= 10)
        {
            rain_detected_debounce_counter = 10; //Max value
        }
        Rain_detected_status = true; //Rain detected 
    } else {
        if(rain_detected_debounce_counter >= 4)
        {
            rain_detected_debounce_counter = rain_detected_debounce_counter - 4;
        }
        else
        {
            rain_detected_debounce_counter = 0; //Min value
        }
        Rain_detected_status = false; //Rain not detected 
    }
}


uint8_t get_ir_left_status(void)
{
    return IR_detect_left_status;
}

uint8_t get_ir_right_status(void)
{
    return IR_detect_right_status;
}

uint8_t get_ir_middle_status(void)
{
    return IR_detect_middle_status;
}

bool get_rain_status(void)
{
    return Rain_detected_status;
}

bool startup_condition(void)
{
    
}



uint8_t app_gpio_IR_direction()
{
    uint8_t var = (IR_detect_left_status<<2u) | (IR_detect_middle_status<<1u) | (IR_detect_right_status);

    switch (var)
    {
        case 0x00: 
            return (uint8_t)CAUTION;
            break;
        case 0x01:
            return (uint8_t)DIRECTION_RIGHT;
            break;
        case 0x02:
            return (uint8_t)DIRECTION_STRAIGHT;
            break;
        case 0x03: 
            return (uint8_t)DIRECTION_SLIGHT_RIGHT;
            break;
        case 0x04:
            return (uint8_t)DIRECTION_LEFT;
            break;
        case 0x06:
            return (uint8_t)DIRECTION_SLIGHT_LEFT; 
            break;
        case 0x07:
            return (uint8_t)PARK;
            break;
        case 0x05:
            return (uint8_t)CAUTION;
            break;
        default:
            break;
    }
}
