#ifndef APP_ADC_H
#define APP_ADC_H

/* ASDK includes */
#include "asdk_adc.h"
#include "asdk_mcu_pins.h"

#define ADC_HIGH_BEAM_THRESHOLD 3000u

extern asdk_mcu_pin_t adc_pins[];
extern asdk_adc_config_t adc_conf;

/* Application specific APIs */
void app_adc_init();
void app_adc_start_conversion();
void app_adc_iteration();
uint32_t app_get_adc_value(asdk_mcu_pin_t p_Pin);

void app_ldr_iteration();
bool app_adc_return_headlight_status();

#endif /* APP_ADC_H */
