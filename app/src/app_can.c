/* Platform dependent includes */
#include "asdk_platform.h"

/* ASDK includes */
#include "asdk_error.h"

/* Application specific includes */
#include <string.h>
#include "app_can.h"

/* Debug Print includes */
#include "debug_print.h"

#include "app_adc.h"
#include "app_gpio.h"
#include "asdk_app.h"

#define CAN_TX_PIN MCU_PIN_4
#define CAN_RX_PIN MCU_PIN_5

volatile uint32_t can_error_count = 0;
volatile uint32_t can_busoff_count = 0;
static uint8_t hazard_counter = 0u;
volatile bool can_bus_off = false;

static int16_t Can_301_Pitch = (int16_t)0u;
static uint8_t Can_300_Throttle = 0u;

static bool VehicleSpeed0KmphStatus = false;
static bool ClearManualBreak = false;
static bool PotHolesHumpsActionStatus = false;
static bool PotHolesHumpsTailLampSwitch = false;

static bool AtStartup = true;

static bool blink_twice_flag = true;


static bool reverse_button = false;
static uint8_t rev_button_count = 0u;

static uint8_t tx_buffer_306[8] = {CAN_306_BYTE0_RIDING_MODE_NEUTRAL, CAN_306_BYTE1_VEH_SPEED_0KMPH, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA};
uint8_t tx_buffer_305[8] = {CAN_305_BYTE0_HEADLIGHT_CMD_DATA, CAN_305_HEADLIGHT_OFF, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA};
uint8_t rx_buffer[8] = {0};
uint8_t rx_buffer_301[8] = {0};

uint32_t rx_accept_can_ids[] = {0x300, 0x301};
uint8_t can_ids_length = sizeof(rx_accept_can_ids) / sizeof(rx_accept_can_ids[0]);

asdk_can_message_t rx_msg = {
    .can_id = 0x300,
    .dlc = 8,
    .message = rx_buffer,
};

asdk_can_message_t rx_msg_301 = {
    .can_id = 0x301,
    .dlc = 8,
    .message = rx_buffer_301,
};

asdk_can_message_t tx_msg_305 = {
    .can_id = 0x305,
    .dlc = 8,
    .message = tx_buffer_305,
};

asdk_can_message_t tx_msg_306 = {
    .can_id = 0x306,
    .dlc = 8,
    .message = tx_buffer_306,
};

uint8_t can_data[8] = {0};
asdk_can_message_t msg = {
    .message = can_data
};

bool msg_received = false;
bool msg_301_received = false;
bool tx_status_busy = 0;
asdk_errorcode_t can_write_status = ASDK_CAN_SUCCESS;

volatile uint16_t tx_can_id = 0;

static void PotHolesHumpsEval(void);
static void HillHoldEval(void);

static void app__can_headlight_tx_iteration();
static void app__can_indicator_hazard_iteration();

void __service_callback(uint8_t VEHICLE_CAN, asdk_can_event_t event, asdk_can_message_t *can_message);

void __service_callback(uint8_t VEHICLE_CAN, asdk_can_event_t event, asdk_can_message_t *can_message)
{
    switch (event)
    {
    case ASDK_CAN_TX_COMPLETE_EVENT:
        tx_can_id = can_message->can_id;
        break;

    case ASDK_CAN_RX_EVENT:       
        
        if (can_message->can_id == 0x300)
        {
            if (can_message->message[5] == 0x01)
            {
                // rev_button_count++;
                // if (rev_button_count = UINT8_MAX)
                // {
                    reverse_button = ~reverse_button;
                //     rev_button_count = 0u;
                // }
            }
        }

        if (AtStartup == true && can_message->can_id == 0x300)
        {
            if ((can_message->message[3] != 0x00u) && 
                (can_message->message[4] == 0x00u) && 
                (((uint8_t)DIRECTION_STRAIGHT) == app_gpio_IR_direction()))
            {
                AtStartup = false;
                set_startup_conditions(true);
            }
            else
            {
                /* Do nothing*/
            }
        }
    

        if (can_message->can_id == 0x300)
        {
            msg_received = true;
            rx_msg.can_id = can_message->can_id;
            rx_msg.dlc = can_message->dlc;
            memcpy(rx_msg.message, can_message->message, can_message->dlc);
        }   
        else if (can_message->can_id == 0x301)    
        {
            msg_301_received = true;
            rx_msg_301.can_id = can_message->can_id;
            rx_msg_301.dlc = can_message->dlc;
            memcpy(rx_msg_301.message, can_message->message, can_message->dlc);
        }
        break;

    case ASDK_CAN_ERROR_EVENT:
        can_error_count++;
        break;
    
    case ASDK_CAN_BUS_OFF_EVENT:
        can_bus_off = true;
        can_busoff_count++;
        break;

    default:
        break;
    }

}

void app_can_init()
{
    asdk_errorcode_t can_status = ASDK_MW_CAN_SERVICE_SUCCESS;

    asdk_can_config_t can_cfg = {
        .mcu_pins = {
            CAN_TX_PIN, /* CAN Tx */
            CAN_RX_PIN, /* CAN Rx */
        },

        .hw_filter = {
           .rx_fifo_acceptance_filter = {
                .can_ids = rx_accept_can_ids,
                .length = can_ids_length,
           }
        },

        .controller_settings = {
            .mode = ASDK_CAN_MODE_STANDARD,
            .max_dlc = ASDK_CAN_DLC_8,
            .can_id_type = ASDK_CAN_ID_STANDARD,
            .bitrate_config.can = {
                .baudrate = ASDK_CAN_BAUDRATE_500K, // tq = 40
                // sampling point = 87.5%
                .bit_time = {
                    .prop_segment = 29,
                    .phase_segment1 = 5,
                    .phase_segment2 = 5,
                    .sync_jump_width = 5,
                },
            },
            .interrupt_config = {
                .intr_num = ASDK_EXTI_INTR_CPU_4,
                .use_interrupt = true,
                .priority = 3,
            },
        },
    };

    can_status = asdk_can_service_init(VEHICLE_CAN, can_cfg);
    ASDK_DEV_ERROR_ASSERT(can_status, ASDK_CAN_SUCCESS);

    can_status = asdk_can_service_install_callback(__service_callback);
    ASDK_DEV_ERROR_ASSERT(can_status, ASDK_CAN_SUCCESS);
}

void app_can_deinit()
{
    asdk_errorcode_t can_deinit_status = asdk_can_deinit(VEHICLE_CAN);
    ASDK_DEV_ERROR_ASSERT(ASDK_CAN_SUCCESS, can_deinit_status);
}

void app_can_iteration()
{
    if (can_bus_off)
    {
        app_can_deinit();
        app_can_init();

        can_bus_off = false;
    }

    if (msg_received)
    {
        msg_received = false;
        
        /* Use rx_msg to read the received CAN message
            rx_msg.can_id
            rx_msg.dlc
            rx_msg.message[0]
         */
         HillHoldEval();                
    }
    else if (msg_301_received)
    {
        msg_301_received = false;
        PotHolesHumpsEval();
    }

    app_can_send(0x306, tx_buffer_306, (uint8_t)2u);

    app__can_headlight_tx_iteration();
}

void app_can_send(uint32_t can_id, uint8_t *data, uint8_t data_length)
{
    msg.can_id = can_id;
    msg.dlc = data_length;
    memcpy(msg.message, data, data_length);

    can_write_status = asdk_can_service_send(VEHICLE_CAN, &msg);
    ASDK_DEV_ERROR_ASSERT(ASDK_MW_CAN_SERVICE_SUCCESS, can_write_status);
}

extern void PotHolesHumps(void)
{
    if (PotHolesHumpsActionStatus == false)
    {
        if ((Can_301_Pitch <= (int16_t)-2) || (Can_301_Pitch >= (int16_t)2))
        {

            tx_buffer_306[CAN_306_BYTE1_VEH_SPEED_INDEX] = CAN_306_BYTE1_VEH_SPEED_3KMPH;
            
            if (PotHolesHumpsTailLampSwitch == false)
            {
                tx_buffer_305[CAN_305_BYTE0_CMD_TYPE_INDEX] = CAN_305_CMD_TYPE_TAIL_LAMP;
                tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = CAN_305_CMD_DATA_SET_TAIL_LAMP;
                app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

                PotHolesHumpsTailLampSwitch = true;
            }
            else
            {
                tx_buffer_305[CAN_305_BYTE0_CMD_TYPE_INDEX] = CAN_305_CMD_TYPE_TAIL_LAMP;
                tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = CAN_305_CMD_DATA_CLR_TAIL_LAMP;
                app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

                PotHolesHumpsTailLampSwitch = false;
            }

        }
        else
        {
            PotHolesHumpsActionStatus = true;
        }
    }
    else
    {
        set_ecu_mode(STATE_MOVE_NORMAL);
    }
}

extern void HillHold(void)
{
    if (VehicleSpeed0KmphStatus == false)
    {
        tx_buffer_306[CAN_306_BYTE1_VEH_SPEED_INDEX] = CAN_306_BYTE1_VEH_SPEED_0KMPH; 
        tx_buffer_305[CAN_305_BYTE0_CMD_TYPE_INDEX] = CAN_305_CMD_TYPE_MANUAL_BREAK;
        tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = CAN_305_CMD_DATA_SET_MANUAL_BREAK;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
        VehicleSpeed0KmphStatus = true;        
    }
    else
    {
        
    }
    if (VehicleSpeed0KmphStatus ==  true)
    {
        if (rx_msg_301.message[1] == 0u)
        {
            if (Can_301_Pitch < (int16_t)-7)
            {
                tx_buffer_306[CAN_306_BYTE0_RIDING_MODE_INDEX] = CAN_306_BYTE0_RIDING_MODE_HOLD_UP;  
            }
            else
            {
                tx_buffer_306[CAN_306_BYTE0_RIDING_MODE_INDEX] = CAN_306_BYTE0_RIDING_MODE_HOLD_DOWN;
            }
            
            tx_buffer_305[CAN_305_BYTE0_CMD_TYPE_INDEX] = CAN_305_CMD_TYPE_MANUAL_BREAK;
            tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = CAN_305_CMD_DATA_CLR_MANUAL_BREAK;
            app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
            ClearManualBreak = true;
            VehicleSpeed0KmphStatus = false;   
        }        
    }
    if ((ClearManualBreak ==  true) && (Can_300_Throttle != 0u))
    {
        set_ecu_mode(STATE_MOVE_NORMAL);
    }
}

static void PotHolesHumpsEval(void)
{
    Can_301_Pitch = rx_msg_301.message[4] | (rx_msg_301.message[5] << 8u);

    if (((Can_301_Pitch <= (int16_t)-2) || (Can_301_Pitch >= (int16_t)2)) && 
        (get_ir_middle_status() == 0)) 
    {         
        set_ecu_mode(STATE_POT_HOLES_HUMPS);
    }
}

static void HillHoldEval(void)
{    
    Can_301_Pitch = rx_msg_301.message[4] | (rx_msg_301.message[5] << 8u);
    Can_300_Throttle = rx_msg.message[3];

    if (((Can_301_Pitch < (int16_t)-7) || (Can_301_Pitch > (int16_t)7)) && (Can_300_Throttle == 0u))
    {
        set_ecu_mode(STATE_HILL_HOLD);
    }
}

static void app__can_headlight_tx_iteration(void)
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_HEADLIGHT_CMD_DATA;
    
    if (true == app_adc_return_headlight_status())
    {
       tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_HEADLIGHT_ON;
    }
    else if (false == app_adc_return_headlight_status())
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_HEADLIGHT_OFF;
    }

    app_can_send(0x305, tx_buffer_305, (uint8_t)8u);
}

void app_can_iteration_300ms()
{
    app__can_indicator_hazard_iteration();
    // app__can_horn_iteration();
}

// void app__can_horn_iteration()
// {
//     tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;
//     if()
//     {
//         horn_count = 4;
//     }
//     if(horn_count%2 == 0)
//     {
//             tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_HORN_CMD_DATA_ENABLE;
//             app_can_send(0x305, tx_buffer_305, (uint8_t)2u);   
//             horn_count--;
//     }
//     else if(horn_count%2 != 0)
//     {
//             tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_HORN_CMD_DATA_DISABLE;
//             app_can_send(0x305, tx_buffer_305, (uint8_t)2u);   
//             horn_count--;
//     }      
// }

static void app__can_indicator_hazard_iteration(void)
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;

    if (false == get_rain_status())
    {
        if(hazard_counter == 0u)
        {
            tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_ON;
            app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
            tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_ON;
            app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
            hazard_counter = 1u;
        }
        else if (hazard_counter == 1u)
        {
            tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_OFF;
            app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
            tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_OFF;
            app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
            hazard_counter = 0u;
        }
    }
    else if (true == get_rain_status())
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_DISABLE;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
    }
}

void app_can_move_forward(uint8_t speed)
{
    tx_buffer_306[CAN_306_BYTE0_RIDING_MODE_INDEX] = CAN_306_BYTE0_RIDING_MODE_FW;
    tx_buffer_306[CAN_306_BYTE1_VEH_SPEED_INDEX] = speed;

    // app_can_send(0x306, tx_buffer_306, (uint8_t)2u);
}

void app_can_move_backward(uint8_t speed)
{
    tx_buffer_306[CAN_306_BYTE0_RIDING_MODE_INDEX] = CAN_306_BYTE0_RIDING_MODE_BW;
    tx_buffer_306[CAN_306_BYTE1_VEH_SPEED_INDEX] = speed;

    // app_can_send(0x306, tx_buffer_306, (uint8_t)2u);
}

void app_can_blink_right_twice()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;

    if(blink_twice_flag == true)
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_ON;

        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        blink_twice_flag = false;
    }
    else
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_OFF;

        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        blink_twice_flag = true;
    }
}

void app_can_blink_left_twice()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;

    if(blink_twice_flag == true)
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_ON;

        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        blink_twice_flag = false;
    }
    else
    {
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_OFF;

        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        blink_twice_flag = true;
    }
}


uint8_t app_can_blink_right_continuous(void)
{
    if (((uint8_t)DIRECTION_RIGHT) == app_gpio_IR_direction())
    {
        tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;

        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_OFF;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_ON;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        return 0x01u;
    }
    else if (get_ir_middle_status() == 0x01u)
    {
        return 0x00u;
    }
    else
    {

    }

    return 0x01u;
}

uint8_t app_can_blink_left_continuous(void)
{
    if (((uint8_t)DIRECTION_LEFT) == app_gpio_IR_direction())
    {
        tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_BYTE0_INDICATOR_CMD_DATA;
        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_RIGHT_OFF;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        tx_buffer_305[CAN_305_BYTE0_DATA_INDEX] = CAN_305_BYTE1_INDICATOR_DATA_LEFT_ON;
        app_can_send(0x305, tx_buffer_305, (uint8_t)2u);

        return 0x01u;
    }
    else if (get_ir_middle_status() == 0x01u)
    {
        return 0x00u;
    }
    else
    {

    }

    return 0x01u;
}

void app_can_pos_60()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_CMD_TYPE_DEGREE;
    tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = (int8_t)60;

    app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
}

void app_can_neg_60()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_CMD_TYPE_DEGREE;
    tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = (int8_t)-60;

    app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
}

void app_can_pos_30()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_CMD_TYPE_DEGREE;
    tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = (int8_t)30;

    app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
}

void app_can_neg_30()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_CMD_TYPE_DEGREE;
    tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = (int8_t)-30;

    app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
}

void app_can_0deg()
{
    tx_buffer_305[CAN_305_BYTE0_CMD_INDEX] = CAN_305_CMD_TYPE_DEGREE;
    tx_buffer_305[CAN_305_BYTE0_CMD_DATA_INDEX] = (int8_t)0;

    app_can_send(0x305, tx_buffer_305, (uint8_t)2u);
}

bool return_reverse_button()
{
    return reverse_button;
}