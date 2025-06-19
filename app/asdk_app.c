/* ASDK User actions starts ************************************************ */

/* ASDK User Action: add application include files */
#include "asdk_system.h"
#include "asdk_clock.h"

#include "debug_uart.h"
#include "scheduler.h"

#include "app_gpio.h"
#include "app_can.h"
#include "app_adc.h"
#include "app_rpi.h"
#include "ultrasonic.h"

#include "asdk_app.h"
#include "app_can.h"
#include "ultrasonic.h"

/* Debug Print includes */
#include "debug_print.h"

/* ASDK User Action: Declare new task below */

static void task_always_run(void);
static void task_1ms(void);
static void task_5ms(void);
static void task_10ms(void);
static void task_100ms(void);
static void task_300ms(void);
static void task_1000ms(void);

static void app_ecu_mode(void);
static void perform_startup();
static void perform_normal_functions();

/* ASDK User Action: Update the scheduler with newly added task */

static scheduler_t scheduler_config[] = {
    /* Task Function               Periodicity   */
    { .task_fn = task_always_run, .periodicty = 0 },
    { .task_fn = task_1ms,         .periodicty = 1 },
    { .task_fn = task_5ms,         .periodicty = 5 },
    { .task_fn = task_10ms,       .periodicty = 10 },
    { .task_fn = task_100ms,      .periodicty = 100 },
    { .task_fn = task_300ms,      .periodicty = 300 },
    { .task_fn = task_1000ms,     .periodicty = 1000 },
};

static size_t scheduler_size = sizeof(scheduler_config) / sizeof(scheduler_t);

static uint8_t ecu_mode = 0;

static bool startup_condition = false;

static uint32_t blink_twice_counter = 0u;

/* below functions are called by main.c */

void asdk_app_init()
{
    asdk_clock_config_t clk_cfg = {
        .clk_source = ASDK_CLOCK_SRC_INT,
        .pll = {
            .input_frequency = 8000000,
            .output_frequency = 160000000,
        },
        .xtal_trim = {0}
    };

    /* Initialize clock */
    asdk_clock_init(&clk_cfg);
    asdk_sys_init();

    /* Initialize scheduler */
    scheduler_init(scheduler_config, scheduler_size);

    /* Initialize UART for debug messages */
    debug_uart_init();

    /* ASDK User Action: Add init calls here */
    app_gpio_init();
    app_can_init();
    app_adc_init();
    app_rpi_init();
    app_adc_start_conversion();

    /* Enabling the interrupt */
    asdk_sys_enable_interrupts();
}

extern void set_ecu_mode(uint8_t mode)
{
    ecu_mode = mode;
}

// extern uint8_t get_ecu_mode()
// {
//     return ecu_mode;
// }
/* ASDK User Action: Define newly added tasks below */

static void task_1ms(void)
{
    ultrasonic_iterations();
}

static void task_5ms(void)
{

}

static void task_10ms(void)
{ 
    // DEBUG_PRINTF("Max Debug Uart Buffer Usage %d\r\n", debug_uart_get_max_usage());
    app_rpi_iteration();
}

static void task_100ms(void)
{
    app_ecu_mode();
    app_gpio_iteration();
    app_ldr_iteration();
    app_adc_iteration();

    app_can_iteration(); 
}

static void task_300ms(void)
{
    app_can_iteration_300ms();
}

static void task_1000ms(void)
{
    
}

static void task_always_run(void)
{
    asdk_can_service_send_iteration(VEHICLE_CAN);
    asdk_can_service_receive_iteration(VEHICLE_CAN);
    debug_uart_iteration();
}

static void app_ecu_mode(void)
{
    if (ultrsonic_get_distance(MCU_PIN_68) < 50u)
    {
        ecu_mode = STATE_MOVE_REVERSE;
    }

    switch(ecu_mode)
    {
        case STATE_STARTUP:
            /* handle startup*/
            perform_startup();
            break;
        case STATE_MOVE_NORMAL:
            /* handle move normal*/
            perform_normal_functions();
            break;
        case STATE_MOVE_LEFT:
            /* handle move left*/
            break;
        case STATE_MOVE_RIGHT:  
            /* handle move right*/
            break;
        case STATE_HILL_HOLD:
            /* handle hill hold*/
            HillHold();
            break;
        case STATE_SIDESTAND_KILL:
            /* handle sidestand kill*/
            break;
        case STATE_MOVE_REVERSE:
            /* handle move reverse*/
            app_can_0deg();
            app_can_move_forward(0x03u);

            if (ultrsonic_get_distance(MCU_PIN_68) > 150u)
            {
                ecu_mode = STATE_MOVE_NORMAL;
            }

            break;
            case STATE_POT_HOLES_HUMPS:
            {
                PotHolesHumps();
                break;
            }
        default:
            /* handle default case*/
            break;
    }
}

void set_startup_conditions(bool status)
{
    startup_condition = status;
}

static void perform_startup()
{
    if (startup_condition == true)
    {
        /* Perform startup actions here */
        startup_condition = false;

        /* start with low speed */
        app_can_move_forward(0x02u);

        /* Set the ECU mode to normal operation after startup */
        ecu_mode = STATE_MOVE_NORMAL;
    }
}

static void perform_normal_functions()
{
    uint8_t checkVar = (uint8_t)app_gpio_IR_direction();
    
    switch (checkVar)
    {
        case DIRECTION_STRAIGHT:
            app_can_0deg();
            app_can_move_forward(0x03u);
            break;
        case DIRECTION_SLIGHT_RIGHT:
            if (blink_twice_counter % 128u == 0u)
            {
                app_can_blink_right_twice();
                app_can_pos_30();
                if (blink_twice_counter == 2048u)
                {
                    blink_twice_counter = 0u;
                    ecu_mode = STATE_MOVE_NORMAL;
                }
            }
            else
            {
                blink_twice_counter++;
            }            
            break;
        case DIRECTION_SLIGHT_LEFT:
            if (blink_twice_counter % 128u == 0u)
            {
                app_can_blink_left_twice();
                app_can_neg_30();
                if (blink_twice_counter == 2048u)
                {
                    blink_twice_counter = 0u;
                    ecu_mode = STATE_MOVE_NORMAL;
                }
            }
            else
            {
                blink_twice_counter++;
            }
            break;
        case DIRECTION_RIGHT:
            app_can_pos_60();
            if (0x00u == app_can_blink_right_continuous())
            {
                ecu_mode = STATE_MOVE_NORMAL;
            }
            break;
        case DIRECTION_LEFT:
            app_can_neg_60();
            if (0x00u == app_can_blink_left_continuous())
            {
                ecu_mode = STATE_MOVE_NORMAL;
            }   
            break;
        case CAUTION:
            break;
        case PARK:
            break;

       
        
        default:
            /* Do nothing */
            break;
    }
}

/* ASDK User actions ends ************************************************** */
