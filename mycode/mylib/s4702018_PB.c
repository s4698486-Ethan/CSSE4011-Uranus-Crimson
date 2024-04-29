/**
********************************************************************************
* @file mylib/s4702018_PB.c
* @author Henry Sommerville - s4702018
* @date 24.02.2024
* @brief PB Peripheral driver C File
********************************************************************************
* External Functions
********************************************************************************
*
*
*
*
*
********************************************************************************
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <s4702018_PB.h>

/* size of stack area used by each thread */
#define PB_STACKSIZE 5 * 1024

/* scheduling PB_THREAD_priority used by each thread */
#define PB_THREAD_PRIORITY 4

K_MSGQ_DEFINE(uart_to_pb_queue, RECEIVE_MSG_SIZE, PB_QUEUE_SIZE, 4);
K_MSGQ_DEFINE(pb_to_uart_queue, SEND_MSG_SIZE, PB_QUEUE_SIZE, 4);

K_THREAD_STACK_DEFINE(PB_THREAD_stack_area, PB_STACKSIZE);
static struct k_thread PB_THREAD;

static const struct gpio_dt_spec button =
    GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios, {0});

/* s4702018_PB_reg_init()
 * inits the Push button
 *
 * Params:
 * None
 *
 * Returns:
 * None.
 */
void s4702018_PB_reg_init(void) {
    int ret;

    if (!gpio_is_ready_dt(&button)) {
        printk("Error: button device %s is not ready\n", button.port->name);
        return;
    }

    ret = gpio_pin_configure_dt(&button, GPIO_INPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure %s pin %d\n", ret,
               button.port->name, button.pin);
        return;
    }
}

/* s4702018_PB_TaskPeripheralControl()
 * PB Task Thread which inits the hardware and controls funcitonality
 *
 * Params:
 * None.
 *
 * Returns:
 * None.
 */
void s4702018_PB_TaskPeripheralControl() {
    // init the push button
    s4702018_PB_reg_init();

    uint8_t receive_data[RECEIVE_MSG_SIZE];
    uint8_t send_data;
    uint8_t status;

    while (1) {
        // check if request has been made for push button status
        if (k_msgq_get(&uart_to_pb_queue, &receive_data, K_NO_WAIT) == 0) {
            // prepare packet
            status = gpio_pin_get_dt(&button);
            send_data = status;

            // send packet to uart thread for processing
            k_msgq_put(&pb_to_uart_queue, &send_data, K_NO_WAIT);
        }
        k_sleep(K_MSEC(100));
    }
}

/* s4702018ThreadPBInit()
 * Initisalizes the Push button thread
 *
 * Params:
 * None.
 *
 * Returns:
 * None.
 */
void s4702018ThreadPBInit() {
    // create and start the push button thread
    k_thread_create(&PB_THREAD, PB_THREAD_stack_area,
                    K_THREAD_STACK_SIZEOF(PB_THREAD_stack_area),
                    s4702018_PB_TaskPeripheralControl, NULL, NULL, NULL,
                    PB_THREAD_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&PB_THREAD, "PB_THREAD");

    k_thread_start(&PB_THREAD);
}
