/**
********************************************************************************
* @file mylib/s4702018_LED.c
* @author Henry Sommerville - s4702018
* @date 23.02.2024
* @brief LED Peripheral driver C File
********************************************************************************
* External Functions
********************************************************************************
* void s4702018_reg_led_init(const struct led* led) - inits the led gpio pins
* void s4702018_reg_led_on(struct led* led) - switches the led on
* void s4702018_reg_led_off(struct led* led) - switches the led off
* void s4702018_reg_led_toggle(const struct led* led) - toggles the led
* void s4702018ThreadLEDInit(void) - inits the led thread
********************************************************************************
*/

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>

#include <s4702018_LED.h>

/* size of stack area used by each thread */
#define LED_STACKSIZE 1024

/* scheduling LED_THREAD_priority used by each thread */
#define LED_THREAD_PRIORITY 4

K_THREAD_STACK_DEFINE(LED_Thread_stack_area, LED_STACKSIZE);
static struct k_thread LED_Thread;

K_MSGQ_DEFINE(LED_command_queue, LED_MSG_SIZE, LED_QUEUE_SIZE, 4);

struct led led0 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED0_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED0_NODE, label, ""),
};

struct led led1 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED1_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED1_NODE, label, ""),
};

struct led led2 = {
    .spec = GPIO_DT_SPEC_GET_OR(LED2_NODE, gpios, {0}),
    .gpio_pin_name = DT_PROP_OR(LED2_NODE, label, ""),
};

/* s4702018_reg_led_init()
 * Initisalizes the given led.
 *
 * Params:
 * led (constr struct led): The struct for led containing its spec and gpio
 *                          pin name to be initisialized.
 * Returns:
 * None.
 * */
void s4702018_reg_led_init(const struct led* led) {
    const struct gpio_dt_spec* spec = &led->spec;
    int ret;

    if (!gpio_is_ready_dt(spec)) {
        printk("Error: %s device is not ready\n", spec->port->name);
        return;
    }

    ret = gpio_pin_configure_dt(spec, GPIO_OUTPUT);
    if (ret != 0) {
        printk("Error %d: failed to configure pin %d (LED '%s')\n", ret,
               spec->pin, led->gpio_pin_name);
        return;
    }
}

/* s4702018_reg_led_on
 * Switches on the given led.
 * Params:
 * led (struct led*): The led struct to switch on.
 *
 * Returns:
 * None
 */
void s4702018_reg_led_on(struct led* led) {
    const struct gpio_dt_spec* spec = &led->spec;
    int ret = gpio_pin_set_dt(spec, 1);
    if (ret < 0) {
        printk("Error %d: failed to turn off pin, pin %d (LED '%s')\n", ret,
               spec->pin, led->gpio_pin_name);
        return;
    }
}

/* s4702018_reg_led_off()
 * Switches the given led off
 * Params:
 * led (struct led*): the led struct to switch off
 *
 * Returns:
 * None
 */
void s4702018_reg_led_off(struct led* led) {
    const struct gpio_dt_spec* spec = &led->spec;
    int ret = gpio_pin_set_dt(spec, 0);
    if (ret < 0) {
        printk("Error %d: failed to turn off pin, pin %d (LED '%s')\n", ret,
               spec->pin, led->gpio_pin_name);
        return;
    }
}

/* s4702018_reg_led_toggle()
 * Toggles the Led on or off. (LED On -> LED Off or LED off -> LED on)
 *
 * Params:
 * led (const struct led*): The led to toggle.
 *
 * Returns:
 * None.
 */
void s4702018_reg_led_toggle(const struct led* led) {
    const struct gpio_dt_spec* spec = &led->spec;

    int ret = gpio_pin_toggle_dt(spec);
    if (ret < 0) {
        printk("Error %d: failed to toggle pin pin %d (LED '%s')\n", ret,
               spec->pin, led->gpio_pin_name);
        return;
    }
}

/* s4702018_reg_led_init()
 * main LED Task thread which calls the init function and process led
 * functionality given commands received from queues
 *
 * Params:
 * None.
 *
 * Returns:
 * None.
 */
void s4702018_LED_TaskPeripheralControl() {
    // Initialize all LEDs
    s4702018_reg_led_init(&led0);
    s4702018_reg_led_init(&led1);
    s4702018_reg_led_init(&led2);

    uint8_t data[LED_MSG_SIZE];

    while (1) {
        // Check for commands
        if (k_msgq_get(&LED_command_queue, data, K_NO_WAIT) == 0) {
            // toggle led
            if (data[1]) {
                s4702018_reg_led_toggle(&led0);
            }
            if (data[2]) {
                s4702018_reg_led_toggle(&led1);
            }
            if (data[0]) {
                s4702018_reg_led_toggle(&led2);
            }
        }

        k_sleep(K_MSEC(100));
    }
}

/* s4702018LEDTaskInit()
 * Initisalizes the thread for led functionality.
 *
 * Params:
 * None.
 *
 * Returns:
 * None.
 */
void s4702018ThreadLEDInit(void) {
    k_thread_create(&LED_Thread, LED_Thread_stack_area,
                    K_THREAD_STACK_SIZEOF(LED_Thread_stack_area),
                    s4702018_LED_TaskPeripheralControl, NULL, NULL, NULL,
                    LED_THREAD_PRIORITY, 0, K_FOREVER);
    k_thread_name_set(&LED_Thread, "LED_Thread");

    k_thread_start(&LED_Thread);
}
