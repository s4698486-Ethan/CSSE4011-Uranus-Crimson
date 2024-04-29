/**
********************************************************************************
* @file includes/s4702018_LED.h
* @author Henry Sommerville - s4702018
* @date 23.02.2024
* @brief LED Peripheral driver Header File
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

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

#ifndef S4702018_LED_H
#define S4702018_LED_H

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)
#define LED2_NODE DT_ALIAS(led2)
#define LED3_NODE DT_ALIAS(led3)

#if !DT_NODE_HAS_STATUS(LED0_NODE, okay)
#error "Unsupported board: led0 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED1_NODE, okay)
#error "Unsupported board: led1 devicetree alias is not defined"
#endif

#if !DT_NODE_HAS_STATUS(LED2_NODE, okay)
#error "Unsupported board: led2 depvicetree alias is not defined"
#endif

#define LED_MSG_SIZE 3
#define LED_QUEUE_SIZE 10

extern struct k_msgq LED_command_queue;

struct led {
    struct gpio_dt_spec spec;
    const char* gpio_pin_name;
};

void s4702018_reg_led_init(const struct led* led);
void s4702018_reg_led_on(struct led* led);
void s4702018_reg_led_off(struct led* led);
void s4702018_reg_led_toggle(const struct led* led);
void s4702018ThreadLEDInit(void);

#endif