/**
********************************************************************************
* @file P1/src/s4702018_main.c
* @author Henry Sommerville - s4702018
* @date 22.02.2024
* @brief main file for prac 2
********************************************************************************
*/

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <limits.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <time.h>

#define MODE_DEFAULT 0x00
#define MODE_GESTURE 0x01

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

int gesture_mode_on = 0;

/* Define GPIO pins for STM32 */
const struct gpio_dt_spec trig_right =
           GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, trig1_gpios);

const struct gpio_dt_spec echo_right =
           GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, echo1_gpios);

const struct gpio_dt_spec trig_left =
           GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, trig2_gpios);

const struct gpio_dt_spec echo_left =
           GPIO_DT_SPEC_GET(ZEPHYR_USER_NODE, echo2_gpios);


uint32_t get_distance(const struct gpio_dt_spec* trig, const struct gpio_dt_spec* echo) {
    uint32_t cycles_spent = 0;
    uint32_t ms_duration;
    volatile uint32_t val;
    uint32_t cm;
    uint32_t stop_time;
    uint32_t start_time;
    uint8_t ret;

    /* Trigger ultrasonic sensor */
        ret = gpio_pin_set_dt(trig, 1);
        if (ret != 0) {
            printk("Error set trig");
            return 0;
        }
        k_sleep(K_MSEC(10));

        ret = gpio_pin_set_dt(trig, 0);
        if (ret !=0) {
            printk("Error set trig");
            return 0;
        }

        do {
		} while (gpio_pin_get_dt(echo) == 0);
		
        start_time = k_cycle_get_32();

		do {
			val = gpio_pin_get_dt(echo);
            stop_time = k_cycle_get_32();
            if (cycles_spent > 1266720) {
                return 0;
            }
        } while (val == 1);

        cycles_spent = stop_time - start_time;
        ms_duration = k_cyc_to_us_floor32(cycles_spent);
        cm = ms_duration * 0.034 / 2;



        return cm;
}

void ultrasonic_inits() {
    int ret;
    ret = gpio_pin_configure_dt(&trig_left, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Error setting trig");
    }
    ret = gpio_pin_configure_dt(&echo_left, GPIO_INPUT);
    if (ret != 0) {
        printk("Error setting echo");
    }

    ret = gpio_pin_configure_dt(&trig_right, GPIO_OUTPUT_INACTIVE);
    if (ret != 0) {
        printk("Error setting trig");
    }
    ret = gpio_pin_configure_dt(&echo_right, GPIO_INPUT);
    if (ret != 0) {
        printk("Error setting echo");
    }
}

int main(void) {
    ultrasonic_inits();
    s4702018_uart_reg_init();
    uint32_t left_distance;
    uint32_t right_distance;
    uint8_t mode = MODE_DEFAULT;
    uint8_t packet[7];
    time_t first_gesture_det_times;
    time_t second_gesture_det_times;
    uint8_t first_gesture_detected;
    uint8_t second_gesture_detected;


    while(1) {
       right_distance = get_distance(&trig_right, &echo_right);
       printk("RIGHT DISTANCE: %d\n", right_distance);
       k_sleep(K_MSEC(500));
       left_distance = get_distance(&trig_left, &echo_left);
       printk("LEFT DISTANCE: %d\n", left_distance);
       k_sleep(K_MSEC(500));

       //Get distance for both ultrasonic.
       if (pushbutton status is 1) {
            mode = MODE_GESTURE;
       } else {
            mode = MODE_DEFAULT; 
       }

        if (mode == MODE_GESTURE) {
            // Check if there is a gesture? not exactly sure how to do this
            

            // ############################### GESTURE 1 #########################//
            // This is the right, left, right gesture.
            if (right_distance >= 45 && right_distance <= 55) {
                first_gesture_det_times = time(NULL);

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((time() - first_gesture_det_times) <= 5) {
                    if (left_distance >= 45 && left_distance <= 55) {
                        second_gesture_det_times = time(NULL);

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.
                        break;
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. 
                    while((time() - second_gesture_det_times) <= 5) {
                        if (right_distance >= 45 && right_distance <= 55) {
                            
                            // SET DISTANCES TO BE SENT VIA UART. Snake right to left.
                            break;
                        }
                    }
                }
            }



            // ############################### GESTURE 2 #########################//
            // This is the left, right, left gesture.
            if (left_distance >= 45 && left_distance <= 55) {
                first_gesture_det_times = time(NULL);

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((time() - first_gesture_det_times) <= 5) {
                    if (right_distance >= 45 && right_distance <= 55) {
                        second_gesture_det_times = time(NULL);

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.
                        break;
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((time() - second_gesture_det_times) <= 5) {
                        if (left_distance >= 45 && left_distance <= 55) {
                            
                            // SET DISTANCES TO BE SENT VIA UART. Snake left to right.
                            break;
                        }
                    }
                }

            // ############################### GESTURE 3 #########################//
            // This is the right-side down, up, down gesture. 
            // TO ACTIVATE: Move hand from (10, 30)cm to (40, 60)cm to (10, 30)cm within
            //              a couple of seconds ON THE RIGHT SENSOR.
            if (right_distance >= 10 && right_distance <= 30) {
                first_gesture_det_times = time(NULL);

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((time() - first_gesture_det_times) <= 5) {
                    if (right_distance >= 40 && right_distance <= 60) {
                        second_gesture_det_times = time(NULL);

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.
                        break;
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((time() - second_gesture_det_times) <= 5) {
                        if (right_distance >= 10 && right_distance <= 30) {
                            
                            // SET DISTANCES TO BE SENT VIA UART. Rotate left.
                            break;
                        }
                    }
                }  
            }



            // ############################### GESTURE 4 #########################//
            // This is the left-side down, up, down gesture. 
            // TO ACTIVATE: Move hand from (10, 30)cm to (40, 60)cm to (10, 30)cm within
            //              a couple of seconds ON THE LEFT SENSOR.
            if (left_distance >= 10 && left_distance <= 30) {
                first_gesture_det_times = time(NULL);

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((time() - first_gesture_det_times) <= 5) {
                    if (left_distance >= 40 && left_distance <= 60) {
                        second_gesture_det_times = time(NULL);

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.
                        break;
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((time() - second_gesture_det_times) <= 5) {
                        if (left_distance >= 10 && left_distance <= 30) {
                            
                            // SET DISTANCES TO BE SENT VIA UART. Rotate left.
                            break;
                        }
                    }
                }  
            }

            second_gesture_detected = 0;

        } else {
            // Send distances via UART
        }
    }
    return 0;
}

