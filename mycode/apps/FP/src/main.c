/**
********************************************************************************
* @file FP/src/s4702018_main.c
* @author Henry Sommerville - s4702018
* @date 28.04.2024
* @brief main file for final project
********************************************************************************
*/

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <limits.h>

#include <sys/time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define PACKET_START 0xAA
#define MODE_DEFAULT 0x01
#define MODE_GESTURE 0x02

#define ZEPHYR_USER_NODE DT_PATH(zephyr_user)

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
    volatile uint32_t stop_time;
    volatile uint32_t start_time;
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
        } while (gpio_pin_get_dt(echo) == 1);

        stop_time = k_cycle_get_32();
        cycles_spent = stop_time - start_time;
        if (cycles_spent > 1266720) {
            return 0;
        }
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
    s4702018_PB_reg_init();
    uint32_t left_distance;
    uint32_t right_distance;
    uint8_t mode = MODE_DEFAULT;
    uint8_t packet[7];
    time_t first_gesture_det_times;
    time_t second_gesture_det_times;
    uint8_t first_gesture_detected;
    uint8_t second_gesture_detected;

    mode = MODE_GESTURE;
    while(1) {
        int gesture_complete = 0;

       right_distance = get_distance(&trig_right, &echo_right);
       printk("RIGHT DISTANCE: %d\n", right_distance);
       left_distance = get_distance(&trig_left, &echo_left);
       printk("LEFT DISTANCE: %d\n", left_distance);

       //Get distance for both ultrasonic.
       uint8_t button = s4702018_button_presses();
       printk("MAIN FILE: %d\n", button);
       if (button == 1) {
            printk("gesture mode: \n");
            mode = MODE_GESTURE;
       } else {
            printk("default mode: \n");
            mode = MODE_DEFAULT; 
       }

        if (mode == MODE_GESTURE) {

            


            //TODO Check if the opposite sensor is zero, or greater than a certain value.
            
            // ############################### GESTURE 1 #########################//
            // This is the right, left, right gesture.
            if (right_distance >= 15 && right_distance <= 30) {
                first_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());// * 1000;

                printk("(right left right): Right _____ ______");
                k_sleep(K_MSEC(500));


                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while(((k_cyc_to_ms_floor32(k_cycle_get_32())/* * 1000*/) - first_gesture_det_times) <= 2000) {
                    // NOT SUCCESSFULLY GETTING INTO THIS LOOP

                    right_distance = get_distance(&trig_right, &echo_right);
                    //printk("RIGHT DISTANCE: %d\n", right_distance);
                    left_distance = get_distance(&trig_left, &echo_left);
                    printk("LEFT DISTANCE: %d\n", left_distance);
                    //printk("==============================");

                    //printk("Waiting in gesture 1 #################### \n");
                    if (left_distance >= 15 && left_distance <= 30) {
                        
                        
                        if (right_distance <= 50) {
                            // Just prevents holding hands at a fixed distance away reading as a gesture by breaking out if right hands
                            // held fixed

                            break;
                        } else {

                            second_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());// * 1000;

                            printk("(right left right): Right Left ______\n");

                            // Note that we have received our second part of the gesture
                            // and then "scan" for the final part of the gesture for 5s.
                            second_gesture_detected = 1;
                            // Also want to break out of inner while loop.

                            k_sleep(K_MSEC(500));
                            break;
                        }
                    }
                    k_sleep(K_MSEC(50));
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. 
                    while((k_cyc_to_ms_floor32(k_cycle_get_32()) - second_gesture_det_times) <= 2000) {

                        right_distance = get_distance(&trig_right, &echo_right);
                        printk("RIGHT DISTANCE: %d\n", right_distance);
                        left_distance = get_distance(&trig_left, &echo_left);
                        //printk("LEFT DISTANCE: %d\n", left_distance);
                        //printk("==============================");

                        if (right_distance >= 15 && right_distance <= 30) {
                            if (left_distance <= 50) {
                                break;
                            } else {
                                printk("(right left right): Right Left Right\n");
                                printk("GESTURE 1: COMPLETE ##################\n");
                                // SET DISTANCES TO BE SENT VIA UART. Snake right to left.

                                gesture_complete = 1;

                                packet[0] = PACKET_START;
                                packet[1] = MODE_GESTURE;
                                packet[2] = 1;
                                packet[3] = 0x01;
                                s4702018_uart_transmit(packet, 4);
                                break;
                            }
                        }

                        k_sleep(K_MSEC(50));
                    }
                }
            }


            // ############################### GESTURE 2 #########################//
            // This is the left, right, left gesture.
            if (left_distance >= 15 && left_distance <= 30) {
                first_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());// * 1000;

                printk("(left right left): Left _____ ______\n");

                k_sleep(K_MSEC(500));

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((k_cyc_to_ms_floor32(k_cycle_get_32()) - first_gesture_det_times) <= 2000) {

                        right_distance = get_distance(&trig_right, &echo_right);
                        printk("RIGHT DISTANCE: %d\n", right_distance);
                        left_distance = get_distance(&trig_left, &echo_left);

                    if (right_distance >= 15 && right_distance <= 30) {

                        if (left_distance <= 50) {
                            break;
                        } else {

                        second_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());
                        printk("(left right left): Left Right ______\n");
                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.

                        k_sleep(K_MSEC(500));

                        break;
                        }
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((k_cyc_to_ms_floor32(k_cycle_get_32()) - second_gesture_det_times) <= 2000) {

                        right_distance = get_distance(&trig_right, &echo_right);
                        left_distance = get_distance(&trig_left, &echo_left);
                        printk("LEFT DISTANCE: %d\n", left_distance);

                        if (left_distance >= 15 && left_distance <= 30) {

                            if (right_distance <= 50) {
                                break;
                            } else {

                                printk("(left right left): Left Right Left\n");
                                printk("############################# GESTURE COMPLETE ##############\n");


                                gesture_complete = 1;

                                // SET DISTANCES TO BE SENT VIA UART. Snake left to right.
                                packet[0] = PACKET_START;
                                packet[1] = MODE_GESTURE;
                                packet[2] = 1;
                                packet[3] = 0x02;
                                s4702018_uart_transmit(packet, 4);
                                break;
                            }
                        }
                    }
                }
            }

            // ############################### GESTURE 3 #########################//
            // This is the right-side down, up, down gesture. 
            // TO ACTIVATE: Move hand from (10, 30)cm to (40, 60)cm to (10, 30)cm within
            //              a couple of seconds ON THE RIGHT SENSOR.
            if (right_distance >= 40 && right_distance <= 60) {
                first_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());// * 1000;

                printk("RIGHT (out in out): Out _____ ______\n");
                k_sleep(K_MSEC(500));
                

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((k_cyc_to_ms_floor32(k_cycle_get_32())- first_gesture_det_times) <= 2000) {

                    right_distance = get_distance(&trig_right, &echo_right);
                    printk("RIGHT DISTANCE: %d\n", right_distance);

                    if (right_distance >= 10 && right_distance <= 30) {
                        second_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.
                        printk("RIGHT (out in out): Out In _____\n");

                        k_sleep(K_MSEC(500));

                        break;
                    }

                    k_sleep(K_MSEC(50));
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((k_cyc_to_ms_floor32(k_cycle_get_32()) - second_gesture_det_times) <= 2000) {

                        right_distance = get_distance(&trig_right, &echo_right);
                        printk("RIGHT DISTANCE: %d\n", right_distance);

                        if (right_distance >= 40 && right_distance <= 60) {
                            printk("RIGHT (out in out): Out In Out\n");
                            printk("#################### GESTURE COMPLETE ################\n");
                            // SET DISTANCES TO BE SENT VIA UART. Rotate left.

                            gesture_complete = 1;

                            packet[0] = PACKET_START;
                            packet[1] = MODE_GESTURE;
                            packet[2] = 1;
                            packet[3] = 0x03;
                            s4702018_uart_transmit(packet, 4);
                            break;
                        }
                    }
                }  
            }



            // ############################### GESTURE 4 #########################//
            // This is the left-side down, up, down gesture. 
            // TO ACTIVATE: Move hand from (10, 30)cm to (40, 60)cm to (10, 30)cm within
            //              a couple of seconds ON THE LEFT SENSOR.
            if (left_distance >= 40 && left_distance <= 60) {
                first_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());//* 1000;

                printk("LEFT (out in out): Out _____  _____\n");
                k_sleep(K_MSEC(500));

                // Note that we have received our first half of the gesture
                // and then "scan" for the second half of the gesture for 5s.
                while((k_cyc_to_ms_floor32(k_cycle_get_32()) - first_gesture_det_times) <= 2000) {

                    left_distance = get_distance(&trig_left, &echo_left);
                    printk("LEFT DISTANCE: %d\n", left_distance);

                    if (left_distance >= 10 && left_distance <= 30) {
                        second_gesture_det_times = k_cyc_to_ms_floor32(k_cycle_get_32());

                        // Note that we have received our second part of the gesture
                        // and then "scan" for the final part of the gesture for 5s.
                        second_gesture_detected = 1;
                        // Also want to break out of inner while loop.

                        printk("LEFT (out in out): Out In  _____\n");
                        k_sleep(K_MSEC(500));

                        break;
                    }
                }

                if (second_gesture_detected) {
                    // Looking for final part of the gesture. If this doesn't
                    // happen then we have only detected the first part of the
                    // gesture and should drop out.
                    while((k_cyc_to_ms_floor32(k_cycle_get_32()) - second_gesture_det_times) <= 2000) {

                        left_distance = get_distance(&trig_left, &echo_left);
                        printk("LEFT DISTANCE: %d\n", left_distance);

                        if (left_distance >= 40 && left_distance <= 60) {

                            printk("LEFT (out in out): Out In Out\n");
                            printk("################## GESTURE COMPLETE #############\n");
                            
                            
                            gesture_complete = 1;

                            // SET DISTANCES TO BE SENT VIA UART. Rotate left.
                            packet[0] = PACKET_START;
                            packet[1] = MODE_GESTURE;
                            packet[2] = 1;
                            packet[3] = 0x04;
                            s4702018_uart_transmit(packet, 4);
                            break;
                        }
                    }
                }  
            } 

            // ############################### GESTURE 5 #########################//
            // This is stop gesture, just hold both hands in front of sensor very 
            // closely. 
            if (1 <= left_distance && left_distance <= 5) {
                if (1 <= right_distance && right_distance <= 5) {

                    printk("STOP GESTURE\n");
                    printk("################## GESTURE COMPLETE #############\n");

                    gesture_complete = 1;

                    packet[0] = PACKET_START;
                    packet[1] = MODE_GESTURE;
                    packet[2] = 1;
                    packet[3] = 0x04;
                    s4702018_uart_transmit(packet, 4);
                }    
            }

            second_gesture_detected = 0;

        } else {
            // Send distances via UART
            packet[0] = PACKET_START;
            packet[1] = MODE_DEFAULT;
            packet[2] = 4;
            packet[3] = (left_distance >> 8) & 0xFF;
            packet[4] = (left_distance) & 0xFF;
            packet[5] = (right_distance >> 8) & 0xFF;
            packet[6] = (right_distance) & 0xFF;
            s4702018_uart_transmit(packet, 7);
        }
        k_sleep(K_MSEC(500));

        if (gesture_complete) {
            k_sleep(K_MSEC(2500));
        }


    }
    return 0;
}