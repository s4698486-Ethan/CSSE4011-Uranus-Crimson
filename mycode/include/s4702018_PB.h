/**
********************************************************************************
* @file includes/s4702018_PB.h
* @author Henry Sommerville - s4702018
* @date 24.02.2024
* @brief PB Peripheral driver Header File
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

#ifndef S4702018_PB_H
#define S4702018_PB_H

#include <zephyr/device.h>
#include <zephyr/kernel.h>

#define SW0_NODE DT_ALIAS(sw0)

#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

#define RECEIVE_MSG_SIZE 1
#define SEND_MSG_SIZE 1
#define PB_QUEUE_SIZE 1

extern struct k_msgq uart_to_pb_queue;
extern struct k_msgq pb_to_uart_queue;

void s4702018ThreadPBInit(void);

#endif