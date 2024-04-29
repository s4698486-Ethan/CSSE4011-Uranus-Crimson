
/**
********************************************************************************
* @file includes/s4702018_UART.h
* @author Henry Sommerville - s4702018
* @date 2.03.2024
* @brief UART Peripheral driver Header File
********************************************************************************
* External Functions
********************************************************************************
* void s4702018_uart_reg_init(void) - init the uart
* void s4702018_uart_transmit(char* buf, uint8_t length) - transmit via uart
********************************************************************************
*/

#ifndef S4702018_UART_H
#define S4702018_UART_H

#define UART_NODE_LABEL uart4
#define UART_BAUDRATE DT_PROP(DT_NODELABEL(UART_NODE_LABEL), current_speed)

void s4702018_uart_reg_init(void);
void s4702018_uart_transmit(char* buf, uint8_t length);

#define COMMAND_PREAMBLE 0xAA
#define MSG_SIZE 10

#endif
