/**
********************************************************************************
* @file mylib/s4702018_UART.c
* @author Henry Sommerville - s4702018
* @date 02.032024
* @brief UART Peripheral driver File
********************************************************************************
* External Functions
********************************************************************************
* void s4702018_uart_reg_init(void) - init the uart
* void s4702018_uart_transmit(char* buf, uint8_t length) - transmit via uart
********************************************************************************
*/

#include <stdio.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>

#include <s4702018_UART.h>

/* Define uart device */
static const struct device* const uart_dev =
    DEVICE_DT_GET(DT_NODELABEL(UART_NODE_LABEL));

/* queue to store up to 10 messages (aligned to 4-byte boundary) */
K_MSGQ_DEFINE(uart_msgq, MSG_SIZE, 10, 4);

/* UART Config Struct, defines rate, and protocol */
struct uart_config uart_cfg = {
    .baudrate = UART_BAUDRATE,
    .parity = UART_CFG_PARITY_NONE,
    .stop_bits = UART_CFG_STOP_BITS_1,
    .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    .data_bits = UART_CFG_DATA_BITS_8,
};

/* rx buffer for uart transmission */
static char rx_buf[MSG_SIZE];

/* variable to track current index for rx buffer */
static int rx_buf_pos = 0;

volatile uint8_t command_length;

/* serial_cb()
 * Read characters from UART until line end is detected. Afterwards push the
 * data to the message queue.
 *
 * Params:
 * dev (const struct device): The uart device struct
 * user_data (void*): user data for callback.
 *
 * Returns:
 * None
 */
void serial_cb(const struct device* dev, void* user_data) {
    uint8_t c;

    if (!uart_irq_update(uart_dev)) {
        return;
    }
    if (!uart_irq_rx_ready(uart_dev)) {
        return;
    }

    /* read until FIFO empty */
    while (uart_fifo_read(uart_dev, &c, 1) == 1) {
        if (rx_buf_pos == 0) {
            // check for start byte
            if (c == COMMAND_PREAMBLE) {
                rx_buf[rx_buf_pos] = c;
                rx_buf_pos++;
            }
        } else if (rx_buf_pos == 1) {
            rx_buf[rx_buf_pos] = c;
            rx_buf_pos++;
            command_length = (c & 0x0F);
        } else if (

            rx_buf_pos > 1 &&
            rx_buf_pos <=
                (command_length +
                 2)) {  // command length + 2 since value doesn't include the preamble byte
                        // and length/type
            rx_buf[rx_buf_pos] = c;
            rx_buf_pos++;
        }

        // check if whole message is received.
        if (rx_buf_pos == (command_length + 2)) {
            // if queue is full, message is dropped
            k_msgq_put(&uart_msgq, &rx_buf, K_NO_WAIT);

            // Reset the buffer (it was copied to the msgq)
            rx_buf[0] = 0x00;
            rx_buf_pos = 0;
        }
    }
}

/* s4702018_uart_transmit()
 * writes the given buffer to uart
 *
 * Params:
 * buf (char*): array of data to write to uart tx
 *
 * Returns:
 * None.
 */
void s4702018_uart_transmit(char* buf, uint8_t length) {
    for (int i = 0; i < length; i++) {
        uart_poll_out(uart_dev, buf[i]);
        k_sleep(K_MSEC(100));
    }
}

/*  s4702018_uart_reg_init()
 * initializes the uart device
 *
 * Params:
 * None.
 *
 * Returns:
 * None
 */
void s4702018_uart_reg_init(void) {
    if (!device_is_ready(uart_dev)) {
        printk("UART device not found!");
        return;
    }

    int rc = uart_configure(uart_dev, &uart_cfg);
    if (rc) {
        printk("Could not configure device\n");
        return;
    }

    /* configure interrupt and callback to receive data */
    int ret = uart_irq_callback_user_data_set(uart_dev, serial_cb, NULL);
    if (ret < 0) {
        if (ret == -ENOTSUP) {
            printk("Interrupt-driven UART API support not enabled\n");
        } else if (ret == -ENOSYS) {
            printk("UART device does not support interrupt-driven API\n");
        } else {
            printk("Error setting UART callback: %d\n", ret);
        }
        return;
    }

    // Enable uart interrupt
    // uart_irq_rx_enable(uart_dev);
}
