/**
********************************************************************************
* @author Henry Sommerville - s4702018
* @date 11.05.2024
* @brief M5 Core sketch for final project using micro-ros
********************************************************************************
*************************************************************************
*/
#include "M5Core2.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t ultrasonic_publisher;
rcl_publisher_t gesture_publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_subscription_t subscriber;

#define PACKET_START 0xAA
#define MODE_DEFAULT 0x01
#define MODE_GESTURE 0x02

#define MAX_DATA_LENGTH 60 

uint8_t data[MAX_DATA_LENGTH];

int x_pos = 110;
int y_pos = 110;
int old_x_pos = 110;
int old_y_pos = 110;

void UART_init() {
  Serial2.begin(115200, SERIAL_8N1, 13, 14);
}

void draw_grid() {
  // Draw Grid bounding box
  // Top line
    M5.Lcd.drawFastHLine(10, 10, 200, YELLOW);
    delay(10);
    // Bottom line
    M5.Lcd.drawFastHLine(10, 210, 200, YELLOW);
    delay(10);
    // Left vertical
    M5.Lcd.drawFastVLine(10, 10, 200, YELLOW);
    delay(10);
    // Right vertical
    M5.Lcd.drawFastVLine(210, 10, 200, YELLOW);
    delay(10);

    // Drawing Grid lines
    // Vertical Grid lines
    for (int c = 0; c < 3; c++) {
      M5.Lcd.drawFastVLine(60 + (c * 50), 10, 200, YELLOW);
      delay(10);
    }

    //Horizontal Lines
    for (int r = 0; r < 3; r++) {
      M5.Lcd.drawFastHLine(10, 60 + (r * 50), 200, YELLOW);
      delay(10);
    }
}

void drawTurtleBot(uint32_t x, uint32_t y) {
  uint8_t padding = 6
  // Draw TurtleBot centred at x and y
  M5.Lcd.fillRect(x - padding, y - padding, 2 * padding, 2 * padding, BLUE);
  delay(10);
}

void lcd_draw_coordinates(int x, int y) {
  // Create black rectangle over old coordinates
  M5.Lcd.fillRect(220, 40, 180, 20, BLACK);
  M5.Lcd.setCursor(220, 40);
  M5.Lcd.printf("X: %d, Y: %d", x, y);
}

void lcd_handler() {
  if (old_x_pos != x_pos || old_y_pos != y_pos) {
    old_x_pos = x_pos;
    old_y_pos = y_pos;
    lcd_draw_coordinates(x_pos, y_pos);

    drawTurtleBot(x_pos, y_pos);

  }
}

void LCD_init() {
  // Draw initial Grid
    M5.Lcd.fillScreen(BLACK);
    draw_grid();
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    M5.Lcd.setCursor(220, 40);
}

// Subriber callback ON_MSG_NEW
//TODO Need to check if this is the right message type.
void subscription_callback(const void* msgin) {
    // Parse received position data and draw on the grid
    const std__msgs__msg__Int16* msg = (const std_msgs_msgs__Int16*)msgin;
    x_pos =(uint8_t)(((msg->data) >> 8) & 0xFF);
    y_pos = (uint8_t)((msg->data) & 0xFF);    
}

// Poll UART for new messages regarding ultrasonic or gesture.
int uart_poll() {
    if (!Serial2.available()) {
        return 0; // No data available
    }
    
    // Read the start byte
    int start = Serial2.read();
    if (start != PACKET_START) {
        return 0; // Invalid start byte
    }

    // Read the length of the packet
    while (!Serial2.available()) {}
    int length = Serial2.read();
    if (length == 0 || length > MAX_DATA_LENGTH) {
        return 0; // Invalid packet length
    }

    // Read the rest of the packet
    for (int i = 0; i < length; i++) {
        while (!Serial2.available()) {}
        data[i] = Serial2.read();
    }

    return 1; // Packet successfully read
}


void handle_ultrasonic_data(uint16_t left, uint16_t right) {
    //TODO: check if this is correct message type. Using 1 int32 to send messages across. Could cause errors since uint16_t?
    msg.data = (left << 16) | right;
    RCSOFTCHECK(rcl_publish(&ultrasonic_publisher, &msg, NULL));
}

void handle_gesture(uint8_t gesture) {
    //TODO: check if this is correct message type. Using 1 int32 to send messages across. Could cause errors since uint16_t?
    msg.data = (int32_t)gesture;
    RCSOFTCHECK(rcl_publish(&gesture_publisher, &msg, NULL));
}

void handle_packet(uint8_t* data) {
    int command = data[0];
    int length = data[1];
    if (command == MODE_DEFAULT && length == 4) {
        // handle ultrasonic measurements
        uint16_t left = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
        uint16_t right = ((uint16_t)data[4] << 8) | (uint16_t)data[5];

        // Call handle function
        handle_ultrasonic_data(left, right);
    } else if (command == MODE_GESTURE) {
        // handle gesture
        gesture = data[3];
        handle_gesture(gesture);
    } else {
        return;
    }
}


void setup() {
    // Initialise M5 Core
    M5.begin();

    UART_init();
    LCD_init();
    set_microros_transports();

    delay(2000);

    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();

    RCCHECK(rcl_init_options_init(&init_options, allocator));
    rmw_init_options_t* rmw_options = rcl_init_options_get_rmw_init_options(&init_options);

    size_t domain_id = 39;

    RCCHECK(rcl_init_options_set_domain_id(&init_options, domain_id));

    // create init_options
    RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "M5_Node", "", &support));

    // create publisher
    RCCHECK(rclc_publisher_init_default(
        &ultrasonic_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "M5Core_UltraSonic_Publisher"
    ));

    RCCHECK(rclc_publisher_init_default(
        &gesture_publisher,
        &node
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "M%Core_Gesture_Publisher"
    ));

    // create subscriber
    // TODO: need to check message type, and topic name
    RCCHECK(rcls_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), 
        "/position:"
    ));

    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
    RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));

    msg.data = 0;

    // Start infinite loop I have done this to avoid globals
    while (1) {
    //  Check if client has lost connection, if so reconnect
        if (!client.connected()) {
        reConnect();
        }

        // Check Serial Communication
        if (uart_poll() == 1) {
            handle_packet(data);
        }

        lcd_handler();
        RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
    }
}

void loop() {

}
