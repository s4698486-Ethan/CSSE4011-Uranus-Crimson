/*
*******************************************************************************
* Copyright (c) 2021 by M5Stack
*                  Equipped with M5Core2 sample source code
*                                M5Core2
* Visit for more information: https://docs.m5stack.com/en/core/core2
* : https://docs.m5stack.com/zh_CN/core/core2
*
* Describe: MQTT.
* Date: 2021/11/5
*******************************************************************************
*/
#include "M5Core2.h"
#include <WiFi.h>
#include <PubSubClient.h>

WiFiClient espClient;
PubSubClient client(espClient);

#define CENTRE_X 110
#define CENTRE_Y 110

#define PADDING 6

#define PACKET_START 0xAA
#define MODE_DEFAULT 0x01
#define MODE_DESTURE 0x02

// Configure the name and password of the connected wifi and your MQTT Serve
// host.
const char* ssid        = "infrastructure";
const char* password    = "3wbq4fAm4uij";
const char* mqtt_server = "csse4011-iot.zones.eait.uq.edu.au";

const int trigPin = 33;
const int echoPin = 32;

long duration;
float distance;

unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

void setupWifi();
void callback(char* topic, byte* payload, unsigned int length);
void reConnect();
void setGrid();
void drawTurtleBot(uint32_t x, uint32_t y);

uint16_t x = 0;
uint16_t y = 0;

uint16_t left;
uint16_t right;

uint32_t wait;

void setup() {
    M5.begin();
    wait = 0;
    setupWifi();
    client.setServer(mqtt_server, 1883);  // Sets the server details.
    client.setCallback(callback);  // Sets the message callback function.
    delay(10);
    M5.Lcd.fillScreen(BLACK);
    delay(10);
    setGrid();
    M5.Lcd.setTextColor(WHITE);
    M5.Lcd.setTextSize(1);
    Serial2.begin(115200, SERIAL_8N1, 13, 14);
}

void loop() { 
    if (!client.connected()) {
      reConnect();
    }

    client.loop();
    // Drawing Bounding Box
    // Top line

    if (Serial2.available()) {
      int start = Serial2.read();
      int command = Serial2.read();
      int length = Serial2.read();
      uint16_t data[length];
      for (int i = 0; i < length; i++) {
        data[i] = Serial2.read();
      }
      left = (data[0] << 8) | data[1];
      right = (data[2] << 8) | data[3];
      //M5.Lcd.setCursor(220, 70);
      //M5.Lcd.printf("Serial2:%d\n", ch);
      //delay(10);
    }

    M5.Lcd.setCursor(220, 10);
    M5.Lcd.printf("Gesture:");
    delay(10);

    M5.Lcd.setCursor(220, 40);
    M5.Lcd.printf("Coords: (%d, %d)", x, y);
    delay(10);

    if (left != 0) {
      drawTurtleBot(left, right);
    } else {
      drawTurtleBot(CENTRE_X, CENTRE_Y);
    }
}

void setGrid() {
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
    M5.Lcd.drawFastVLine(60, 10, 200, YELLOW);
    delay(10);   

    M5.Lcd.drawFastVLine(110, 10, 200, YELLOW);
    delay(10); 

    M5.Lcd.drawFastVLine(160, 10, 200, YELLOW);
    delay(10); 

    // Horizontal grid lines
    M5.Lcd.drawFastHLine(10, 60, 200, YELLOW);
    delay(10);

    M5.Lcd.drawFastHLine(10, 110, 200, YELLOW);
    delay(10);

    M5.Lcd.drawFastHLine(10, 160, 200, YELLOW);
    delay(10);
}

void drawTurtleBot(uint32_t x, uint32_t y) {
  // Draw TurtleBot centred at x and y
  M5.Lcd.fillRect(x - PADDING, y - PADDING, 2 * PADDING, 2 * PADDING, BLUE);
  delay(10);
}

void setupWifi() {
    delay(10);
    //M5.Lcd.printf("Connecting to %s", ssid);
    WiFi.mode(
        WIFI_STA);  // Set the mode to WiFi station mode. 
    WiFi.begin(ssid, password);  // Start Wifi connection. 

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        //M5.Lcd.print(".");
    }
    //M5.Lcd.printf("\nSuccess\n");
}

void callback(char* topic, byte* payload, unsigned int length) {
    M5.Lcd.print("Message arrived [");
    M5.Lcd.print(topic);
    M5.Lcd.print("] ");
    for (int i = 0; i < length; i++) {
        M5.Lcd.print((char)payload[i]);
    }
    M5.Lcd.println();
}

void reConnect() {
    while (!client.connected()) {
        //M5.Lcd.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "45815018-";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect. 
        if (client.connect(clientId.c_str())) {
            client.subscribe("45815018");
            client.subscribe("un45815018");
            continue;
        } else {
            delay(5000);
        }
    }
}
