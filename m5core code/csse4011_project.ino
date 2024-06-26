/**
********************************************************************************
* @author Henry Sommerville - s4702018
* @date 08.05.2024
* @brief M5 Core sketch for final project
********************************************************************************
*************************************************************************
*/
#include "M5Core2.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

WiFiClient espClient;
PubSubClient client(espClient);

#define CENTRE_X 110
#define CENTRE_Y 110

#define PADDING 6

#define PACKET_START 0xAA
#define MODE_DEFAULT 0x01
#define MODE_GESTURE 0x02
#define MODE_POSITION 0x03

#define MAX_DATA_LENGTH 60 // Maximum length of data packet, adjust as needed

float actual_x = 2;
float actual_y = 2;
float x_pos = 2;
float y_pos = 2;
float old_x_pos = 110;
float old_y_pos = 110;

uint8_t data[MAX_DATA_LENGTH];


void setupWifi(const char* ssid, const char* password) {
    delay(10);
    WiFi.mode(WIFI_STA);  // Set the mode to WiFi station mode. 
    WiFi.begin(ssid, password);  // Start Wifi connection. 

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
    if (strcmp(topic, "s4702018/M5/position") == 0) {
        // M5.Lcd.print("Message arrived [");
        // M5.Lcd.print(topic);
        // M5.Lcd.print("] ");
        
        // Assuming payload is a null-terminated string
        payload[length] = '\0';

        // Create a JSON document
        StaticJsonDocument<200> doc;

        // Parse the JSON payload
        DeserializationError error = deserializeJson(doc, payload);

        // Check for parsing errors
        if (error) {
            // M5.Lcd.print("JSON parsing failed: ");
            // M5.Lcd.println(error.c_str());
            return;
        }

        // Extract command, x, and y values
        int command = doc["command"];

        if (command == MODE_POSITION) {
            float x = doc["x"];
            float y = doc["y"];
            actual_x = x;
            actual_y = y;
            if (x > 2.0) {
                x = 2.0;
            } else if (x < 0.0) {
                x = 0;
            } else {
                x = (x - 0.0) / (2.0 - 0.0) * ((210 - 10) + 210);
            }
            // Calculate pixel coordinates within the grid
            
            if (y > 2.0) {
                y = 2.0;
            } else if (y < 0.0) {
                y = 0;
            } else {
                y = (y - 0.0) / (2.0 - 0.0) * ((210 - 10) + 210);
            }

            x_pos = x;
            y_pos = y;
        }
    }
}

void reConnect() {
    while (!client.connected()) {
        //M5.Lcd.print("Attempting MQTT connection...");
        // Create a random client ID
        String clientId = "4702018";
        clientId += String(random(0xffff), HEX);
        // Attempt to connect. 
        if (client.connect(clientId.c_str())) {
            client.subscribe("s4702018/M5/ultrasonic");
            client.subscribe("s4702018/M5/position");
            continue;
        } else {
            delay(1000);
        }
    }
}

void MQTT_init(const char* mqtt_server, const char* ssid, const char* password) {
    setupWifi(ssid, password);
    client.setServer(mqtt_server, 1883);  // Sets the server details.
    client.setCallback(callback);  // Sets the message callback function.
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

void drawTurtleBot(float x, float y) {    
    // Draw TurtleBot centred at x and y
    M5.Lcd.fillRect(x - PADDING, y - PADDING, 2 * PADDING, 2 * PADDING, BLUE);
    delay(10);
}

void lcd_draw_coordinates(float x, float y) {
    // Create black rectangle over old coordinates
    M5.Lcd.fillRect(220, 40, 180, 20, BLACK);
    M5.Lcd.setCursor(220, 40);
    char x_str[8]; // Enough to hold a float with one decimal place
    char y_str[8]; // Enough to hold a float with one decimal place
    dtostrf(x, 6, 1, x_str); // Format grid_x to one decimal place
    dtostrf(y, 6, 1, y_str); // Format grid_y to one decimal place
    M5.Lcd.printf("X:%fY:%f",x, y);
}

void lcd_handler() {

  if (old_x_pos != x_pos || old_y_pos != y_pos) {
    old_x_pos = x_pos;
    old_y_pos = y_pos;
    lcd_draw_coordinates(actual_x, actual_y);

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
    drawTurtleBot(x_pos, y_pos);
    lcd_draw_coordinates(x_pos, y_pos);
}

void UART_init() {
  Serial2.begin(115200, SERIAL_8N1, 13, 14);
}

void handle_gesture(uint8_t gesture) {
    M5.Lcd.fillRect(5, 212, 200, 220, BLACK);
    M5.Lcd.setCursor(10, 215);
    M5.Lcd.printf("Gesture: %d", gesture);
    const char* mqttTopic = "s4702018/M5/ultrasonic";
    unsigned long timeStamp = millis();
    
    StaticJsonDocument<200> doc;
    doc["timestamp"] = timeStamp;
    doc["gesture"] = gesture;
    doc["left"] = 0;
    doc["right"] = 0;

    String jsonString;
    serializeJson(doc, jsonString);
    

    if (client.publish(mqttTopic, jsonString.c_str())) {
        Serial.println("Published to MQTT");
    } else {
        Serial.println("Failed to publish to MQTT");
    }
}

void handle_packet(uint8_t* data) {
  int command = data[0];
  int length = data[1];
  if (command == MODE_DEFAULT && length == 4) {
    // handle ultrasonic measurements
    uint16_t left = ((uint16_t)data[2] << 8) | (uint16_t)data[3];
    uint16_t right = ((uint16_t)data[4] << 8) | (uint16_t)data[5];

    // In normal. Hence, display 

    // Call handle function
    handle_ultrasonic_data(left, right);
  } else if (command == MODE_GESTURE) {
    // handle gesture
    uint8_t gesture = data[2];
    handle_gesture(gesture);
  } else {
    return;
  }
}

void handle_ultrasonic_data(uint16_t left, uint16_t right) {
    M5.Lcd.fillRect(5, 212, 200, 220, BLACK);
    M5.Lcd.setCursor(10, 215);
    M5.Lcd.printf("Default");
  // publish to mqtt
    const char* mqttTopic = "s4702018/M5/ultrasonic";
    unsigned long timeStamp = millis();
    
    StaticJsonDocument<200> doc;
    doc["timestamp"] = timeStamp;
    doc["gesture"] = 0;
    doc["left"] = left;
    doc["right"] = right;

    String jsonString;
    serializeJson(doc, jsonString);
    

    if (client.publish(mqttTopic, jsonString.c_str())) {
      // idk if this does anything
        Serial.println("Published to MQTT");
    } else {
        Serial.println("Failed to publish to MQTT");
    }
}

int uart_poll() {
  int start = 0;
  int length = 0;
  if (Serial2.available()) {
      start = Serial2.read();
      if (start == PACKET_START) {
        while (!Serial2.available()) {}
        data[0] = Serial2.read();
        while (!Serial2.available()) {}
        length = Serial2.read();
        data[1] = length;
        if (length != 0 && length <= MAX_DATA_LENGTH) {
          for (int i = 2; i < length + 2; i++) {
            while (!Serial2.available()) {}
              data[i] = Serial2.read();
            }
            return 1;
        } else {
          return 0;
        }
      } else {
        return 0 ;
      }
      return 0;
      
    }
    return 0;
}



void setup() {

  const char* ssid = "infrastructure";
  const char* password = "a3s7fSbjs4qS";
  const char* mqtt_server = "csse4011-iot.zones.eait.uq.edu.au";

 
  // Initialise M5 Core
    M5.begin();

    // Setup MQTT Communication
    MQTT_init(mqtt_server, ssid, password);
    
    // Initialise Serial Communication
    UART_init();

    //init LCD
    LCD_init();


    // Start infinite loop I have done this to avoid globals
    while (1) {
      //Check if client has lost connection, if so reconnect
      if (!client.connected()) {
        reConnect();
      }

      // Run Client Loop
      client.loop();


      // Check Serial Communication
      if (uart_poll() == 1) {
        handle_packet(data);
      }

      lcd_handler();
    }
}

void loop() {

}
