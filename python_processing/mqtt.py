import paho.mqtt.client as mqtt
import json
import time
import packet as packet
import util as util

class MQTT():
    def __init__(self, transmit_queue, receive_queue) -> None:
        self.transmit_queue = transmit_queue
        self.receive_queue = receive_queue
        
    def run(self):
        """
        MQTT RX Thread
        """
        self.mqtt_client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client.on_subscribe = self.on_subscribe
        self.mqtt_client.on_unsubscribe = self.on_unsubscribe

        self.mqtt_client.username_pw_set(username="s4702018", password="a3s7fSbjs4qS")


        self.mqtt_client.user_data_set([])
    # Connect to MQTT broker
        self.mqtt_client.connect("csse4011-iot.zones.eait.uq.edu.au")
        print("running")



        while True:
            self.mqtt_client.loop()
            # Start MQTT loop
            data = util.get_queue_data(self.receive_queue)
            if data is not None:
                x = data[0]
                y = data[1]
                position_data = {
                    "command" : packet.MODE_DEFAULT,
                    "x" : x,
                    "y" : y,
                }

                json_data = json.dumps(position_data)
                json_data += '\n'
                print(f"Publishing: {json_data  }")

                self.mqtt_client.publish(json_data)
                
                time.sleep(0.1)

    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            print("Connected!")
            client.subscribe("URANUS-CRIMSON_M5CoreUpload")

    def on_subscribe(self, client, userdata, mid, reason_code_list, properties):
    # Since we subscribed only for a single channel, reason_code_list contains
    # a single entry
        if reason_code_list[0].is_failure:
            print(f"Broker rejected you subscription: {reason_code_list[0]}")
        else:
            print(f"Broker granted the following QoS: {reason_code_list[0].value}")

    def on_unsubscribe(self, client, userdata, mid, reason_code_list, properties):
        # Be careful, the reason_code_list is only present in MQTTv5.
        # In MQTTv3 it will always be empty
        if len(reason_code_list) == 0 or not reason_code_list[0].is_failure:
            print("unsubscribe succeeded (if SUBACK is received in MQTTv3 it success)")
        else:
            print(f"Broker replied with failure: {reason_code_list[0]}")
        client.disconnect()
        
    def on_message(self, client, userdata, message):
        userdata.append(message.payload)

        try:
            # Check if userdata is empty
            if not self.mqtt_client.user_data_get():
                print("Empty payload received.")
                return

            # Get the latest appended message payload
            latest_payload = self.mqtt_client.user_data_get()[-1]
            print(latest_payload)

            # Parse JSON payload
            json_data = latest_payload.decode('utf-8')
            data_dict = json.loads(json_data)

            # Not a gesture
            if data_dict.get('gesture') == 0:
                # Extract distance
                left_reading = data_dict.get('left')
                right_reading = data_dict.get('right')

                if left_reading is not None and right_reading is not None:
                    self.transmit_queue.put([packet.MODE_DEFAULT, left_reading, right_reading])
            else:
                self.transmit_queue.put([packet.MODE_GESTURE, 0, 0])


        except json.decoder.JSONDecodeError as e:
            print("Error decoding JSON:", e)
        except Exception as e:
            print("An error occurred:", e)

        # Remove the message payload from userdata after processing
        userdata.remove(message.payload)
    
    def on_publish(self, client, userdata,result):             #create function for callback
        print("data published \n")
        pass