def mqtt_rx_thread(self):
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

        while True:
            self.mqtt_client.loop()
            # Start MQTT loop
            
            time.sleep(0.01)
     
    def on_connect(self, client, userdata, flags, reason_code, properties):
        if reason_code.is_failure:
            print(f"Failed to connect: {reason_code}. loop_forever() will retry connection")
        else:
            # we should always subscribe from on_connect callback to be sure
            # our subscribed is persisted across reconnections.
            print("Connected!")
            client.subscribe("s4702018_Ultrasonic_Measurements")

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
            
            # Extract distance
            distance = data_dict.get('distance')

            self.PointUltrasonic.config(text=f"Distance: {distance/100:.2f}")

            if distance is not None:
                # Put data into queue
                self.ultrasound_queue.put([2, distance/100])
                # print(distance/100)
                try:
                    self.page.delete("ultra")
                except:
                    print("Oval not drawn yet")
                    return

                width = 5
                centre_x = 2 * 100
                centre_y = 400 - (distance/100 * 100)

                self.page.create_oval(centre_x - width,
                                    centre_y - width,
                                    centre_x + width,
                                    centre_y + width,
                                    fill="red",
                                tags="ultra")

            else:
                print("Distance not found in the JSON payload.")
        except json.decoder.JSONDecodeError as e:
            print("Error decoding JSON:", e)
        except Exception as e:
            print("An error occurred:", e)

        # Remove the message payload from userdata after processing
        userdata.remove(message.payload)