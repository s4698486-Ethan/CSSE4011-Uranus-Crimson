import threading
import mqtt as mqtt
import ROS_Move_Node as rmn
import time
import queue
import rclpy
import uart as uart


if __name__ == '__main__':
    

    mqtt_ros = queue.Queue(maxsize=10)
    ros_mqtt = queue.Queue(maxsize=10)

    # mqttobject = mqtt.MQTT(mqtt_ros, ros_mqtt)
    
    
    # t1 = threading.Thread(target=mqttobject.run)
    # t1.daemon = True
    # t1.start()

    t2 = threading.Thread(target=rmn.ros_main, args=[ros_mqtt, mqtt_ros])
    t2.daemon = True
    t2.start()
    # mqttobject.run()

    uart_object = uart.Uart(mqtt_ros, ros_mqtt)

    t3 = threading.Thread(target=uart_object.run)
    t3.daemon = True
    t3.start()


    while(1):
        time.sleep(100000000)

    exit(1)