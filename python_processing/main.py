import threading
import mqtt as mqtt
import ROS_Move_Node as rmn
import time
import queue
import rclpy


if __name__ == '__main__':
    print("Here")
    rclpy.init(args=None)

    mqtt_ros = queue.Queue(maxsize=10)
    ros_mqtt = queue.Queue(maxsize=10)
    mqttobject = mqtt.MQTT(mqtt_ros, ros_mqtt)
    publisher = rmn.MoveTurtle(ros_mqtt, mqtt_ros)
    
    t1 = threading.Thread(target=mqttobject.run)
    t1.daemon = True
    t1.start()

    t2 = threading.Thread(target=rmn.ros_main, args=publisher)
    t2.daemon = True
    t2.start()
    # mqttobject.run()

    while(1):
        time.sleep(10)

    exit(1)