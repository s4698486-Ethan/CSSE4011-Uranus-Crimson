import threading
import mqtt as mqtt
import ROS_Move_Node as rmn
import ROS_Pos_Node as rpn
import time
import queue
import rclpy
import execute_launch as launcher


if __name__ == '__main__':

    print("HERR")
    

    mqtt_move = queue.Queue()
    move_mqtt = queue.Queue()

    mqtt_pos = queue.Queue()
    pos_mqtt = queue.Queue()



    rclpy.init()
    print("HELO")
    

    mqttobject = mqtt.MQTT(mqtt_move, move_mqtt, mqtt_pos, pos_mqtt)
    turtle_position = rpn.TurtlePos(pos_mqtt, mqtt_pos)
    move_turtle = rmn.MoveTurtle(move_mqtt, mqtt_move)

    t1 = threading.Thread(target=mqttobject.run)
    t1.start()

    print("HEreee")

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(move_turtle)
    executor.add_node(turtle_position)

    # Spin in a separate thread
    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    launch_thread = threading.Thread(target=launcher.execute_command())
    launch_thread.start()

    while(1):
        time.sleep(1000)


    exit(1)