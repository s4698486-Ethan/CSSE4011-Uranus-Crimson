import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import packet as packet
import time


class TurtlePos(Node):
    def __init__(self, pos_mqtt, mqtt_pos):
        super().__init__('position_node')
        self.pos_mqtt = pos_mqtt
        self.mqtt_pos = mqtt_pos
        
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.subscription = self.create_subscription(Odometry, '/odom', self.cb, 10)

    def cb(self, msg):
        # Extract x, y coordinates from the transform message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("TurtleBot3 coordinates - x: {}, y: {}".format(x, y))

        self.pos_mqtt.put([packet.MODE_POSITION, x, y])
        




def ros_pos_node_run(mqtt_pos, pos_mqtt):
    subscriber = TurtlePos(mqtt_pos, pos_mqtt)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
