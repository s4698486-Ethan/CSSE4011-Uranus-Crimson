import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import packet as packet
import time


class TurtlePos(Node):
    def __init__(self, pos_mqtt, mqtt_pos):
        super().__init__('position_node')
        self.pos_mqtt = pos_mqtt
        self.mqtt_pos = mqtt_pos
        self.odom_x = 0
        self.odom_y = 0
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.subscription = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_2, 10)

    def cb(self, msg):
        # Extract x, y coordinates from the transform message
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y

        
        
    def cb_2(self, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        print("odom coordinates - x: {}, y: {}".format(self.odom_x, self.odom_y))
        print("AMCL POSE - x: {}, y: {}".format(x, y))
        self.pos_mqtt.put([packet.MODE_POSITION, x, y])



def ros_pos_node_run(mqtt_pos, pos_mqtt):
    subscriber = TurtlePos(mqtt_pos, pos_mqtt)
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()
