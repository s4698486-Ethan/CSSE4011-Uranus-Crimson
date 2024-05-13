import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

class Pos_Sub(Node):
    def __init__(self):
        super().__init__('position_node')
        
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.subscription = self.create_subscription(Odometry, '/odom', self.cb, 10)
    def cb(self, msg):
        # Extract x, y coordinates from the transform message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        print("TurtleBot3 coordinates - x: {}, y: {}".format(x, y))

def main(args=None):
    rclpy.init(args=args)
    subscriber = Pos_Sub()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()