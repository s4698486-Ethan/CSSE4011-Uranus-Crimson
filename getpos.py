import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class Pos_Sub(Node):
    def __init__(self):
        super().__init__('position_node')
        
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.odom_x = 0
        self.odom_y = 0
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.cb, 10)
        self.amcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.cb_2, 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.init_pose = PoseWithCovarianceStamped()
        self.init_pose.header.frame_id = 'map'
        self.init_pose.pose.pose.position.x = 0.0
        self.init_pose.pose.pose.position.y = 0.0
        self.init_pose.pose.pose.position.z = 0.0
        self.init_pose.pose.pose.orientation.z = -0.65
        self.init_pose.pose.pose.orientation.w = 0.8

        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)
    def node_callback(self):
        #print("pub")
        #elf.init_pose_pub.publish(self.init_pose)
        pass

    def cb(self, msg):
        self.timer = None
        # Extract x, y coordinates from the transform message
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        
    def cb_2(self, msg):
        position = msg.pose.pose.position
        x = position.x
        y = position.y
        print("odom coordinates - x: {}, y: {}".format(self.odom_x, self.odom_y))
        print("AMCL POSE - x: {}, y: {}".format(x, y))
def main(args=None):
    rclpy.init(args=args)
    subscriber = Pos_Sub()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()