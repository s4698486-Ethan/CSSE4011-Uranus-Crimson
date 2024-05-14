import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np

class Pos_Sub(Node):
    def __init__(self):
        super().__init__('position_node')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.previous_scan = None

    def lidar_callback(self, msg):
        # Convert scan data to numpy array for easier manipulation
        scan_data = np.array(msg.ranges)

        # Filter out invalid values (e.g., infinity or nan)
        scan_data = scan_data[np.isfinite(scan_data)]

        if self.previous_scan is not None:
            if len(scan_data) > 0:
                # Calculate the difference between the current scan and the previous scan
                # For simplicity, let's use the mean difference as an estimate of the robot's motion
                motion_estimate = np.mean(self.previous_scan - scan_data)
                print("Motion estimate:", motion_estimate)
            else:
                print("No valid scan data.")
            
            # Update the previous scan for the next iteration
            self.previous_scan = scan_data
        else:
            # If it's the first scan, just store it for comparison in the next iteration
            self.previous_scan = scan_data

def main(args=None):
    rclpy.init(args=args)
    subscriber = Pos_Sub()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
