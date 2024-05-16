# Import the ROS client library for Python 
import rclpy 
 
# Enables the use of rclpy's Node class
from rclpy.node import Node 
 
# Base class to handle exceptions
from tf2_ros import TransformException 
 
# Stores known frames and offers frame graph requests
from tf2_ros.buffer import Buffer
 
# Easy way to request and receive coordinate frame transform information
from tf2_ros.transform_listener import TransformListener 
 
# Handle float64 arrays
from std_msgs.msg import Float64MultiArray 
 
# Math library
import math 

import packet as packet

import time
 
 
class FrameListener(Node):
  """
  Subclass of the Node class.
  The class listens to coordinate transformations and 
  publishes the 2D pose at a specific time interval.
  """
  def __init__(self, pos_mqtt):
    """
    Class constructor to set up the node
    """
    self.pos_mqtt = pos_mqtt
    
    # Initiate the Node class's constructor and give it a name
    super().__init__('map_base_link_frame_listener')
 
    # Declare and acquire `target_frame` parameter
    self.declare_parameter('target_frame', 'base_link')
    self.target_frame = self.get_parameter(
      'target_frame').get_parameter_value().string_value
 
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)
      
    # Create publisher(s)  
      
    # This node publishes the 2d pose.
    # Maximum queue size of 1. 
    #self.publisher_2d_pose = self.create_publisher(
    #  Float64MultiArray, 
    #  '/map_to_base_link_pose2d', 
    #  1)
 
    # Call on_timer function on a set interval
    timer_period = 0.1
    self.timer = self.create_timer(timer_period, self.on_timer)
     
    # Current position and orientation of the target frame with respect to the 
    # reference frame. x and y are in meters, and yaw is in radians.
    self.current_x = 0.0
    self.current_y = 0.0 
    self.current_yaw = 0.0
     
  def on_timer(self):
    """
    Callback function.
    This function gets called at the specific time interval.
    """
    # Store frame names in variables that will be used to
    # compute transformations
    from_frame_rel = self.target_frame
    to_frame_rel = 'map'
   
    trans = None
     
    try:
      now = rclpy.time.Time()
      trans = self.tf_buffer.lookup_transform(
                  to_frame_rel,
                  from_frame_rel,
                  now)
    except TransformException as ex:
      self.get_logger().info(
        f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
      time.sleep(0.5)
      return
       
    # Publish the 2D pose
    self.current_x = trans.transform.translation.x
    self.current_y = trans.transform.translation.y    
    roll, pitch, yaw = self.euler_from_quaternion(
      trans.transform.rotation.x,
      trans.transform.rotation.y,
      trans.transform.rotation.z,
      trans.transform.rotation.w)      
    self.current_yaw = yaw    
    msg = Float64MultiArray()
    msg.data = [self.current_x, self.current_y, self.current_yaw]   
    print (msg.data)
    self.pos_mqtt.put([packet.MODE_POSITION, -self.current_y, self.current_x])
    #self.publisher_2d_pose.publish(msg) 
   
  def euler_from_quaternion(self, x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
      
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
     
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
     
    return roll_x, pitch_y, yaw_z # in radians
 
def main(args=None):
  
  # Initialize the rclpy library
  rclpy.init(args=args)
  
  # Create the node
  frame_listener_node = FrameListener()
  
  # Spin the node so the callback function is called.
  # Publish any pending messages to the topics.
  try:
    rclpy.spin(frame_listener_node)
  except KeyboardInterrupt:
    pass
  
  # Shutdown the ROS client library for Python
  rclpy.shutdown()
  
if __name__ == '__main__':
  main()