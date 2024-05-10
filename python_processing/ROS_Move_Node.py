'''
Copyright (c) 2015, Mark Silliman
All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

# A very basic TurtleBot script that moves TurtleBot forward indefinitely. Press CTRL + C to stop.  To run:
# On TurtleBot:
# roslaunch turtlebot_bringup minimal.launch
# On work station:
# python goforward.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import threading
import mqtt as mqtt
import util as util
import packet as packet

gesture = 0
left_distance = 0
right_distance = 0
move_cmd = Twist()
max_distance = 4
scalar = 0.1

class MoveTurtle(Node):
    def __init__(self, transmit_queue,receive_queue):
        super().__init__('move_node')
        self.receive_queue = receive_queue
        self.transmit_queue = transmit_queue
        
	    # Create a publisher which can "talk" to TurtleBot and tell it to move
        # Tip: You may need to change cmd_vel_mux/input/navi to /cmd_vel if you're not using TurtleBot2
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
     

        # Twist is a datatype for velocity
        #move_cmd = Twist()
	    # let's go forward at 0.2 m/s
     
        move_cmd.linear.x = 0.2
	    # let's turn at 0 radians/s
        move_cmd.angular.z = 0.0
        # Create a timer that will gate the node actions twice a second
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)
        
    # 
    def node_callback(self):
        data = util.get_queue_data(self.data_queue)
        if (data is not None):
            if data[0] == packet.MODE_DEFAULT:
                print(f"linear x: {util.normalize(0, 0.26, data[1]) * 0.26}. angular z: {util.normalize(0, 0.26, data[2]) * 0.26}")
                move_cmd.linear.x = util.normalize(0, 0.26, data[1]) * 0.26
                move_cmd.angular.z = util.normalize(0, 0.26, data[2]) * 0.26
                self.publisher.publish(move_cmd)
            elif data[0] == packet.MODE_GESTURE:
                print("gesture received!")
        
        # Get position and publish to mqtt
        self.transmit_queue.put([1,1])

    def snaking(self):
        # Snaking back and forth. Just some fixed value.
        move_cmd.linear.x = 1
        angular_freq = 0
        step = 0.1

        if gesture == 2:
            # Start snaking other way
            step = -0.1
            while True:
                # Basically if we start rotating too far, then make
                # it start rotating back the other way.
                if angular_freq >= 3 or angular_freq <= -3:
                    step *= -1
                elif angular_freq <= -3:
                    # One snake each way is enough.
                    break
                move_cmd.angular.z += step

        # Reset back to no gesture after doing full snake.
        gesture = 0

        # Then will just return back to init.

def ros_main (publisher):
    
    rclpy.spin(publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()
 

# MQTT thread that receives, decodes data and then sets global flag for 
# the ROS. Set gesture, left and right distance values based on that.