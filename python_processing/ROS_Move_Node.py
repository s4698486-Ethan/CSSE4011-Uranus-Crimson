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
import time
import threading
import mqtt as mqtt
import util as util
import packet as packet
import numpy as np

gesture = 0
left_distance = 0
right_distance = 0
move_cmd = Twist()
max_distance = 4
scalar = 0.1

class MoveTurtle(Node):
    def __init__(self, move_mqtt, mqtt_move, pos_move):
        super().__init__('move_node')
        self.mqtt_move = mqtt_move
        self.move_mqtt = move_mqtt
        self.pos_move = pos_move

        self.test = 1
        self.oob_flag = 0
        
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
        timer_period = 0.00001  # seconds
        self.timer = self.create_timer(timer_period, self.node_callback)
        
    # 
    def node_callback(self):
        turn_around = util.get_queue_data(self.pos_move)
        if (turn_around is not None):
            
            self.turn_around()
            data = util.get_queue_data(self.mqtt_move)
            self.pos_move.queue.clear()
            self.mqtt_move.queue.clear()
            #with self.pos_move.mutex:
            return
        data = util.get_queue_data(self.mqtt_move)
        if (data is not None):
            if data[0] == packet.MODE_DEFAULT:
                if (data[1] < 75 and data[2] < 75):
                    print(f"linear x: {util.normalize(0, 0.26, 0, 75, data[1])}. angular z: {0.26 - util.normalize(0, 0.52, 0, 75, data[2])}")
                    move_cmd.linear.x = util.normalize(0, 0.26, 0, 75, data[1])
                    move_cmd.angular.z = 1.82 - util.normalize(0, 3.64, 0, 75, data[2])
                    self.publisher.publish(move_cmd)
            elif data[0] == packet.MODE_GESTURE:
                print("gesture received!")
                self.gesture(data[1])

        
        
        time.sleep(0.01)

    def turn_around(self):
        print("turn arround")
        move_cmd.linear.x = -0.2
        move_cmd.angular.z = 0.0
        init_t = time.time()
        new_t = time.time()
        while(new_t - init_t < 0.3):
            self.publisher.publish(move_cmd)
            new_t = time.time()

        move_cmd.linear.x = 0.0
        move_cmd.angular.z = np.pi / 2
        init_t = time.time()
        new_t = time.time()
        while(new_t - init_t < 2):
            self.publisher.publish(move_cmd)
            new_t = time.time()
        move_cmd.linear.x = 0.15
        move_cmd.angular.z = 0.0
        init_t = time.time()
        new_t = time.time()
        while(new_t - init_t < 5):
            self.publisher.publish(move_cmd)
            new_t = time.time()
        move_cmd.linear.x = 0.0
        move_cmd.angular.z = 0.0
        init_t = time.time()
        new_t = time.time()
        while(new_t - init_t < 0.8):
            self.publisher.publish(move_cmd)
            new_t = time.time()
        
        #with self.pos_move.mutex:
        #    self.pos_move.queue.clear()
        self.oob_flag = 0
        

    def gesture(self, gesture):
        # Snaking back and forth. Just some fixed value.
        
        if gesture == 1:
            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            
                
            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            
            move_cmd.linear.x = 0.0
            move_cmd._angular.z = 0.0    
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()
                
        elif gesture == 2:
            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            
                
            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.8
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 0.3):
                self.publisher.publish(move_cmd)
                new_t = time.time()

            move_cmd.linear.x = 0.26
            move_cmd.angular.z = 0.0
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            
            move_cmd.linear.x = 0.0
            move_cmd._angular.z = 0.0    
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 1):
                self.publisher.publish(move_cmd)
                new_t = time.time()
        elif gesture == 3:
            # Donuts!!!
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.82
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 5):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            move_cmd.linear.x = 0.0
            move_cmd._angular.z = 0.0    
            self.publisher.publish(move_cmd)

        elif gesture == 4:
            # Donuts 2!!!
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = -1.82
            init_t = time.time()
            new_t = time.time()
            while (new_t - init_t < 5):
                self.publisher.publish(move_cmd)
                new_t = time.time()
            move_cmd.linear.x = 0.0
            move_cmd._angular.z = 0.0    
            self.publisher.publish(move_cmd)
        elif gesture == 5:
            # STOP!!!
            init_t = time.time()
            new_t = time.time()
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            while (new_t - init_t < 5):
                self.publisher.publish(move_cmd)
                new_t = time.time()
        # Reset back to no gesture
        gesture = 0
        
        # Then will just return back to init.

def ros_move_node_run(move_mqtt, mqtt_move):

    publisher = MoveTurtle(move_mqtt, mqtt_move)
    rclpy.spin(publisher)
    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    publisher.destroy_node()
    rclpy.shutdown()
 

# MQTT thread that receives, decodes data and then sets global flag for 
# the ROS. Set gesture, left and right distance values based on that.