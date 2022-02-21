# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
import time
import transforms3d 
import numpy as np

from px4_msgs.msg import Timesync
from px4_msgs.msg import OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand
from px4_msgs.msg import VehicleLocalPosition
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import VehicleControlMode




class Waypoint():
    
    position_error = 0.1 # maximum error in meters between the drone and the waypoint 

    def __init__(self, x = 0.0, y = 0.0, z = 0.0):
        self.x = x
        self.y = y
        self.z = z

    def __str__(self):
        return "Waypoint x,y,z: %f, %f, %f" % (self.x, self.y,self.z)

    #define method to test if waypoint has been reached (if the drone is close enough to the waypoint)
    def __eq__(self, other):
        if abs(self.x - other.x) < self.position_error and abs(self.y - other.y) < self.position_error and abs(self.z - other.z) < self.position_error:
            return True
        else:
            return False

class Odometry():

    def __init__(self, x = 0.0, y = 0.0, z = 0.0, pitch = 0.0, roll = 0.0, yaw = 0.0):  #change default values here
        
        #Location Coordinates
        self.x = x #meters
        self.y = y
        self.z = z

        #Rotation Parameters
        self.pitch = pitch #angle in radians # [-PI:PI]
        self.roll = roll # [-PI:PI]
        self.yaw = yaw  # [-PI:PI]

        #Speed Parameters
        self.vx = 0
        self.vy = 0
        self.vz = 0

    def __str__(self):
        return "Position x,y,z: %f, %f, %f" % (self.x, self.y,self.z)


class FlyDrone(Node):

    def __init__(self):
        super().__init__('FlyDrone')

        #Status flags for control lopp[]
        self.ARMED = False #set to true by vehicle_status_callback() if the drone was actually armed
        self.TAKE_OFF = False #signals when the drone took off
        self.MISSION_COMPLETED = False 
        self.START_MISSION = False
        self.LAND = False #signals if drone has landed
    

        self.home_waypoint = Waypoint(0,0,10) # set take off altitude here
        self.current_pos_waypoint = Waypoint(0,0,0) #current position, starts in (0,0,0)
        #define trajectory
        self.waypoints_list = [Waypoint(-5,-5,2.5),Waypoint(1,1,2.5),Waypoint(1,3,2.5),Waypoint(1,3,7.5)]
        self.waypoint_counter = 0

        self.odom_pos = Odometry() #current position, starts in (0,0,0), contains speed and yaw also
        self.goal_pos = Odometry() #next position to reach 
        self.timestamp = 0
        self.counter = 0

        #PUBLISHERS
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', 10)
        self.offboard_control_pub = self.create_publisher(OffboardControlMode, 'fmu/offboard_control_mode/in', 10)
        #self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.safety_check_pub = self.create_publisher(TrajectorySetpoint, '/safety_check', 10)

        #SUBSCRIPTIONS
        self.timesync_sub = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, 10)
        self.odom_sub = self.create_subscription(VehicleOdometry,'/fmu/vehicle_odometry/out', self.odom_callback, 10)
        self.vehicle_status_sub = self.create_subscription(VehicleControlMode,'/fmu/vehicle_control_mode/out', self.vehicle_status_callback, 10)



        timer_period = 0.2  # seconds, #minimum rate: > 2Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
    
    def timer_callback(self):
        self.control_loop() #takes off and controls the position of the drone
        self.publish_offboard_control() #mandatory to publish to keep dron in offboard control   
        
        self.compute_path(Waypoint(0,0,0), Waypoint(2,2,2))


    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

    def vehicle_status_callback(self, msg):
        if msg.flag_armed == True:
            self.ARMED = True 

    def odom_callback(self, msg):
        self.current_pos_waypoint.x = msg.x
        self.current_pos_waypoint.y = msg.y
        self.current_pos_waypoint.z = float(-msg.z)


        self.odom_pos.x = msg.x
        self.odom_pos.y = msg.y
        self.odom_pos.z = float(-msg.z)

        rotation = transforms3d.euler.quat2euler(msg.q) #convert to Euler coordinates
        self.odom_pos.pitch = rotation[0] 
        self.odom_pos.roll = rotation[1]
        self.odom_pos.yaw = rotation[2]
       
        self.odom_pos.vx = msg.vx       
        self.odom_pos.vy = msg.vy
        self.odom_pos.vz = msg.vz


     


    def publish_vehicle_command(self, command, param1, param2 = 0.0, param3 = 0.0):
        #publish specific commands, eg land, arm

        msg = VehicleCommand()
        msg.timestamp = self.timestamp

        msg.param1 = param1
        msg.param2 = param2
        msg.param3 = param3
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def publish_offboard_control(self):
        msg = OffboardControlMode()
        msg.timestamp = self.timestamp
        msg.position = True
        msg.velocity = False 
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        self.offboard_control_pub.publish(msg)

    def publish_trajectory_setpoint(self, x = 0, y = 0, z = 0, yaw = 0.0):
        #unused values should be set to NaN
        msg = TrajectorySetpoint()
        msg.timestamp = self.timestamp
        msg.x = float(x)
        msg.y = float(y)
        msg.z = float(-z) #z axis points down in PX4 - NED coordinates     
        msg.yaw = float("nan"); # [-PI:PI] #not set for now
        msg.vx = float("nan")
        msg.vy = float("nan")
        msg.vz = float("nan")
        msg.acceleration[0] = float("nan")
        msg.acceleration[1] = float("nan")
        msg.acceleration[2] = float("nan")
        msg.jerk[0] = float("nan")
        msg.jerk[1]  = float("nan")
        msg.jerk[2]  = float("nan")
        msg.thrust[0] = float("nan")
        msg.thrust[1] = float("nan")
        msg.thrust[2] = float("nan")
        #self.trajectory_setpoint_pub.publish(msg) #do not publish directly to setpoint topic
        self.safety_check_pub.publish(msg) #publish to safety node to check if the drone can move without killing anyone (hopefully)

    #Functions that returns a waypoint close enough to the current drone position
    #so that flight is smooth
    #Change default speed factor for slower/faster movement
    def compute_path(self, curr_waypoint, goal_waypoint, speed = 0.5):
        alpha = speed #constant that controls speed
        curr_pos = np.array([curr_waypoint.x, curr_waypoint.y, curr_waypoint.z])
        goal_pos = np.array([goal_waypoint.x, goal_waypoint.y, goal_waypoint.z])
        next_pos = curr_pos + alpha * (goal_pos - curr_pos) / np.linalg.norm(goal_pos - curr_pos)
        waypoint = Waypoint(next_pos[0], next_pos[1], next_pos[2])
        return waypoint

    

    def control_loop(self):

        small_waypoint = Waypoint(0,0,0)
        if self.ARMED == False and self.MISSION_COMPLETED == False:          
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0) #switch to offboard control
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0) #arm drone            
            print("ARMED cmd sent")

        if self.TAKE_OFF == False and self.ARMED == True and self.current_pos_waypoint != self.home_waypoint: #make sure you reach the take off altitude before moving the drone elsewhere
            small_waypoint = self.compute_path(self.current_pos_waypoint, self.home_waypoint, 1) #for take off, set speed factor to a higher value                              
            print("Taking off")

        if self.TAKE_OFF == False and self.current_pos_waypoint == self.home_waypoint:
            self.START_MISSION = True            
            self.TAKE_OFF = True
            print("Starting mission")

        if self.START_MISSION == True:
            if self.waypoints_list: #as long as there are waypoints in the list, navigate to the next one 
                self.next_waypoint = self.waypoints_list[0]
                self.current_pos_waypoint = Waypoint(self.odom_pos.x,self.odom_pos.y,self.odom_pos.z)
                small_waypoint = self.compute_path(self.current_pos_waypoint, self.next_waypoint)
                print("Heading to ", small_waypoint, "number ", self.waypoint_counter)

                if self.current_pos_waypoint == self.next_waypoint:
                    print("Waypoint " + str(self.waypoint_counter) + " reached")
                    self.waypoints_list.pop(0) #remove waypoint when reached
                    self.waypoint_counter += 1

            else:
                self.START_MISSION = False
                self.MISSION_COMPLETED = True #when all the waypoints were reached, mission is completed            
                print("Reached final destination, heading home")
            

        if self.MISSION_COMPLETED == True and self.LAND == False: # return to starting position   
            small_waypoint = self.compute_path(self.current_pos_waypoint, self.home_waypoint) 
            print("Heading HOME ", self.home_waypoint)  
            if self.current_pos_waypoint == self.home_waypoint:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0) #land at the end of mission
                self.LAND = True
                print("LAND cmd sent")
        
        if self.LAND == True:
            print("Mission completed, chillin'")

        print("Current position: ", self.current_pos_waypoint)


        self.publish_trajectory_setpoint(small_waypoint.x, small_waypoint.y, small_waypoint.z)

def main(args=None):
    rclpy.init(args=args)

    fly_drone = FlyDrone()

    rclpy.spin(fly_drone)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fly_drone.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()