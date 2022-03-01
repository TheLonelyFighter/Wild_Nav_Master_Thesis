
import rclpy
from rclpy.node import Node

from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleControlMode
from px4_msgs.msg import VehicleOdometry
from px4_msgs.msg import Timesync
from px4_msgs.msg import VehicleCommand

import numpy as np






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


class SafetyCheck(Node):

    def __init__(self):
        super().__init__('safety_check')

        #FLAGS
        self.ARMED = True
        self.ALL_GOOD = False #drone will land ASAP if this is set to False
        self.SAFETY_CHECK_FAILED = False


        #Define safety area coordinates where flight is allowed
        self.distance_list = []
        self.max_altitude = 3 # meters
        self.max_distance_from_home = 2 # meters, distance from initial position (0,0,0), radius of a circle in XoY plane where drone can fly
        self.max_distance_between_waypoints = 0.5 # meters, maximum distance between current position and next waypoint

        self.current_waypoint_pos = Waypoint()
        self.home_waypoint = Waypoint(0,0,0)
        self.timestamp = 0

        timer_period = 1  #seconds, timer for performing safety checks periodically
        self.timer = self.create_timer(timer_period, self.timer_callback)
    

        #PUBLISHERS
        self.trajectory_out_pub = self.create_publisher(TrajectorySetpoint, '/fmu/trajectory_setpoint/in', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/vehicle_command/in', 10)
        

        #SUBSCRIPTIONS
        self.trajectory_in_sub = self.create_subscription(TrajectorySetpoint,'/safety_check', self.trajectory_in_callback, 10)
        self.timesync_sub = self.create_subscription(Timesync, '/fmu/timesync/out', self.timesync_callback, 10)
        self.odom_sub = self.create_subscription(VehicleOdometry,'/fmu/vehicle_odometry/out', self.odom_callback, 10)
        self.vehicle_status_sub = self.create_subscription(VehicleControlMode,'/fmu/vehicle_control_mode/out', self.vehicle_status_callback, 10)


    def timer_callback(self):
        something = 1 
        #check to see if drone is still in safety zone

    def trajectory_in_callback(self, waypoint_from_path_node):
        #perform safety check here

        #TODO
        #if safety checks fail ONCE, then next setpoints sent by path node should not be published
        next_waypoint = Waypoint(waypoint_from_path_node.x,waypoint_from_path_node.y,waypoint_from_path_node.z)

        print("Current Position ",  self.current_waypoint_pos)
        print("Next Waypoint ", next_waypoint)

        self.ALL_GOOD  = self.perform_safety_check(next_waypoint)
        if (self.ALL_GOOD == True and self.SAFETY_CHECK_FAILED == False):            
            #print("All good")
            self.trajectory_out_pub.publish(waypoint_from_path_node)
        else:
            print("Something went wrong, landing")
            print("ALL_GOOD flag: ", self.ALL_GOOD)
            print("SAFETY_CHECK_FAILED flag: ", self.SAFETY_CHECK_FAILED)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND, 0.0, 0.0) #land if safety checks not met

    
    def odom_callback(self, msg):
        self.current_waypoint_pos.x = msg.x
        self.current_waypoint_pos.y = msg.y
        self.current_waypoint_pos.z = float(msg.z) #Don't forget PX4 uses NED coordinate system
        #print(self.current_waypoint_pos)

    def timesync_callback(self, msg):
        self.timestamp = msg.timestamp

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

    def vehicle_status_callback(self, msg):
        if msg.flag_armed == True:
            self.ARMED = True 
    
    def perform_safety_check(self, next_waypoint):
        all_good = True
        distance_consecutive = self.calculate_distance(self.current_waypoint_pos, next_waypoint)
        self.distance_list.append(distance_consecutive)
        self.distance_list.sort()
        f = open("distances.txt", "w")
        f.write(str(self.distance_list))
        f.close()
        distance_home = self.calculate_distance(self.home_waypoint, next_waypoint)

        if distance_consecutive > 0.4:
            print("Distance between 2 consectutive waypoints: ", distance_consecutive)

        if (self.current_waypoint_pos.z >= self.max_altitude):
            all_good = False
            self.SAFETY_CHECK_FAILED = True
            print("Maximum altidude exceeded, drone will land. Altidude:", self.current_waypoint_pos.z)
        if (distance_consecutive >= self.max_distance_between_waypoints):
            all_good = False
            self.SAFETY_CHECK_FAILED = True
            print("Maximum distance between 2 consecutive waypoints exceeded, drone will land. Distance:", distance_consecutive)

        if (distance_home >= self.max_distance_from_home):
            all_good = False
            self.SAFETY_CHECK_FAILED = True
            print("Drone too far away from home, will land. Distance: ", distance_home)
       
        return all_good

    def calculate_distance(self, waypoint_1, waypoint_2):
        point_1 = np.array((waypoint_1.x, waypoint_1.y))
        point_2 = np.array((waypoint_2.x, waypoint_2.y))
        distance = np.linalg.norm(point_1 - point_2)
        return distance


def main(args=None):
    rclpy.init(args=args)

    safety_check = SafetyCheck()
    print("Safety check module running...")

    rclpy.spin(safety_check)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
