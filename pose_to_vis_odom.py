import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleVisualOdometry
from px4_msgs.msg import VehicleMocapOdometry
from px4_msgs.msg import Timesync



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_client_node/dashgo/pose',
            self.optitrack_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.current_timestamp = 0
        self.last_timestamp = 0

        self.timesync_sub = self.create_subscription(Timesync, 'Timesync_PubSubTopic', self.timesync_callback, 10)
        self.visual_odom_pub = self.create_publisher(VehicleVisualOdometry, '/VehicleVisualOdometry_PubSubTopic', 10)
        #self.mocap_odom_pub = self.create_publisher(VehicleMocapOdometry, '/VehicleMocapOdometry_PubSubTopic', 10)



    def timesync_callback(self, msg):
        self.last_timestamp = self.current_timestamp
        self.current_timestamp = msg.timestamp

    def optitrack_callback(self, msg):
        #print ("Publishing odom messages")

        #seconds = int (self.timestamp)
        #nanoseconds = int (self.timestamp)

        visual_odom_msg = VehicleVisualOdometry()
        
        visual_odom_msg.x = msg.pose.position.x
        visual_odom_msg.y = msg.pose.position.y
        visual_odom_msg.z = msg.pose.position.z
        visual_odom_msg.timestamp = self.current_timestamp #seconds * 10**6 + int (nanoseconds / 1000)
        visual_odom_msg.timestamp_sample = self.last_timestamp
        visual_odom_msg.q[0] = msg.pose.orientation.x
        visual_odom_msg.q[1] = msg.pose.orientation.y
        visual_odom_msg.q[2] = msg.pose.orientation.z
        visual_odom_msg.q[3] = msg.pose.orientation.w

        self.visual_odom_pub.publish(visual_odom_msg)
        print(visual_odom_msg)
        
        
        # print(self.timestamp)
        # mocap_odom_msg = VehicleMocapOdometry()
        # mocap_odom_msg.x = msg.pose.position.x
        # mocap_odom_msg.y = msg.pose.position.y
        # mocap_odom_msg.z = msg.pose.position.z
        # mocap_odom_msg.timestamp = seconds * 10**6 + int (nanoseconds / 1000)
        # mocap_odom_msg.q[0] = msg.pose.orientation.x
        # mocap_odom_msg.q[1] = msg.pose.orientation.y
        # mocap_odom_msg.q[2] = msg.pose.orientation.z
        # mocap_odom_msg.q[3] = msg.pose.orientation.w

        # self.mocap_odom_pub.publish(mocap_odom_msg)


        
    


    

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()