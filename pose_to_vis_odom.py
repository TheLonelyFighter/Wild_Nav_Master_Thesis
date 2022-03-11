import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleVisualOdometry


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/vrpn_client_node/dashgo/pose',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.visual_odom_pub = self.create_publisher(VehicleVisualOdometry, '/VehicleVisualOdometry_PubSubTopic', 10)

    def listener_callback(self, msg):
        print(msg)

        seconds = int (msg.header.stamp.sec)
        nanoseconds = int (msg.header.stamp.nanosec)

        visual_odom_msg = VehicleVisualOdometry()
        visual_odom_msg.x = msg.pose.position.x
        visual_odom_msg.y = msg.pose.position.y
        visual_odom_msg.z = msg.pose.position.z
        visual_odom_msg.timestamp = seconds * 10**6 + int (nanoseconds / 1000)
        visual_odom_msg.q[0] = msg.pose.orientation.x
        visual_odom_msg.q[1] = msg.pose.orientation.y
        visual_odom_msg.q[2] = msg.pose.orientation.z
        visual_odom_msg.q[3] = msg.pose.orientation.w

        self.visual_odom_pub.publish(visual_odom_msg)


        

    

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