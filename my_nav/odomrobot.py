import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf2_ros import TransformBroadcaster
import math

class OdometryNode(Node):
    def __init__(self):
        super().__init__('move_robot_node')
        self.odom_publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )
        self.twist_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # Replace with the actual topic name
            self.twist_callback,
            10
        )
        self.odom_broadcaster = TransformBroadcaster(self)

        # Initialize odometry variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()

    def twist_callback(self, msg):
        # Calculate time difference
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9

        # Calculate linear and angular velocity
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        # Calculate odometry values
        delta_x = linear_velocity * math.cos(self.theta) * dt
        delta_y = linear_velocity * math.sin(self.theta) * dt
        delta_theta = angular_velocity * dt

        self.x += delta_x
        self.y += delta_y
        self.theta += delta_theta
                
        # Create an Odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.twist.twist = msg
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.z = math.sin(self.theta / 2.0)
        odom_msg.pose.pose.orientation.w = math.cos(self.theta / 2.0)
        
        # Publish the odometry message
        self.odom_publisher.publish(odom_msg)

        # Broadcast the transform
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.0),
            (0.0, 0.0, math.sin(self.theta / 2.0), math.cos(self.theta / 2.0)),
            current_time.to_msg(),
            'base_link',
            'odom'
        )

        self.last_time = current_time
