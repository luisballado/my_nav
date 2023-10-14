import rclpy
#Import math Library
import math 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 

class MoveRobotNode(Node):

    def __init__(self):
        super().__init__('move_robot_node')

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback,10)
        self.subscription_odom = self.create_subscription(Odometry, '/odom', self.odom_callback,10)

        self.linear_velocity = 1.0
        self.angular_velocity = 0.0
        self.timer = self.create_timer(1.0, self.move_robot)
        

    def stop_robot(self):

        msg = Twist()
        msg.linear.x  = 0.0
        msg.angular.z = 0.0

        self.publisher.publish(msg)
        self.get_logger().info('Deteniendo el robot')

    def odom_callback(self, msg):
        
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        quat_x = msg.pose.pose.orientation.x
        quat_y = msg.pose.pose.orientation.y
        quat_z = msg.pose.pose.orientation.z
        quat_w = msg.pose.pose.orientation.w

        self.get_logger().info("Posición: ({}, {}, {})".format(x, y, z))
        self.get_logger().info("Orientación: ({}, {}, {})".format(quat_x, quat_y, quat_z))

        #self.msg = Odometry()
        #msg.header.stamp = self.get_clock().now().to_msg()
        #self.msg.pose.pose.position.x = x
        #self.msg.pose.pose.position.y = y
        #self.msg.pose.pose.position.z = z
        #self.msg.pose.pose.orientation.x = quat_x
        #self.msg.pose.pose.orientation.y = quat_y
        #self.msg.pose.pose.orientation.z = quat_z
        #self.msg.pose.pose.orientation.w = quat_w
        #print("HOLAAAAa")
        #print(x)
        #print(y)
        #print(z)
        
        
        
    def scan_callback(self, msg):

        ranges = msg.ranges

        obstacle_threshold = 1.0
        
        obstacle_right = any(r < obstacle_threshold for r in ranges[90:135])
        obstacle_center = any(r < obstacle_threshold for r in ranges[135:225])
        obstacle_left = any(r < obstacle_threshold for r in ranges[225:270])

        self.linear_velocity = 0.7
        self.angular_velocity = 0.0
        if obstacle_left:
            self.linear_velocity = 0.00
            self.angular_velocity = -0.2#0.2
            self.get_logger().info('Izq: %f - angl vel: %f' %(self.linear_velocity, self.angular_velocity))
        elif obstacle_right:
            self.linear_velocity = 0.00
            self.angular_velocity = 0.2#-0.2
            self.get_logger().info('Der: %f - angl vel: %f' %(self.linear_velocity, self.angular_velocity))
        elif obstacle_center:
            self.angular_velocity = 0.2
            self.linear_velocity = 0.0
            self.get_logger().info('Fren: %f - angl vel: %f' %(self.linear_velocity, self.angular_velocity))
                        
    def move_robot(self):

        msg = Twist()
        msg.linear.x  = self.linear_velocity
        msg.angular.z = self.angular_velocity
        self.publisher.publish(msg)
        self.get_logger().info('Moviendo el robot:: veloc linear: %f velc ang: %f' % (self.linear_velocity, self.angular_velocity))

def main(args=None):
    
    rclpy.init(args=args)
    node = MoveRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
