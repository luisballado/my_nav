import math
import time
from math import pow, atan2, sqrt, asin, cos, sin
from math import radians, degrees
from nav_msgs.msg import Odometry
from rclpy.node import Node
import rclpy
import os
from odometria.euler_from_quaternion import *
from geometry_msgs.msg import Twist, Pose, Quaternion
from geometry_msgs.msg import Pose2D
from rclpy.executors import MultiThreadedExecutor

class MoveRobotNode(Node):
    
    def __init__(self):

        super().__init__('move_robot_node')

        self.get_logger().info("Node move_robot_node Started")

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        
        self.odom_subs = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

        self.k1 = 0.5
        self.k2 = 1

        self.pose = Pose2D()

        self.pose.x = 0.0
        self.pose.y = 0.0
        self.theta = 0.0

        self.goal_pose = Pose2D()
        self.goal_pose.x = 0.0
        self.goal_pose.y = 0.0
        
        self.start_path = False
        
        #self.odom_subs
        #ejecuta cada cierto tiempo
        self.timer = self.create_timer(0.005, self.move2goal)
                
    def do_something(self):
        os.system('clear')
        self.get_logger().info("Haciendo algo")
        time.sleep(0.5)
        
    def stop_robot(self):

        print("DETENIENDO ROBOT")

        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0

        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)

        print("ROBOT DETENIDO")

            
    #valor de a
    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y-self.pose.y),2))

    #alpha
    def alpha(self, goal_pose):

        alph = atan2(
            (goal_pose.y - self.pose.y),
            (goal_pose.x-self.pose.x)
        ) - 2*self.theta
        
        return alph

    def linear_vel(self,goal_pose):

        v = (self.k1 *
             self.euclidean_distance(goal_pose) *
             cos(self.alpha(goal_pose)))

        if v > 1:
            v = 1.0

        if v < -1:
            v = -1.0
            
        return v
    
    #_w_
    def angular_vel(self,goal_pose):

        w = self.k2 * self.alpha(goal_pose)
        + self.k1 * sin(self.alpha(goal_pose)) * cos(self.alpha(goal_pose))

        if w > 1:
            w = 1.0

        if w < -1:
            w = -1.0
        
        return w

    def odom_callback(self,msg):

        self.pose.x = round(msg.pose.pose.position.x,4)
        self.pose.y = round(msg.pose.pose.position.y,4)
        
        orientation = msg.pose.pose.orientation
        euler = euler_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w
        )
        roll,pitch,yaw = euler
        
        self.theta = round(msg.pose.pose.orientation.z,4)
        #self.theta = yaw
        

        """
        self.get_logger().info(
            "Posición: ({}, {})".format(
                self.pose.x, self.pose.y)
        )
        
        self.get_logger().info(
            "Theta: ({})".format(
                self.theta)
        )        
        """
    def move2goal(self):
        
        #mover hacia el objetivo
        goal_pose = self.goal_pose
        
        if not self.start_path:
                        
            #Datos
            point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
            
            self.goal_pose.x = float(point_x)
            self.goal_pose.y = float(point_y)

            self.start_path = True

        vel_msg = Twist()
        distance_tolerance = 0.15
        
        if self.euclidean_distance(goal_pose) >= distance_tolerance:
            
            velocidad_linear = self.linear_vel(goal_pose)
            velocidad_angular = self.angular_vel(goal_pose)

            os.system('clear')
            
            self.get_logger().info(
                "Posición: ({}, {})".format(
                    self.pose.x, self.pose.y)
            )

            self.get_logger().info(
                "Theta: ({})".format(
                    self.theta)
            )
            
            self.get_logger().info(
                "Objetivo: ({}, {})".format(
                    goal_pose.x, goal_pose.y)
            )

            self.get_logger().info("Vlineal: ({})".format(velocidad_linear))
            self.get_logger().info("VAngular: ({})".format(velocidad_angular))
            self.get_logger().info(
                "D Euclidiana: ({})".format(
                    round(self.euclidean_distance(goal_pose),3))
            )

            self.get_logger().info("Alpha: ({})".format(self.alpha(goal_pose)))
            
            vel_msg.linear.x = velocidad_linear #* cos(self.theta)
            vel_msg.linear.y = 0.0 #velocidad_linear * sin(self.theta)
            vel_msg.linear.z = 0.0

            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = velocidad_angular

            self.cmd_vel_pub.publish(vel_msg)
            
        else:

            vel_msg.linear.x = 0.0
            vel_msg.angular.z = 0.0

            self.cmd_vel_pub.publish(vel_msg)
            
            self.start_path = False
            answer = input("Quieres ingresar otro punto? y/n:   ")
            
            if answer == "y":
                pass
            else:
                rclpy.shutdown()
                exit(-1)
