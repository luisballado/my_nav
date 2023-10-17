import math
import time

from my_nav.euler_from_quaternion import *

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry 
from rclpy.node import Node
import rclpy
import os

from math import pow, atan2, sqrt
from math import radians, degrees

from geometry_msgs.msg import Pose2D
import time
class MoveRobotNode(Node):
    
    def __init__(self):
        super().__init__('move_robot_node')
        
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)

        self.scan_subs = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)

        #suscribirse al topico de odometria
        self.odom_subs = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)

        #iniciar moviendo del robot
        #al crear el robot
        self.linear_velocity = 0.2
        self.angular_velocity = 0.0
        
        self.pose = Pose2D()
               
        #ejecutar move_robot cada 1seg
        #self.timer = self.create_timer(1.0, self.move_robot)
        self.timer = self.create_timer(1.0, self.odom_callback)
        
    def stop_robot(self):

        print("DETENIENDO ROBOT")
                
        msg = Twist()
        msg.linear.x  = 0.0
        msg.angular.z = 0.0
            
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)

        print("ROBOT DETENIDO")
    
    def odom_callback(self, msg):
        #cuando el suscriptor recibe un mensaje se actualiza la posicion

        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        euler = euler_from_quaternion(
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w)
            
        roll,pitch,yaw = euler
            
        self.pose.x = round(position.x,4)
        self.pose.y = round(position.y,4)
        self.pose.theta = yaw

        x = self.pose.x
        y = self.pose.y
        theta = yaw
        #x = msg.pose.pose.position.x
        #y = msg.pose.pose.position.y
        #z = msg.pose.pose.position.z
        
        #quat_x = msg.pose.pose.orientation.x
        #quat_y = msg.pose.pose.orientation.y
        #quat_z = msg.pose.pose.orientation.z
        #quat_w = msg.pose.pose.orientation.w
                
        self.get_logger().info(
            "Posición: ({}, {})".format(
                x, y)
        )
        
        self.get_logger().info(
            "Theta: ({})".format(
                theta)
        )

    def euclidean_distance(self, goal_pose):
        #Distancia euclidiana entre la posicion actual y la meta
        return sqrt(pow((goal_pose.x - self.pose.x),2)+pow((goal_pose.y - self.pose.y),2))

    def steering_angle(self, goal_pose):
        #Direccion del angulo
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def linear_vel(self,goal_pose,constant=1.5):
        return constant * self.euclidean_distance(goal_pose)

    def angular_vel(self, goal_pose, constant=15):
        goal_angle = self.steering_angle(goal_pose)
        current_angle = math.fmod(self.pose.theta,2*math.pi)
        return constant * (math.fmod((goal_angle - current_angle + math.pi),(2 * math.pi)) - math.pi)

    def move2goal(self):
        #mover hacia el objetivo
        goal_pose = Pose2D()

        #Entrada de datos
        point_x,point_y = input("Ingrese las coordenadas x,y:: ").split(",")
        goal_pose.x = float(point_x)
        goal_pose.y = float(point_y)

        distance_tolerance = 0.15

        vel_msg = Twist()

        while rclpy.ok() and self.euclidean_distance(goal_pose) >= distance_tolerance:
            
            velocidad_linear = self.linear_vel(goal_pose)
            velocidad_angular = self.angular_vel(goal_pose)

            print("v_lineal: ",velocidad_linear)
            print("v_angular: ",velocidad_angular)                 
            print("pose actual x: ",self.pose.x)
            print("pose actual y: ",self.pose.y)
            print("pose objetivo x: ",goal_pose.x)
            print("pose objetivo y: ",goal_pose.y)
            print("angulo actual:",self.pose.theta)
            print("angulo objetivo:",self.steering_angle(goal_pose))
            print("distancia euclidiana:",self.euclidean_distance(goal_pose))
            print("--------------------------------------------------------")
                        
	    # Linear velocity in the x-axis.
            vel_msg.linear.x = 42.42640687119285 #velocidad_linear
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            
	    # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 11.780972450961723 # velocidad_angular
            
	    # Publishing our vel_msg
            self.cmd_vel_pub.publish(vel_msg)

            time.sleep(1)
            
        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0.0
        vel_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(vel_msg)
        
        answer = input("Quieres ingresar otro punto? y/n:  ")

        if answer == "y":
            self.move2goal()
        else:
            rclpy.shutdown()
            sys.exit()

                    
    def scan_callback(self, msg):

        os.system('clear')
        
        ranges = msg.ranges
        
        obstacle_threshold = 1.0
                
        obstacle_right = any(r < obstacle_threshold for r in ranges[90:135])
        obstacle_center = any(r < obstacle_threshold for r in ranges[135:225])
        obstacle_left = any(r < obstacle_threshold for r in ranges[225:270])
        
        self.linear_velocity = 0.2
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
        
        #publicar a velocidades
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info('Moviendo el robot:: veloc linear: %f velc ang: %f' % (self.linear_velocity, self.angular_velocity))
    
