import rclpy
from my_nav.moverobot import *

def main(args=None):
    
    rclpy.init(args=args)

    node = MoveRobotNode()
        
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()
                
if __name__ == '__main__':
    main()
