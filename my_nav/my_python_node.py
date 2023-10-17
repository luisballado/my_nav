import rclpy
from my_nav.moverobot import *
import signal

def main(args=None):
    
    rclpy.init(args=args)

    node = MoveRobotNode()
    node.move2goal()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_robot()
        node.destroy_node()
        #rclpy.shutdown()
                
if __name__ == '__main__':
    main()
