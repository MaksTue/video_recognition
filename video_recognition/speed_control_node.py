import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32

class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('speed_control_node')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Int32, '/classification_result', self.speed_control_callback, 10
        )
        self.current_speed = 0.1 
        
    def speed_control_callback(self, msg):
        classNumber = msg.data
        
        if classNumber == 4:       
            self.current_speed = 0.7
            
        if classNumber == 1:
            self.current_speed = 0.3
            
        if classNumber == 8:
            self.current_speed = 1.2
            
        if classNumber == 14:
            self.current_speed = 0.0
            
              
        print(self.current_speed,'Changing speed')   
        self.publish_speed()
            
        
    def publish_speed(self):
        twist_msg = Twist()
        twist_msg.linear.x = self.current_speed
        twist_msg.angular.z = 0.0
        
        self.publisher.publish(twist_msg) 
        
    def start_moving(self):
        
        self.publish_speed()     

def main(args=None):
    rclpy.init(args=args)
    node = SpeedControlNode()
    node.start_moving()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
