import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

class VelHandler(Node):
    def __init_(self):
        super().__init__('control')
        self.forwardPub = self.create_publisher(Float64, 'speed_metersec', 1)
        self.anglePub = self.create_publisher(Float64, 'wheelAngleCmd', 1)
        self.twistSub = self.create_subscription(Twist, 'cmd_vel', 1)
    
    def callback(self,msg):
        forwardSpeed, wheelAngle = Float64(), Float64()
        forwardSpeed.data = msg.linear.x
        wheelAngle.data = msg.angular.z
        self.forwardPub.publish(forwardSpeed)
        self.anglePub.publish(wheelAngle)
    
def main(args=None):
    rclpy.init(args=args)
    control = VelHandler()
    rclpy.spin(control)
    rclpy.shutdown()
if __name__ == '__main__':
    main()