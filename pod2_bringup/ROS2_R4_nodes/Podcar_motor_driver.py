#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from std_msgs.msg import String

Fwd_Speed_limit = 0.0
Bkd_speed_limit = 0.0
'''String to send command to R4'''
R4Command = String()
oldR4Command = String()
R4_DHB_Channel = 1
OSMC_range = 400 #512 #256 #128                                                         # To test the OSMC at lower speed. Can be set to high as well like 4095
OSMC_scale_factor = 2
class Podcar_Motor_Driver(Node):

    def __init__(self):
        super().__init__('PodCar_Motor_Driver')
        self.linear_velocity = 0.0
        self.pub = self.create_publisher(String,'R4_Command',10)
        self.subscriber = self.create_subscription(AckermannDrive, 'ackermann_cmd', self.speed_callback, 10)
        self.old_value = None
        self.timer = self.create_timer(0.1, self.timed_Publisher)    
        print("init")
    
    def speed_callback(self, msg):

        remapped_linear_x = msg.speed*OSMC_scale_factor
        remapped_linear_x = int(abs(remapped_linear_x*OSMC_range))
        
        if remapped_linear_x < 1:
         remapped_linear_x = 1
              
        if msg.speed >= 0.0:
         direction = 0       
        else:
         direction = 1      
          
        R4Command.data = "O:"+str(remapped_linear_x)+','+str(direction)+','+str(R4_DHB_Channel) 
        
    def timed_Publisher (self):
        if R4Command.data != oldR4Command.data:
          self.pub.publish(R4Command)
          print(R4Command)
        oldR4Command.data = R4Command.data
        
         
def main(args = None):

     rclpy.init(args=args)
     PodCar_Motor_Driver = Podcar_Motor_Driver()
     rclpy.spin(PodCar_Motor_Driver)
     PodCar_Motor_Driver.destroy_node()

     rclpy.shutdown()

if __name__ == '__main__':
     main()    

