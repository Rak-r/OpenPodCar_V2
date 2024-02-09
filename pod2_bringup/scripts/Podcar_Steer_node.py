#!/usr/bin/env python3

'''This node converts the standard ROS2 twist message (Linear x (in m/s) and Angular z(rad/sec) into ackermann messages which includes the fields

float32 steering angle --> This is the desired steering angle for the virtual front centre wheel (like bicycle)
float32 steering angle velocity --> Desired rate of change of the steering angle (rad/sec)
float32 speed --> Desired forward speed (m/s))'''
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import numpy as np
import math

wheel_track = 0.70                                                              # seperation between front left and front right wheels (in meters)
wheelbase = 1.6                                                                 # the distance between the centre point of front wheels and centre point of rear wheels (in meters)
steering_limit = 0.436                                                          # Limit the steer angle turning for left and right (radians) to represent 25 degrees as measured for our vehicle
min_turning_radius = wheelbase/math.sin(steering_limit)                         # Calculate the minimum turning radius (returns result in meters)
AINSTEER_centre_voltage = 1.25                                                  # steering caliberation based on placemnet of Linear actuator
desired_AISTEER_feedback_voltage = AINSTEER_centre_voltage                      # desired value for the steering motor in volts, 1.25v represents nearly centre position
AINSTEER_right_limit = 2.5                                                      # Max AINSTEER voltage limit when turning full right (in volts)
AINSTEER_left_limit = 0.1                                                       # Max AINSTEER voltage limit when turning full left (in volts)
Right_AINSTEER_max = AINSTEER_right_limit - AINSTEER_centre_voltage
Max_theta = 0.436                                                               # Maximum angle the vehicle can achieve (in radians)
scaling_factor =  Right_AINSTEER_max/Max_theta                                  # no. of messges to receive from DHB to get the actual position of the linear actuator
max_messages = 20 
                                                          
ain_steer = np.zeros(max_messages, dtype = np.float32)
                                                                        # a zero-initialized array to store the messgaes from R4_AINSTEER topic
index = 0                                                                       # array index to perform array operations
avg_AINSTEER = 0
old_R4_Command = String()                                                       # Message object  of string type for R4 
R4_Command = String()                                                           # Message object  of string type for R4 
R4_DHB_Channel = 1
debug = False
deadBand = 0.15                                               #Tune the dead band at the desired steering angle depending on noise from feedback, how much does the average vary by when disired position reached
direction = 0
class Podcar(Node):
    def __init__(self):
        super().__init__('PodCar_Steer')

        self.Ackermann_pub = self.create_publisher(AckermannDrive, 'ackermann_cmd', 10)
        self.Steer_pub = self.create_publisher(String, 'R4_Command', 10)
        self.twist_sub = self.create_subscription(Twist, 'cmd_vel', self.ackermann_callback, 10)
        self.AINSTEER_sub = self.create_subscription(String, 'R4_AINSTEER', self.AINSTEER_callback, 10)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
        self.desired_angle = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)

    def virtual_steering_angle(self, wheelbase, lin_v, ang_v):                   # Function to calculate the steering angle for the front virtual wheel in centre like a bicycle Kinematic model
        if lin_v == 0.0 and ang_v != 0.0:                                        # Deals with the case where angular velocity is not zero and linear velocity is zero
            # desired_turning_radius = (ang_v/(abs(ang_v))*min_turning_radius)   # this is how gazebo handles this case
            desired_turning_radius = 4/ang_v
            
        elif abs(ang_v) < 0.001:                                                 # Deals with the case where any one of the velocity is zero to handle division error
            desired_turning_radius = 1000000000.0
        else:
            if abs(lin_v)< 0.01: 
                lin_v=0.1
                                                                                 # Deals with the case where vehicle is moving and turning
            desired_turning_radius = abs(lin_v)/ang_v
            # print('Desired_turning_Radius:', desired_turning_radius)
        
        # if abs(desired_turning_radius) >= min_turning_radius:
        #     desired_turning_radius_limit = min_turning_radius
        #     if desired_turning_radius <=  -min_turning_radius:
        #         desired_turning_radius = desired_turning_radius_limit
        #     # if debug:
        #     print('gazebo case:', desired_turning_radius)
        
        desired_virtual_angle = math.atan(wheelbase/desired_turning_radius)

        if abs(desired_virtual_angle) > steering_limit:
            desired_virtual_angle_limit = steering_limit
            if desired_virtual_angle < 0:
                desired_virtual_angle_limit = -desired_virtual_angle_limit
            
            desired_virtual_angle = desired_virtual_angle_limit
                
        return desired_virtual_angle
    
    def ackermann_callback(self, msg):                                          # callback subcribes to twist messages and calculates the desired steering angle by caling the virtual steering angle function
        global wheelbase
        global desired_AISTEER_feedback_voltage
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z
        self.desired_angle = self.virtual_steering_angle(wheelbase, self.linear_velocity, self.angular_velocity)

        desired_AISTEER_feedback_voltage = (self.desired_angle)*scaling_factor + AINSTEER_centre_voltage
         
        # if debug:
        # print('desired_angle:', self.desired_angle)
        # print('Desired Feedback:', desired_AISTEER_feedback_voltage )

    
    def AINSTEER_callback(self, msg):                                           # function to take the feedback from AINSTEER (0V to 5V) over the topic /R4_AINSTEER
        global index
        global avg_AINSTEER 
        steer_feedback = msg.data
        if index < max_messages:
            ain_steer[index] = steer_feedback
            index += 1
            if index == max_messages:
        
                avg_AINSTEER = (ain_steer.sum())/max_messages
                
                index = 0
            '''print('messages in the array :',ain_steer)'''
        else:
            print('no data')

    def timer_callback(self):                                                   # timer callback to publish Ackermann drive messages (speed, steering angle velocity & steering angle) at a defined rate of 10 hz
        data = AckermannDrive()
        data.speed = self.linear_velocity
        data.steering_angle = self.desired_angle
        self.Ackermann_pub.publish(data)                                        # Publish the ackermann messages
        #print('desired_angle:', self.desired_angle)
        feedback_error = desired_AISTEER_feedback_voltage  - avg_AINSTEER
        #print('avg_AINSTEER', avg_AINSTEER)
        #print('Desired feedback:', desired_AISTEER_feedback_voltage)
        if feedback_error >= 0.0:
         direction = 0       
        else:
         direction = 1
        # if debug:
        
        if feedback_error > 0 and feedback_error < deadBand:
          R4_Command.data = "D:"+str(0)+','+str(direction)+','+str(R4_DHB_Channel) 
        else:  
            R4_Command.data = "D:"+str(4095)+','+str(direction)+','+str(R4_DHB_Channel)
        if R4_Command.data != old_R4_Command.data:
            self.Steer_pub.publish(R4_Command)                                  # Publish the R4 Protocol messages
        # print('R4_Command:',R4_Command)
        old_R4_Command.data = R4_Command.data  
       

def main(args=None):
    rclpy.init(args=args)
    Podcar_Steer_node = Podcar()
    rclpy.spin(Podcar_Steer_node)
    rclpy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

