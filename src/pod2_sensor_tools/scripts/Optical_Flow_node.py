#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, TwistWithCovariance, Quaternion, Transform, Point, Vector3, Twist, PoseWithCovariance
from tf2_ros import TransformBroadcaster, TransformStamped
import numpy as np
import signal
from time import time
import math

FOV_DEG = 42.0                                              # field of view in degrees
FOV_RADIAN = np.radians(FOV_DEG)                            # field of view in degrees
RES = 35                                                    # resolution in pixels
scaler = 9.38 #15.0 #20.0 #9.4453 #7.5                      # magic number from manufacturer's data
h =  0.150 #0.155 #0.083                                    # sensor height above the surface in meters
pixel_to_dist = 2*h*np.tan(FOV_RADIAN/2)/(RES*scaler)

slope = 0.406652563002354
intercept = 0.508426291436033

wheelbase = 1.05
AINSTEER_centre_voltage = 1.25                                                  # steering caliberation based on placemnet of Linear actuator
desired_AISTEER_feedback_voltage = AINSTEER_centre_voltage                      # desired value for the steering motor in volts, 1.25v represents nearly centre position
AINSTEER_right_limit = 2.5                                                      # Max AINSTEER voltage limit when turning full right (in volts)
AINSTEER_left_limit = 0.1                                                       # Max AINSTEER voltage limit when turning full left (in volts)
Right_AINSTEER_max = AINSTEER_right_limit - AINSTEER_centre_voltage
Max_theta = 0.436                                                               # Maximum angle the vehicle can achieve (in radians)
scaling_factor =  Right_AINSTEER_max/Max_theta
max_messages = 20                                                               # no. of messges to receive from DHB to get the actual position of the linear actuator
                                                          
ain_steer = np.zeros(max_messages, dtype = np.float32)
                                                                                # a zero-initialized array to store the messgaes from R4_AINSTEER topic
index = 0                                                                       # array index to perform array operations
avg_AINSTEER = 0

class OpticalFlowPublisher(Node):
    def __init__(self):
        super().__init__('optical_flow_publisher')

        #  declare parameter

        self.declare_parameters(
            namespace='',
            parameters=[
                ('use_AINSTEER_feedback', True),
                ('publish_tf', False),
            ])
        
        self.use_AINSTEER_feedback = self.get_parameter('use_AINSTEER_feedback').value
        self.publish_tf = self.get_parameter('publish_tf').value
        # publisher
        self.odometry_publisher = self.create_publisher(Odometry, 'optical_flow_odom', 10)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        # self.raw_values_pub = self.create_publisher(Odometry, '/raw_value_odom', 10)
        # subscriber
        self.AINSTEER_sub = self.create_subscription(String, 'R4_AINSTEER', self.AINSTEER_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)                           # transform broacaster odom --> base_link
        self.ser = serial.Serial()
        self.vels = []
        #store prev time 
        self.old_time = time()

        # Initialize serial connection
        self.input_data = ''
        self.string_complete = False
        self.init_serial()
        # setup callback for ctrl C
        signal.signal(signal.SIGINT, self.handler)
        
        #Initialize actual distance variables
        self.dx = 0.0
        self.dy = 0.0
        self.abs_x = 0.0
        self.abs_y = 0.0
        self.vx_ms = 0.0
        self.vy_ms = 0.0
        
        # initialise theta and angular velocity variables

        self.abs_theta = 0.0
        self.current_theta = 0.0
        self.old_theta = 0.0
        self.delta_theta = 0.0
        self.v_angular = 0.0
      
    '''The field of view (FOV) of the optical flow sensor is 42.0 degrees and resolution in pixels is 35 from the given data, using optics of lens and trigonometry,        
the FOV is equals to the arc tangent horizontal FOV in meters or mm and the height at which the snesor is mounted above the ground.'''    
    def pix2speed(self, dx_pix, dy_pix, dt_sec):
        # global old_accumulated_x_dist
        # global old_accumulated_y_dist
        dx_m = (pixel_to_dist*dx_pix) #.round(3)                                   # This stores the value of distnace in x direction in mm or meters.
        dy_m = (pixel_to_dist*dy_pix) #.round(3)                                   # This stores the value of distnace in y direction in mm or meters.
        # self.abs_x += dx_m
        # self.abs_y += dy_m
        
        vx_ms = dx_m / dt_sec                                                      # Calculate the linear velocity in x 
        vy_ms = dy_m / dt_sec                                                      # Calculate the linear velocity in y 
        
        # print(f' Absolute X : {self.abs_x}, Absolute Y : {self.abs_y}')
        return (dx_m, dy_m)                                                       

    
    #### FOLLOWING FIRST APPROACH ######
    ####################################

    def calculate_theta(self, vy,vx):
                                             
        v_magnitude = math.sqrt((vx)**2 + (vy)**2)                                             # Find the magnitude for the velocity
        self.current_theta = math.atan2(vy,vx)                                                      # Find the current rotating angle of the robot
        
        d_theta = self.current_theta - self.old_theta                                           # calculate change in rotating  angle
                             
        self.old_theta = self.current_theta                                                          # Update the rotating angle
        
        return  d_theta
    
    
    
    ###### FOLLOWING SECOND APPROACH ########
    #########################################
    
    '''
    Find the angular velocity using steering angle feedback.

    1.) subscribe to the steer feedback message from linear actuator node on the topic /R4_AINSTEER.

    2.) Using linear.x velocity Optical flow measurement and the relation:  v = rw

    where r = radius of the arc, 
          w = angular velocity
          v = linear velocity

    3.) r can be determined using the ackermann eqautions and wheelbase, tan(theta) = wheelbase/r
        r = wheelbase / tan(theta)

    4.) substitute r in (2), w = v*tan(theta)/wheelbase

    '''
    def AINSTEER_callback(self, msg):                                             # function to take the feedback from AINSTEER (0V to 5V) over the topic /R4_AINSTEER
        global index
        global avg_AINSTEER 
        steer_feedback = msg.data
        if index < max_messages:
            ain_steer[index] = steer_feedback
            index += 1
            if index == max_messages:
                #avg_AINSTEER = np.mean(ain_steer)
                avg_AINSTEER = (ain_steer.sum())/max_messages
                
                index = 0
                # self.current_theta = (avg_AINSTEER - AINSTEER_centre_voltage)/scaling_factor
                self.current_theta = avg_AINSTEER*slope - intercept
        else:
            print('no data')


    def yaw_to_quaternion(self, yaw):
        # Convert yaw angle to quaternion
        quaternion = Quaternion()
       
        # Yaw is rotation around the z-axis
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
       
        return quaternion


    '''Function to take the average of calculated velocities. Used to make the values near to what the robot actually behaves using gamepad.
    To do this, take the average of vx and vy and calibrate the sensor output to match the actual robot velocity. Could be done by tuning the scaling factor'''
    
    def averager(self, new_vels: np.array):
        
        if len(new_vels) > 1:
            
            self.vels.append(new_vels)
            
            self.vels = self.vels[-50:]
            # print('10 values', self.vels)
            avg_vels = np.mean(self.vels, axis=0)
            # print('avg' ,avg_vels[0])
            return avg_vels  
        else:
            print(f' Not much velocities to average {self.vels}')

    '''Function to extract the values from the incoming string and returns the dx and dy in pixels'''

    def extract_data(self,input_data):
        try:

            data = input_data.strip().split(',')
            dx = int(data[0].split(':')[1].strip())
            dy = int(data[1].split(':')[1].strip())
            return dx, dy                                                     # call the function for pixel to distance                                          
        except ValueError as e:
            print(f'Invalid data:{e}')

    # callback handler for ctrl C
    def handler(self,signum, frame):
        print('Control C detedted port closed', signum)
        self.ser.close()
        exit()

    def init_serial(self):
        try:
            print('trying to open port')
            self.ser = serial.Serial('/dev/ttyACM0', 115200)
            print('succeded')
        except:
            print('No port found')
            exit()    

    # print('start reading the data')
    
    def timer_callback(self):                                                                  # callback to read the incoming sensor data at its actual rate
        
        t = TransformStamped()
        odom_msg = Odometry()
        # raw_value_odom = Odometry()                                                            #raw value (dx, dy) based odometry message
        if self.ser.isOpen() == True:                                                          # checks if seriall port is open
            
            while self.ser.in_waiting != 0:                                                    # check for more incoming bytes
                
                read_bytes = self.ser.read()                                                   # read the incoming bytes
                if read_bytes.decode() == '\n':
                    self.string_complete = True
                    break
                else:
                    self.input_data += read_bytes.decode()

            if self.string_complete:
                try:
                    
                    current_time_sec = time()
                    dt_sec = current_time_sec - self.old_time
                    self.old_time = current_time_sec

                    (dx_pix, dy_pix) = self.extract_data(self.input_data)                           # Extract x and y values from input_data
                    (self.dx, self.dy) = self.pix2speed(dx_pix, dy_pix, dt_sec)                     # call the function for pixel to distance                                          
                    
                    # NOTE This may not be appropiate because the pose should be trasnformed to global frame before passing into message
                    #https://github.com/introlab/rtabmap_ros/issues/1149
                    
                    self.abs_x += self.dx                                                           
                    self.abs_y += self.dy
                    # print(f' Absolute X : {self.abs_x}, Absolute Y : {self.abs_y} ', '\n')
                    self.vx_ms = self.dx/dt_sec
                    self.vy_ms = self.dy/dt_sec
                    # print('Before Average values: ', self.vx_ms, self.vy_ms)
                    avg_vels = self.averager(np.array((self.vx_ms, self.vy_ms)))
                    # print('AVG_Velocities: ', avg_vels)
                    

                    if not self.use_AINSTEER_feedback: 
                        # Feed x and y values on the 'optical_flow_odom' topic
                        
                        self.delta_theta = self.calculate_theta(self.vy_ms, self.vx_ms)
                        self.abs_theta += self.delta_theta

                        #  generate the odometry message
                        odom_msg.header.stamp = self.get_clock().now().to_msg()                 # This is synced to time the data is received, but actually it should use time stamp
                                                                                                # coming from Teensy board
                        odom_msg.header.frame_id = 'odom'
                        odom_msg.child_frame_id = 'base_link'                                   # NOTE if feeding velocity data, try to pass the velocity in x as input to EKF                        
                        # fill in the pose field of odometry message
                        
                        # odom_msg.pose.pose.position.x = self.abs_x
                        # odom_msg.pose.pose.position.y = self.abs_y
                        # odom_msg.pose.pose.position.z = h
                        # odom_msg.pose.pose.orientation.z = math.sin(self.abs_theta/2.0)
                        # odom_msg.pose.pose.orientation.w = math.cos(self.abs_theta/2.0)
                        odom_msg.twist.twist.linear.x = self.vx_ms
                        # odom_msg.twist.twist.linear.y = self.vy_ms
                        
                        # covariance_matrix = np.array([0.04, 0.02, 0.0, 0.0, 0.0, 0.1])
                        # odom_msg.twist.covariance = covariance_matrix.flatten()
                    else:

                        self.v_angular = self.vx_ms*math.tan(self.current_theta)/wheelbase
                        self.delta_theta = self.v_angular*dt_sec
                        self.abs_theta += self.delta_theta
                        print(f'Vyaw : {self.v_angular},  Yaw : {self.abs_theta}')
                        print(f'X : {self.abs_x}')
                        print('\n')

                        #  generate the odometry message
                        odom_msg.header.stamp = self.get_clock().now().to_msg()
                        odom_msg.header.frame_id = 'odom'
                        odom_msg.child_frame_id = 'base_link'

                        # odom_msg.pose.pose.position.x = self.abs_x
                        # odom_msg.pose.pose.position.y = self.abs_y
                        # odom_msg.pose.pose.position.z = h
                        
                        # odom_msg.pose.covariance[7] = 0.005  
                        # odom_msg.pose.covariance[35] = 10000.0                                               # make EKF to not believe this measurement fully
                        # fill the twist field of the message
                        
                        odom_msg.twist.twist.linear.x = self.vx_ms
                        # odom_msg.twist.twist.linear.y = self.vy_ms
                        # odom_msg.twist.twist.angular.z = self.v_angular
                        

                        # Assign covariance matrix to the twist covariance
                        # odom_msg.twist.covariance[0] = 0.04
                        # odom_msg.twist.covariance[6] = 0.002
                        # odom_msg.twist.covariance[11] = 0.05
                    t.header = odom_msg.header
                    t.header.frame_id = 'odom'
                    t.child_frame_id = 'base_link'
                    t.transform.translation.x = odom_msg.pose.pose.position.x
                    t.transform.translation.y = odom_msg.pose.pose.position.y
                    t.transform.translation.z = odom_msg.pose.pose.position.z 
                    # t.transform.rotation.z = odom_msg.pose.pose.orientation.z
                    # t.transform.rotation.w = odom_msg.pose.pose.orientation.w
                    
                    self.odometry_publisher.publish(odom_msg)                                                  # publish the odometry data
                    # if self.publish_tf:
                    #     self.tf_broadcaster.sendTransform(t)                                                 # Send the transformation
                except ValueError as e:
                    print("Invalid input - closing the serial port")
                    self.ser.close()
                    exit()

                self.input_data = ''
                self.string_complete = False
            
        else:
            print('Serial Port Failed')
            self.ser.close()
            exit()

                 
def main(args=None):
    rclpy.init(args=args)

    optical_flow_publisher = OpticalFlowPublisher()
    try:
        rclpy.spin(optical_flow_publisher)
    except:
        
        optical_flow_publisher.destroy_node()
        rclpy.shutdown()
    

if __name__ == '__main__':
    main()