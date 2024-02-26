#!/usr/bin/env python3
import serial
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
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
scaler = 19.5 #15.0 #20.0 #9.4453 #7.5                                        # magic number from manjufacturer's data
h =  0.150 #0.155 #0.083                                                   # sensor height above the surface in meters
pixel_to_dist = 2*h*np.tan(FOV_RADIAN/2)/(RES*scaler)
print(type(pixel_to_dist))
old_accumulated_x_dist = 0                                              # intialize the variable to store the old calculated distance values in x direction
old_accumulated_y_dist = 0 
                                                                 # intialize the variable to store the old calculated distance values in y direction


class OpticalFlowPublisher(Node):
    def __init__(self):
        super().__init__('optical_flow_publisher')
        self.odometry_publisher = self.create_publisher(Odometry, 'optical_flow_odom', 10)
        self.transform_br = TransformBroadcaster(self)
        self.timer_ = self.create_timer(0.01, self.timer_callback)
        #self.publish_timer = self.create_timer(0.05, self.odom_callback)
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

        #Initialize actual distance variables
        self.vx_ms = 0.0
        self.vy_ms = 0.0
        
      
    '''The field of view (FOV) of the optical flow sensor is 42.0 degrees and resolution in pixels is 35 from the given data, using optics of lens and trigonometry,        
the FOV is equals to the arc tangent horizontal FOV in meters or mm and the height at which the snesor is mounted above the ground.'''    
    def pix2speed(self, dx_pix, dy_pix, dt_sec):
        # global old_accumulated_x_dist
        # global old_accumulated_y_dist
        dx_m = (pixel_to_dist*dx_pix) #.round(3)                                   # This stores the value of distnace in x direction in mm or meters.
        dy_m = (pixel_to_dist*dy_pix) #.round(3)                                   # This stores the value of distnace in y direction in mm or meters.
        vx_ms = dx_m / dt_sec                                                      # Calculate the linear velocity in x 
        vy_ms = dy_m / dt_sec                                                      # Calculate the linear velocity in y 
        

        return (vx_ms, vy_ms)                                                       


    '''Function to take the average of calcluated velocities. Used to make the values near to what the robot actually behaves using gamepad.
    To do this, take the average of vx and vy and caliberate the sensor output to match the actual robot velocity. Could be done by tuning the scaling factor'''
    
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
        
        current_time_sec = time()
        dt_sec = current_time_sec - self.old_time
        self.old_time = current_time_sec
        
        if self.ser.isOpen() == True:                                                          # checks if serail port is open
            
            while self.ser.in_waiting != 0:                                                    # check for more incoming bytes
                
                read_bytes = self.ser.read()                                                   # read the incoming bytes
                if read_bytes.decode() == '\n':
                    self.string_complete = True
                    break
                else:
                    self.input_data += read_bytes.decode()

            if self.string_complete:
                try:
                    
                    (dx_pix, dy_pix) = self.extract_data(self.input_data)                           # Extract x and y values from input_data
                    (self.vx_ms, self.vy_ms) = self.pix2speed(dx_pix, dy_pix, dt_sec)               # call the function for pixel to distance                                          
                    
                    print('Before Average values: ', self.vx_ms, self.vy_ms)
                    avg_vels = self.averager(np.array((self.vx_ms, self.vy_ms)))
                    print('AVG_Velocities: ', avg_vels)
                    print("#####################")
                    
                    # self.extract_data(self.input_data)                                          # Extract x and y values from input_data
                except ValueError as e:
                    print("Invalid input - stopped")
                    self.ser.close()
                    exit()
                    
                self.input_data = ''
                self.string_complete = False
                # Feed x and y values on the 'optical_flow_odom' topic
                odom_msg = Odometry()
                odom_msg.header.stamp = self.get_clock().now().to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'

                # fill the pose field of the message
                          
                odom_msg.twist.twist.linear.x = self.vx_ms
                odom_msg.twist.twist.linear.y = self.vy_ms
                covaraince  = np.diag([0.01, 0.01, 0, 0, 0, 0])
               
                odom_msg.twist.covariance = list(covaraince.flatten())
                self.odometry_publisher.publish(odom_msg)                                                 # publish the odometry data
                
                #transform broadcaster
                # transform_msg = TransformStamped()
                # transform_msg.header.stamp = self.get_clock().now().to_msg()
                # transform_msg.header.frame_id = 'odom'
                # transform_msg.child_frame_id = odom_msg.child_frame_id
                # transform_msg.transform = Transform(translation = Vector3(x = odom_msg.pose.pose.position.x,
                #                                                 y = odom_msg.pose.pose.position.y,
                #                                                 z = odom_msg.pose.pose.position.z))

                # self.transform_br.sendTransform(transform_msg)                                            # broadcast the odom --> base_link transform
            
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