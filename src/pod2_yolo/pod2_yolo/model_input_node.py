#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# from ultralytics import YOLO
from sensor_msgs.msg import Image
from cv_bridge import CvBridge,CvBridgeError
from rclpy.qos import qos_profile_sensor_data


    
class Realtime_Input(Node):
    def __init__(self):
        super().__init__('yolo_subscriber')

        self.rgb_image_subcriber = self.create_subscription(Image, '/rgbd_image_real', self.image_callback, 10)
        # self.depth_image_subscriber = self.create_subscription(Image, '/D435/depth/image_rect_raw', self.depth_callback, 10)
        self.brg = CvBridge()
        # self.timer = self.create_timer(0.1, self.timer_callback)
    
    def image_callback(self, msg):
        
        img = self.brg.imgmsg_to_cv2(msg, 'bgr8')
  
      
    def depth_callback(self, msg):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, 'passthrough' )
        except CvBridgeError as e:
            print('no image received')


def main(args = None):
    rclpy.init(args=args)
    yolo_subscriber = Realtime_Input()
    rclpy.spin(yolo_subscriber)
    yolo_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()

