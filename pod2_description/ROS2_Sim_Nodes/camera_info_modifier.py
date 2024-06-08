#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo

class CameraInfoModifier(Node):

    def __init__(self):
        super().__init__('camera_info_modifier')

        self.declare_parameter('input_topic', '/depth_camera_info')
        self.declare_parameter('output_topic', '/camera_info_modified')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self.subscription = self.create_subscription(
            CameraInfo,
            input_topic,
            self.camera_info_callback,
            10
        )
        self.publisher = self.create_publisher(CameraInfo, output_topic, 10)

        self.get_logger().info(f'Subscribed to {input_topic}')
        self.get_logger().info(f'Publishing modified CameraInfo to {output_topic}')

    def camera_info_callback(self, msg):
        modified_msg = CameraInfo()
        modified_msg.header = msg.header
        
        modified_msg.height = 480
        modified_msg.width = 640
        modified_msg.distortion_model = 'plumb_bob'
        modified_msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        modified_msg.k = [
            617.4435424804688, 0.0, 317.70989990234375,
            0.0, 617.878662109375, 238.01991271972656,
            0.0, 0.0, 1.0
        ]
        modified_msg.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        modified_msg.p = [
            617.4435424804688, 0.0, 317.70989990234375, 0.0,
            0.0, 617.878662109375, 238.01991271972656, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        modified_msg.binning_x = 0
        modified_msg.binning_y = 0
        modified_msg.roi.x_offset = 0
        modified_msg.roi.y_offset = 0
        modified_msg.roi.height = 0
        modified_msg.roi.width = 0
        modified_msg.roi.do_rectify = False

        self.publisher.publish(modified_msg)
        self.get_logger().info('Published modified CameraInfo')

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoModifier()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
