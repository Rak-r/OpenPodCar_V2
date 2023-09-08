#! /usr/bin/env python3
import rclpy
import sys
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from tf2_ros.transform_broadcaster import TransformBroadcaster
#from turtlesim.msg import Pose
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry

class DynamicBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_broadcaster')
       
        #self.get_logger().info("Broadcasting pose of : {}".format(self.name_))
        self.tfb_ = TransformBroadcaster(self)
        self.sub_pose = self.create_subscription(TFMessage, '/model/podcar/pose', self.handle_pose, 1)

    def handle_pose(self, msg):
        
        tfs = TFMessage()
        tfs.header.stamp = self.get_clock().now().to_msg()
        tfs.header.frame_id="odom"
        tfs._child_frame_id = "base_link"
        tfs.transform.translation.x = msg.x
        tfs.transform.translation.y = msg.y
        tfs.transform.translation.z = msg.z  

        r = R.from_euler('xyz',[0,0,msg.theta])

        tfs.transform.rotation.x = r.as_quat()[0]
        tfs.transform.rotation.y = r.as_quat()[1]
        tfs.transform.rotation.z = r.as_quat()[2]
        tfs.transform.rotation.w = r.as_quat()[3]

        self.tfb_.sendTransform(tfs)    

def main(args = None):
    rclpy.init(args=args)
    node = DynamicBroadcaster()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()