import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import tf2_ros
import tf_transformations as tf
from geometry_msgs.msg import TransformStamped

class podcar_tf_broadcaster(Node):
    def __init__(self):
        super().__init__('tf_frame_publisher')

        #declare the object of transform broadcaster class
        self.trans_br = tf2_ros.TransformBroadcaster(self)

        #create the subscriber to this node which will subscribe and publish the transforms accordingly
        self.subscriber_ = self.create_subscription(Odometry, 'odometry/groundtruth', self.callback, 1)

    #callback function
    def callback(self, msg):

        #create a transform stamped message object
        #set the values of all the parts in the message
        #header.stamp give sthe current time of broadcasting
        #header.frame_id = parent frame id
        #header.child_frame_id = child frame_id where the properties should be executed namly; linear velocity an dangular velocity.
        t = TransformStamped()
        t.header.stamp = self.get_clock().now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = msg.z
        #using quaternion from euler function to convert the raw, pitch, yaw values to x,y,z,w
        quat = tf.quaternion_from_euler(0.0, 0.0, msg.theta)
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        #send the transform using sendTransform function of tf_tranformations
        self.trans_br.sendTransform(t)

def main():
    rclpy.init()
    node = podcar_tf_broadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()



