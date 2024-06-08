#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

################
# SimplePubSub #
################
class SimplePubSub(Node):

    def __init__(self):
        super().__init__('R4_Publisher')
        self.subscription = self.create_subscription(String, 'R4', self.listener_callback, 10)
        #self.subscription  # prevent unused variable warning
        self.myPublishers = {}
        self.topics = []
       
    #####################
    # listener_callback #
    #####################
    def listener_callback(self, msg):

        data = msg.data
        
        # Split message into parts
        dataParts = data.split(";")

        for d in dataParts:
            if len(d) > 1:
                parts = d.split(':')
                topic = parts[0]
                if topic == "T" or topic == "PNum":
                    continue
                data = parts[1]

                # If we haven't seen this topic before....
                if not topic in self.topics:
                    # Add its name
                    self.topics.append(topic)
                    # Append the publisher to a dictionary of publishers
                    # NOTE queue length was 10 
                    self.myPublishers[topic] = self.create_publisher(String, "R4_" + topic, 1)
                    self.get_logger().info('New topic seen: "%s"' % topic)

                # Create the payload String
                payLoadS = String()
                payLoadS.data = data

                # Publish on the topic
                self.myPublishers[topic].publish(payLoadS)

########
# Main #
########
def main(args=None):
    rclpy.init(args=args)
    pubSub = SimplePubSub()
    rclpy.spin(pubSub)
    pubSub.destroy_node()
    rclpy.shutdown()

##################
# Start properly #
##################
if __name__ == '__main__':
    main()