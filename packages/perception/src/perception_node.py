#!/usr/bin/env python3

# import torch
import os
import rospy


from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage

class PerceptionNode(DTROS):
    """
    
    """
    def __init__(self, node_name):
        #Initialize DTROS parent class
        super(PerceptionNode, self).__init__(
            node_name = node_name,
            node_type = NodeType.PERCEPTION
        )

        self.pub = rospy.Publisher('chatter', String, queue_size=10)


        # Get vehicle name
        self.veh_name = rospy.get_namespace().strip("/")

        camera_topic = f'/{self.veh}/camera_node/image/compressed'
        self.camera_feed_sub = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.model_callback,
            queue_size=1,
            buff_size=2**24
        )

        # get prediction model
        self.model = None

    def model_callback(self, img_msg):
        """
        This function processes received input image to predict (x,y, theta)
        pose.
        """
        rate = rospy.Rate(1) # 1Hz
        message = "Hello from %s" % os.environ['VEHICLE_NAME']
        rospy.loginfo("Received input image!")
        self.pub.publish(message)
        rate.sleep()


if __name__ == '__main__':
    # create the node
    node = PerceptionNode(node_name='preception_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()