#!/usr/bin/env python3

import os
import numpy as np
from PIL import Image
from sensor_msgs.msg import CompressedImage
import rospy
import rospkg
import cv2
from duckietown.dtros import DTROS, NodeType
"""
publishes input camerage of a goal constantly
"""


class GoalPublisherNode(DTROS):
    def __init__(self, node_name):
        super(GoalPublisherNode, self).__init__(
            node_name=node_name,
            node_type=NodeType.GENERIC)
        self.veh = rospy.get_namespace().strip('/')
        # image topic hardcoded for now
        rospack = rospkg.RosPack()
        perception_path = rospack.get_path('perception')
        image = np.asarray(Image.open(os.path.join(perception_path, "files/goal.jpg")))
        self.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        #Publisher to publish predicted pose
        self.pub = rospy.Publisher(
            f'{self.node_name}/image_goal/compressed',
            CompressedImage,
            queue_size=1)

    def run(self):
        # publish message every 1 second
        rate = rospy.Rate(15)  # 1Hz
        while not rospy.is_shutdown():
            msg = CompressedImage()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = self.data
            self.pub.publish(msg)
            rate.sleep()

if __name__ == "__main__":
    node = GoalPublisherNode(node_name='goal_publisher')
    node.run()
    rospy.spin()
    