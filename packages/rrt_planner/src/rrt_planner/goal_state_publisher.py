#!/usr/bin/env python3

import os
import numpy as np
from PIL import Image
from sensor_msgs.msg import CompressedImage
import rospy
import rospkg
import cv2
"""
publishes input camerage of a goal constantly
"""

if __name__ == "__main__":
    # Get vehicle name
    veh = rospy.get_namespace().strip("/")
    rospy.init_node('goal_publisher', anonymous=True)
    # image topic hardcoded for now
    goal_img_topic = f'/{ veh}/camera_node/image_goal/compressed'
    rospack = rospkg.RosPack()
    #Publisher to publish predicted pose
    pub = rospy.Publisher(
        goal_img_topic,
        CompressedImage,
        queue_size=1)


    perception_path = rospack.get_path('perception')
    image = np.asarray(Image.open(os.path.join(perception_path, "files/goal.jpg")))

    rate = rospy.Rate(0.1)
    while not rospy.is_shutdown():            
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
        pub.publish(msg)
        rate.sleep()
