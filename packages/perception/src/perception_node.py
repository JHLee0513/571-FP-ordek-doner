#!/usr/bin/env python3

import torch
from torchvision import transforms
from network import LocalizationModel
import numpy as np
from cv_bridge import CvBridge

import os
import rospy


from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from msg import PredictedPose

class PerceptionNode(DTROS):
    """
    
    """
    def __init__(self, node_name):
        #Initialize DTROS parent class
        super(PerceptionNode, self).__init__(
            node_name = node_name,
            node_type = NodeType.PERCEPTION
        )

        self.pub = rospy.Publisher(
            f'/{self.veh}/perception/PredictedPose',
            PredictedPose,
            queue_size=1)

        # Get vehicle name
        self.veh = rospy.get_namespace().strip("/")

        camera_topic = f'/{self.veh}/camera_node/image/compressed'
        self.camera_feed_sub = rospy.Subscriber(
            camera_topic,
            CompressedImage,
            self.model_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.bridge = CvBridge()

        # get prediction model
        self.model = LocalizationModel()
        self.model.load_state_dict(torch.load("./model_aug1.pth"))

        # preprocessing
        self.size = (128, 128)
        self.transform = transforms.compose([
            transforms.ToPILImage(),
            transforms.Resize(self.size),
            transforms.ToTensor()
        ])

    def model_callback(self, img_msg):
        """
        This function processes received input image to predict (x,y, theta)
        pose.
        """

        def normalize_angle(angle):
            while angle >= np.pi:
                angle -= 2 * np.pi
            while angle <= -np.pi:
                angle += 2 * np.pi
            return angle

        # decompress img msg
        cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        # transform img to model input
        input = self.transform(cv_image)
        # run model
        predicted_pose = self.model(input)
        predicted_pose[2] = normalize_angle(predicted_pose[2])
        # publish output

        message = PredictedPose()
        message.x = predicted_pose[0]
        message.t = predicted_pose[1]
        message.theta = predicted_pose[2]
        
        self.pub.publish(message)

        # rate = rospy.Rate(1) # 1Hz
        # message = "Hello from %s" % os.environ['VEHICLE_NAME']
        # rospy.loginfo("Received input image!")
        # self.pub.publish(message)
        # rate.sleep()


if __name__ == '__main__':
    # create the node
    node = PerceptionNode(node_name='preception_node')
    # run node
    node.run()
    # keep spinning
    rospy.spin()