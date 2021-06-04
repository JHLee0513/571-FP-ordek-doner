#!/usr/bin/env python3

import torch
from torchvision import transforms
from perception.network import LocalizationModel
import numpy as np
from cv_bridge import CvBridge

import os
import rospy

from duckietown.dtros import DTROS, NodeType, TopicType
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from perception.msg import PredictedPose
import rospkg

class PerceptionNode():
    """
    
    """
    def __init__(self, node_name, camera_topic=None):
        #Initialize DTROS parent class
        # super(PerceptionNode, self).__init__(
            # node_name = node_name,
            # node_type = NodeType.PERCEPTION
        # )
        self.node_name = node_name
        # Get vehicle name
        self.veh = rospy.get_namespace().strip("/")
        # Set input camera topic
        if camera_topic is None:
            # default to input image topic of the vehicle
            self.input_topic = f'/{self.veh}/camera_node/image/compressed'
        else:
            self.input_topic = camera_topic
        
        #Publisher to publish predicted pose
        self.pub = rospy.Publisher(
            f'/{self.veh}/{self.node_name}/perception/PredictedPose',
            PredictedPose,
            queue_size=1)

        # subscriber for receiving the input
        self.camera_feed_sub = rospy.Subscriber(
            self.input_topic,
            CompressedImage,
            self.model_callback,
            queue_size=1,
            buff_size=2**24
        )

        self.bridge = CvBridge()

        # get prediction model
        self.model = LocalizationModel()
        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path('perception'), "model_aug1.pth")
        self.model.load_state_dict(torch.load(model_path))

        # preprocessing, hardcode invariant
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
        rate = rospy.Rate(1) # 1Hz
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
        rate.sleep()


# if __name__ == '__main__':
#     # create the node
#     node = PerceptionNode(node_name='preception_node')
#     # run node
#     node.run()
#     # keep spinning
#     rospy.spin()