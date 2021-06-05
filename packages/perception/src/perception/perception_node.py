#!/usr/bin/env python3

import torch
from torchvision import transforms
from perception.network import LocalizationModel
import numpy as np
from cv_bridge import CvBridge
import cv2

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
        self.device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
        self.node_name = node_name
        # Get vehicle name
        self.veh = rospy.get_namespace().strip("/")
        # Set input camera topic
        if camera_topic is None:
            # default to input image topic of the vehicle
            self.input_topic = f'/{self.veh}/camera_node/image/compressed'
        else:
            self.input_topic = camera_topic
        self.x_voxel = 1.89/366
        self.y_voxel = 1.26/246
        self.bridge = CvBridge()

        # get prediction model
        self.model = LocalizationModel().to(self.device)
        rospack = rospkg.RosPack()
        model_path = os.path.join(rospack.get_path('perception'), "files/model_aug1.pth")
        self.model.load_state_dict(torch.load(model_path, map_location=self.device))

        # preprocessing, hardcode invariant
        self.size = (128, 128)
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize(self.size),
            transforms.ToTensor()
        ])

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
        # cv_image = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        np_arr = np.fromstring(img_msg.data, np.uint8)
        cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # transform img to model input
        input = self.transform(cv_image)
        input = input.to(self.device).unsqueeze(0)
        # run model
        predicted_pose = self.model(input)[0]
        predicted_pose[2] = normalize_angle(predicted_pose[2])
        # publish output

        message = PredictedPose()
        message.x = torch.round(predicted_pose[1] / self.x_voxel)
        message.y = torch.round(predicted_pose[0] / self.y_voxel)
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
