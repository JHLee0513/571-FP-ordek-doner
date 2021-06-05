#!/usr/bin/env python3

# import perception
# from perception import PerceptionNode
import os.path, pkgutil

import os
from rrt_planner.environments.CarEnvironment import CarEnvironment
from rrt_planner.RRTPlannerNonholonomic import RRTPlannerNonholonomic
from perception.perception_node import PerceptionNode
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import matplotlib.pyplot as plt
from perception.msg import PredictedPose

class PlannerNode(DTROS):
    """
    This node is a ROS node for generating and executing navigation plan,
    and it utilizes the image based localization perception.
    """
    def __init__(self, node_name):
        #Initialize DTROS parent class
        super(PlannerNode, self).__init__(
            node_name = node_name,
            node_type = NodeType.PLANNING
        )

        # Get vehicle name
        self.veh = rospy.get_namespace().strip("/")

        # Get perception node for goal pose prediction
        self.goal_pose_node = PerceptionNode('goal_classifier', camera_topic=f'/{self.veh}/camera_node/image_goal/compressed')

        # Get perception node for curr pose prediction
        self.curr_pose_node = PerceptionNode('curr_classifier', camera_topic=f'/{self.veh}/camera_node/image_start/compressed')

        # pose handling
        self.goal_sub = rospy.Subscriber(
            f'/{self.veh}/goal_classifier/perception/PredictedPose',
            PredictedPose,
            self.plan,
            queue_size=1
        )
        self.curr_sub = rospy.Subscriber(
            f'/{self.veh}/curr_classifier/perception/PredictedPose',
            PredictedPose,
            self.updatePose,
            queue_size=1
        )

        # Planning variables
        self.planning_env = None
        self.planner = None
        self.planned_trajectory = None
        self.goalpose = (0,0,0)
        self.currpose = (0,0,0)
        self.seed = 2021

    
    def updatePose(self, pose):
        self.currpose = (pose.x, pose.y, pose.theta)

    def plan(self):
        self.planning_env = CarEnvironment("../environments/mapfile.pgm", self.currpose, self.goalpose, self.seed)
        self.planner = RRTPlannerNonholonomic(self.planning_env, seed=self.seed, bias=0.2)

        self.planning_env.init_visualizer()

        # Plan
        plan_result = self.planner.plan(np.asarray(self.currpose), np.asarray(self.goalpose))

        # Visualize the final path
        tree = None
        visited = None
        tree = self.planner.tree
        self.planning_env.visualize_plan(plan_result.plan, tree, visited)
        plt.show()


    def run(self):
        # have goal image publishing running
        # get preidcted pose from network
        # plan
        # run plan
        # Base on some criterion replan.

        # send motor signals along the plan
        # after moving for x seconds, check current pose for deviation
        # or some replanning criterion
        # otherwise continue until termination
        return NotImplementedError()

if __name__ == '__main__':
    # Initialize the node
    sensor_fusion_node = PlannerNode(node_name='sensor_fusion_node')
    sensor_fusion_node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()