#!/usr/bin/env python3

# import perception
# from perception import PerceptionNode
import os.path, pkgutil

import os
from rrt_planner.environments.CarEnvironment import CarEnvironment
from rrt_planner.RRTPlannerNonholonomic import RRTPlannerNonholonomic
from perception.perception_node import PerceptionNode
from duckietown_msgs.msg import Twist2DStamped
import rospy
from duckietown.dtros import DTROS, NodeType
import numpy as np
import matplotlib.pyplot as plt
from perception.msg import PredictedPose
import rospkg
import time

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
        # Get perception nodes
        self.goal_pose_node = PerceptionNode('goal_classifier', camera_topic=f'/{self.veh}/goal_state_publisher/image_goal/compressed')
        self.curr_pose_node = PerceptionNode('curr_classifier')
        # ROS interface
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

        self.control_pub = rospy.Publisher(
            '/control', 
            Twist2DStamped,
            queue_size=1
        )

        # Planning variables
        self.planning_env = None
        self.planner = None
        self.planned_trajectory = None
        self.goalpose = (0,0,0)
        self.currpose = (50, 50, 0.273915)
        self.seed = 2021
        rospack = rospkg.RosPack()
        # self.map_path = os.path.join(rospack.get_path('rrt_planner'),"environments/mapfile.pgm")
        self.map_path = os.path.join(rospack.get_path('rrt_planner'),"environments/map2.png")
        # print("Intialization complete!")

    def updatePose(self, pose):
        self.currpose = (pose.x, pose.y, pose.theta)

    def plan(self, msg):
        # self.goalpose = (msg.x, msg.y, msg.theta)
        self.goalpose = (50, 50, 0)
        # self.currpose = (420, 100, 0.5)
        self.currpose = (100, 100, 0.5)
        start = np.asarray(self.currpose).reshape((3,1))
        goal = np.asarray(self.goalpose).reshape((3,1))

        print("Goal received. Planning with start (%f, %f, %f) and goal (%f, %f, %f)" % (start[0,0], start[1,0], start[2,0], goal[0,0], goal[1,0], goal[2,0]))

        self.planning_env = CarEnvironment(self.map_path, start, goal, self.seed)
        self.planner = RRTPlannerNonholonomic(self.planning_env, seed=self.seed, bias=0.2)

        self.planning_env.init_visualizer()

        # Plan
        plan_result, plan_result_states = self.planner.plan(start, goal)

        # Visualize the final path
        tree = None
        visited = None
        tree = self.planner.tree
        # For debugging purposes
        # self.planning_env.visualize_plan(plan_result_states.plan, tree, visited)
        self.run_plan(plan_result)
        # Prevent constant planning compute overhead
        time.sleep(3)


    def run_plan(self, plan_result):
        plan_states = plan_result.plan
        print(plan_states)
        rate = rospy.Rate(10)
        for action in plan_states:
            linear = float(action[0])
            angular = float(action[1])
            msg = Twist2DStamped()
            msg.v = linear
            msg.omega = angular
            self.control_pub.publish(msg)
            rate.sleep()
            # print("published control! %f, %f" % (linear, angular))

if __name__ == '__main__':
    # Initialize the node
    import time
    time.sleep(1)
    sensor_fusion_node = PlannerNode(node_name='sensor_fusion_node')
    # sensor_fusion_node.run()
    # Keep it spinning to keep the node alive
    rospy.spin()