#!/usr/bin/env python3

import os
import rospy


from duckietown.dtros import DTROS, NodeType, TopicType

class PlannerNode(DTROS):
    """
    ss
    """
    def __init__(self, node_name):
        #Initialize DTROS parent class
        super(PlannerNode, self).__init__(
            node_name = node_name,
            node_type = NodeType.PLANNING
        )

        # Get vehicle name

        # Get perception node for goal pose prediction
        # Get perception node for curr pose prediction
        
    
    def plan(self):
        pass

    def run(self):
        # have goal image publishing running
        # get preidcted pose from network
        # plan
        # run plan
        # Base on some criterion replan.
        return NotImplementedError()