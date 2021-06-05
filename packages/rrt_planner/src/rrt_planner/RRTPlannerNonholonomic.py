import sys
import time
from typing import Any

import numpy as np

from rrt_planner.environments.CarEnvironment import CarEnvironment
from rrt_planner.RRTPlannerBase import RRTPlannerBase
from rrt_planner.PlanResult import PlanResult


class RRTPlannerNonholonomic(RRTPlannerBase):
    def __init__(
            self,
            planning_env: CarEnvironment,
            seed: int,
            bias: float = 0.05,
            max_iter: int = 10000,
            num_control_samples: int = 25):
        RRTPlannerBase.__init__(
            self,
            planning_env=planning_env,
            bias=bias,
            max_iter=max_iter,
            seed=seed)
        self.num_control_samples = num_control_samples  # Number of controls to sample
        self.control_tree = {}

    def plan(
            self,
            start_config: np.ndarray,
            goal_config: np.ndarray) -> PlanResult:
        # TODO: YOUR IMPLEMENTATION HERE
        self.control_tree[0] = (-1, (0,0))
        plan_time = time.time()

        # Start with adding the start configuration to the tree.
        self.tree.AddVertex(start_config)
        k = 0
        for i in range(self.max_iter):
            q_rand = self.sample(goal_config)                
            qid, q_near = self.nearestVertex(q_rand)
            q_new, q_new_cost, control = self.extend(q_near, q_rand)
            if (self.env.state_validity_checker(q_new) and
                self.env.edge_validity_checker(q_near, q_new)):
                k += 1
                self.tree.AddVertex(q_new,  cost = self.tree.costs[qid] + q_new_cost)
                self.tree.AddEdge(qid,k)
                self.control_tree[k] = (int(qid), control)
                print(control)
                if (self.env.goal_criterion(q_new, goal_config)):
                    break
                
        edges = self.tree.edges
        vertices = self.tree.vertices
        costs = self.tree.costs
        plan = []

        start, _ = self.nearestVertex(goal_config)
        cost = costs[start]
        while (start != 0):
            # plan.append(vertices[start])
            if start in self.control_tree.keys():
                # print(self.control_tree[start].shape)
                plan.append(self.control_tree[start])
            start = edges[start]
        plan.append((start_config, (0, 0)))
        plan = plan[::-1]
        plan_time = time.time() - plan_time
        print("Cost: %f" % cost)
        print("Planning Time: %fs" % plan_time)
        # print(plan)

        plan_result = PlanResult(
            plan=plan,
            cost=cost,
            time=plan_time)
        return plan_result
        # print(plan_result)
        # return PlanResult(
            # plan=np.concatenate(plan, axis=2),
            # cost=cost,
            # time=plan_time)

    def extend(
            self,
            x_near: np.ndarray,
            x_rand: np.ndarray) -> Any:
        """ Extend method for non-holonomic RRT

            Generate n control samples, with n = self.num_control_samples
            Simulate trajectories with these control samples
            Compute the closest closest trajectory and return the resulting state (and cost)
        """
        # TODO: YOUR IMPLEMENTATION HERE

        x_chosen = None
        min_dist = None
        min_cost = None

        for i in range(self.num_control_samples):
            linear_vel, steer_angle = self.env.sample_action()
            x_new, cost = self.env.simulate_car(x_near, x_rand, linear_vel, steer_angle)
            if x_new is not None:
                dist = self.env.compute_distance(x_new, x_rand)
                if x_chosen is None:
                    x_chosen = x_new
                    min_dist = dist
                    min_cost = cost
                elif min_dist > dist:
                    x_chosen = x_new
                    min_dist = dist
                    min_cost = cost


        return x_chosen, min_cost, (linear_vel, steer_angle)

    def nearestVertex(self, config):
        vertices = np.squeeze(np.array(self.tree.vertices), axis=-1).T
        min_idx = self.env.compute_distance(vertices, config.reshape((3,1))).argmin()
        return min_idx, self.tree.vertices[min_idx]