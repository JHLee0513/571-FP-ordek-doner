from typing import Any

import numpy as np

from rrt_planner.environments.EnvironmentBase import EnvironmentBase
from rrt_planner.PlannerBase import PlannerBase
from rrt_planner.RRTTree import RRTTree


class RRTPlannerBase(PlannerBase):
    def __init__(
            self,
            planning_env: EnvironmentBase,
            bias: float,
            max_iter: int,
            seed: int):
        PlannerBase.__init__(
            self,
            planning_env=planning_env)
        self.tree = RRTTree(self.env)
        self.bias = bias
        self.max_iter = max_iter
        self.seed = seed
        self.rng = np.random.default_rng(seed)

    def extend(
            self,
            x_near: np.ndarray,
            x_rand: np.ndarray) -> Any:
        """ Extend a state from two given states
            @param x_near:
            @param x_rand:
        """
        raise NotImplementedError

    def sample(self, goal: np.ndarray) -> np.ndarray:
        """ Draw a random sample point from the map

            @return: a numpy array of the randomly drawn state
        """
        if self.rng.random() < self.bias:
            return goal

        return self.env.sample()
