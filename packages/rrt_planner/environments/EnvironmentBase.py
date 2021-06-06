from typing import Optional

import numpy as np
from matplotlib import pyplot as plt
from PIL import Image
from rrt_planner.RRTTree import RRTTree


class EnvironmentBase:
    def __init__(
            self,
            map_file: str,
            state_dim: int,
            start: np.ndarray,
            goal: np.ndarray,
            seed: int):
        self.map_file = map_file
        self.start = start
        self.goal = goal
        self.state_dim = state_dim
        self.seed = seed

        self.rng = np.random.default_rng(seed)

        # Obtain the boundary limits
        # Check if file exists
        # self.map = np.loadtxt(map_file)
        self.map = 1 - np.asarray(Image.open(map_file)) / 254
        self.map = np.rot90(self.map)
        # print(self.map.max())
        self.xlimit = [0, np.shape(self.map)[0] - 1]
        self.ylimit = [0, np.shape(self.map)[1] - 1]
        # print(self.xlimit, self.ylimit)

        # Variables for the visualizer
        self.fig = None
        self.ax1 = None
        self.ax1_img = None

    def check_start_and_goal_validity(self):
        # Check if start and goal are within limits and collision free
        if not self.state_validity_checker(self.start) or not self.state_validity_checker(self.goal):
            raise ValueError('Start and Goal state must be within the map limits')
            exit(0)

    def sample(self) -> np.ndarray:
        """ Draw a random sample point from the map

            @return: a numpy array of the randomly drawn state
        """
        raise NotImplementedError

    def goal_criterion(
            self,
            config: np.ndarray,
            goal_config: np.ndarray) -> bool:
        """ Return True if config is close enough to goal

            @param config: a [d x 1] numpy array of a state
            @param goal_config: a [d x 1] numpy array of goal state
        """
        raise NotImplementedError

    def compute_distance(
            self,
            start_config: np.ndarray,
            end_config: np.ndarray) -> float:
        """ A function which computes the distance between two configurations

            @param start_config: a [d x 1] numpy array of current state
            @param end_config: a [d x 1] numpy array of goal state
        """
        raise NotImplementedError

    def state_validity_checker(
            self,
            config: np.ndarray) -> bool:
        """ Check validity of state
            Return True if all states are valid

            @param config: = a [d x n] numpy array of n states.
        """
        raise NotImplementedError

    def edge_validity_checker(
            self,
            config1: np.ndarray,
            config2: np.ndarray) -> bool:
        """ Return True if edge is valid

            @param config1: a [d x 1] numpy array of state
            @param config2: a [d x 1] numpy array of state
        """
        assert (config1.shape == (self.state_dim, 1))
        assert (config2.shape == (self.state_dim, 1))
        n = max(self.xlimit[1], self.ylimit[1])
        x_vals = np.linspace(config1[0], config2[0], n).reshape(1, n)
        y_vals = np.linspace(config1[1], config2[1], n).reshape(1, n)
        configs = np.vstack((x_vals, y_vals))
        return self.state_validity_checker(configs)

    def init_visualizer(self):
        """
            Initialize visualizer
        """
        self.fig = plt.figure()
        self.ax1 = self.fig.add_subplot(1, 1, 1)

        # plot image
        visit_map = 1 - np.copy(self.map)  # black is obstacle, white is free space
        self.ax1_img = self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

    def visualize_plan(
            self,
            plan: Optional[np.ndarray] = None,
            tree: Optional[RRTTree] = None,
            visited=None):
        """
            Visualize the final path
            @param plan: a [d x n] numpy array of states
        """
        raise NotImplementedError
