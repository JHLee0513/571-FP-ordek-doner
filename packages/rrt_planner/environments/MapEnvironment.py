from typing import Optional

import numpy as np
from matplotlib import pyplot as plt

from environments.EnvironmentBase import EnvironmentBase
from planners.RRTTree import RRTTree


class MapEnvironment(EnvironmentBase):
    def __init__(
            self,
            map_file: str,
            start: np.ndarray,
            goal: np.ndarray,
            seed: int,
            epsilon: float = 0.01):
        EnvironmentBase.__init__(
            self,
            map_file=map_file,
            state_dim=2,
            start=start,
            goal=goal,
            seed=seed)
        self.epsilon = epsilon

        self.check_start_and_goal_validity()
        self.clear = np.argwhere(self.map == 0)

    def sample(self) -> np.ndarray:
        """ Sample random clear point from the map

        @return: numpy array of sampled point
        """
        
        idx = self.rng.choice(len(self.clear))
        return self.clear[idx, :].reshape((2, 1))

    def goal_criterion(
            self,
            config: np.ndarray,
            goal_config: np.ndarray) -> bool:
        """ Return True if config is close enough to goal

            @param config: a [2 x 1] numpy array of a state
            @param goal_config: a [2 x 1] numpy array of goal state
        """
        return self.compute_distance(config, goal_config) < self.epsilon

    def compute_distance(
            self,
            start_config: np.ndarray,
            end_config: np.ndarray) -> float:
        """ A function which computes the distance between
            two configurations. 

            @param start_config: a [2 x 1] numpy array of current state
            @param end_config: a [2 x 1] numpy array of goal state
        """
        # TODO: YOUR IMPLEMENTATION HERE
        # return np.sqrt(((start_config - end_config)**2).sum())
        return np.linalg.norm(start_config - end_config)

    def state_validity_checker(
            self,
            config: np.ndarray) -> bool:
        """ Check validity of state
            Return True if all states are valid

            @param config: = a [2 x n] numpy array of n states.
        """
        # TODO: YOUR IMPLEMENTATION HERE

        if (0 > config[0, :]).any() or (config[0, :] >= self.map.shape[0]).any():
            return False
        if (0 > config[1, :]).any() or (config[1, :] >= self.map.shape[1]).any():
            return False

        if (self.map[config.astype(np.int)[0, :], config.astype(np.int)[1, :]] == 1).any():
            return False
        
        return True

    def h(
            self,
            config: np.ndarray):
        """ Heuristic function for A*

            @param config: a [2 x 1] numpy array of state
        """
        # TODO: YOUR IMPLEMENTATION HERE
        return self.compute_distance(config, self.goal)

    def visualize_plan(
            self,
            plan: Optional[np.ndarray] = None,
            tree: Optional[RRTTree] = None,
            visited=None):
        """
            Visualize the final path
            @param plan: a final [2 x n] numpy array of states
        """
        visit_map = 1 - np.copy(self.map)  # black is obstacle, white is free space

        self.ax1.cla()

        if visited is not None:
            visit_map[visited == 1] = 0.5
        self.ax1.imshow(visit_map, interpolation="nearest", cmap="gray")

        if tree is not None:
            for idx in range(len(tree.vertices)):
                if idx == tree.GetRootID():
                    continue
                econfig = tree.vertices[idx]
                sconfig = tree.vertices[tree.edges[idx]]
                x = [sconfig[0], econfig[0]]
                y = [sconfig[1], econfig[1]]
                self.ax1.plot(y, x, 'r')

        if plan is not None:
            for i in range(np.shape(plan)[1] - 1):
                x = [plan[0, i], plan[0, i + 1]]
                y = [plan[1, i], plan[1, i + 1]]
                plt.plot(y, x, 'b', linewidth=3)
                self.fig.canvas.draw()
                plt.pause(.025)

        self.fig.canvas.draw()
        plt.pause(1e-10)
