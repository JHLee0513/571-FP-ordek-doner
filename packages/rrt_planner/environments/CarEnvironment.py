from typing import Optional

import numpy as np
from matplotlib import pyplot as plt

from rrt_planner.environments.EnvironmentBase import EnvironmentBase
from rrt_planner.RRTTree import RRTTree


class CarEnvironment(EnvironmentBase):
    """ Car Environment. Car is represented as a circular robot.

        Robot state: [x, y, theta]
    """

    def __init__(
            self,
            map_file: str,
            start: np.ndarray,
            goal: np.ndarray,
            seed: int,
            radius=15,
            delta_step=10,
            max_linear_vel=20,
            max_steer_angle=1.):
        EnvironmentBase.__init__(
            self,
            map_file=map_file,
            state_dim=3,
            start=start,
            goal=goal,
            seed=seed)
        self.radius = radius

        self.delta_step = delta_step  # Number of steps in simulation rollout
        self.max_linear_vel = max_linear_vel
        self.max_steer_angle = max_steer_angle

        self.check_start_and_goal_validity()

    def sample(self) -> np.ndarray:
        """ Sample random clear point from the map

        @return: numpy array of sampled point
        """
        clear = np.argwhere(self.map == 0)
        idx = self.rng.choice(len(clear))
        xy = clear[idx, :].reshape((2, 1))
        theta = self.rng.uniform(0, 2 * np.pi)
        return np.concatenate([xy, np.array([[theta]])])

    def sample_action(self):
        # Sample random direction of motion and random steer angle
        linear_vel = (0.5 + 0.5 * self.rng.random()) * self.max_linear_vel
        if self.rng.random() > 0.5:
            linear_vel = -1 * linear_vel
        steer_angle = (2 * self.rng.random() - 1) * self.max_steer_angle  # uniformly distributed
        # print(linear_vel, steer_angle)
        return linear_vel, steer_angle

    def simulate_car(self, x_near, x_rand, linear_vel, steer_angle):
        """ Simulates a given control from the nearest state on the graph to the random sample.

            @param x_near: a [3 x 1] numpy array. Nearest point on the current graph to the random sample
            @param x_rand: a [3 x 1] numpy array. Random sample
            @param linear_vel: a Python float. Linear velocity of the car (control)
            @param steer_angle: a Python float. Steering angle of the car (control)

            @return: x_new: a [3 x 1] numpy array
                     delta_t: a Python float. Time required to execute trajectory
        """

        # Set vehicle constants
        dt = 0.1  # Step by 0.1 seconds
        L = 7.5  # Car length

        # Simulate forward from xnear using the controls (linear_vel, steer_angle) to generate the rollout
        x = x_near
        rollout = [x]
        for i in range(self.delta_step):
            x_rollout = np.zeros_like(x)
            x_rollout[0] = x[0] + linear_vel * np.cos(x[2]) * dt
            x_rollout[1] = x[1] + linear_vel * np.sin(x[2]) * dt
            x_rollout[2] = x[2] + (linear_vel / L) * np.tan(steer_angle) * dt
            x_rollout[2] = x_rollout[2] % (2 * np.pi)

            x = x_rollout
            rollout.append(x_rollout)  # maintain history
        rollout = np.concatenate(rollout, axis=1)  # Shape: [3 x delta_step]

        # Find the closest point to x_rand on the rollout
        # This is x_new. Discard the rest of the rollout
        min_ind = np.argmin(self.compute_distance(rollout, x_rand))
        x_new = rollout[:, min_ind].reshape(3, 1)
        rollout = rollout[:, :min_ind + 1]  # don't need the rest
        delta_t = rollout.shape[1]

        # Check for validity of the path
        if self.state_validity_checker(rollout):
            return x_new, delta_t
        else:
            return None, None

    def angular_difference(
            self,
            start_config: np.ndarray,
            end_config: np.ndarray) -> float:
        """ Compute angular difference

            @param start_config: a [3 x n] numpy array of states
            @param end_config: a [3 x 1] numpy array of goal state
        """
        th1 = start_config[2, :]
        th2 = end_config[2, :]
        ang_diff = th1 - th2
        ang_diff = ((ang_diff + np.pi) % (2 * np.pi)) - np.pi
        ang_diff = ang_diff * (180 / np.pi)  # convert to degrees
        return ang_diff

    def compute_distance(
            self,
            start_config: np.ndarray,
            end_config: np.ndarray) -> float:
        """ Distance function: alignment, xy distance, angular difference
                - alignment: measure of how far start_config is from line defined by end_config
                             similar to projection of start_config xy position end_config line

            @param start_config: a [3 x n] numpy array of states
            @param end_config: a [3 x 1] numpy array of goal state
        """

        ang_diff = np.abs(self.angular_difference(start_config, end_config)) / 180
        e_to_s = start_config[:2, :] - end_config[:2, :]  # Shape: [2 x n]
        euclidean_distance = np.linalg.norm(e_to_s, axis=0)  # Shape: [n]
        e_to_s = e_to_s / euclidean_distance[None, :]
        e_vec = np.array([np.cos(end_config[2, 0]), np.sin(end_config[2, 0])])
        alignment = 1 - np.abs(e_vec.dot(e_to_s))  # Shape: [n]

        # alignment is in [0,1], euclidean_distance can be large, ang_diff is between [0,1]
        return 50 * alignment + euclidean_distance + 50 * ang_diff

        # New distance function for experiment
        # ang_diff = np.abs(self.angular_difference(start_config, end_config))
        # return np.sqrt(np.linalg.norm(start_config[:2,:] - end_config[:2,:], axis=0)**2 + ang_diff**2)

    def goal_criterion(
            self,
            config: np.ndarray,
            goal_config: np.ndarray) -> bool:
        """ Return True if config is close enough to goal

            @param config: a [3 x 1] numpy array of a state
            @param goal_config: a [3 x 1] numpy array of goal state
        """
        if np.linalg.norm(config[:2, :] - goal_config[:2, :]) < 10 and \
                np.abs(self.angular_difference(config, goal_config)) < 5:
            print(f'Goal reached! State: {config[:, 0]}, Goal state: {goal_config[:, 0]}')
            print(f'xy_diff: {np.linalg.norm(config[:2, :] - goal_config[:2, :]):.03f}, ' \
                  f'ang_diff: {np.abs(self.angular_difference(config, goal_config))[0]:.03f}')
            return True
        else:
            return False

    def out_of_limits_violation(
            self,
            config: np.ndarray) -> bool:
        """ Check limit violations

            @param config: a [d x n] numpy array of n states
        """
        out_of_limits = np.stack([config[0, :] <= self.radius,
                                  config[0, :] >= (self.xlimit[1] - self.radius),
                                  config[1, :] <= self.radius,
                                  config[1, :] >= (self.ylimit[1] - self.radius)])
        return np.any(out_of_limits)

    def collision_violation(
            self,
            config: np.ndarray) -> bool:
        """ Check whether car is in collision with obstacle

            @param config: a [d x n] numpy array of n states
        """
        theta = np.linspace(0, 2 * np.pi, 50)
        xs = self.radius * np.cos(theta) + config[0, :, None]  # Shape: [n x 50]
        ys = self.radius * np.sin(theta) + config[1, :, None]  # Shape: [n x 50]
        xys = np.stack([xs, ys], axis=0)  # Shape: [2 x n x 50]

        cfloor = np.round(xys.reshape(2, -1)).astype("int")
        try:
            values = self.map[cfloor[0, :], cfloor[1, :]]
        except IndexError:
            return False

        return np.sum(values) > 0

    def state_validity_checker(
            self,
            config: np.ndarray) -> bool:
        """ Check validity of state
            Return True if all states are valid

            @param config: = a [d x n] numpy array of n states.
        """
        valid_position = ~self.collision_violation(config)
        valid_limits = ~self.out_of_limits_violation(config)
        # print(valid_position, valid_limits)
        return valid_position and valid_limits

    def plot_car(
            self,
            config: np.ndarray):
        """ Plot the car

            @param config: a [3 x 1] numpy array
        """
        config = config[:, 0]

        # Plot car as a circle
        ax = plt.gca()
        circle1 = plt.Circle(config[:2][::-1], self.radius, fill=True, facecolor='w')
        circle2 = plt.Circle(config[:2][::-1], self.radius, fill=False, color='k')
        ax.add_artist(circle1)
        ax.add_artist(circle2)

        # Now plot a line for the direction of the car
        theta = config[2]
        ed = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]]) @ np.array([[self.radius * 1.5, 0]]).T
        ed = ed[:, 0]
        ax.plot([config[1], config[1] + ed[1]], [config[0], config[0] + ed[0]], 'b-', linewidth=3)

    def visualize_plan(
            self,
            plan: Optional[np.ndarray] = None,
            tree: Optional[RRTTree] = None,
            visited=None):
        """
            Visualize the final path
            @param plan: a [3 x n] numpy array of states
        """
        visit_map = 1 - np.copy(self.map)  # black is obstacle, white is free space
        self.ax1.cla()
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
            for i in range(np.shape(plan)[1]):
                self.plot_car(plan[:, i:i + 1])
                self.fig.canvas.draw()
                plt.pause(.025)

        self.fig.canvas.draw()
        plt.pause(1e-10)
        # plt.pause(60)
