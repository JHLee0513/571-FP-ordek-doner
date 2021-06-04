import numpy as np

from rrt_planner.environments.EnvironmentBase import EnvironmentBase
from rrt_planner.PlanResult import PlanResult


class PlannerBase:
    def __init__(self, planning_env: EnvironmentBase):
        self.env = planning_env

    def plan(
            self,
            start_config: np.ndarray,
            goal_config: np.ndarray) -> PlanResult:
        raise NotImplementedError
