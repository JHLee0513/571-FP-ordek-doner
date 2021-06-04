import numpy as np


class PlanResult:
    def __init__(
            self,
            plan: np.ndarray,
            cost: float,
            num_states: int = -1,
            time: float = 1e10):
        self.plan = plan
        self.cost = cost
        self.num_states = num_states
        self.time = time
