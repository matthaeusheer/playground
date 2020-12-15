from dataclasses import dataclass

from path_planning.tree import Node, Pose


@dataclass
class RrtConfig:
    grid_size: int = 100
    start_node: Node = Node(Pose(10, 10))
    end_node: Node = Node(Pose(90, 65))
    eps: float = 3
    max_steps: int = 1000
    clamp_dist: float = 4
    fast_plot: bool = False
