import random
import time

import matplotlib.pyplot as plt

from path_planning.config import RrtConfig
from path_planning.geometry import intersects
from path_planning.tree import Node, Pose
from path_planning.visualization import init_plot, plot_tree
from path_planning.obstacle import Obstacle

from typing import Optional

DEFAULT_OBSTACLES = [Obstacle(-10, 40, 60, 40),
                     Obstacle(60, 10, 50, 20),
                     Obstacle(-10, -10, 5, 20),
                     Obstacle(65, 86, 5, 5),
                     Obstacle(60, 50, 20, 30),
                     Obstacle(80, 90, 10, 20)]


class SimulationState:
    """Container class holding the state of the path finding algorithm."""

    def __init__(self, cfg: RrtConfig, obstacles: list[Obstacle] = None) -> None:
        self._cfg: RrtConfig = cfg
        self.ax: plt.Axes = init_plot(cfg)
        self.rr_tree: Node = cfg.start_node
        self.obstacles: list[Obstacle] = obstacles if obstacles else DEFAULT_OBSTACLES
        self.step_counter: int = 0
        self.global_closest: Node = cfg.start_node
        self.global_closest_dist: float = 1e6
        self.running: bool = True

    def update(self, new_node: Node) -> None:
        dist_to_end = new_node.distance_to(self._cfg.end_node)
        if dist_to_end < self.global_closest_dist:
            self.global_closest_dist = dist_to_end
            self.global_closest = new_node
        print(f'[{self.step_counter}] (new node / closest global) = '
              f'({dist_to_end:.2f} / global {self.global_closest_dist:.2f})')
        if new_node.distance_to(self._cfg.end_node) < self._cfg.eps:
            self.running = False
            print('Target found or steps done!')
            plot_tree(self.ax, self.rr_tree, self._cfg.grid_size, self._cfg.start_node, self._cfg.end_node,
                      obstacles=self.obstacles, reached_target=True, last_node=new_node, fast_plot=self._cfg.fast_plot)
            input()
        elif self.step_counter == self._cfg.max_steps:
            print('Max steps reached - target not found.')
            self.running = False
            input()
        else:
            self.step_counter += 1
            start_time = time.time()
            plot_tree(self.ax, self.rr_tree, self._cfg.grid_size, self._cfg.start_node, self._cfg.end_node,
                      obstacles=self.obstacles, fast_plot=self._cfg.fast_plot)
            print(f'\tplotting took {time.time() - start_time:.2f}')


def rrt_path_finder(cfg: RrtConfig, simulation_state: SimulationState, obstacles: list[Obstacle] = None):
    """Main RRT path planning loop."""
    if obstacles is None:
        obstacles = DEFAULT_OBSTACLES

    rr_tree = simulation_state.rr_tree
    while simulation_state.running:
        sampled_node = sample_new_node(obstacles, cfg.grid_size)
        nearest_node = find_closest_node(sampled_node, rr_tree, obstacles)
        if nearest_node is not None:
            new_node = insert_new_node(nearest_node, sampled_node, cfg.clamp_dist)
            simulation_state.update(new_node)


def sample_new_node(obstacles: list[Obstacle], grid_size: int) -> Node:
    while True:
        new_node = Node(Pose(x=random.uniform(0, grid_size), y=random.uniform(0, grid_size)))
        if not any([obs.contains(new_node.pose) for obs in obstacles]):
            return new_node


def find_closest_node(new_node: Node, rr_tree: Node, obstacles: list[Obstacle]) -> Optional[Node]:
    """For a new node, find the nearest node in the tree. Return this nearest node if there is an obstacle-free
    connection between the new and nearest node, else return None."""
    closest_node = None
    closest_distance = 1e6
    for node in rr_tree.traverse():
        distance = new_node.distance_to(node)
        if distance < closest_distance:
            closest_node = node
            closest_distance = distance
    all_object_lines = []
    for obs in obstacles:
        all_object_lines.extend(obs.segments)
    seg_to_closest = [(new_node.pose.x, new_node.pose.y), (closest_node.pose.x, closest_node.pose.y)]
    if any([intersects(seg, seg_to_closest) for seg in all_object_lines]):
        return None
    else:
        return closest_node


def insert_new_node(nearest_node: Node, new_node: Node, clamp_distance: float) -> Node:
    """If a nearest node has been found, insert the new node into the tree appropriately and also return the Node."""
    distance_to_new = new_node.distance_to(nearest_node)
    if distance_to_new > clamp_distance:
        dist_vec = Pose.normalize(Pose.subtract(nearest_node.pose, new_node.pose))
        new_pose = Pose.add(nearest_node.pose, Pose(dist_vec.x * clamp_distance, dist_vec.y * clamp_distance))
        new_node = Node(new_pose)
    nearest_node.children.append(new_node)
    new_node.parent = nearest_node
    return new_node
