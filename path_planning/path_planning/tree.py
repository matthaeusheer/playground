import math
from dataclasses import dataclass

import numpy as np

from typing import Generator


@dataclass
class Pose:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other: 'Pose') -> float:
        return math.sqrt((self.x - other.x) ** 2 + (self.y - other.y) ** 2)

    @staticmethod
    def subtract(p1: 'Pose', p2: 'Pose') -> 'Pose':
        return Pose(p2.x - p1.x, p2.y - p1.y)

    @staticmethod
    def add(p1: 'Pose', p2: 'Pose') -> 'Pose':
        return Pose(p1.x + p2.x, p1.y + p2.y)

    @staticmethod
    def normalize(p: 'Pose') -> 'Pose':
        norm = Pose.distance_to(p, Pose(0, 0))
        return Pose(p.x/norm, p.y/norm)

    def __str__(self) -> str:
        return f'{self.x:.1f}, {self.y:.1f}'


class Node:
    def __init__(self, pose, children=None, parent=None):
        self.pose = pose
        self.children = children if children is not None else []
        self.parent = parent

    def traverse(self) -> Generator['Node', None, None]:
        yield self
        if self.children is not None:
            for child in self.children:
                yield from child.traverse()

    def distance_to(self, other: 'Node') -> float:
        return self.pose.distance_to(other.pose)

    def num_elements(self) -> int:
        return sum(1 for _ in self.traverse())

    def to_array(self) -> np.ndarray:
        n_nodes = self.num_elements()
        array = np.empty((n_nodes, 2))
        for idx, node in enumerate(self.traverse()):
            array[idx][0] = node.pose.x
            array[idx][1] = node.pose.y
        return array

    def adjacency_nodes(self) -> np.ndarray:
        adjacency_list = []  # [idx1, idx2] for adjacent nodes with indices idx1 and idx2
        for idx1, node1 in enumerate(self.traverse()):
            for idx2, node2 in enumerate(self.traverse()):
                if node1 is node2.parent:
                    adjacency_list.append([idx1, idx2])
        return np.array(adjacency_list)

    def __str__(self) -> str:
        kids_str = ', '.join([f'Node({kid.pose})' for kid in self.children])
        parent_str = 'None' if self.parent is None else f'Node({self.parent.pose})'
        return f'Node({self.pose}), children: [{kids_str}], parent: {parent_str})'
