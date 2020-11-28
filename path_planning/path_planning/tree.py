import math
from dataclasses import dataclass

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


class Node:
    def __init__(self, pose, children=None, parent=None):
        self.pose = pose
        self.children = children if children is not None else []
        self.parent = parent

    def traverse(self) -> Generator['Node', None, None]:
        if self.children is not None:
            for child in self.children:
                yield from child.traverse()
        yield self

    def distance_to(self, other: 'Node') -> float:
        return self.pose.distance_to(other.pose)
