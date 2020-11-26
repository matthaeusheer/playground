import math
import time
from dataclasses import dataclass, field

from typing import Generator


@dataclass
class Pose:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def distance_to(self, other: 'Pose') -> float:
        return math.sqrt((self.x-other.x)**2 + (self.y-other.y)**2)


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
