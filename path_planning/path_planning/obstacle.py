from path_planning.tree import Pose


class Obstacle:
    def __init__(self, x, y, width, height) -> None:
        """A rectangle obstacle."""
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def contains(self, pose: Pose) -> bool:
        if pose.x < self.x or pose.x > self.x + self.width:
            return False
        if pose.y < self.y or pose.y > self.y + self.height:
            return False
        return True

    @property
    def segments(self) -> list[list[tuple]]:
        """Return a list of all four segments of the rectangle, each segment is list of start and end position tuple."""
        return [[(self.x, self.y), (self.x + self.width, self.y)],
                [(self.x + self.width, self.y), (self.x + self.width, self.y + self.height)],
                [(self.x + self.width, self.y + self.height), (self.x, self.y + self.height)],
                [(self.x, self.y + self.height), (self.x, self.y)]]
