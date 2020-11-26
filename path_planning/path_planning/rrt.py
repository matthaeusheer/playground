import random
import time

import matplotlib.pyplot as plt
import matplotlib.patches as patches

from path_planning.tree import Node, Pose


def rrt_path_finder():
    grid_size = 100
    start = Node(Pose(10, 10))
    end = Node(Pose(100, 100))
    eps = 3
    max_steps = 1000
    coarseness = 5
    obstacles = [Obstacle(-10, 40, 60, 40),
                 Obstacle(60, 10, 50, 20),
                 Obstacle(-10, -10, 5, 20),
                 Obstacle(65, 86, 5, 5),
                 Obstacle(60, 50, 20, 30),
                 Obstacle(80, 90, 10, 20)]

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_ylim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_aspect('equal')
    ax.grid(alpha=0.4)
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)

    rr_tree = start

    counter = 0
    found_target = False
    global_closest = 1e6
    while not found_target:
        not_valid_sample = True
        while not_valid_sample:
            new_node = Node(Pose(x=int(random.randint(0, grid_size // coarseness) * coarseness),
                                 y=int(random.randint(0, grid_size // coarseness)) * coarseness))
            if not any([obs.contains(new_node.pose) for obs in obstacles]):
                not_valid_sample = False
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
            continue
        closest_node.children.append(new_node)
        new_node.parent = closest_node
        global_closest = closest_distance if closest_distance < global_closest else global_closest
        print(f'[{counter}] (new node / closest global) =  ({closest_distance:.2f} / global {global_closest:.2f})')
        if new_node.distance_to(end) < eps or counter == max_steps:
            found_target = True
            print('Target found or steps done!')
            plot_tree(ax, rr_tree, grid_size, start, end, obstacles=obstacles,
                      reached_target=True, last_node=new_node)

            input()
        counter += 1
        plot_tree(ax, rr_tree, grid_size, start, end, obstacles=obstacles)


def plot_tree(ax, rr_tree: Node, grid_size: int, start: Node, end: Node,
              reached_target: bool = False, last_node: Node = None, obstacles: list['Obstacle'] = None) -> None:
    # Add the patch to the Axes
    if obstacles is not None:
        for obs in obstacles:
            rect = patches.Rectangle((obs.x, obs.y), obs.width, obs.height,
                                     linewidth=1, edgecolor='none', facecolor='black')
            ax.add_patch(rect)

    ax.set_xlim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_ylim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_aspect('equal')
    ax.grid()
    ax.scatter([start.pose.x], [start.pose.y], c='red', s=50)
    ax.scatter([end.pose.x], [end.pose.y], c='green', s=70)
    nodes = []
    connections = []

    for node in rr_tree.traverse():
        nodes.append(node)
        for child in node.children:
            x1, y1 = node.pose.x, node.pose.y
            x2, y2 = child.pose.x, child.pose.y
            connections.append([[x1, x2], [y1, y2]])

    # Draw connections to children
    for connection in connections:
        ax.plot(connection[0], connection[1], 'k--', linewidth=0.5)
    # Draw the nodes themselves
    x_nodes = [node.pose.x for node in nodes]
    y_nodes = [node.pose.y for node in nodes]
    colors = [end.distance_to(node) for node in nodes]
    ax.scatter(x_nodes, y_nodes, c=colors, s=10, cmap='jet')

    if reached_target:
        node = last_node
        while node.parent is not None:
            ax.plot([node.pose.x, node.parent.pose.x], [node.pose.y, node.parent.pose.y], 'r-')
            plt.draw()
            plt.pause(1e-17)
            node = node.parent
    plt.draw()
    plt.pause(1e-17)
    ax.clear()


class Obstacle:
    def __init__(self, x, y, width, height) -> None:
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
        return [[(self.x, self.y), (self.x+self.width, self.y)],
                [(self.x+self.width, self.y), (self.x+self.width, self.y+self.height)],
                [(self.x+self.width, self.y+self.height), (self.x, self.y+self.height)],
                [(self.x, self.y+self.height), (self.x, self.y)]]


def intersects(s0, s1) -> bool:
    """Assumes line segments are stored in the format [(x0,y0),(x1,y1)]"""
    dx0 = s0[1][0]-s0[0][0]
    dx1 = s1[1][0]-s1[0][0]
    dy0 = s0[1][1]-s0[0][1]
    dy1 = s1[1][1]-s1[0][1]
    p0 = dy1*(s1[1][0]-s0[0][0]) - dx1*(s1[1][1]-s0[0][1])
    p1 = dy1*(s1[1][0]-s0[1][0]) - dx1*(s1[1][1]-s0[1][1])
    p2 = dy0*(s0[1][0]-s1[0][0]) - dx0*(s0[1][1]-s1[0][1])
    p3 = dy0*(s0[1][0]-s1[1][0]) - dx0*(s0[1][1]-s1[1][1])
    return (p0*p1<=0) & (p2*p3<=0)

if __name__ == '__main__':
    rrt_path_finder()
    plt.show()
