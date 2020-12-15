import sys
from dataclasses import dataclass
import random
import time
from functools import partial

import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np

from path_planning.tree import Node, Pose
from path_planning.obstacle import Obstacle
from path_planning import pg_utils


@dataclass
class RrtConfig:
    grid_size: int = 100
    start_node: Node = Node(Pose(10, 10))
    end_node: Node = Node(Pose(90, 65))
    eps: float = 3
    max_steps: int = 1000
    clamp_dist: float = 3
    fast_plot: bool = True


pg.setConfigOptions(antialias=True)
pg.setConfigOption('background', 'w')
app = pg.mkQApp()

view = pg.PlotWidget()
view.resize(800, 600)
view.setWindowTitle('RRT Path Finder')
view.setAspectLocked(True)

graph_item = pg.GraphItem()
view.setXRange(0, 100)
view.setYRange(0, 100)
view.addItem(graph_item)

start_end_item = pg.ScatterPlotItem()

cfg = RrtConfig()
start_end_points = [(cfg.start_node.pose.x, cfg.start_node.pose.y),
                    (cfg.end_node.pose.x, cfg.end_node.pose.y)]
start_end_item.setData(pos=start_end_points)
start_end_item.setPen([QtGui.QPen(QtGui.QColor('green')), QtGui.QPen(QtGui.QColor('red'))])
view.addItem(start_end_item)

obstacles = [Obstacle(-10, 40, 60, 40),
             Obstacle(60, 10, 50, 20),
             Obstacle(-10, -10, 5, 20),
             Obstacle(65, 86, 5, 5),
             Obstacle(60, 50, 20, 30),
             Obstacle(80, 90, 10, 20)]

obstacle_items = []
if obstacles is not None:
    for obs in obstacles:
        rect = pg_utils.RectangleItem([obs.x, obs.y],
                                      [obs.width, obs.height])
        view.addItem(rect)

view.show()


def rrt_path_finder(cfg: RrtConfig):
    rr_tree = cfg.start_node

    counter = 1
    found_target = False
    global_closest = 1e6
    while not found_target:
        last_time = time.time()
        not_valid_sample = True
        while not_valid_sample:
            new_node = Node(Pose(x=random.uniform(0, cfg.grid_size),
                                 y=random.uniform(0, cfg.grid_size)))
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

        distance_to_new = new_node.distance_to(closest_node)
        if distance_to_new > cfg.clamp_dist:
            dist_vec = Pose.normalize(Pose.subtract(closest_node.pose, new_node.pose))
            new_pose = Pose.add(closest_node.pose, Pose(dist_vec.x * cfg.clamp_dist, dist_vec.y * cfg.clamp_dist))
        else:
            new_pose = new_node.pose
        new_node = Node(new_pose)
        closest_node.children.append(new_node)
        new_node.parent = closest_node
        dist_to_end = new_node.distance_to(cfg.end_node)
        global_closest = dist_to_end if dist_to_end < global_closest else global_closest
        print(f'[{counter}] (new node / closest global) =  ({closest_distance:.2f} / global {global_closest:.2f})')
        if new_node.distance_to(cfg.end_node) < cfg.eps or counter == cfg.max_steps:
            found_target = True
            print('Target found or steps done!')
            plot_tree(rr_tree, end_node=cfg.end_node, reached_target=True)
            input()
            sys.exit(0)
        counter += 1
        plot_tree(rr_tree, end_node=cfg.end_node, reached_target=False)
        print(f'\ttook {time.time() - last_time:.2f}s')


def plot_tree(rr_tree: Node, end_node: Node, reached_target: bool = False) -> None:
    global app
    plot_start = time.time()
    nodes = rr_tree.to_array()
    connections = rr_tree.adjacency_nodes()
    # cmap = pg.ColorMap(np.array([0, 100]), np.array([pg.mkColor('r'), pg.mkColor('b')]))
    # distances = [end_node.distance_to(node) for node in rr_tree.traverse()]
    # print(distances)
    # brushes = [pg.mkBrush(col) for col in cmap.map(distances)]
    graph_item.setData(pos=nodes, adj=np.array(connections))
    app.processEvents()
    print(f'\tPlotting took {time.time() - plot_start:.2f}s')


def intersects(s0, s1) -> bool:
    """Assumes line segments are stored in the format [(x0,y0),(x1,y1)]"""
    dx0 = s0[1][0] - s0[0][0]
    dx1 = s1[1][0] - s1[0][0]
    dy0 = s0[1][1] - s0[0][1]
    dy1 = s1[1][1] - s1[0][1]
    p0 = dy1 * (s1[1][0] - s0[0][0]) - dx1 * (s1[1][1] - s0[0][1])
    p1 = dy1 * (s1[1][0] - s0[1][0]) - dx1 * (s1[1][1] - s0[1][1])
    p2 = dy0 * (s0[1][0] - s1[0][0]) - dx0 * (s0[1][1] - s1[0][1])
    p3 = dy0 * (s0[1][0] - s1[1][0]) - dx0 * (s0[1][1] - s1[1][1])
    return (p0 * p1 <= 0) & (p2 * p3 <= 0)


if __name__ == '__main__':
    timer = QtCore.QTimer()
    timer.timeout.connect(partial(rrt_path_finder, RrtConfig()))
    timer.start(0)

    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
