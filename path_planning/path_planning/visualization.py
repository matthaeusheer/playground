import time

from matplotlib import pyplot as plt, patches as patches

from path_planning.config import RrtConfig
from path_planning.tree import Node
from path_planning.obstacle import Obstacle


def init_plot(cfg: RrtConfig) -> plt.Axes:
    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-cfg.grid_size * 0.2, cfg.grid_size * 1.2)
    ax.set_ylim(-cfg.grid_size * 0.2, cfg.grid_size * 1.2)
    ax.set_aspect('equal')
    ax.grid(alpha=0.4, linestyle='--')
    ax.get_xaxis().set_visible(False)
    ax.get_yaxis().set_visible(False)
    return ax


def plot_tree(ax, rr_tree: Node, grid_size: int, start: Node, end: Node,
              reached_target: bool = False, last_node: Node = None, obstacles: list[Obstacle] = None,
              fast_plot: bool = False) -> None:
    # Add obstacles to the Axes
    if obstacles is not None:
        for obs in obstacles:
            rect = patches.Rectangle((obs.x, obs.y), obs.width, obs.height,
                                     linewidth=1, edgecolor='none', facecolor='gray')
            ax.add_patch(rect)

    ax.set_xlim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_ylim(-grid_size * 0.2, grid_size * 1.2)
    ax.set_aspect('equal')
    ax.grid()

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
        ax.plot(connection[0], connection[1], 'k--', linewidth=0.5, zorder=8)
    # Draw the nodes themselves
    x_nodes = [node.pose.x for node in nodes]
    y_nodes = [node.pose.y for node in nodes]
    if fast_plot:  # use plt.plot instead of plt.scatter which is super slow
        ax.plot(x_nodes, y_nodes, 'b.', zorder=10)
    else:
        colors = [end.distance_to(node) for node in nodes]
        ax.scatter(x_nodes, y_nodes, c=colors, s=15, vmin=0, vmax=90, zorder=10)
    ax.scatter([start.pose.x], [start.pose.y], c='red', s=60, zorder=11)
    ax.scatter([end.pose.x], [end.pose.y], c='green', s=100, zorder=11)
    if reached_target:
        # Draw the actual path from goal to start
        node = last_node
        while node.parent is not None:
            ax.plot([node.pose.x, node.parent.pose.x], [node.pose.y, node.parent.pose.y], 'r-')
            plt.draw()
            plt.pause(1e-17)
            node = node.parent
            time.sleep(0.05)
    plt.draw()
    plt.pause(1e-17)
    ax.clear()
