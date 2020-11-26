from path_planning.tree import Node, Pose


def test_traversal():
    root = Node(Pose(1, 1), children=[Node(Pose(2, 2)),
                                      Node(Pose(3, 3), children=[Node(Pose(5, 5)),
                                                                 Node(Pose(4, 4))]),
                                      Node(Pose(6, 6))])
    for node in root.traverse():
        print(node)


if __name__ == '__main__':
    test_traversal()