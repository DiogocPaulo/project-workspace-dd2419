import py_trees
import py_trees_ros
import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint, Trigger

from project_master import behaviours

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

def main():
    rclpy.init()
    node = ExploreMaster()
    end_points = [
        (-1.6, 0.9, 0.0),
        (-1.6, -0.9, 0.0),
        (1.6, -0.9, 0.0),
        (1.6, 0.9, 0.0),
    ]

    root = behaviours.EndPointSelector("Exploration", end_points)
    behaviour_tree = py_trees.trees.BehaviourTree(root)
    behaviour_tree.setup(timeout=15)
    rate = node.create_rate(10)

    try:
        while rclpy.ok():
            rclpy.spin_once(tree_node, timeout_sec=0.1)
            behaviour_tree.tick()
            rate.sleep()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
