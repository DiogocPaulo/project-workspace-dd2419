import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint, Trigger

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

def main():
    rclpy.init()
    node = ProjectMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
