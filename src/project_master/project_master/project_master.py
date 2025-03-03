import rclpy
from rclpy.node import Node
from project_interface.srv import GoToPoint

def ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

def main():
    rclpy.init()
    node = Pathing()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
