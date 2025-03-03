import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

        self.point_client = self.create_client(GoToPoint, "/navigation_point")
        while not self.point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GoToPoint service not yet avaliable, waiting ...")
        self.point_request = GoToPoint.Request()

    def send_end_point(self, x, y, yaw):
        self.point_request.x = x
        self.point_request.y = y
        self.point_request.yaw = yaw

        self.point_future = self.point_client.call_async(self.point_request)
        rclpy.spin_until_future_complete(self, self.point_future)
        if self.point_future.result() is not None:
            response = self.point_future.result()
            self.get_logger().info(f"GoToPoint Response: {response.message}")
        else:
            self.get_logger().info("GoToPoint service failed")

        


def main():
    rclpy.init()
    node = ProjectMaster()

    node.send_end_point(-1.5, 0.5, 1.5)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
