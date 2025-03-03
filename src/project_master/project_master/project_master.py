import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint, Trigger

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

        self.point_client = self.create_client(GoToPoint, "/navigation_point")
        self.reached_destination_service = self.create_service(Trigger, "/reached_destination", self.reached_destination)
        while not self.point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")

        self.send_end_point(-1.5, 0.5, 1.5)

    def reached_destination(self, request, response):
        self.send_end_point(1.5, 0.5, 1.5)

        response.success = True
        response.message = "Received reached destination trigger"
        return response


    def send_end_point(self, x, y, yaw):
        request = GoToPoint.Request()
        request.x = x
        request.y = y
        request.yaw = yaw

        self.point_future = self.point_client.call_async(request)
        rclpy.spin_until_future_complete(self, self.point_future)
        if self.point_future.result() is not None:
            response = self.point_future.result()
            self.get_logger().debug(f"GoToPoint Response: {response.message}")
        else:
            self.get_logger().warn("GoToPoint service failed")

        


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
