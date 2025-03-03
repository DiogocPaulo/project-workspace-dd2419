import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint, Trigger

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

        self.point_client = self.create_client(GoToPoint, "/navigation_point")
        self.reached_destination_service = self.create_service(Trigger, "/reached_destination", self.reached_destination_callback)
        while not self.point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")

        self.send_end_point(-1.5, 0.5, 0.0)

    def reached_destination_callback(self, request, response):
        self.get_logger().info("Master - Reached destination")

        # Send new end point
        go_to_point_request = GoToPoint.Request()
        go_to_point_request.x = 1.5
        go_to_point_request.y = 0.5
        go_to_point_request.yaw = 1.5
        future = self.point_client.call_async(go_to_point_request)

        rclpy.spin_until_future_complete(self, future)

        if future is not None:
            go_to_point_response = future.result()
            self.get_logger().info(f"GoToPoint response: {go_to_point_response.success}, {go_to_point_response.message}")
            response.success = go_to_point_response.success
            response.message = go_to_point_response.message
        else:
            self.get_logger().warn("Service call failed")
            response.success = False
            response.message = "Failed to call GoToPoint service"

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
            self.get_logger().info(f"GoToPoint Response: {response.message}")
        else:
            self.get_logger().info("GoToPoint service failed")

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
