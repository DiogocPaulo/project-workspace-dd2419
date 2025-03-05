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

        self.end_points = [
            (0.5, 0, 0.0),
            (0.5, -0.5, 0.0),
            (-0.5, 0.0, 0.0),
        ]
        self.i = 0

    def reached_destination_callback(self, request, response):
        self.get_logger().info("Master - Reached destination")

        # Send new end point
        go_to_point_request = GoToPoint.Request()
        go_to_point_request.x = self.end_points[self.i][0]
        go_to_point_request.y = self.end_points[self.i][1]
        go_to_point_request.yaw = self.end_points[self.i][2]
        self.i = (self.i + 1) % 3

        # Send new point async
        future = self.point_client.call_async(go_to_point_request)
        future.add_done_callback(self.go_to_point_response_callback)


        response.success = True
        response.message = "Sending new end point"
        return response

    def go_to_point_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"GoToPoint response: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().warn(f"Service call failed: {e}")
            

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
