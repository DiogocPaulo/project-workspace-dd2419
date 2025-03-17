import rclpy
from rclpy.node import Node
from project_interfaces.srv import GoToPoint, Trigger

class ExploreMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        self.reached_destination_service = self.create_service(Trigger, "/reached_destination", self.reached_destination_callback)
        self.end_point_client = self.create_client(GoToPoint, "/navigation_point")
        while not self.end_point_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")

        self.end_points = [
            (-1.6, 0.9, 0.0),
            (-1.6, -0.9, 0.0),
            (1.6, -0.9, 0.0),
            (1.6, 0.9, 0.0),
        ]
        self.i = 0

    def reached_destination_callback(self, request, response):
        next_x = self.end_points[self.i][0]
        next_y = self.end_points[self.i][1]
        next_yaw = self.end_points[self.i][2]
        self.send_end_point()
        self.i = (self.i + 1) % 4

        response.success = True
        response.message = f"Sending end point: ({next_x}, {next_y}) at {next_yaw} radians"
        return response

    def send_end_point(self, x, y, yaw):
        # Create new GoToPoint service for navigation node
        navigation_request = GoToPoint.Request()
        navigation_request.x = x
        navigation_request.y = y
        navigation_request.yaw = yaw

        # Handle request and response to navigation node async
        future = self.end_point_client.call_async(navigation_request)
        future.add_done_callback(self.navigation_response_callback)

    def navigation_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"Navigation response: {response.success}, {response.message}")
        except Exception as e:
            self.get_logger().warn(f"Service call to navigation node failed: {e}")

def main():
    rclpy.init()
    node = ExploreMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
