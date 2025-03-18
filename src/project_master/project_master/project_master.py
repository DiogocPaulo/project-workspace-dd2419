import rclpy
from rclpy.node import Node
import rclpy.time
from project_interfaces.srv import GoToPoint
from std_msgs.msg import Header
from project_interfaces.msg import ArmTaskMessage
from project_interfaces.srv import PickObject
from geometry_msgs.msg import Point
from project_interfaces.srv import GoToPoint, Trigger

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")


        self.clock = self.get_clock()

        self.arm_publisher = self.create_publisher(ArmTaskMessage, "/Arm_Task", 10)

        self.client = self.create_client(PickObject, 'PickObject')

        
        # self.send_arm_task(0.2,0.2,0.0,"PICKUP")

        # self.point_client = self.create_client(GoToPoint, "/navigation_point")
        # self.reached_destination_service = self.create_service(Trigger, "/reached_destination", self.reached_destination_callback)
        # while not self.point_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")

        self.end_points = [
            (0.5, 0.0, 0.0),
            (0.5, -0.5, 0.0),
            (-0.5, -0.5, 0.0),
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
        self.i = (self.i + 1) % 4

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

    def send_arm_task(self, x, y, z, task):
        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        self.get_logger().info("ARMTASK!!!!!!!!!!!!!!!!!!!!!!!!!!")
        arm_msg = ArmTaskMessage()
        arm_msg.header = Header()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.header.frame_id = "base_link"
        arm_msg.point = Point()
        arm_msg.point.x = x
        arm_msg.point.y = y
        arm_msg.point.z = z
        arm_msg.description = task

        
        
        self.arm_publisher.publish(arm_msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=5))

    def send_arm_request(self, x, y, z, task):
        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        self.get_logger().info("ARMTASK!")
        arm_msg = PickObject.Request()
        arm_msg.header = Header()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.header.frame_id = "base_link"
        arm_msg.point = Point()
        arm_msg.point.x = x
        arm_msg.point.y = y
        arm_msg.point.z = z
        arm_msg.description = task

        future = self.client.call_async(arm_msg)
        rclpy.spin_until_future_complete(self, future)

        response = future.result() 
        if response is not None:
            self.get_logger().info(f"Response received: {response.result}")
        else:
            self.get_logger().error("No response received!")


        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        

        


def main():
    rclpy.init()
    node = ProjectMaster()

   # node.send_end_point(-1.5, 0.5)

    node.send_arm_request(0.2,0.0,-0.03,"PICKUP")
    node.send_arm_request(0.2,0.0,-0.03,"DROPOFF")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
