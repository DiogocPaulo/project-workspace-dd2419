import rclpy
from rclpy.node import Node
import rclpy.time
from project_interfaces.srv import GoToPoint
from std_msgs.msg import Header
from project_interfaces.msg import ArmTaskMessage

def ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")

        self.point_client = self.create_client(GoToPoint, "/navigation_point")
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("GoToPoint service not yet avaliable, waiting ...")
        self.point_request = GoToPoint.Request()

        self.arm_publisher = self.create_publisher(ArmTaskMessage, "/Arm_Task", 10)

        self.clock = self.get_clock()


    def send_end_point(self, x, y):
        self.point_request.x = x
        self.point_request.y = y

        self.point_future = self.point_client.call_async(self.point_client)
        rclpy.spin_until_future_complete(self, self.point_future)
        if self.point_future.result() is not None:
            response = self.point_future.result()
            self.get_logger().info(f"GoToPoint Response: {response.message}")
        else:
            self.get_logger().info("GoToPoint service failed")

    def send_arm_task(self, x, y, z, task):
        arm_msg = ArmTaskMessage()
        arm_msg.header = Header()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.header.frame_id = "map"
        arm_msg.point = Point()
        arm_msg.point.x = x
        arm_msg.point.y = y
        arm_msg.point.z = z
        arm_msg.description = task

        
        
        self.arm_publisher.publish(arm_msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=20))

        

        


def main():
    rclpy.init()
    node = ProjectMaster()

    node.send_end_point(-1.5, 0.5)
    node.send_arm_task(0.2,0.2,0.0,"PICKUP")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
