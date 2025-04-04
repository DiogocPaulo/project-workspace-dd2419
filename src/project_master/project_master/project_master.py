import rclpy
from rclpy.node import Node
import rclpy.time
from project_interfaces.srv import GoToPoint
from std_msgs.msg import Header
from project_interfaces.msg import ArmTaskMessage
from project_interfaces.srv import PickObject
from geometry_msgs.msg import Point
from project_interfaces.srv import GoToPoint, Trigger
import tf2_ros
from geometry_msgs.msg import TransformStamped
import math

class ProjectMaster(Node):

    def __init__(self):
        super().__init__("project_master")


        self.clock = self.get_clock()

        self.arm_publisher = self.create_publisher(ArmTaskMessage, "/Arm_Task", 10)

        self.client = self.create_client(PickObject, 'PickObject')

        
        # self.reached_destination_service = self.create_service(Trigger, "/reached_destination", self.reached_destination_callback)
        # self.end_point_client = self.create_client(GoToPoint, "/navigation_point")
        # while not self.end_point_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().debug("GoToPoint service not yet avaliable, waiting ...")

        self.end_points = [
            (-1.6, 0.9, 0.0),
            (-1.6, -0.9, 0.0),
            (1.6, -0.9, 0.0),
            (1.6, 0.9, 0.0),
        ]

        self.i = 0

        self.objects = []

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # self.send_arm_request(0.2,0.0,0.0,"PICKUP")
            

    # def send_end_point(self, x, y, yaw):
    #     request = GoToPoint.Request()
    #     request.x = x
    #     request.y = y
    #     request.yaw = yaw

    #     self.point_future = self.point_client.call_async(request)
    #     rclpy.spin_until_future_complete(self, self.point_future)
    #     if self.point_future.result() is not None:
    #         response = self.point_future.result()
    #         self.get_logger().info(f"GoToPoint Response: {response.message}")
    #     else:
    #         self.get_logger().info("GoToPoint service failed")

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
            if response.result == 0:
                self.get_logger().info(f"REQUEST SUCCESFUL!")
            elif response.result == 1:
                self.get_logger().info(f"REQUEST FAILED!: COULD NOT FIND KINEMATIC SOLUTION")
            else:
                self.get_logger().info(f"REQUEST FAILED!: PICKUP DID NOT PICK UP OBJECT")
        else:
            self.get_logger().error("NO RESPONSE RECIEVED!")


        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        
    class Object:
        def __init__(self,type,x,y):
            self.x = x
            self.y = y
            self.type = type

    def process_map_file(self, file_path):
        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split(" ")
                O = self.Object(parts[0],float(parts[1])/1000,float(parts[2])/1000)
                self.objects.append(O)

    def publish_transforms(self):
        for Object in self.objects:
            self.publish_transform(Object.type,Object.x,Object.y,0)


    def publish_transform(self, name, x, y, theta):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = name

        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0

        # Convert yaw to quaternion
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"Published transform for {name} at ({x}, {y})")


    # def reached_destination_callback(self, request, response):
    #     next_x = self.end_points[self.i][0]
    #     next_y = self.end_points[self.i][1]
    #     next_yaw = self.end_points[self.i][2]
    #     self.send_end_point(next_x, next_y, next_yaw)
    #     self.i = (self.i + 1) % 4

    #     response.success = True
    #     response.message = f"Sending end point: ({next_x}, {next_y}) at {next_yaw} radians"
    #     return response

    # def send_end_point(self, x, y, yaw):
    #     # Create new GoToPoint service for navigation node
    #     navigation_request = GoToPoint.Request()
    #     navigation_request.x = x
    #     navigation_request.y = y
    #     navigation_request.yaw = yaw

    #     # Handle request and response to navigation node async
    #     future = self.end_point_client.call_async(navigation_request)
    #     future.add_done_callback(self.navigation_response_callback)

    # def navigation_response_callback(self, future):
    #     try:
    #         response = future.result()
    #         self.get_logger().info(f"Navigation response: {response.success}, {response.message}")
    #     except Exception as e:
    #         self.get_logger().warn(f"Service call to navigation node failed: {e}")
        


def main():
    rclpy.init()
    node = ProjectMaster()

   # node.send_end_point(-1.5, 0.5)

    
    # node.process_map_file("/home/robot/project-workspace-dd2419/maps/Map_test.txt")
    # node.publish_transforms()
    node.send_arm_request(0.2,0.0,0.0,"PICKUP")
    # node.send_arm_request(0.15,-0.15,0.0,"DROPOFF")   



    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
