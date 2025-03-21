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

        self.objects = []

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

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
        


def main():
    rclpy.init()
    node = ProjectMaster()

   # node.send_end_point(-1.5, 0.5)

    # node.send_arm_request(0.2,0.0,0.0,"PICKUP")
    # node.send_arm_request(0.15,-0.15,0.0,"DROPOFF")
    node.process_map_file("/home/robot/project-workspace-dd2419/maps/Map1.txt")
    node.publish_transforms()



    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
