import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
# from pick_up.action import PickupAction

import math
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy.time
from builtin_interfaces.msg import Time

class MultiServoPublisher(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")
        # self.timer = self.create_timer(5.0, self.publish_pose)
        self.publisher = self.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)
        self.i = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)
        

        self.publish_pose()
        # self.server = ActionServer(self,PickupAction,'PickupCube',self.pickup_callback)

    def publish_pose(self): #Test function (not the actual function)
        self.get_logger().info(f"Lets go!--------------------------------------------------------------------------")
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 1000 #arm speed (milliseconds)

        # pose publish
        # If not already in the base pose, go to it:
        pose = [3000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)
        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0


        # TODO: Get transform between arm base and map (SOLVED)
        tf_future = self.tfBuffer.wait_for_transform_async(
            target_frame = 'arm_base',
            source_frame = 'map',
            time = zero_time # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
        )

        rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        try:
            t = self.tfBuffer.lookup_transform(
                'arm_base',
                'map',
                zero_time
        )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to arm_base: {ex}'
            )

        #rclpy.sleep(1.5) #Give arm time to do its thing

        # TODO: Get position of object in map frame (SOLVED)

        
        world_position = Point()
        world_position.x = 0.1
        world_position.y = 0.1
        world_position.z = 0.1


        self.get_logger().info(f"Received goal: Pickup object at position {world_position} in map frame")

        # TODO: Transform the object into the arm base frame (SOLVED)
        position = do_transform_point(world_position,t)

        # TODO: Calculate the arm parameters to "hawk" over the object's position
        # First calculate the base rotation
        base_rotation_angle = math.atan2(position.position.y,position.position.x)

        distance = distance(position.position.x,position.position.y)

        alpha,beta = self.CalcKinematics1(distance,position.position.z + 0.2)


        self.get_logger().info(f"Received parameter: base={base_rotation_angle} servo5={alpha} servo4={beta}")

        # TODO: transform angle from radians to arm parameters

        # placeholder
        # pose = [12000,12000,8000,20000,6700,6000,move_time,move_time,move_time,move_time,move_time,move_time]
        # msg.data = pose
        # self.publisher.publish(msg)




















    # def pickup_callback(self,goal_handle):
    #     msg = Int16MultiArray()
    #     msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
    #     move_time = 1000 #arm speed (milliseconds)

    #     # pose publish
    #     # If not already in the base pose, go to it:
    #     pose = [3000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
    #     msg.data = pose
    #     self.publisher.publish(msg)

    #     # TODO: Get transform between arm base and map (SOLVED)
    #     tf_future = self.tf_buffer.wait_for_transform_async(
    #         target_frame = 'arm_base',
    #         source_frame = 'map',
    #         time = 0 # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
    #     )

    #     rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

    #     try:
    #         t = self.tf_buffer.lookup_transform(
    #             'arm_base',
    #             'map',
    #             0)
    #     except TransformException as ex:
    #         self.get_logger().info(
    #             f'Could not transform map to arm_base: {ex}'
    #         )

    #     rclpy.sleep(1.5) #Give arm time to do its thing

    #     # TODO: Get position of object in map frame (SOLVED)
    #     world_position = goal_handle.request.position
    #     self.get_logger().info(f"Received goal: Pickup object at position {world_position} in map frame")

    #     # TODO: Transform the object into the arm base frame (SOLVED)
    #     position = do_transform_point(world_position,t)

    #     # TODO: Calculate the arm parameters to "hawk" over the object's position
    #     # First calculate the base rotation
    #     base_rotation_angle = math.atan2(position)


    #     # TODO: transform angle from radians to arm parameters

    #     # TODO: Publish said pose to the arm
    #     # placeholder
    #     pose = [12000,12000,8000,20000,6700,base_rotation_angle,move_time,move_time,move_time,move_time,move_time,move_time]
    #     msg.data = pose
    #     self.publisher.publish(msg)

    #     # TODO: See and confirm the object is in the right position (using the arm camera)

    #     # TODO: If yes (the object is there and in correct position) then calculate and publish a movement to move into pre pickup pose
    #     # TODO: Close gripper
    #     # TODO: Move arm back to base position
    #     # TODO: Confirm object is in gripper using arm camera

    #     # TODO: If no (the object is not there or in incorrect position)
    #     # TODO: Go back to base position
    #     # TODO: If found recalculate the new postion, move to it and try again

    #     # TODO: Done


    def CalcKinematics1(self,x,y): #Servos 5 & 4
        l1 = 0.101
        l2 = 0.095


        c2 = (x**2 + y**2 - l1**2 - l2**2) / (2*l1*l2)
        v2 = math.acos(c2)

        alpha = math.atan2(y,x)

        beta = math.acos((x**2 + y**2 + l1**2 - l2**2)/(2*l1*math.sqrt(x**2 + y**2)))

        v1 = alpha - beta 
        

        return v1, v2

        


def main():
    rclpy.init()
    node = MultiServoPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()

