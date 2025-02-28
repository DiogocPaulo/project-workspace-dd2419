import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

from rclpy.action import ActionServer
from geometry_msgs.msg import Point
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point

import math
import tf2_ros
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

import rclpy.time
from builtin_interfaces.msg import Time

from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import geometry_msgs.msg
from tf2_ros import TransformBroadcaster

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray

class MultiServoPublisher(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")
        # self.timer = self.create_timer(5.0, self.publish_pose)
        self.publisher = self.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)
        self.publisher_sim = self.create_publisher(JointState, '/joint_states', 10)
        self.i = 0

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)
        self.clock = self.get_clock()
        
        # self.server = ActionServer(self,PickupAction,'PickupCube',self.pickup_callback)

        self.position = Point()
        self.position.x = 0.13
        self.position.y = 0.13   
        self.position.z = -0.2
        self.desired_grip_angle = math.pi/2

        self.publisher_marker = self.create_publisher(Marker, '/visualization_marker', 10)

        self.timer = self.create_timer(1.0, self.publish_object_marker)
                

        self.publish_pose_sim()

    def publish_pose(self): #Test function (not the actual function)
        self.get_logger().info(f"Applying to the real world")
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

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

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing

        # TODO: Get position of object in map frame (SOLVED)

        
        world_position = Point()
        world_position.x = 0.13
        world_position.y = 0.13
        world_position.z = -0.05


        self.get_logger().info(f"Received goal: Pickup object at position {world_position} in map frame")

        # TODO: Transform the object into the arm base frame (SOLVED)
        position = do_transform_point(world_position,t)

        # TODO: Calculate the arm parameters to "hawk" over the object's position

        # perform IK
        base,v1,v2,v3 = CalcKinematics(position.x,position.y,position.z,math.pi/2)

        #Convert angles to robot arm
        base = 12000 + int(math.degrees(base)*10)
        v1 = 12000 + int(math.degrees(v1)*10)
        v2 = 12000 + int(math.degrees(v2)*10)
        v3 = 12000 + int(math.degrees(v3)*10)

        self.get_logger().info(f"Received parameter: base={base} servo5={v1} servo4={v1}")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=10))
        
        pose = [3000,12000,12000,12000,12000,base,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.get_logger().info(f"Done")




    def publish_pose_sim(self): #Test function (not the actual function)
        self.get_logger().info(f"Simulating!")

        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','r_joint']

        move_time = 2000 #arm speed (milliseconds)

        # pose publish
        # If not already in the base pose, go to it:
        
        self.publish_JointStates(joint_names,[0.0,0.0,0.0,0.0,0.0,0.0])

        self.clock.sleep_for(rclpy.duration.Duration(seconds=1))



        # TODO: Calculate the arm parameters to "hawk" over the object's position
        # First calculate the base rotation

        
        base,v1,v2,v3 = self.CalcKinematics(self.position.x,self.position.y,self.position.z,self.desired_grip_angle)

        #Convert angles to robot arm
        base_arm = 12000 - int(math.degrees(base)*100)
        v1_arm = 12000 - int(math.degrees(v1)*100)
        v2_arm = 12000 - int(math.degrees(v2)*100)
        v3_arm = 12000 - int(math.degrees(v3)*100)

        self.get_logger().info(f"Received parameter: base={base_arm} servo5={v1_arm} servo4={v2_arm} servo3={v3_arm}")

        self.publish_JointStates(joint_names,[base,v1,v2,v3,0.0,0.0])
        self.clock.sleep_for(rclpy.duration.Duration(seconds=5))



        self.publish_JointStates(joint_names,[0,0,0,0.0,0.0,0.0])


        self.get_logger().info("Done!")


















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


    def CalcKinematics(self,x,y,z,desired_grip_angle): #Servos 5 & 4
        self.get_logger().info(f'Position: {x},{y}')
        l1 = 0.101
        l2 = 0.095
        l4 = 0.168

        base_rotation_angle = (math.atan2(y,x))
        d = self.distance_calc(0,0,x,y)

        #Calc gripper
        py = math.sin(desired_grip_angle)*l4 + z
        px = d - math.cos(desired_grip_angle)*l4

        # Calc joint 1-3
        l3 = math.sqrt(px**2 + py**2)

        c2 = (l1**2 + l2**2 - l3**2) / (2 * l1 * l2)
        v2 = math.acos(c2)

        c1 = (l1**2 + l3**2 - l2**2) / (2*l1*l3)
        v1 = math.acos(c1)

        base_angle = math.atan2(py,px)

        v3 = (math.pi/2 - base_angle) + (math.pi - v1 - v2) + (math.pi/2 - desired_grip_angle) - math.pi

        return base_rotation_angle,(math.pi/2) - v1 - base_angle,math.pi - v2, -v3
    
    def publish_JointStates(self, names, states):
        self.get_logger().info("***Publishing!***")
        msg = JointState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()

        
        msg.name = names
        msg.position = states
        msg.velocity = []  # Optional, leave empty if not used
        msg.effort = []    # Optional, leave empty if not used

        self.publisher_sim.publish(msg)
        self.get_logger().info(f'Publishing joint states: {msg.position}')

    def distance_calc(self, x1, y1, x2, y2):
        return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)


    def publish_object_marker(self):
        # Create the marker
        marker = Marker()
        marker.header.frame_id = "base_link"  # This is the robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE  # You can also use CUBE, ARROW, etc.
        marker.action = Marker.ADD
        marker.pose.position.x = self.position.x  # Position of the object (relative to the base)
        marker.pose.position.y = self.position.y
        marker.pose.position.z = self.position.z+0.065
        marker.scale.x = 0.05  # Scale of the sphere (radius)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 1.0  # Red color

        # Publish the marker
        self.publisher_marker.publish(marker)
        self.get_logger().info("Publishing object marker")

        


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

