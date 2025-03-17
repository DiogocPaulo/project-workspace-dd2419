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

from project_interfaces.srv import PickObject
from project_interfaces.action import PickUpObject
from project_interfaces.msg import ArmTaskMessage

from rclpy.action import ActionClient

class MultiServoPublisher(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")
        # self.timer = self.create_timer(5.0, self.publish_pose)
        self.publisher = self.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)
        self.publisher_sim = self.create_publisher(JointState, '/joint_states', 10)
        self.i = 0

        self.task_subscription = self.create_subscription(
            ArmTaskMessage,           # Message type
            '/Arm_Task',        # Topic name
            self.task_callback,  # Callback function
            10                # Queue size
        )
        self.task_subscription

        # self.task_publisher = self.create_publisher(ArmTaskMessage, "/Arm_Task", 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)
        self.clock = self.get_clock()
        
        # self.service = self.create_service(PickObject, 'PickObject', self.pickup_callback_sim)
        # self.client = self.create_client(PickObject, 'PickObject')


        # self._action_server = ActionServer(
        #     self,
        #     PickUpObject,
        #     'PickUpObject',
        #     self.pickup_callback_sim
        # )
        # self._action_client = ActionClient(self, PickUpObject, 'PickUpObject')

        arm_msg = ArmTaskMessage()
        arm_msg.header = Header()
        arm_msg.header.stamp = self.get_clock().now().to_msg()
        arm_msg.header.frame_id = "map"
        arm_msg.point = Point()
        arm_msg.point.x = 0.2
        arm_msg.point.y = 0.2
        arm_msg.point.z = 0.0
        arm_msg.description = "DROPOFF"

        # self.publisher_marker = self.create_publisher(Marker, '/visualization_marker', 10)

        # self.timer = self.create_timer(1.0, self.publish_object_marker)

        # request = PickObject.Request()
        # request.target_position = self.position
        # future = self.client.call_async(request)
        # self.get_logger().info(f"RESPONSE: {future}")

        # goal_msg = PickUpObject.Goal()
        # goal_msg.target_position = self.position
        # self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.result_callback)

        # self.task_publisher.publish(arm_msg)

    def task_callback(self,msg):
        if msg.description == "PICKUP":
            self.pickup_callback(msg)
        elif msg.description == "DROPOFF":
            self.dropoff_callback(msg)
        else:
            return

        



    def pickup_callback(self, message):
        self.get_logger().info(f'Received pickup request at {message.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0

        pose = [3000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        # Transform ---------------------------------------
        tf_future = self.tfBuffer.wait_for_transform_async(
            target_frame = 'arm_base',
            source_frame = message.header.frame_id,
            time = zero_time # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
        )

        rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        try:
            t = self.tfBuffer.lookup_transform(
                'arm_base',
                message.header.frame_id,
                zero_time
        )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to arm_base: {ex}'
            )
        # Transform ---------------------------------------

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing
        position = do_transform_point(message,t)
        position = position.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)


        if base_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {request.target_position}')
            return



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"PICKUP INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        pose = [3000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        pose = [14000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        pose = [14000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.get_logger().info(f"PICKUP COMPLETE")

        return

    def dropoff_callback(self, message):
        self.get_logger().info(f'Received dropoff request at {message.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0

        pose = [14000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        # # Transform ---------------------------------------
        # tf_future = self.tfBuffer.wait_for_transform_async(
        #     target_frame = 'arm_base',
        #     source_frame = message.header.frame_id,
        #     time = zero_time # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
        # )

        # rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        # try:
        #     t = self.tfBuffer.lookup_transform(
        #         'arm_base',
        #         message.header.frame_id,
        #         zero_time
        # )
        # except TransformException as ex:
        #     self.get_logger().info(
        #         f'Could not transform map to arm_base: {ex}'
        #     )
        # # Transform ---------------------------------------

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing
        # position = do_transform_point(message.point,t)
        position = message.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)


        if base_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {request.target_position}')
            return



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"DROPOFF INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        pose = [14000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        pose = [3000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        pose = [12000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.get_logger().info(f"DROPOFF COMPLETE")

        return


    def pickup_callback_sim(self, msg):
        self.get_logger().info(f'Received pickup request at {msg.point}')
        position = msg.point
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','r_joint']


        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics2(position.x,position.y,position.z)


        if base_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {position}')
            return



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"PICKUP INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        self.publish_JointStates(joint_names,[base_arm,v1_arm,v2_arm,v3_arm,0.0,1.0])

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        self.publish_JointStates(joint_names,[base_arm,v1_arm,v2_arm,v3_arm,0.0,0.0])

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        self.publish_JointStates(joint_names,[0.0,0.0,0.0,0.0,0.0,0.0])

        self.get_logger().info(f"PICKUP COMPLETE")
        return 

    def dropoff_callback_sim(self, msg):
        self.get_logger().info(f'Received dropoff request at {msg.point}')
        position = msg.point
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','r_joint']


        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics2(position.x,position.y,position.z)


        if base_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {position}')
            return



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"DROPOFF INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        self.publish_JointStates(joint_names,[base_arm,v1_arm,v2_arm,v3_arm,0.0,0.0])

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        
        self.publish_JointStates(joint_names,[base_arm,v1_arm,v2_arm,v3_arm,0.0,1.0])

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))

        self.publish_JointStates(joint_names,[0.0,0.0,0.0,0.0,0.0,1.0])

        self.get_logger().info(f"DROPOFF COMPLETE")
        return 

    def feedback_callback(self, feedback):
        self.get_logger().info(f'Feedback: {feedback.message}')

    def result_callback(self, future):
        # This is called once the result of the action is available
        result = future.result()
        
        # Check the result's status and print it
        if result:
            self.get_logger().info(f'Result received: Status = {result.status}')
        else:
            self.get_logger().error('Action failed!')



        



    def CalcKinematics(self,x,y,z,desired_grip_angle): #Servos 5 & 4
        # self.get_logger().info(f'Position: {x},{y}')
        l1 = 0.101
        l2 = 0.095
        l4 = 0.168

        try:
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
        except ValueError as e:
            return -1,-1,-1,-1

    def FindKinematics(self,x,y,z):

        offset = 500

        self.get_logger().info(f"OFFSET = {offset}")

        for angle in range(0 + offset,9000 - offset):
            angle = math.radians(angle/100)

            base,v1,v2,v3 = self.CalcKinematics(x,y,z,angle)
            base_arm = 12000 + int(math.degrees(base)*100)
            v1_arm = 12000 - int(math.degrees(v1)*100)
            v2_arm = 12000 + int(math.degrees(v2)*100)
            v3_arm = 12000 - int(math.degrees(v3)*100)

            if(base_arm < (0 + offset) or base_arm > (24000 - offset)): continue
            if(v1_arm < (6000 + offset) or v1_arm > (18000 - offset)): continue
            if(v2_arm < (3000 + offset) or v2_arm > (21000 - offset)): continue
            if(v3_arm < (3000 + offset) or v3_arm > (21000 - offset)): continue

            self.get_logger().info("Configuration has been found!")
            return base_arm,v1_arm,v2_arm,v3_arm


        self.get_logger().info("No configuration found!")
        return -1,-1,-1,-1

    def FindKinematics2(self,x,y,z):

        offset = 500

        self.get_logger().info(f"OFFSET = {offset}")

        for angle in range(0 + offset,9000 - offset):
            angle = math.radians(angle/100)

            base,v1,v2,v3 = self.CalcKinematics(x,y,z,angle)
            base_arm = 12000 - int(math.degrees(base)*100)
            v1_arm = 12000 - int(math.degrees(v1)*100)
            v2_arm = 12000 + int(math.degrees(v2)*100)
            v3_arm = 12000 - int(math.degrees(v3)*100)

            if(base_arm < (0 + offset) or base_arm > (24000 - offset)): continue
            if(v1_arm < (6000 + offset) or v1_arm > (18000 - offset)): continue
            if(v2_arm < (3000 + offset) or v2_arm > (21000 - offset)): continue
            if(v3_arm < (3000 + offset) or v3_arm > (21000 - offset)): continue

            self.get_logger().info("Configuration has been found!")
            return base,v1,v2,v3

        self.get_logger().info("No configuration found!")
        return -1,-1,-1,-1


    
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

