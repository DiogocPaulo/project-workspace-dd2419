import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

from rclpy.action import ActionServer
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

from project_interfaces.srv import PickObject, JointMove, GetDetectedList
from project_interfaces.msg import ArmTaskMessage

from rclpy.action import ActionClient

from project_interfaces.msg import DetectedData, DetectedDataArray
from geometry_msgs.msg import TransformStamped, Point


class MultiServoPublisher(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")
        self.publisher = self.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)
        self.publisher_sim = self.create_publisher(JointState, '/joint_states', 10)
        self.service_sim = self.create_service(PickObject,'PickObjectSim',self.sim_callback)
        self.i = 0


        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer,self)
        self.clock = self.get_clock()
        
        self.service = self.create_service(PickObject, 'PickObject', self.task_callback)

        self.service_fine_tune = self.create_service(JointMove, 'MoveArm', self.adjust_callback)

        self.pos_subscriber = self.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)

        self.handle_camera = False

        self.detected_objects = []
        self.detected_boxes = []
        self.detected_threshhold = 0.05
        self.take_detected_flag = False



        self.l1 = 0.101
        self.l2 = 0.095
        self.l3 = 0.16
        self.off_base = 0.068

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000

    def task_callback(self,request,response):
        if request.description == "PICKUP":
            response.result = self.pickup_callback(request)
        elif request.description == "DROPOFF":
            response.result = self.dropoff_callback(request)
        elif request.description == "LOOK":
            response.result = self.look_callback(request)
        elif request.description == "LOOKUP":
            response.result = self.lookup_callback(request)
        elif request.description == "RETURN":
            response.result = self.return_callback(request)
        else:
            response.result = 2
            return response
        return response
    
    class detected_entity:
        def __init__(self,label,x,y,timestamp):
            self.x = x
            self.y = y
            self.label = label
            self.timestamp = timestamp

    def pickup_callback(self, request):
        self.get_logger().info(f'Received pickup request at {request.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0


        position = request.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)


        if v1_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {position}')
            return 1



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"PICKUP INITIATED")

        ose = [3000,12000,self.v3,self.v2,self.v1,self.base,int(move_time/2),int(move_time/2),int(move_time/2),(move_time/2),int(move_time/2),int(move_time/2)]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=1.0))
        
        pose = [3000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=3))
        
        pose = [18000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=3))

        pose = [18000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)

        self.get_logger().info(f"PICKUP COMPLETE")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=5))

        return 0

    def dropoff_callback(self, request):
        self.get_logger().info(f'Received dropoff request at {request.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0

        position = request.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)


        if v1_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {position}')
            return 1



        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"DROPOFF INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=0.5))
        
        pose = [18000,12000,v3_arm,v2_arm,v1_arm,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
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

        self.clock.sleep_for(rclpy.duration.Duration(seconds=5))

        return 0

    def look_callback(self, request):
        self.get_logger().info(f'Received look request at {request.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0
    
        # Transform ---------------------------------------
        tf_future = self.tfBuffer.wait_for_transform_async(
            target_frame = 'arm_base',
            source_frame = request.header.frame_id,
            time = zero_time # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
        )

        rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        try:
            t = self.tfBuffer.lookup_transform(
                'arm_base',
                request.header.frame_id,
                zero_time
        )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to arm_base: {ex}'
            )
        # Transform ---------------------------------------

        self.clock.sleep_for(rclpy.duration.Duration(seconds=0.5)) #Give arm time to do its thing
        position = do_transform_point(request,t)
        position = position.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)
        
        pose = [18000,12000,6500,21000,12000,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        self.base = base_arm
        self.v3 = v3_arm
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=4))
    
        return 0

    def lookup_callback(self, request):
        self.get_logger().info(f'Received look up request at {request.point}')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0
    
        # Transform ---------------------------------------
        tf_future = self.tfBuffer.wait_for_transform_async(
            target_frame = 'arm_base',
            source_frame = request.header.frame_id,
            time = zero_time # Get latest transform instead of timestamped, since we want to pickup when the robot is standing still
        )

        rclpy.spin_until_future_complete(self,tf_future, timeout_sec=1)

        try:
            t = self.tfBuffer.lookup_transform(
                'arm_base',
                request.header.frame_id,
                zero_time
        )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform map to arm_base: {ex}'
            )
        # Transform ---------------------------------------

        self.clock.sleep_for(rclpy.duration.Duration(seconds=0.5)) #Give arm time to do its thing
        position = do_transform_point(request,t)
        position = position.point
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics(position.x,position.y,position.z)
        
        pose = [18000,12000,8000,21000,12000,base_arm,move_time,move_time,move_time,move_time,move_time,move_time]
        self.base = base_arm
        self.v3 = v3_arm
        msg.data = pose
        self.publisher.publish(msg)

        self.clock.sleep_for(rclpy.duration.Duration(seconds=4))
    
        return 0
    
    def return_callback(self, request):
        self.get_logger().info(f'Received return request')
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        move_time = 2000 #arm speed (milliseconds)

        zero_time = Time()
        zero_time.sec = 0
        zero_time.nanosec = 0

        pose = [3000,12000,12000,12000,12000,12000,move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose
        self.publisher.publish(msg)
    
        return 0
    
    def adjust_callback(self, request):
        #self.get_logger().info(f'Received adjust request')

        self.take_detected_flag = True
        objects = self.detected_objects
        boxes =self.detected_boxes
        self.take_detected_flag = False

        self.get_logger().info(f'Objects:{len(objects)}')


        # self.get_logger().info(f'Received camera detections')

        target = request.target
        closest_obj = None
        if target == 'objects':
            closest_obj = min(objects, key=lambda DetectedData: DetectedData.distance)
        elif target == 'boxes':
            closest_obj = min(boxes, key=lambda DetectedData: DetectedData.distance)

        step_size = 100
        eps = 8

        base = self.base
        v3 = self.v3

        while closest_obj.distance > eps:
            if closest_obj.distance < 50:
                step_size = 50

            if closest_obj.distance < 30:
                step_size = 20
            
            if closest_obj.distance < 15:
                step_size = 10

            #difference in x-axis
            if(closest_obj.diff_x > 0):
                base += step_size
            elif(closest_obj.diff_x < 0):
                base -= step_size

            #difference in y-axis
            if(closest_obj.diff_y > 0):
                v3 += step_size
            elif(closest_obj.diff_y < 0):
                v3 -= step_size

            if not (base < 23900 and base > 100):
                base = self.base

            if  not (v3 < 20900 and v3 > 3100):
                v3 = self.v3

            
            msg = Int16MultiArray()
            msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
            move_time = 150
            pose = [11000,12000,v3,self.v2,self.v1,base,move_time,move_time,move_time,move_time,move_time,move_time]
            msg.data = pose
            self.publisher.publish(msg)

            self.clock.sleep_for(rclpy.duration.Duration(seconds=0.3))

            self.take_detected_flag = True
            objects = self.detected_objects
            boxes = self.detected_boxes
            self.take_detected_flag = False

            if target == 'objects':
                closest_obj = min(objects, key=lambda DetectedData: DetectedData.distance)
            elif target == 'boxes':
                closest_obj = min(boxes, key=lambda DetectedData: DetectedData.distance)
    
        return 0











    def sim_callback(self, request,response):
        self.get_logger().info(f'Received simulation request')

        position = request.point
        
        joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5','r_joint']

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2)) #Give arm time to do its thing
        base_arm,v1_arm,v2_arm,v3_arm = self.FindKinematics2(position.x,position.y,position.z+0.03)

        if base_arm == -1:
            self.get_logger().info(f'COULD NOT FIND KINEMATIC SOLUTION FOR POSITION: {position}')
            return

        self.get_logger().info(f"APPLYING SERVO ANGLES: BASE={base_arm} SERVO5={v1_arm} SERVO4={v2_arm} SERVO3={v3_arm}")
        self.get_logger().info(f"PICKUP INITIATED")

        self.clock.sleep_for(rclpy.duration.Duration(seconds=1))
        
        self.publish_JointStates(joint_names,[base_arm,v1_arm,v2_arm,v3_arm,0.0,0.0])

        self.publish_object_marker(request.point)

        response.result = 0

        self.clock.sleep_for(rclpy.duration.Duration(seconds=2))
        return response

        

    def CalcKinematics(self,x,y,z,desired_grip_angle): #Servos 5 & 4
        # self.get_logger().info(f'Position: {x},{y}')
        # l1 = 0.101
        # l2 = 0.095
        # l4 = 0.168

        l1 = self.l1
        l2 = self.l2
        l4 = self.l3

        try:
            base_rotation_angle = (math.atan2(y,x))
            d = self.distance_calc(0,0,x,y)

            #Calc gripper
            py = math.sin(desired_grip_angle)*l4 + z
            px = d - math.cos(desired_grip_angle)*l4

            # Calc joint 1-3
            l3 = math.sqrt((px**2) + (py**2))

            c2 = (l1**2 + l2**2 - l3**2) / (2 * l1 * l2)
            v2 = math.acos(c2)

            c1 = (l1**2 + l3**2 - l2**2) / (2*l1*l3)
            v1 = math.acos(c1)

            base_angle = math.atan2(py,px)

            v3 = (math.pi/2 - base_angle) + (math.pi - v1 - v2) + (math.pi/2 - desired_grip_angle) - math.pi

            # self.get_logger().info(f"ANGLES: V1={v1} SERVO4={v2} SERVO3={desired_grip_angle}")

            return base_rotation_angle,(math.pi/2) - v1 - base_angle,math.pi - v2, -v3
        except ValueError as e:
            return base_rotation_angle,-1,-1,-1

    def FindKinematics(self,x,y,z):

        offset = 150

        self.get_logger().info(f"OFFSET = {offset}")

        for angle in range(0 + offset,9000 - offset):
            angle = math.radians(angle/100)

            base,v1,v2,v3 = self.CalcKinematics(x,y,z,angle)
            base_arm = 12000 + int(math.degrees(base)*100)
            v1_arm = 12000 - int(math.degrees(v1)*100)
            v2_arm = 12000 + int(math.degrees(v2)*100)
            v3_arm = 12000 - int(math.degrees(v3)*100)

            if(base_arm < (0 + offset) or base_arm > (24000 - offset)): continue
            if(v1_arm < (3000 + offset) or v1_arm > (14000 - offset)): continue
            if(v2_arm < (1000 + offset) or v2_arm > (21000 - offset)): continue
            if(v3_arm < (3000 + offset) or v3_arm > (14000 - offset)): continue

            self.get_logger().info("Configuration has been found!")
            return base_arm,v1_arm,v2_arm,v3_arm


        self.get_logger().info("No configuration found!")
        return base_arm,-1,-1,-1

    def FindKinematics2(self,x,y,z):

        offset = 500

        self.get_logger().info(f"OFFSET = {offset}")
        base = 0

        for angle in range(0 + offset,9000 - offset):
            angle = math.radians(angle/100)

            base,v1,v2,v3 = self.CalcKinematics(x,y,z-0.032,angle)
            base_arm = 12000 - int(math.degrees(base)*100)
            v1_arm = 12000 - int(math.degrees(v1)*100)
            v2_arm = 12000 + int(math.degrees(v2)*100)
            v3_arm = 12000 - int(math.degrees(v3)*100)

            if(base_arm < (0 + offset) or base_arm > (24000 - offset)): continue
            if(v1_arm < (3000 + offset) or v1_arm > (14000 - offset)): continue
            if(v2_arm < (1000 + offset) or v2_arm > (21000 - offset)): continue
            if(v3_arm < (3000 + offset) or v3_arm > (14000 - offset)): continue

            self.get_logger().info("Configuration has been found!")
            return -base,-v1,-v2,-v3

        self.get_logger().info("No configuration found!")
        return base,-1,-1,-1


    
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


    def publish_object_marker(self,point):
        # Create the marker
        marker = Marker()
        marker.header.frame_id = "base_link"  # This is the robot's base frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "object_namespace"
        marker.id = 0
        marker.type = Marker.SPHERE  # You can also use CUBE, ARROW, etc.
        marker.action = Marker.ADD
        marker.pose.position.x = point.y  # Position of the object (relative to the base)
        marker.pose.position.y = point.x
        marker.pose.position.z = point.z+ self.off_base
        marker.scale.x = 0.05  # Scale of the sphere (radius)
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.a = 1.0  # Fully opaque
        marker.color.r = 1.0  # Red color   

        # Publish the marker
        self.publisher_marker.publish(marker)
        self.get_logger().info("Publishing object marker")












    def adjust_callback(self,request,response):
        self.get_logger().info("Recieved Adjust request")
        objects = request.objects
        boxes = request.boxes
        target = request.target

        closest_obj = None
        if target == 'objects' and len(objects)>0:
            closest_obj = min(objects, key=lambda DetectedData: DetectedData.distance)
        elif target == 'boxes' and len(objects)>0:
            closest_obj = min(boxes, key=lambda DetectedData: DetectedData.distance)

        if target == 'objects':
            eps = 10
        else:
            eps = 30

        if closest_obj == None:
            response.result = 2
            response.target = "None"
            return response
        
        response.target = closest_obj.label

        #Check if within threshhold:
        if closest_obj.distance <= eps:
            self.get_logger().info("Adjusting complete")
            response.result = 0
            return response

        step_size = 400
        move_time = 150

        if closest_obj.distance < 100:
            step_size = 200
            move_time = 75
        if closest_obj.distance < 30:
            step_size = 120
            move_time = 35 
        if closest_obj.distance < 10:
            step_size = 100
            move_time = 35

        base = self.base
        v3 = self.v3


        if(closest_obj.diff_x > 0):
            base += step_size
        elif(closest_obj.diff_x < 0):
            base -= step_size

        #difference in y-axis
        if(closest_obj.diff_y > 0):
            v3 += step_size
        elif(closest_obj.diff_y < 0):
            v3 -= step_size

        if not (base < 23900 and base > 100):
            base = self.base

        if  not (v3 < 20900 and v3 > 3100):
            v3 = self.v3

        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        pose = [18000,12000,int(v3),int(self.v2),int(self.v1),int(base),move_time,move_time,move_time,move_time,move_time,move_time]
        msg.data = pose

        self.publisher.publish(msg)

        time = (move_time/1000)
        self.clock.sleep_for(rclpy.duration.Duration(seconds=time))

        response.result = 1
        return response


    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

        

        


        


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
