
#!/usr/bin/env python

import numpy as np
import py_trees
import py_trees_ros

import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from project_interfaces.srv import GoToPoint, Trigger
from nav_msgs.msg import Odometry
from project_interfaces.msg import Point, Workspace
from project_interfaces.srv import PickObject
from std_msgs.msg import Header
import rclpy.time
from geometry_msgs.msg import Point as GeometryPoint
import math
from project_interfaces.srv import GetDetectedList, JointMove
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import time

class ServiceClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, service_type, service_name, **kwargs):
        super().__init__(name)
        self.service_type = service_type
        self.service_name = service_name
        self.request_args = kwargs
        self.client = None
        self.future = None
        self.sent_request = False

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.client = self.node.create_client(self.service_type, self.service_name)
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service {self.service_name} ...")
            return py_trees.common.Status.RUNNING
            
        if not self.sent_request:
            try:
                request = self.service_type.Request()

                for key, value in self.request_args.items():
                    setattr(request, key, value)

                self.future = self.client.call_async(request)
                self.sent_request = True
                self.node.get_logger().info(f"{self.name} - Sent request to {self.service_name}")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
                if response.success:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response.success}, {response.message}")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response.success}, {response.message}")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING



            
        
class ArmClient(py_trees.behaviour.Behaviour):
    def __init__(self, name, x, y, z, task, **kwargs):
        super().__init__(name)


        self.x = x
        self.y = y
        self.z = z
        self.task = task
        self.client = None
        self.future = None
        self.sent_request = False
        self.request_args = kwargs
        self.clock = None

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.client = self.node.create_client(PickObject, 'PickObject')
        self.clock = self.node.get_clock()
        return True

    def initialise(self):
        self.sent_request = False
        self.future = None

    def update(self):
        if not self.client.service_is_ready():
            self.node.get_logger().info(f"{self.name} - Waiting for service PickObject ...")
            return py_trees.common.Status.RUNNING
            
        if not self.sent_request:
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "map"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = self.z
                request.description = self.task

                self.future = self.client.call_async(request)
                self.sent_request = True
                self.node.get_logger().info(f"{self.name} - Sent request to PickObject")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE

        if self.future.done():
            try:
                response = self.future.result()
                if response == 0:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response}, task successful")
                    return py_trees.common.Status.SUCCESS
                else:
                    self.node.get_logger().info(f"{self.name} - Service call response: {response}, task failed")
                    return py_trees.common.Status.FAILURE
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Service call failed with exception: {e}")
                return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.RUNNING

class ReachedEndPoint(py_trees.behaviour.Behaviour):
    """
    A behaviour that checks is an end point has been reached
    """
    def __init__(self, name, x, y, tolerance=0.1):
        super().__init__(name)
        self.current_point = (None, None)
        self.end_point = (x, y)
        self.tolerance = tolerance

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False

        qos_profile = QoSProfile(
            depth=1,
            history=HistoryPolicy.KEEP_LAST,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.node.create_subscription(Odometry, "/odom", self.odom_callback, qos_profile)
        return True

    def odom_callback(self, msg: Odometry):
        self.current_point = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def update(self):
        if self.current_point == (None, None):
            # self.node.get_logger().info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )

        if distance <= self.tolerance:
            # self.node.get_logger().info(f"{self.name}: Reached end point of ({self.end_point[0], self.end_point[1]})")
            self.node.get_logger().info(f"GYATT")
            return py_trees.common.Status.SUCCESS
        else:
            # self.node.get_logger().info(f"{self.name}: Distance to end point is {distance:.2f}")
            return py_trees.common.Status.RUNNING

def create_offset_end_points(workspace_vertices, offset_distance=0.5):
    vertices = [np.array(vertex) for vertex in workspace_vertices]
    num_vertices = len(vertices)

    end_points = []

    def compute_offset_normal(p, q):
        edge_vector = q - p
        norm_edge = np.linalg.norm(edge_vector)
        if norm_edge == 0:
            return np.array([0, 0])
        perp_vector = np.array([-edge_vector[1], edge_vector[0]])
        normal_vector = perp_vector / np.linalg.norm(perp_vector)
        cross_product = np.cross(edge_vector, perp_vector)
        if cross_product < 0:
            normal_vector = -normal_vector
        return normal_vector

    # Compute and add offset midpoints for each edge.
    for i in range(num_vertices):
        prev = vertices[i - 1]
        current = vertices[i]
        nxt = vertices[(i + 1) % num_vertices]

        normal1 = compute_offset_normal(prev, current)
        normal2 = compute_offset_normal(current, nxt)

        p1 = current + normal1 * offset_distance
        d1 = current - prev  # direction of the incoming edge
        p2 = current + normal2 * offset_distance
        d2 = nxt - current   # direction of the outgoing edge

        denom = np.cross(d1, d2)
        if np.abs(denom) < 1e-6:
            offset_vertex = current + normal1 * offset_distance
        else:
            t = np.cross((p2 - p1), d2) / denom
            offset_vertex = p1 + t * d1

        end_points.append((offset_vertex[0], offset_vertex[1], 0.0))

        edge_vector = nxt - current
        midpoint = (current + nxt) / 2.0

        normal_vector = compute_offset_normal(current, nxt)
        offset_mid = midpoint + normal_vector * offset_distance
        end_points.append((offset_mid[0], offset_mid[1], 0.0))

    return end_points

class CollectionMaster(Node):

    def __init__(self):
        super().__init__("explore_master")

        self.get_logger().info(f"INITIALIZING COLLECTION MASTER")

        workspace_file = "workspaces/small_workspace.tsv"
        self.workspace_vertices = self.read_workspace(workspace_file, skip_header=True)
        self.workspace_publisher = self.create_publisher(Workspace, "/workspace", 10)
        self.end_points_broadcaster = TransformBroadcaster(self)

        self.i = 0

        self.end_points = create_offset_end_points(self.workspace_vertices, 0.5)
        self.objects, self.boxes = self.process_map_file("maps/Map_test.txt")

        self.publish_transforms()

        self.create_timer(0.1, self.tick_tree) # Tick tree every 100 ms
        self.create_timer(2, self.publish_workspace)
        #self.create_timer(2, self.broadcast_end_points)
        self.create_timer(2, self.publish_transforms)

        root = self.create_collection_tree()
        self.tree = py_trees_ros.trees.BehaviourTree(root=root)
        tree_string = py_trees.display.ascii_tree(root)
        self.get_logger().info(f"Behavior Tree Structure:\n{tree_string}")
        self.tree.setup(timeout=15, node=self)

    def publish_workspace(self):
        workspace_msg = Workspace()
        workspace_msg.header.stamp = self.get_clock().now().to_msg()
        workspace_msg.header.frame_id = "map"

        for vertex in self.workspace_vertices:
            point_msg = Point()
            point_msg.x = vertex[0]
            point_msg.y = vertex[1]
            workspace_msg.points.append(point_msg)

        self.workspace_publisher.publish(workspace_msg)
        self.get_logger().info("Published workspace vertices", once=True)

    def broadcast_end_points(self):
        for i, point in enumerate(self.end_points):
            transform = TransformStamped()
            transform.header.frame_id = 'map'
            transform.header.stamp = self.get_clock().now().to_msg()
            transform.child_frame_id = f'EndPoint{i}'
            
            transform.transform.translation.x = point[0]
            transform.transform.translation.y = point[1]
            transform.transform.translation.z = 0.0
            
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0
            
            self.end_points_broadcaster.sendTransform(transform)

    class Object:
        def __init__(self,type,x,y):
            self.x = x
            self.y = y
            self.type = type

    def process_map_file(self, file_path):
        objects = []
        boxes = []
        with open(file_path, "r") as file:
            for line in file:
                parts = line.strip().split(" ")
                O = self.Object(parts[0],float(parts[1])/100,float(parts[2])/100)
                if O.type == "B" or O.type == "b":
                    boxes.append(O)
                else:
                    objects.append(O)

        return objects,boxes

    def publish_transforms(self):
        object_number = 0
        box_number = 0
        for object in self.objects:
            self.publish_transform("Object-"+str(object_number),object.x,object.y,0)
            object_number+=1
        for box in self.boxes:
            self.publish_transform("Box-"+str(box_number),box.x,box.y,0)
            box_number+=1


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

        self.end_points_broadcaster.sendTransform(t)
        # self.get_logger().info(f"Published transform for {name} at ({x}, {y})")

    def create_rob_coordinates(self, point1, point2, offset):
        direction = np.array(point2) - np.array(point1)
        
        unit_direction = direction / np.linalg.norm(direction)
        
        new_endpoint = np.array(point1) + unit_direction * offset

        rob_x = point2[0] - new_endpoint[0]
        rob_y = point2[1] - new_endpoint[1]
        
        return rob_x, rob_y
    
    def create_collection_tree(self):
        root = py_trees.composites.Selector("CollectionRoot", memory=True)
        Collection_sequence = py_trees.composites.Sequence("Collection", memory=True)
        prev_rob_x = 0
        prev_rob_y = 0

        objects_copy = self.objects.copy()

        while len(objects_copy)>0:
            ################# Pick Up Phase ##################

            # Picks the closest object
            distance = 100000
            closest = None
            O_i = 0
            for i, O in enumerate(objects_copy):
                distance_O = np.linalg.norm(np.array([O.x,O.y]) - np.array([prev_rob_x,prev_rob_y]))
                if distance_O < distance:
                    closest = O
                    distance = distance_O
                    O_i = i

            point_selector = py_trees.composites.Selector(f"EndPoint{self.i}", memory=True)



            rob_x,rob_y = self.create_rob_coordinates((prev_rob_x,prev_rob_y),(closest.x,closest.y),0.1)
            x = closest.x
            y = closest.y
            yaw = 0.0

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{self.i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{self.i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=x,
                y=y,
                yaw=yaw
            )

            look_service = Look(
                name=f"LOOK",
                x=x+0.1,
                y=y+0.1,
                t='objects'
            )

            adjust_service = Adjust(
                name=f"ADJUST", 
                t='objects'
            )

            pick_service = Pick(
                name=f"PICK", 
                t='objects',
                task = "PICKUP"
            )

            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{self.i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{self.i}",
                x=x,
                y=y
            )

            retry_endpoint = py_trees.decorators.FailureIsRunning(
                name=f"RetryEndpoint{self.i}",
                child=end_point_check
            )

            retry_on_endpoint_failure.add_children([pathing_service, retry_endpoint])
            service_check_sequence.add_child(retry_on_endpoint_failure)

            fallback = py_trees.behaviours.Success(name=f"SkipToNext{self.i}")

            point_selector.add_children([service_check_sequence, fallback])
            Collection_sequence.add_child(point_selector)
            # Collection_sequence.add_child(look_service)
            # Collection_sequence.add_child(adjust_service)
            # Collection_sequence.add_child(pick_service)

            objects_copy.pop(O_i)

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            # self.i += 1

            # ################# Drop Off Phase ##################

            # Picks the closest box
            distance = 100000
            closest = None
            O_i = 0
            for i, O in enumerate(self.boxes):
                distance_O = np.linalg.norm(np.array([O.x,O.y]) - np.array([prev_rob_x,prev_rob_y]))
                if distance_O < distance:
                    closest = O
                    distance = distance_O
                    O_i = i

            point_selector = py_trees.composites.Selector(f"EndPoint{self.i}", memory=True)


            rob_x,rob_y = self.create_rob_coordinates((prev_rob_x,prev_rob_y),(closest.x,closest.y),0.1)
            x = closest.x
            y = closest.y
            yaw = 0.0

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{self.i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{self.i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=x,
                y=y,
                yaw=yaw
            )

            look_service = Look(
                name=f"LOOK",
                x=0.1,
                y=-0.1,
                t='boxes'
            )

            adjust_service = Adjust(
                name=f"ADJUST", 
                t='boxes'
            )

            drop_service = Drop(
                name=f"DROP", 
                t='boxes',
                task = "DROPOFF"
            )



            retry_on_endpoint_failure = py_trees.composites.Sequence(f"RetryOnEndpointFailure{self.i}", memory=False)
            
            end_point_check = ReachedEndPoint(
                name=f"ReachedEndPoint{self.i}",
                x=x,
                y=y
            )

            retry_endpoint = py_trees.decorators.FailureIsRunning(
                name=f"RetryEndpoint{self.i}",
                child=end_point_check        
            )

            retry_on_endpoint_failure.add_children([pathing_service, retry_endpoint])
            service_check_sequence.add_child(retry_on_endpoint_failure)

            fallback = py_trees.behaviours.Success(name=f"SkipToNext{self.i}")

            point_selector.add_children([service_check_sequence, fallback])
            Collection_sequence.add_child(point_selector)

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            # Collection_sequence.add_child(look_service)
            # Collection_sequence.add_child(adjust_service)
            # Collection_sequence.add_child(drop_service)

            # self.i += 1

            
    
        repeater = py_trees.decorators.Repeat(
            name="RepeatCollection", 
            child=Collection_sequence,
            num_success=2
        )
        
        root.add_child(repeater)
        return root

    def tick_tree(self):
        try:
            self.tree.tick()
        except Exception as e:
            self.get_logger().error(f"Exception during tree tick: {e}")

    def read_workspace(self, filename, skip_header=False):
        workspace_vertices = []
        try:
            with open(filename, "r") as file:
                if skip_header:
                    next(file)
                for line in file:
                    line = line.strip()
                    if line:
                        parts = line.split("\t")
                        if len(parts) == 2:
                            x, y = parts
                            workspace_vertices.append((float(x)/100, float(y)/100))
                            self.get_logger().info(f"Added vertex: ({float(x)/100}, {float(y)/100})")
                        else:
                            self.get_logger().warn("Skipping invalid line: {line}")
            return workspace_vertices
        except FileNotFoundError:
            self.get_logger().warn(f"File {filename} not found")
            return []
        








class Look(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, x, y, t, **kwargs):
        super().__init__(name)
        self.x = x
        self.y = y
        self.type = t
        self.node = None
        self.pickup_client = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.logger.info("Arm Request service not yet avaliable, waiting ...")

        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.clock = self.node.get_clock()

        self.future = None


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def update(self):
        if self.stage == 0:
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = 0.0
                request.description = "LOOK"

                self.future = self.pickup_client.call_async(request)
                self.stage = 1
                self.node.get_logger().info(f"{self.name} - Sent request to Look")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    self.stage = 2

            return py_trees.common.Status.RUNNING
                
        elif self.stage == 2:
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 3

            return py_trees.common.Status.RUNNING

        elif self.stage == 3:
            if self.future.done():
                response = self.future.result()
                
                if self.type == "objects":
                    if len(response.objects) > 0:
                        return py_trees.common.Status.SUCCESS
                    else:
                        return py_trees.common.Status.FAILURE
                elif self.type == "boxes":
                    if len(response.boxes) > 0:
                        return py_trees.common.Status.SUCCESS
                    else:
                        return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        

class Adjust(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.camera_client = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14

        self.eps = 5
        self.step_size = 200

        if self.type == "boxes":
            self.eps = 15

    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.logger.error(f"{self.name} - Setup failed: {e}")
            return False


        self.camera_client = self.node.create_client(GetDetectedList, 'get_detected_list')
        while not self.camera_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)

        self.joint_publisher = self.node.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)

        self.clock = self.node.get_clock()


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):
        if self.stage == 0:
            # self.node.get_logger().info(f"Stage: 0")
            request = GetDetectedList.Request()

            self.future = self.camera_client.call_async(request)
            self.stage = 1

            return py_trees.common.Status.RUNNING

        elif self.stage == 1:
            # self.node.get_logger().info(f"Stage: 1")
            if self.future.done():
                response = self.future.result()
                
                objects = response.objects
                boxes = response.boxes

                closest_obj = None
                if self.type == 'objects':
                    closest_obj = min(objects, key=lambda DetectedData: DetectedData.distance)
                elif self.type == 'boxes':
                    closest_obj = min(boxes, key=lambda DetectedData: DetectedData.distance)



                #Check if within threshhold:
                if closest_obj.distance <= self.eps:
                    return py_trees.common.Status.SUCCESS

                base = self.base
                v3 = self.v3

                if closest_obj.distance < 10:
                    self.step_size = 100
                #difference in x-axis
                if(closest_obj.diff_x > 0):
                    base += self.step_size
                elif(closest_obj.diff_x < 0):
                    base -= self.step_size

                #difference in y-axis
                if(closest_obj.diff_y > 0):
                    v3 += self.step_size
                elif(closest_obj.diff_y < 0):
                    v3 -= self.step_size

                if not (base < 23900 and base > 100):
                    base = self.base

                if  not (v3 < 20900 and v3 > 3100):
                    v3 = self.v3

                move_command = JointMove.Request()
                move_command.base = int(base)
                move_command.v1 = int(self.v1)
                move_command.v2 = int(self.v2)
                move_command.v3 = int(v3)
                
                msg = Int16MultiArray()
                msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
                move_time = 100
                pose = [11000,12000,int(v3),int(self.v2),int(self.v1),int(base),move_time,move_time,move_time,move_time,move_time,move_time]
                msg.data = pose
                self.start_time = time.time()
                self.joint_publisher.publish(msg)

                self.stage = 2

            return py_trees.common.Status.RUNNING

        elif self.stage == 2:
            # self.node.get_logger().info(f"Stage: 2")
            if time.time() - self.start_time >= 0.12:
                self.stage = 0

            return py_trees.common.Status.RUNNING
        

class Pick(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, task, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.x = None
        self.y = None
        self.task = task

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.clock = self.node.get_clock()

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):

        # First two stages is to pass some time to allow the correct joint readings to be read
        if self.stage == 0:
            self.node.get_logger().info(f"Stage: 0")
            self.start_time = time.time()

            self.stage = 1
            return py_trees.common.Status.RUNNING


        elif self.stage == 1:
            self.node.get_logger().info(f"Stage: 1")
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
            self.node.get_logger().info(f"Stage: 2")
            l1 = 0.101
            l2 = 0.095
            base = math.radians((12000 - self.base) / 100)
            alpha = math.radians((12000 - self.v1) / 100)
            beta = math.radians((12000 + self.v2) / 100)
            charlie = math.radians((12000 - (self.v3 - 50)) / 100)

            distance_y = l1 + self.off_base #math.sin(alpha)*l1 + math.sin(beta)*l2

            distance = l2 + math.tan((math.pi/2)-charlie)*distance_y + 0.085  #math.cos(alpha)*l1 + math.cos(beta)*l2

            if distance < 0.25: distance -= 0.01

            distance_z = 0 - self.off_base

            distance_y = math.sin(-base)*distance
            distance_x = math.cos(base)*distance

            self.x = distance_x
            self.y = distance_y

            self.stage = 3

            return py_trees.common.Status.RUNNING

        # Pick upp target
        elif self.stage == 3:
            self.node.get_logger().info(f"Stage: 3")
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = -0.15
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            self.node.get_logger().info(f"Stage: 4")
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        
class Drop(py_trees.behaviour.Behaviour):
    """
    A behaviour that determines the next end point and navigates to it.
    """
    def __init__(self, name, t, task, **kwargs):
        super().__init__(name)
        self.type = t
        self.node = None
        self.request_args = kwargs
        self.clock = None
        self.stage = 0
        self.future = None
        self.x = None
        self.y = None
        self.task = task

        self.base = 12000
        self.v1 = 12000
        self.v2 = 12000
        self.v3 = 12000
        self.off_base = 0.14


    def setup(self, **kwargs):
        try:
            self.node = kwargs.get("node")
        except Exception as e:
            self.node.get_logger.error(f"{self.name} - Setup failed: {e}")
            return False

        self.clock = self.node.get_clock()

        self.pickup_client = self.node.create_client(PickObject, 'PickObject')
        while not self.pickup_client.wait_for_service(timeout_sec=1.0):
            self.node.get_logger().info('Service not available, waiting...')

        self.pos_subscriber = self.node.create_subscription(
            JointState, '/servo_pos_publisher', self.pos_callback, 10)


        return True

    def initialise(self):
        self.stage = 0
        self.future = None

    def pos_callback(self,msg):
        self.base = msg.position[5]
        self.v1 = msg.position[4]
        self.v2 = msg.position[3]
        self.v3 = msg.position[2]

    def update(self):

        # First two stages is to pass some time to allow the correct joint readings to be read
        if self.stage == 0:
            self.node.get_logger().info(f"Stage: 0")
            self.start_time = time.time()

            self.stage = 1
            return py_trees.common.Status.RUNNING


        elif self.stage == 1:
            self.node.get_logger().info(f"Stage: 1")
            if time.time() - self.start_time >= 0.5:
                self.stage = 2

            return py_trees.common.Status.RUNNING

        # Stage 2 estimates the position of the target given where the arm is pointing
        elif self.stage == 2:
            self.node.get_logger().info(f"Stage: 2")
            l1 = 0.101
            l2 = 0.095
            base = math.radians((12000 - self.base) / 100)
            alpha = math.radians((12000 - self.v1) / 100)
            beta = math.radians((12000 + self.v2) / 100)
            charlie = math.radians((12000 - (self.v3 - 50)) / 100)

            distance_y = l1 + self.off_base #math.sin(alpha)*l1 + math.sin(beta)*l2

            distance = l2 + math.tan((math.pi/2)-charlie)*distance_y + 0.085  #math.cos(alpha)*l1 + math.cos(beta)*l2

            if distance < 0.25: distance -= 0.03

            distance_z = 0 - self.off_base

            distance_y = math.sin(-base)*distance
            distance_x = math.cos(base)*distance

            self.x = distance_x
            self.y = distance_y

            self.stage = 3

            return py_trees.common.Status.RUNNING

        # Pick upp target
        elif self.stage == 3:
            self.node.get_logger().info(f"Stage: 3")
            try:
                request = PickObject.Request()

                request.header = Header()
                request.header.stamp = self.node.get_clock().now().to_msg()
                request.header.frame_id = "arm_base"
                request.point = GeometryPoint()
                request.point.x = self.x
                request.point.y = self.y
                request.point.z = 0.0
                request.description = self.task

                self.future = self.pickup_client.call_async(request)
                self.stage = 4
                self.node.get_logger().info(f"{self.name} - Sent request to Pickup")
                return py_trees.common.Status.RUNNING
            except Exception as e:
                self.node.get_logger().error(f"{self.name} - Failed to send request: {e}")
                return py_trees.common.Status.FAILURE


        elif self.stage == 4:
            self.node.get_logger().info(f"Stage: 4")
            if self.future.done():
                response = self.future.result()
                if response.result == 0:
                    return py_trees.common.Status.SUCCESS
                else:
                    return py_trees.common.Status.FAILURE

            return py_trees.common.Status.RUNNING
        

def main():
    rclpy.init()
    node = CollectionMasterMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == "__main__":
    main()
