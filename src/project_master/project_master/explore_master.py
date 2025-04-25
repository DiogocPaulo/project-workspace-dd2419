
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

from project_master import behaviours

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
    def __init__(self, name, x, y, tolerance=0.2):
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
            self.node.get_logger().info(f"{self.name}: Waiting for current point ...")
            return py_trees.common.Status.RUNNING
        
        distance = np.hypot(
            self.current_point[0] - self.end_point[0],
            self.current_point[1] - self.end_point[1]
        )

        if distance <= self.tolerance:
            self.node.get_logger().info(f"{self.name}: Reached end point of ({self.end_point[0], self.end_point[1]})")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info(f"{self.name}: Distance to end point is {distance:.2f}")
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

class ExploreMaster(Node):

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
        self.create_timer(2, self.broadcast_end_points)
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
                O = self.Object(parts[0],float(parts[1])/1000,float(parts[2])/1000)
                if O.type == "B" or O.type == "b":
                    boxes.append(O)
                else:
                    objects.append(O)

        return objects,boxes

    def publish_transforms(self):
        object_number = 0
        box_number = 0
        for object in self.objects:
            self.publish_transform(object.type+"-"+str(object_number),object.x,object.y,0)
            object_number+=1
        for box in self.boxes:
            self.publish_transform(box.type+"-"+str(box_number),box.x,box.y,0)
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
        self.get_logger().info(f"Published transform for {name} at ({x}, {y})")

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



            rob_x,rob_y = self.create_rob_coordinates((closest.x,closest.y),(prev_rob_x,prev_rob_y),0.05)
            x = closest.x
            y = closest.y
            yaw = 0.0

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{self.i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{self.i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=rob_x,
                y=rob_y,
                yaw=yaw
            )

            pick_service = ArmClient(
                name=f"PickupObject",
                x=x,
                y=y,
                z=0.0,
                task='PICKUP'
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
            service_check_sequence.add_child(pick_service)

            fallback = py_trees.behaviours.Success(name=f"SkipToNext{self.i}")

            point_selector.add_children([service_check_sequence, fallback])
            Collection_sequence.add_child(point_selector)

            objects_copy.pop(O_i)

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            self.i += 1

            ################# Drop Off Phase ##################

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


            rob_x,rob_y = self.create_rob_coordinates((closest.x,closest.y),(prev_rob_x,prev_rob_y),0.05)
            x = closest.x
            y = closest.y
            yaw = 0.0

            service_check_sequence = py_trees.composites.Sequence(f"ServiceCheck{self.i}", memory=True)

            pathing_service = ServiceClient(
                name=f"GoToPoint{self.i}",
                service_type=GoToPoint,
                service_name="/pathing_end_point",
                x=rob_x,
                y=rob_y,
                yaw=yaw
            )

            drop_service = ArmClient(
                name=f"DropObject",
                x=x,
                y=y,
                z=0.0,
                task='DROPOFF'
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
            service_check_sequence.add_child(drop_service)

            fallback = py_trees.behaviours.Success(name=f"SkipToNext{self.i}")

            point_selector.add_children([service_check_sequence, fallback])
            Collection_sequence.add_child(point_selector)

            prev_rob_x = rob_x
            prev_rob_y = rob_y

            self.i += 1

            
    
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
