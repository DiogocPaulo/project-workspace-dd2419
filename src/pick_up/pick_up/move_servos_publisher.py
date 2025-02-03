"""import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int16MultiArray, MultiArrayDimension, MultiArrayLayout

import time


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('move_servos_publisher')
        self.publisher_ = self.create_publisher(Int16MultiArray, '/multi_servo_cmd_sub', 10)
        timer_period = 5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        # pick up stuff
        # data_sets = [[2000,12000,20000,3000,-1,3000,500,500,500,500,500,500],
        #             [16000,-1,-1,-1,-1,-1,500,500,500,500,500,500],
        #             [16000,12000,12000,12000,12000,12000,500,500,500,500,500,500],
        #             [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]]

        # msg = Int64MultiArray()

        # msg.data = data_sets[self.i]
        # self.publisher_.publish(msg)

        # self.i += 1
        # if self.i == 4:
        #     self.i = 0

        
        # random movement
        data_sets = [[5000,13000,14000,10000,16000,8000,500,500,500,500,500,500],
                    [12000,12000,12000,12000,12000,12000,500,500,500,500,500,500]]
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=0, stride=0)], data_offset=0)
        msg.data = data_sets[self.i]
        self.publisher_.publish(msg)

        self.i += 1
        if self.i == 2:
            self.i = 0

        
        # self.get_logger().info('Publishing: "%s"' % str(msg.data))
        # self.i += 1   

def main():
    rclpy.init()
    node = MinimalPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension

class MultiServoPublisher(Node):
    def __init__(self):
        super().__init__("multi_servo_publisher")
        self.timer = self.create_timer(5.0, self.publish_pose)
        self.publisher = self.create_publisher(Int16MultiArray, "/multi_servo_cmd_sub", 10)
        self.i = 0
        # self.publish_pose()

    def publish_pose(self):
        msg = Int16MultiArray()
        msg.layout = MultiArrayLayout(dim=[MultiArrayDimension(label="", size=12, stride=12)], data_offset=0)
        
        data_sets = [[3000,12000,12000,12000,12000,12000,1000,1000,1000,1000,1000,1000],
                    [3000,12000,8000,20000,6700,12000,1000,1000,1000,1000,1000,1000],
                    [12000,12000,8000,20000,6700,12000,1000,1000,1000,1000,1000,1000],
                    [12000,12000,12000,12000,12000,12000,1000,1000,1000,1000,1000,1000]]
        msg.data = data_sets[self.i]
        self.publisher.publish(msg)
        self.get_logger().info(f"Published servo command message")
        self.i += 1
        if self.i == 4:
            self.i = 0


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

