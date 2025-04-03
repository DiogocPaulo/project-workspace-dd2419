import numpy as np
from nav_msgs.msg import Odometry

class RobotState:
    """
    A class to keep track of the robots current state.
    Uses an odometry message to update the state.
    """
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.velocity = 0.0

    def update_state(self, odom_msg: Odometry):
        self.x = odom_msg.pose.pose.position.x
        self.y = odom_msg.pose.pose.position.y

        q = odom_msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.yaw = np.arctan2(siny_cosp, cosy_cosp)

        odom_velocity = odom_msg.twist.twist.linear.x
        self.velocity = odom_velocity

    def distance_to_state(self, x, y):
        return np.hypot(self.x - x, self.y - y)
