import numpy as np
from nav_msgs.msg import Odometry

# Robot parameters
base = 0.3                  # Wheelbase of the vehicle
lookahead_gain = 0.1        # Look-ahead distance gain
lookahead_min = 0.3         # Minimum look-ahead distance
distance_threshold = 0.2    # Stop distance threshold
yaw_threshold = 0.2         # Stop yaw threshold
target_velocity = 0.22      # Robot's target velocity

class RobotState:
    """Using odometry message to update the current state of the robot"""
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
        self.velocity = odom_velocity + (target_velocity - odom_velocity)

    def distance_to_state(self, x, y):
        return np.hypot(self.x - x, self.y - y)
