#!/usr/bin/env python

import math
import numpy as np
import matplotlib.pyplot as plt

def compute_new_pose(initial_pose: np.ndarray, translation: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    """Compute a new pose given an initial pose, translation vector, and rotation matrix."""
    # Ensure inputs are in the correct format
    initial_pose = np.asarray(initial_pose, dtype=float)
    translation = np.asarray(translation, dtype=float).flatten()
    rotation_matrix = np.asarray(rotation_matrix, dtype=float)

    if initial_pose.shape != (3,):
        raise ValueError(f"Initial pose must be a 3-element array, got {initial_pose.shape}")
    if translation.shape != (2,):
        raise ValueError(f"Translation must be a 2-element vector, got {translation.shape}")
    if rotation_matrix.shape != (2, 2):
        raise ValueError(f"Rotation matrix must be 2x2, got {rotation_matrix.shape}")

    # Extract initial position and orientation
    x, y, theta = initial_pose
    
    # Compute new position: apply translation in world frame
    x_new = x + translation[0]
    y_new = y + translation[1]
    
    # Compute new orientation: combine initial theta with rotation from matrix
    # Extract rotation angle from the 2x2 matrix
    delta_theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    theta_new = theta + delta_theta
    # Normalize theta to [-pi, pi]
    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
    
    return np.array([x_new, y_new, theta_new])

def plot_poses(initial_pose: np.ndarray, new_pose: np.ndarray):
    """Plot the initial and new poses as arrows."""
    plt.figure(figsize=(8, 8))
    
    arrow_len = 0.5  # Length of the pose arrows for visualization
    
    # Plot initial pose
    plt.quiver(initial_pose[0], initial_pose[1], 
               arrow_len * math.cos(initial_pose[2]),
               arrow_len * math.sin(initial_pose[2]), 
               color='b', label='Initial Pose', width=0.005)
    
    # Plot new pose
    plt.quiver(new_pose[0], new_pose[1], 
               arrow_len * math.cos(new_pose[2]),
               arrow_len * math.sin(new_pose[2]), 
               color='g', label='New Pose', width=0.005)
    
    # Set plot properties
    plt.legend()
    plt.grid(True)
    plt.axis('equal')
    plt.title("Initial and New Poses")
    plt.xlabel("X (m)")
    plt.ylabel("Y (m)")
    
    # Adjust plot limits to ensure poses are fully visible
    poses = np.vstack((initial_pose[:2], new_pose[:2]))
    plt.xlim(min(poses[:, 0]) - 1, max(poses[:, 0]) + 1)
    plt.ylim(min(poses[:, 1]) - 1, max(poses[:, 1]) + 1)
    
    plt.show()

if __name__ == "__main__":
    # Example usage
    # Initial pose: [x, y, theta]
    initial_pose = np.array([0.0, 0.0, 0.0])
    
    # Translation vector: [dx, dy]
    translation = np.array([1.0, 0.5])
    
    # Rotation matrix: 30 degrees rotation
    theta_rot = math.radians(30)
    rotation_matrix = np.array([[math.cos(theta_rot), -math.sin(theta_rot)],
                               [math.sin(theta_rot), math.cos(theta_rot)]])
    
    # Compute new pose
    new_pose = compute_new_pose(initial_pose, translation, rotation_matrix)
    
    # Print results
    print(f"Initial Pose: {initial_pose}")
    print(f"Translation: {translation}")
    print(f"Rotation Matrix:\n{rotation_matrix}")
    print(f"New Pose: {new_pose}")
    
    # Plot the poses
    plot_poses(initial_pose, new_pose)