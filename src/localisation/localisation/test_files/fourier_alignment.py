import numpy as np
import scipy.fftpack as fft
import matplotlib.pyplot as plt

def fourier_alignment(boundary1, boundary2):
    """Aligns boundary2 to boundary1 using Fourier phase correlation."""
    def to_polar(points):
        """Converts Cartesian points to polar coordinates."""
        r = np.linalg.norm(points, axis=1)
        theta = np.arctan2(points[:, 1], points[:, 0])
        return r, theta
    
    # Convert both boundaries to polar coordinates
    r1, theta1 = to_polar(boundary1)
    r2, theta2 = to_polar(boundary2)
    
    # Convert polar to complex form for FFT (e^(i*theta) representation)
    complex_boundary1 = r1 * np.exp(1j * theta1)
    complex_boundary2 = r2 * np.exp(1j * theta2)
    
    # Fourier transforms of both point sets
    f1 = fft.fft(complex_boundary1)
    f2 = fft.fft(complex_boundary2)
    
    # Cross-correlation in the frequency domain (normalized)
    cross_power = np.conj(f1) * f2 / np.abs(f1) / np.abs(f2)
    phase_shift = fft.ifft(cross_power)
    
    # Find the phase shift (rotation) between the two point sets
    rotation_angle = np.angle(phase_shift[0])
    
    # Apply the rotation
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)], 
                                [np.sin(rotation_angle), np.cos(rotation_angle)]])
    
    aligned_boundary = np.dot(boundary2, rotation_matrix.T)
    
    return aligned_boundary, rotation_angle

# Example usage:
boundary1 = np.array([[0, 0], [1, 0], [1, 1], [0, 1]])  # Square
boundary2 = np.array([[0, 0], [0, 1], [-1, 1], [-1, 0]])  # Rotated square

aligned_boundary, rotation_angle = fourier_alignment(boundary1, boundary2)

# Plotting
# Plotting
plt.figure(figsize=(6,6))
plt.plot(boundary1[:, 0], boundary1[:, 1], 'o-', label='Boundary 1', color='blue')
plt.plot(boundary2[:, 0], boundary2[:, 1], 'o-', label='Boundary 2', color='red')
plt.plot(aligned_boundary[:, 0], aligned_boundary[:, 1], 'o-', label='Aligned Boundary 2', color='green')
plt.legend()
plt.gca().set_aspect('equal', adjustable='box')
plt.title('PCA Alignment of Boundaries')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
plt.show()
