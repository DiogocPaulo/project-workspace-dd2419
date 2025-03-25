import numpy as np
import matplotlib.pyplot as plt

def align_boundaries_by_centering(source, target):
    """Aligns boundary2 to boundary1 by centering both sets and then translating boundary2 to boundary1's centroid."""
    # Step 1: Compute the centroids of both boundaries
    source_centroid = np.mean(source, axis=0)
    target_centroid = np.mean(target, axis=0)
    
    # Step 2: Move source to the position of targets's centroid
    source_aligned = source + (target_centroid - source_centroid)

    return source_aligned

def pca_alignment(source, target):
    """Aligns target to source using PCA after centering both sets."""
    def compute_pca(points):
        mean = np.mean(points, axis=0)
        centered = points - mean
        U, S, Vt = np.linalg.svd(centered, full_matrices=False)
        return Vt, mean  # Principal components + centroid
    
    # Center both point sets by subtracting their centroids
    source_mean = np.mean(source, axis=0)
    target_mean = np.mean(target, axis=0)
    
    source_centered = source - source_mean
    target_centered = target - target_mean
    
    # Compute PCA for both centered point sets
    V_source, _ = compute_pca(source_centered)
    V_target, _ = compute_pca(target_centered)
    
    # Compute initial rotation (align principal axes)
    R_init = V_source.T @ V_target  # Align the principal components
    
    # Apply the rotation to target
    target_aligned = (target_centered @ R_init) + source_mean

    return target_aligned, R_init

def fourier_alignment(source, target):
    """Aligns target to source using Fourier phase correlation."""
    def to_polar(points):
        """Converts Cartesian points to polar coordinates."""
        r = np.linalg.norm(points, axis=1)
        theta = np.arctan2(points[:, 1], points[:, 0])
        return r, theta
    
    # Convert both boundaries to polar coordinates
    r1, theta1 = to_polar(source)
    r2, theta2 = to_polar(target)
    
    # Convert polar to complex form for FFT (e^(i*theta) representation)
    complex_source = r1 * np.exp(1j * theta1)
    complex_target = r2 * np.exp(1j * theta2)
    
    # Fourier transforms of both point sets
    f1 = np.fft.fft(complex_source)
    f2 = np.fft.fft(complex_target)
    
    # Cross-correlation in the frequency domain (normalized)
    cross_power = np.conj(f1) * f2 / np.abs(f1) / np.abs(f2)
    phase_shift = np.fft.ifft(cross_power)
    
    # Find the phase shift (rotation) between the two point sets
    rotation_angle = np.angle(phase_shift[0])
    
    # Apply the rotation
    rotation_matrix = np.array([[np.cos(rotation_angle), -np.sin(rotation_angle)], 
                                [np.sin(rotation_angle), np.cos(rotation_angle)]])
    
    aligned_target = np.dot(target, rotation_matrix.T)
    
    return aligned_target, rotation_angle

def generate_square(size, center=(0, 0), n_points=100):
    """Generate points forming a square"""
    half = size / 2
    corners = np.array([
        [-half, -half], [half, -half],
        [half, half], [-half, half]
    ]) + center
    points = []
    for i in range(4):
        start, end = corners[i], corners[(i + 1) % 4]
        t = np.linspace(0, 1, n_points // 4)
        segment = (1 - t[:, None]) * start + t[:, None] * end
        points.append(segment)
    return np.vstack(points)

def test_pre_alignments():
    """Test center alignment, PCA+center, and Fourier+center on two squares"""
    # Generate two squares
    source = generate_square(2, center=(0, 0), n_points=20)
    true_angle = np.pi / 4  # 45 degrees
    true_R = np.array([[np.cos(true_angle), -np.sin(true_angle)],
                      [np.sin(true_angle), np.cos(true_angle)]])
    true_t = np.array([1, 1])
    target = (true_R @ source.T).T + true_t

    # Add noise
    source += np.random.normal(0, 0.05, source.shape)
    target += np.random.normal(0, 0.05, target.shape)

    # Apply pre-alignments
    center_aligned = align_boundaries_by_centering(np.copy(source), np.copy(target))
    pca_aligned, _ = pca_alignment(np.copy(source), np.copy(target))
    pca_centered_aligned, _ = pca_alignment(np.copy(center_aligned), np.copy(target))
    fourier_aligned, _ = fourier_alignment(np.copy(source), np.copy(target))
    fourier_centered_aligned, _ = fourier_alignment(np.copy(center_aligned), np.copy(target))

    # Plot the results
    plt.figure(figsize=(15, 10), dpi=100)
    plt.xticks(np.arange(-3, 4, 1))
    plt.yticks(np.arange(-3, 4, 1))

    plt.subplot(2, 3, 1)
    plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
    plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
    plt.scatter(center_aligned[:, 0], center_aligned[:, 1], label='Center Aligned', alpha=0.6)
    plt.title('Center Alignment')
    plt.legend()

    plt.subplot(2, 3, 2)
    plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
    plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
    plt.scatter(pca_aligned[:, 0], pca_aligned[:, 1], label='PCA Aligned', alpha=0.6)
    plt.title('PCA Alignment')
    plt.legend()

    plt.subplot(2, 3, 3)
    plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
    plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
    plt.scatter(pca_centered_aligned[:, 0], pca_centered_aligned[:, 1], label='Centering + PCA', alpha=0.6)
    plt.title('Full Alignment')
    plt.legend()

    plt.subplot(2, 3, 4)
    plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
    plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
    plt.scatter(fourier_aligned[:, 0], fourier_aligned[:, 1], label='Fourier Aligned', alpha=0.6)
    plt.title('Fourier Alignment')
    plt.legend()

    plt.subplot(2, 3, 5)
    plt.scatter(source[:, 0], source[:, 1], label='Source', alpha=0.6)
    plt.scatter(target[:, 0], target[:, 1], label='Target', alpha=0.6)
    plt.scatter(fourier_centered_aligned[:, 0], fourier_centered_aligned[:, 1], label='Fourier + Center Aligned', alpha=0.6)
    plt.title('Fourier + Center Alignment')
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test_pre_alignments()