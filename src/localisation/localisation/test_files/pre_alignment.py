import numpy as np
from scipy.fft import fft2, ifft2
import matplotlib.pyplot as plt

# --- Pre-alignment Algorithms ---

def center_alignment(source_points, target_points):
    """Translate source centroid to target centroid"""
    src_centroid = np.mean(source_points, axis=0)
    tgt_centroid = np.mean(target_points, axis=0)
    
    t = tgt_centroid - src_centroid
    transform = np.eye(3)
    transform[0:2, 2] = t
    
    print(f"Center Alignment - Source Centroid: {src_centroid}, Target Centroid: {tgt_centroid}, Translation: {t}")
    return transform

def pca_pre_alignment(source_points, target_points):
    """Pre-align using PCA with center alignment"""
    # Step 1: Center alignment
    center_transform = center_alignment(source_points, target_points)
    src_h = np.ones((source_points.shape[0], 3))
    src_h[:, 0:2] = source_points
    src_centered = (center_transform @ src_h.T).T[:, 0:2]
    
    # Step 2: PCA for rotation
    src_centroid = np.mean(src_centered, axis=0)
    tgt_centroid = np.mean(target_points, axis=0)
    
    src_centered -= src_centroid
    tgt_centered = target_points - tgt_centroid
    
    src_cov = src_centered.T @ src_centered
    tgt_cov = tgt_centered.T @ tgt_centered
    src_eigvals, src_eigvecs = np.linalg.eigh(src_cov)
    tgt_eigvals, tgt_eigvecs = np.linalg.eigh(tgt_cov)
    
    src_order = np.argsort(src_eigvals)[::-1]
    tgt_order = np.argsort(tgt_eigvals)[::-1]
    src_eigvecs = src_eigvecs[:, src_order]
    tgt_eigvecs = tgt_eigvecs[:, tgt_order]
    
    if np.linalg.det(src_eigvecs) * np.linalg.det(tgt_eigvecs) < 0:
        tgt_eigvecs[:, 1] *= -1
    
    R = tgt_eigvecs @ src_eigvecs.T
    
    rotation_transform = np.eye(3)
    rotation_transform[0:2, 0:2] = R
    
    # Combine: center alignment first, then rotation
    final_transform = rotation_transform @ center_transform
    
    print(f"PCA - Rotation Matrix:\n{R}")
    return final_transform

def fourier_pre_alignment(source_points, target_points, grid_size=64):
    """Pre-align using Fourier phase correlation with center alignment"""
    # Step 1: Center alignment
    center_transform = center_alignment(source_points, target_points)
    src_h = np.ones((source_points.shape[0], 3))
    src_h[:, 0:2] = source_points
    src_centered = (center_transform @ src_h.T).T[:, 0:2]
    
    # Step 2: Fourier for residual translation
    def points_to_image(points, grid_size, bounds):
        x_min, y_min = np.min(points, axis=0)
        x_max, y_max = np.max(points, axis=0)
        img = np.zeros((grid_size, grid_size))
        for x, y in points:
            i = int((y - y_min) / (y_max - y_min) * (grid_size - 1))
            j = int((x - x_min) / (x_max - x_min) * (grid_size - 1))
            i = min(max(i, 0), grid_size - 1)
            j = min(max(j, 0), grid_size - 1)
            img[i, j] += 1
        return img
    
    all_points = np.vstack((src_centered, target_points))
    bounds = (np.min(all_points[:, 0]), np.max(all_points[:, 0]),
             np.min(all_points[:, 1]), np.max(all_points[:, 1]))
    
    src_img = points_to_image(src_centered, grid_size, bounds)
    tgt_img = points_to_image(target_points, grid_size, bounds)
    
    src_fft = fft2(src_img)
    tgt_fft = fft2(tgt_img)
    
    cross_power = src_fft * np.conj(tgt_fft)
    cross_power /= np.abs(cross_power) + 1e-10
    shift = ifft2(cross_power)
    
    shift_real = np.abs(shift)
    peak = np.unravel_index(np.argmax(shift_real), shift_real.shape)
    dy, dx = peak
    if dy > grid_size // 2:
        dy -= grid_size
    if dx > grid_size // 2:
        dx -= grid_size
    
    x_range = bounds[1] - bounds[0]
    y_range = bounds[3] - bounds[2]
    tx = dx * x_range / grid_size
    ty = dy * y_range / grid_size
    
    fourier_transform = np.eye(3)
    fourier_transform[0:2, 2] = [tx, ty]
    
    # Combine: center alignment first, then Fourier translation
    final_transform = fourier_transform @ center_transform
    
    print(f"Fourier - Additional Translation: [{tx}, {ty}]")
    return final_transform

# --- Test Utilities ---

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
    source = generate_square(2, center=(0, 0), n_points=100)
    true_angle = np.pi / 4  # 45 degrees
    true_R = np.array([[np.cos(true_angle), -np.sin(true_angle)],
                      [np.sin(true_angle), np.cos(true_angle)]])
    true_t = np.array([1, 1])
    target = (true_R @ source.T).T + true_t

    # Add noise
    source += np.random.normal(0, 0.05, source.shape)
    target += np.random.normal(0, 0.05, target.shape)
    
    # Apply pre-alignments
    center_transform = center_alignment(source, target)
    pca_transform = pca_pre_alignment(source, target)
    fourier_transform = fourier_pre_alignment(source, target)
    
    src_h = np.ones((source.shape[0], 3))
    src_h[:, 0:2] = source
    center_aligned = (center_transform @ src_h.T).T[:, 0:2]
    pca_aligned = (pca_transform @ src_h.T).T[:, 0:2]
    fourier_aligned = (fourier_transform @ src_h.T).T[:, 0:2]
    
    # Verify centroids
    src_centroid = np.mean(source, axis=0)
    tgt_centroid = np.mean(target, axis=0)
    center_centroid = np.mean(center_aligned, axis=0)
    pca_centroid = np.mean(pca_aligned, axis=0)
    fourier_centroid = np.mean(fourier_aligned, axis=0)
    
    print(f"Source Centroid: {src_centroid}")
    print(f"Target Centroid: {tgt_centroid}")
    print(f"Center Aligned Centroid: {center_centroid}")
    print(f"PCA Aligned Centroid: {pca_centroid}")
    print(f"Fourier Aligned Centroid: {fourier_centroid}")
    
    # Compute error metric
    from scipy.spatial.distance import cdist
    center_error = np.mean(np.min(cdist(center_aligned, target), axis=1))
    pca_error = np.mean(np.min(cdist(pca_aligned, target), axis=1))
    fourier_error = np.mean(np.min(cdist(fourier_aligned, target), axis=1))
    
    # Visualization
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 6))
    
    ax1.scatter(target[:, 0], target[:, 1], c='blue', s=10, label='Target')
    ax1.scatter(source[:, 0], source[:, 1], c='red', s=10, label='Source')
    ax1.scatter(center_aligned[:, 0], center_aligned[:, 1], c='green', s=10, label='Center Aligned')
    ax1.plot([tgt_centroid[0]], [tgt_centroid[1]], 'b+', markersize=15, label='Target Center')
    ax1.plot([center_centroid[0]], [center_centroid[1]], 'g+', markersize=15, label='Center')
    ax1.legend()
    ax1.set_title(f'Center Alignment (Err: {center_error:.4f})')
    ax1.axis('equal')
    
    ax2.scatter(target[:, 0], target[:, 1], c='blue', s=10, label='Target')
    ax2.scatter(source[:, 0], source[:, 1], c='red', s=10, label='Source')
    ax2.scatter(pca_aligned[:, 0], pca_aligned[:, 1], c='green', s=10, label='PCA Pre-aligned')
    ax2.plot([tgt_centroid[0]], [tgt_centroid[1]], 'b+', markersize=15, label='Target Center')
    ax2.plot([pca_centroid[0]], [pca_centroid[1]], 'g+', markersize=15, label='PCA Center')
    ax2.legend()
    ax2.set_title(f'PCA + Center (Err: {pca_error:.4f})')
    ax2.axis('equal')
    
    ax3.scatter(target[:, 0], target[:, 1], c='blue', s=10, label='Target')
    ax3.scatter(source[:, 0], source[:, 1], c='red', s=10, label='Source')
    ax3.scatter(fourier_aligned[:, 0], fourier_aligned[:, 1], c='green', s=10, label='Fourier Pre-aligned')
    ax3.plot([tgt_centroid[0]], [tgt_centroid[1]], 'b+', markersize=15, label='Target Center')
    ax3.plot([fourier_centroid[0]], [fourier_centroid[1]], 'g+', markersize=15, label='Fourier Center')
    ax3.legend()
    ax3.set_title(f'Fourier + Center (Err: {fourier_error:.4f})')
    ax3.axis('equal')
    
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    test_pre_alignments()