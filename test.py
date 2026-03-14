import numpy as np


def weiszfeld(points, eps=1e-6, max_iter=200, initial_guess=None):
    """
    Compute approximate geometric median using Weiszfeld's algorithm.

    Parameters:
    -----------
    points : ndarray (n_points, dim)
        Input points (2D or higher)
    eps : float
        Convergence tolerance
    max_iter : int
        Maximum iterations
    initial_guess : ndarray or None
        Starting point (defaults to centroid)

    Returns:
    --------
    geometric_median : ndarray
        Approximate geometric median
    """
    points = np.asarray(points)
    if points.ndim != 2:
        raise ValueError("points should be 2D array (n_points × dim)")

    n, dim = points.shape

    # Good default start: centroid
    if initial_guess is None:
        y = np.mean(points, axis=0)
    else:
        y = np.asarray(initial_guess, dtype=float)
        if y.shape != (dim,):
            raise ValueError("initial_guess must have same dimension as points")

    for iteration in range(max_iter):
        y_old = y.copy()

        # Distances to current estimate
        distances = np.linalg.norm(points - y, axis=1)  # shape (n,)

        # Avoid division by zero (very rare in float)
        # Small epsilon safeguard + common practical fix
        too_close = distances < 1e-10
        if np.any(too_close):
            # If y coincides with a point → that point is likely the median
            if np.sum(too_close) == 1:
                return points[np.argmax(too_close)]
            # Otherwise continue with tiny epsilon
            distances[too_close] = 1e-10

        weights = 1.0 / distances  # shape (n,)
        numerator = np.sum(points * weights[:, np.newaxis], axis=0)  # shape (dim,)
        denominator = np.sum(weights)

        y = numerator / denominator

        # Check convergence
        if np.linalg.norm(y - y_old) < eps:
            print(f"Converged after {iteration + 1} iterations")
            return y

    print(f"Warning: Did not converge within {max_iter} iterations")
    return y

import matplotlib.pyplot as plt

# Example: noisy circle + one outlier
np.random.seed(42)
n = 60
theta = np.linspace(0, 2*np.pi, n, endpoint=False)
r = 1.0 + np.random.normal(0, 0.15, n)
x = r * np.cos(theta) + 3
y = r * np.sin(theta) + 4

# Add one strong outlier
x = np.append(x, 12)
y = np.append(y, 1)

points = np.column_stack((x, y))

# Compute
median_weisz = weiszfeld(points, eps=1e-7, max_iter=300)

# Compare with centroid
centroid = np.mean(points, axis=0)

plt.figure(figsize=(9,7))
plt.scatter(points[:,0], points[:,1], s=60, alpha=0.7, label='points')
plt.scatter(centroid[0], centroid[1], c='C1', s=180, marker='*', label='centroid (arithmetic mean)')
plt.scatter(median_weisz[0], median_weisz[1], c='C3', s=180, marker='X', label='geometric median (Weiszfeld)')
plt.legend()
plt.axis('equal')
plt.title("Geometric Median vs Centroid\n(notice robustness to outlier)")
plt.grid(alpha=0.3)
plt.show()