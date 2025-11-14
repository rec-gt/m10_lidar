import math
import time

import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __repr__(self):
        return f"({self.x}, {self.y})"


# Function to find the orientation of three points (p, q, r)
# 0 --> Collinear
# 1 --> Clockwise
# 2 --> Counterclockwise
def orientation(p, q, r):
    val = (q.y - p.y) * (r.x - q.x) - \
          (q.x - p.x) * (r.y - q.y)
    if val == 0:
        return 0  # Collinear
    elif val > 0:
        return 1  # Clockwise
    else:
        return 2  # Counterclockwise


# Function to calculate squared Euclidean distance
def distSq(p1, p2):
    return (p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2


# Global variable to store the bottom-most point
p0 = None


def compare_points(p1, p2):
    global p0

    # Find orientation
    o = orientation(p0, p1, p2)
    if o == 0:  # Collinear
        if distSq(p0, p2) >= distSq(p0, p1):
            return -1  # p2 is farther or equal, so p1 comes first
        else:
            return 1  # p1 is farther, so p2 comes first
    else:
        if o == 2:  # Counterclockwise
            return -1
        else:
            return 1


def graham_scan(points):
    global p0

    n = len(points)
    if n < 3:
        return points  # Convex hull is the points themselves if less than 3

    # Find the bottom-most point (and left-most in case of tie)
    min_y = points[0].y
    min_idx = 0
    for i in range(1, n):
        y = points[i].y
        if (y < min_y) or (y == min_y and points[i].x < points[min_idx].x):
            min_y = y
            min_idx = i

    # Swap the bottom-most point with the first point
    points[0], points[min_idx] = points[min_idx], points[0]
    p0 = points[0]

    # Sort the remaining n-1 points by polar angle with respect to p0
    # If angles are same, sort by distance from p0
    points[1:] = sorted(points[1:], key=cmp_to_key(compare_points))

    # Remove collinear points that are not farthest from p0
    m = 1
    for i in range(1, n):
        while i < n - 1 and orientation(p0, points[i], points[i + 1]) == 0:
            i += 1
        points[m] = points[i]
        m += 1
    n = m

    if n < 3:
        return points[:n]

    # Create an empty stack and push first three points
    stack = []
    stack.append(points[0])
    stack.append(points[1])
    stack.append(points[2])

    # Process remaining points
    for i in range(3, n):
        while len(stack) > 1 and orientation(stack[-2], stack[-1], points[i]) != 2:
            stack.pop()
        stack.append(points[i])

    return stack


# Helper for sorting with a custom comparison function (Python 2 style)
from functools import cmp_to_key

app = QApplication([])
win = pg.GraphicsLayoutWidget(show=True, title="M10 Lidar Real-time Plot")
p = win.addPlot(title="M10 Lidar Real-time Plot")
p.setXRange(-11000, 11000)
p.setYRange(-11000, 11000)
scatter = p.scatterPlot(size=5, pen=pg.mkPen(color='r', width=2), symbol='o')
scatter_center = p.plot(size=10, pen=pg.mkPen(color='y', width=3), symbol='o')
scatter_center.setData(x=[0], y=[0])
app.processEvents()

# Example Usage:
if __name__ == "__main__":
    points_data = [(0, 3), (1, 1), (2, 2), (4, 4), (0, 0), (1, 2), (3, 1), (3, 3)]
    points = [Point(x, y) for x, y in points_data]

    hull = graham_scan(points)
    print("Convex Hull Points:")
    for p in hull:
        print(p)

