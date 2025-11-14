from Utils import ConfigSystem, M10Lidar, PlotLidar, Utils


class DetectSystem:
    @staticmethod
    def __is_inside_boundary(point, boundary):
        inside = False

        n = len(boundary)
        px, py = point
        for i in range(n):
            x1, y1 = boundary[i]
            x2, y2 = boundary[(i + 1) % n]

            if min(y1, y2) < py <= max(y1, y2):
                x_intersect = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
                if px < x_intersect:
                    inside = not inside

        return inside

    def is_one_detected(self, points, boundary):
        for point in points:
            if self.__is_inside_boundary(point, boundary):
                return True
        return False


detectSystem = DetectSystem()
configSystem = ConfigSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()

configSystem.read()
m10Lidar.connect()
plotLidar.init()

while True:
    m10Lidar.listen()
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(configSystem.boundary_points)
    plotLidar.plot_calibration_point(configSystem.calibration_point)
    if detectSystem.is_one_detected(m10Lidar.points, configSystem.boundary_points):
        print("detected")
