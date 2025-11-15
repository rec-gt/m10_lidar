from Utils import ConfigSystem, M10Lidar, PlotLidar, Utils, Debouncer, RS485Client, ModbusRTUServer


class DetectSystem:
    is_detected = False

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
        self.is_detected = False
        for point in points:
            if self.__is_inside_boundary(point, boundary):
                self.is_detected = True
        return self.is_detected


configSystem = ConfigSystem()
detectSystem = DetectSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
modbusRTUServer = ModbusRTUServer()

configSystem.read()
m10Lidar.connect()
plotLidar.init()
modbusRTUServer.init()

debouncer = Debouncer()


def cb():
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(configSystem.boundary_points)
    plotLidar.plot_calibration_point(configSystem.calibration_point)
    if detectSystem.is_one_detected(m10Lidar.points, configSystem.boundary_points):
        print("detected")


import random


def cb2():
    if detectSystem.is_detected:
        modbusRTUServer.update_hr(0, random.randint(100, 999))
        pass


modbusRTUServer.start_server_thread()
while True:
    m10Lidar.listen()
    debouncer.auto_counter(100, cb)
    debouncer.auto_timeout(1, cb2)
