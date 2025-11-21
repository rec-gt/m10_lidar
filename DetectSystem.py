import json

from Main import SystemConfig, M10Lidar, PlotLidar, Debouncer, ModbusRTUServer


class DetectSystem:
    is_detected = False
    boundary = []

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

    def set_boundary(self, boundary):
        self.boundary = boundary

    def is_one_detected(self, points):
        self.is_detected = False
        for point in points:
            if self.__is_inside_boundary(point, self.boundary):
                self.is_detected = True
        return self.is_detected


systemConfig = SystemConfig()
detectSystem = DetectSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
modbusRTUServer = ModbusRTUServer()

systemConfig.read()
m10Lidar.connect()
plotLidar.init()
modbusRTUServer.init()

debouncer = Debouncer()

detectSystem.set_boundary(
    systemConfig.boundary_points if systemConfig.boundary_profile == "DEFAULT" else systemConfig.polygon_points)


def cb():
    (
        plotLidar
        .plot_cloud_points(m10Lidar.xs, m10Lidar.ys)
        .plot_boundary(detectSystem.boundary)
    )

    detectSystem.is_one_detected(m10Lidar.points)


def cb2():
    lidar_err = 1 if m10Lidar.curr_err > 0 else 0
    inbound_detection = 1 if detectSystem.is_detected else 0

    modbusRTUServer.update_ir(0, [lidar_err, inbound_detection])


modbusRTUServer.start_server_thread()

while True:
    m10Lidar.listen()
    debouncer.auto_counter(100, cb)
    debouncer.auto_timeout(1, cb2)
