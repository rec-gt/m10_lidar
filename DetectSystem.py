from Main import SystemConfig, M10Lidar, PlotLidar, ModbusRTUServer
from Utils import Debouncer


class DetectSystem:
    points = []
    boundary = []

    is_inbound = False
    consecutive_count = 0
    is_detected = False

    @staticmethod
    def __is_inside_boundary(point, boundary):
        if point[0] is None or point[1] is None:
            return False

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

    def set_points(self, points):
        self.points = points

    def set_boundary(self, boundary):
        self.boundary = boundary

    def listen(self):
        self.is_inbound = False
        for point in self.points:
            if self.__is_inside_boundary(point, self.boundary):
                self.is_inbound = True

        if self.is_inbound:
            self.consecutive_count += 1
        else:
            self.consecutive_count = 0

        self.is_detected = self.consecutive_count > 1


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

detectSystem.set_boundary(systemConfig.boundary)


def cb():
    (
        plotLidar
        .plot_cloud_points(m10Lidar.points_clean)
        .plot_boundary(detectSystem.boundary)
    )

    detectSystem.set_points(m10Lidar.points)
    detectSystem.listen()


def cb2():
    lidar_err = 1 if m10Lidar.curr_err > 0 else 0
    inbound_detection = 1 if detectSystem.is_detected else 0
    modbusRTUServer.update_ir(0, [lidar_err, inbound_detection])
    print(inbound_detection)


modbusRTUServer.start_server_thread()

while True:
    m10Lidar.listen()
    debouncer.auto_counter(100, cb)
    debouncer.auto_timeout(1, cb2)
