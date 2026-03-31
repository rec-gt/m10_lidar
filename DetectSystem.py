import threading

from shapely.geometry import Point, Polygon

from Main import SystemConfig, M10Lidar, PlotLidar, ModbusRTUServer
from Utils import Debouncer


class DetectSystem:
    points = []
    boundary = []
    boundary_polygon = None
    is_inbound = False
    consecutive_count = 0
    is_detected = False

    @staticmethod
    def __is_inside_boundary(point, polygon):
        if point[0] is None or point[1] is None:
            return False

        point = Point(point[0], point[1])
        inside = polygon.contains(point)
        return inside

    def set_points(self, points):
        self.points = points

    def set_boundary(self, boundary):
        self.boundary = boundary
        self.boundary_polygon = Polygon(boundary)

    def listen(self):
        self.is_inbound = False
        for point in self.points:
            if self.__is_inside_boundary(point, self.boundary_polygon):
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
debouncer2 = Debouncer()

detectSystem.set_boundary(systemConfig.boundary)


def cb():
    (
        plotLidar
        .plot_cloud_points(m10Lidar.points_clean)
        .plot_boundary(detectSystem.boundary)
    )
    detectSystem.set_points(m10Lidar.points_clean)
    detectSystem.listen()


def cb3():
    lidar_err = 1 if m10Lidar.curr_err > 0 else 0
    inbound_detection = 1 if detectSystem.is_detected else 0
    modbusRTUServer.update_ir(0, [lidar_err, inbound_detection])
    print(inbound_detection)


modbusRTUServer.start_server_thread()


def thd1():
    while True:
        m10Lidar.listen()


thread1 = threading.Thread(target=thd1)

thread1.start()

while True:
    debouncer.auto_counter(10, cb)
    debouncer2.auto_timeout(1, cb3)
