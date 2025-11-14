import math
import os
import time
from enum import Enum

import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import QApplication


class OSConfig(Enum):
    WINDOWS = 0,
    LINUX = 1


CONFIG = {
    "OS": os.getenv('OSS'),
    "PLOTTING": True,
    # "PLOTTING": os.getenv('PLOTTING') == 'True',
}


# load_dotenv()
#
#
# with open("polygon.json", 'r') as json_file:
#     polygon = json.load(json_file)
#

class OSWindows:
    plot_app = QApplication([]) if CONFIG["PLOTTING"] else None
    ser_m10 = serial.Serial("COM25", 460800, timeout=1)
    ser_to_mcu = serial.Serial("COM22", 9600, timeout=1)


class OSLinux:
    plot_app = pg.mkQApp("") if CONFIG["PLOTTING"] else None
    ser_m10 = serial.Serial("/dev/ttyACM0", 460800, timeout=1) if CONFIG["OS"] == OSConfig.LINUX.name else None
    ser_to_mcu = serial.Serial("/dev/ttyS0", 9600, timeout=1) if CONFIG["OS"] == OSConfig.LINUX.name else None


#
# class Service:
#     curr_os = OSWindows()
#
#     ser_m10 = None
#     ser_to_mcu = None
#
#     plot_app = None
#     plot_win = None
#     plot_plt = None
#     scatter = None
#     relay = None
#     prevTime = time.time()
#
#     state = 0b00000000
#
#     distance_cloud = {
#         8: [],
#         23: [],
#         38: [],
#         53: [],
#         68: [],
#         83: [],
#         98: [],
#         113: [],
#         128: [],
#         143: [],
#         158: [],
#         173: [],
#         188: [],
#         203: [],
#         218: [],
#         233: [],
#         248: [],
#         263: [],
#         278: [],
#         293: [],
#         308: [],
#         323: [],
#         338: [],
#         353: [],
#     }
#
#     sin_values = []
#     cos_values = []
#
#     counter = 0
#     curr_err = 0

#
#     def conn(self):
#         if self.ser_m10 and self.ser_m10.is_open:
#             self.ser_m10.close()
#             self.ser_m10 = None
#
#         if self.ser_to_mcu and self.ser_to_mcu.is_open:
#             self.ser_to_mcu.close()
#             self.ser_to_mcu = None
#
#         self.ser_m10 = self.curr_os.ser_m10
#         self.ser_to_mcu = self.curr_os.ser_to_mcu
#
#     def parse_data(self, data):
#         start_angle = (data[0] * 256 + data[1]) / 100.0
#         speed = data[2] * 256 + data[3]
#         distances = []
#
#         for x in range(4, 87, 2):
#             distance = data[x] * 256 + data[x + 1]
#             if distance != 0 and distance != 65535:
#                 distances.append(distance)
#             else:
#                 distances.append(None)
#
#         return speed, start_angle, distances
#
#     def get_angle_distance_pairs_from_cloud(self):
#         res = []
#         for angle, distances in self.distance_cloud.items():
#             delta_angle = 0
#             for distance in distances:
#                 if distance is not None:
#                     res.append([angle + delta_angle, distance])
#                 delta_angle += 360 / 1008
#         return res
#
#     def radian_distance_to_coordinate(self, angle_distance_pairs):
#         sin_values = []
#         cos_values = []
#
#         for (angle, distance) in angle_distance_pairs:
#             radian = angle * math.pi / 180
#             sin_theta = math.sin(radian)
#             cos_theta = math.cos(radian)
#             sin_values.append(distance * sin_theta)
#             cos_values.append(distance * cos_theta)
#
#         return sin_values, cos_values
#
#     def inbound_detect(self):
#         length = len(self.sin_values)
#         for i in range(length):
#             x = self.sin_values[i]
#             y = self.cos_values[i]
#
#             if self.is_point_in_polygon(x, y, polygon):
#                 print(x, y)
#                 return True
#
#         return False
#
#     def is_point_in_polygon(self, x, y, polygon):
#         n = len(polygon)
#         inside = False
#
#         px, py = x, y
#         for i in range(n):
#             x1, y1 = polygon[i]
#             x2, y2 = polygon[(i + 1) % n]
#
#             # Check if the point is within the y-range of the edge
#             if min(y1, y2) < py <= max(y1, y2):
#                 # Calculate the x-coordinate where the ray intersects the edge
#                 x_intersect = x1 + (py - y1) * (x2 - x1) / (y2 - y1)
#                 if px < x_intersect:  # The ray crosses the edge
#                     inside = not inside
#
#         return inside
#
#     def handle_comm_mcu(self):
#         currTime = time.time()
#         if currTime - self.prevTime > 2:
#             string = "1"
#             self.prevTime = currTime
#
#             string += "1" if self.inbound_detect() else "0"
#             self.ser_to_mcu.write(bytes(b'AT+STATUS=' + bytes(string, 'utf-8') + b'\r\n'))
#
#     def loop(self):
#         while True:
#             self.handle_comm_mcu()
#
#             if self.curr_err == 1:
#                 try:
#                     self.conn()
#                     self.curr_err = 0
#                 except Exception as e:
#                     print(e)
#                     continue
#
#             if self.curr_err == 0:
#                 try:
#                     if self.ser_m10.in_waiting > 0:
#                         data = self.ser_m10.read(1)
#                         if data[0] == 0xA5:
#                             data = self.ser_m10.read(1)
#                             if data[0] == 0x5A:
#                                 data = self.ser_m10.read(88)
#                                 speed, start_angle, distances = self.parse_data(data)
#                                 self.distance_cloud[start_angle] = distances
#                                 angle_distance_pairs = self.get_angle_distance_pairs_from_cloud()
#                                 self.sin_values, self.cos_values = self.radian_distance_to_coordinate(
#                                     angle_distance_pairs)
#
#                                 self.counter += 1
#                                 if self.counter == 36:
#                                     self.counter = 0
#                                     self.plot_circle(self.sin_values, self.cos_values)
#
#                 except Exception as e:
#                     print(e)
#                     self.curr_err = 1
#                     continue
#
#     def print_data(self, speed, start_angle, distances, last_angle):
#         if last_angle - start_angle > 100:
#             print("*******************************")
#
#         print("转速:", speed, end="\t")
#         print("起始角度:", start_angle, end="\t")
#         print("数据【距离（mm）】*42个点：", end="\t")
#
#         for distance in distances:
#             print(distance, end="\t")
#         print("\n")
#
#
# service = Service()
# service.curr_os = OSWindows() if CONFIG["OS"] == OSS_ENUM.WINDOWS.name else OSLinux()
# service.init_relay()
# service.init_plot()
# service.conn()
# service.loop()


curr_os = OSWindows()


class Utils:
    @staticmethod
    def point_to_xs_ys(points):
        xs = []
        ys = []
        for point in points:
            xs.append(point[0])
            ys.append(point[1])
        return xs, ys


class Debouncer:
    prev_time = time.time()

    def auto_timeout(self, timeout, callback):
        curr_time = time.time()
        if curr_time - self.prev_time >= timeout:
            callback()
            self.prev_time = curr_time


class M10Lidar:
    ser_m10 = None
    curr_err = 0
    distance_cloud = {
        8: [],
        23: [],
        38: [],
        53: [],
        68: [],
        83: [],
        98: [],
        113: [],
        128: [],
        143: [],
        158: [],
        173: [],
        188: [],
        203: [],
        218: [],
        233: [],
        248: [],
        263: [],
        278: [],
        293: [],
        308: [],
        323: [],
        338: [],
        353: [],
    }
    xs = []
    ys = []
    points = []

    @staticmethod
    def __parse_data(raw_data):
        start_angle = (raw_data[0] * 256 + raw_data[1]) / 100.0
        speed = raw_data[2] * 256 + raw_data[3]
        distances = []

        for x in range(4, 87, 2):
            distance = raw_data[x] * 256 + raw_data[x + 1]
            if distance != 0 and distance != 65535:
                distances.append(distance)
            else:
                distances.append(None)

        return speed, start_angle, distances

    @staticmethod
    def __parse_angle_distance_pairs(distance_cloud):
        res = []
        for angle, distances in distance_cloud.items():
            delta_angle = 0
            for distance in distances:
                if distance is not None:
                    res.append([angle + delta_angle, distance])
                delta_angle += 360 / 1008
        return res

    @staticmethod
    def __transform_to_coordinates(angle_distance_pairs):
        _xs = []
        _ys = []
        for (angle, distance) in angle_distance_pairs:
            radian = angle * math.pi / 180
            sin_theta = math.sin(radian)
            cos_theta = math.cos(radian)
            _xs.append(distance * sin_theta)
            _ys.append(distance * cos_theta)
        return _xs, _ys

    @staticmethod
    def __transform_to_points(_xs, _ys):
        res = []
        size = len(_xs)
        for i in range(0, size):
            x = _xs[i]
            y = _ys[i]
            res.append([x, y])
        return res

    def connect(self):
        if self.ser_m10 and self.ser_m10.is_open:
            self.ser_m10.close()
            self.ser_m10 = None

        if self.ser_m10:
            self.ser_m10.close()
            self.ser_m10 = None

        self.ser_m10 = curr_os.ser_m10
        if not self.ser_m10.is_open:
            self.ser_m10.open()

    def listen(self):
        if self.curr_err == 1:
            try:
                self.connect()
                self.curr_err = 0
            except Exception as e:
                print(e)

        if self.curr_err == 0:
            try:
                if self.ser_m10.in_waiting > 0:
                    data = self.ser_m10.read(1)
                    if data[0] == 0xA5:
                        data = self.ser_m10.read(1)
                        if data[0] == 0x5A:
                            data = self.ser_m10.read(88)
                            speed, start_angle, distances = self.__parse_data(data)
                            self.distance_cloud[start_angle] = distances
                            angle_distance_pairs = self.__parse_angle_distance_pairs(self.distance_cloud)
                            self.xs, self.ys = self.__transform_to_coordinates(angle_distance_pairs)
                            self.points = self.__transform_to_points(self.xs, self.ys)


            except Exception as e:
                print(e)
                self.curr_err = 1


class PlotLidar:
    plot_app = None
    plot_win = None
    plot_plt = None

    scatter_calibrate = None
    scatter_dynamic = None
    scatter_center = None
    scatter_range = None

    counter = 0

    def init(self):
        if CONFIG["PLOTTING"]:
            self.plot_app = curr_os.plot_app
            self.plot_win = pg.GraphicsLayoutWidget(show=True, title="M10 Lidar Real-time Plot")
            self.plot_plt = self.plot_win.addPlot(title="M10 Lidar Real-time Plot")
            self.plot_plt.setXRange(-11000, 11000)
            self.plot_plt.setYRange(-11000, 11000)

            self.scatter_dynamic = self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='r', width=1), symbol='o')
            self.scatter_center = self.plot_plt.scatterPlot(size=6, pen=pg.mkPen(color='g', width=6), symbol='o')
            self.scatter_range = [self.plot_plt.plot(pen=pg.mkPen(color='y', width=2)),
                                  self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='y', width=2))]
            self.scatter_calibrate = self.plot_plt.scatterPlot(size=6, pen=pg.mkPen(color='y', width=6), symbol='x')
            self.scatter_center.setData(x=[0], y=[0])
            # self.scatter_range.setData(x=xs, y=ys)
            # self.scatter_range = self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='g', width=1), symbol='o')
            # self.scatter_range.setData(x=xs, y=ys)

    def update_cloud_points(self, xs, ys):
        try:
            if CONFIG["PLOTTING"]:
                self.counter += 1
                if self.counter >= 100:
                    self.scatter_dynamic.setData(x=xs, y=ys)
                    self.plot_app.processEvents()
                    time.sleep(0.05)

                    self.counter = 0
        except Exception as e:
            print(e)

    def plot_boundary(self, boundary_points):
        if CONFIG["PLOTTING"]:
            xs, ys = Utils.point_to_xs_ys(boundary_points)
            for sr in self.scatter_range:
                sr.setData(x=xs, y=ys)

    def plot_calibration_point(self, calibration_point):
        x = calibration_point[0]
        y = calibration_point[1]
        if CONFIG["PLOTTING"]:
            self.scatter_calibrate.setData(x=[x], y=[y])

# m10Lidar = M10Lidar()
# plotLidar = PlotLidar()
#
# m10Lidar.connect()
# plotLidar.init()
#
# while True:
#     m10Lidar.listen()
#     plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
