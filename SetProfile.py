import math
import time
import json

import numpy as np
import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import QApplication
import os
from dotenv import load_dotenv
from enum import Enum

load_dotenv()

CFG_OSS = os.getenv('OSS')
CFG_PLOTTING = os.getenv('PLOTTING') == 'True'
TOP_L_POINT = json.loads(os.getenv('TOP_L_POINT'))
TOP_R_POINT = json.loads(os.getenv('TOP_R_POINT'))
BOTTOM_L_POINT = json.loads(os.getenv('BOTTOM_L_POINT'))
BOTTOM_R_POINT = json.loads(os.getenv('BOTTOM_R_POINT'))

profile_coordinates = []


class OSS_ENUM(Enum):
    WINDOWS = 0,
    LINUX = 1


class OSWindows:
    plot_app = QApplication([]) if CFG_PLOTTING else None
    ser_to_lidar = serial.Serial("COM25", 460800, timeout=1)
    ser_to_mcu = serial.Serial("COM22", 9600, timeout=1)


class OSLinux:
    plot_app = pg.mkQApp("") if CFG_PLOTTING else None
    ser_to_lidar = serial.Serial("/dev/ttyACM0", 460800, timeout=1) if CFG_OSS == OSS_ENUM.LINUX.name else None
    ser_to_mcu = serial.Serial("/dev/ttyS0", 9600, timeout=1) if CFG_OSS == OSS_ENUM.LINUX.name else None


class Service:
    curr_os = OSWindows()

    ser_to_lidar = None
    ser_to_mcu = None

    plot_app = None
    plot_win = None
    plot_p = None
    scatter = None
    prevTime = time.time()

    state = 0b00000000

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

    sin_values = []
    cos_values = []

    counter = 0
    curr_err = 0

    def init_plot(self):
        self.plot_app = self.curr_os.plot_app

        if CFG_PLOTTING:
            self.plot_win = pg.GraphicsLayoutWidget(show=True, title="M10 Lidar Real-time Plot")
            self.plot_p = self.plot_win.addPlot(title="M10 Lidar Real-time Plot")
            self.plot_p.setXRange(-11000, 11000)
            self.plot_p.setYRange(-11000, 11000)
            self.scatter = self.plot_p.scatterPlot(size=3, pen=pg.mkPen(color='r', width=1), symbol='o')
            scatter_center = self.plot_p.scatterPlot(size=10, pen=pg.mkPen(color='y', width=3), symbol='o')
            scatter_center.setData(x=[0], y=[0])

    def plot_circle(self, sin_values, cos_values):
        if CFG_PLOTTING:
            self.scatter.setData(x=sin_values, y=cos_values)
            self.plot_app.processEvents()
            time.sleep(0.05)

    def shrink_coordinates(self, points, shrink_amount=1):
        for point in points:
            if point[0] > 0:
                point[0] -= shrink_amount
            if point[0] < 0:
                point[0] += shrink_amount

            if point[1] > 0:
                point[1] -= shrink_amount
            if point[1] < 0:
                point[1] += shrink_amount

        return points

    def set_profile_coordinates(self, sin_values, cos_values):
        coordinates = {
            "x": sin_values,
            "y": cos_values
        }

        polygon = []
        for i in range(0, len(coordinates["x"])):
            polygon.append([coordinates["x"][i], coordinates["y"][i]])

        shrinked_polygon = self.shrink_coordinates(polygon, 50)

        with open("polygon.json", 'w') as json_file:
            json.dump(shrinked_polygon, json_file, indent=4)
        print("Updated polygon profile")

    def conn(self):
        if self.ser_to_lidar and self.ser_to_lidar.is_open:
            self.ser_to_lidar.close()
            self.ser_to_lidar = None

        if self.ser_to_mcu and self.ser_to_mcu.is_open:
            self.ser_to_mcu.close()
            self.ser_to_mcu = None

        self.ser_to_lidar = self.curr_os.ser_to_lidar
        self.ser_to_mcu = self.curr_os.ser_to_mcu

    def parse_data(self, data):
        start_angle = (data[0] * 256 + data[1]) / 100.0
        speed = data[2] * 256 + data[3]
        distances = []

        for x in range(4, 87, 2):
            distance = data[x] * 256 + data[x + 1]
            if distance != 0 and distance != 65535:
                distances.append(distance)
            else:
                distances.append(None)

        return speed, start_angle, distances

    def get_angle_distance_pairs_from_cloud(self):
        res = []
        for angle, distances in self.distance_cloud.items():
            delta_angle = 0
            for distance in distances:
                if distance is not None:
                    res.append([angle + delta_angle, distance])
                delta_angle += 360 / 1008
        return res

    def radian_distance_to_coordinate(self, angle_distance_pairs):
        sin_values = []
        cos_values = []

        for (angle, distance) in angle_distance_pairs:
            radian = angle * math.pi / 180
            sin_theta = math.sin(radian)
            cos_theta = math.cos(radian)
            sin_values.append(distance * sin_theta)
            cos_values.append(distance * cos_theta)

        return sin_values, cos_values

    def inbound_detect(self):
        length = len(self.sin_values)
        for i in range(length):
            x = self.sin_values[i]
            y = self.cos_values[i]
            if (TOP_L_POINT[0] <= x <= BOTTOM_R_POINT[0]) and (TOP_L_POINT[1] <= y <= BOTTOM_R_POINT[1]):
                print(x, y)
                return True
        return False

    def handle_comm_mcu(self):
        currTime = time.time()
        if currTime - self.prevTime > 2:
            string = "1"
            self.prevTime = currTime

            string += "1" if self.inbound_detect() else "0"
            self.ser_to_mcu.write(bytes(b'AT+STATUS=' + bytes(string, 'utf-8') + b'\r\n'))

    def loop(self):
        while True:
            self.handle_comm_mcu()

            if self.curr_err == 1:
                try:
                    self.conn()
                    self.curr_err = 0
                except Exception as e:
                    print(e)
                    continue

            if self.curr_err == 0:
                try:
                    if self.ser_to_lidar.in_waiting > 0:
                        data = self.ser_to_lidar.read(1)
                        if data[0] == 0xA5:
                            data = self.ser_to_lidar.read(1)
                            if data[0] == 0x5A:
                                data = self.ser_to_lidar.read(88)
                                speed, start_angle, distances = self.parse_data(data)
                                self.distance_cloud[start_angle] = distances
                                angle_distance_pairs = self.get_angle_distance_pairs_from_cloud()
                                self.sin_values, self.cos_values = self.radian_distance_to_coordinate(
                                    angle_distance_pairs)

                                self.counter += 1
                                if self.counter == 36:
                                    self.counter = 0
                                    self.plot_circle(self.sin_values, self.cos_values)
                                    self.set_profile_coordinates(self.sin_values, self.cos_values)

                except Exception as e:
                    print(e)
                    self.curr_err = 1
                    continue


service = Service()
service.curr_os = OSWindows() if CFG_OSS == OSS_ENUM.WINDOWS.name else OSLinux()
service.init_plot()
service.conn()
service.loop()
