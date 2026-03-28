import json
import math
import time
import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import QApplication

import threading

from pymodbus import ModbusDeviceIdentification
from pymodbus.datastore import ModbusSequentialDataBlock, ModbusDeviceContext, ModbusServerContext
from pymodbus.server import StartSerialServer

import platform

from Utils import Utils

CONFIG_SYSTEM = None
CONFIG_POINTS = None
CONFIG_POLYGON = None

with open("Config_System.json", 'r') as f:
    CONFIG_SYSTEM = json.load(f)

with open("Config_Points.json", 'r') as f:
    CONFIG_POINTS = json.load(f)

try:
    with open("Config_Polygon.json", 'r') as f:
        CONFIG_POLYGON = json.load(f)
except Exception:
    CONFIG_POLYGON = []


### === Configuration === ###

class OSConfig:
    __os_name = platform.system().upper()
    __OS_LINUX = CONFIG_SYSTEM["OS"]["LINUX"]
    __OS_WINDOWS = CONFIG_SYSTEM["OS"]["WINDOWS"]

    plot_app = None
    ser_m10 = None
    modbus_rtu_port = None

    if CONFIG_SYSTEM["PLOTTING"]:
        if __os_name == __OS_LINUX["NAME"]:
            plot_app = pg.mkQApp("")
        if __os_name == __OS_WINDOWS["NAME"]:
            plot_app = QApplication([])

    if __os_name == __OS_LINUX["NAME"]:
        ser_m10 = serial.Serial(__OS_LINUX["PORTS"]["LIDAR"], 460800, timeout=1)
    if __os_name == __OS_WINDOWS["NAME"]:
        ser_m10 = serial.Serial(__OS_WINDOWS["PORTS"]["LIDAR"], 460800, timeout=1)

    if __os_name == __OS_LINUX["NAME"]:
        modbus_rtu_port = __OS_LINUX["PORTS"]["IOT_CONTROLLER"]
    if __os_name == __OS_WINDOWS["NAME"]:
        modbus_rtu_port = __OS_WINDOWS["PORTS"]["IOT_CONTROLLER"]


class SystemConfig:
    boundary_points = (())
    calibration_point = ()
    polygon_points = (())
    boundary_profile = "DEFAULT"
    boundary = (())

    def __init__(self):
        self.boundary_points = (())
        self.calibration_point = ()
        self.polygon_points = (())
        self.boundary_profile = "DEFAULT"
        self.boundary = (())

    def read(self):
        self.boundary_points = (
            (CONFIG_POINTS["TOP_LEFT_POINT"]["x"], CONFIG_POINTS["TOP_LEFT_POINT"]["y"]),
            (CONFIG_POINTS["TOP_RIGHT_POINT"]["x"], CONFIG_POINTS["TOP_RIGHT_POINT"]["y"]),
            (CONFIG_POINTS["BOTTOM_RIGHT_POINT"]["x"], CONFIG_POINTS["BOTTOM_RIGHT_POINT"]["y"]),
            (CONFIG_POINTS["BOTTOM_LEFT_POINT"]["x"], CONFIG_POINTS["BOTTOM_LEFT_POINT"]["y"]),
            (CONFIG_POINTS["TOP_LEFT_POINT"]["x"], CONFIG_POINTS["TOP_LEFT_POINT"]["y"]),
        )

        self.calibration_point = (
            CONFIG_POINTS["CALIBRATION_POINT"]["x"], CONFIG_POINTS["CALIBRATION_POINT"]["y"])

        self.polygon_points = CONFIG_POLYGON

        self.boundary_profile = CONFIG_SYSTEM["BOUNDARY_PROFILE"]

        if self.boundary_profile == "DEFAULT":
            self.boundary = self.boundary_points
        else:
            self.boundary = self.polygon_points


curr_os = OSConfig()


### === System Core === ###


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
    xs_clean = []
    ys_clean = []
    points = []
    points_clean = []

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
                # if distance is not None:
                #     res.append([angle + delta_angle, distance])
                if distance is not None:
                    res.append([angle + delta_angle, distance])
                else:
                    res.append([None, None])
                delta_angle += 360 / 1008
        return res

    @staticmethod
    def __transform_to_coordinates(angle_distance_pairs):
        _xs = []
        _ys = []
        for (angle, distance) in angle_distance_pairs:
            if angle is None or distance is None:
                _xs.append(None)
                _ys.append(None)
            else:
                radian = angle * math.pi / 180
                sin_theta = math.sin(radian)
                cos_theta = math.cos(radian)
                _xs.append(distance * sin_theta)
                _ys.append(distance * cos_theta)
        return _xs, _ys

    @staticmethod
    def __clean_coordinates(xs, ys):
        _xs = []
        _ys = []
        for i in range(0, len(xs)):
            if xs[i] is not None:
                _xs.append(xs[i])
                _ys.append(ys[i])
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

                            # print(len(self.distance_cloud[8]))
                            # print(len(self.distance_cloud[23]))
                            # print(len(self.distance_cloud[38]))
                            # print(len(self.distance_cloud[53]))
                            # print(len(self.distance_cloud[68]))
                            # print(len(self.distance_cloud[83]))
                            # print(len(self.distance_cloud[98]))
                            # print(len(self.distance_cloud[113]))
                            # print(len(self.distance_cloud[128]))
                            # print(len(self.distance_cloud[143]))
                            # print(len(self.distance_cloud[158]))
                            # print(len(self.distance_cloud[173]))
                            # print(len(self.distance_cloud[188]))
                            # print(len(self.distance_cloud[203]))
                            # print(len(self.distance_cloud[218]))
                            # print(len(self.distance_cloud[233]))
                            # print(len(self.distance_cloud[248]))
                            # print(len(self.distance_cloud[263]))
                            # print(len(self.distance_cloud[278]))
                            # print(len(self.distance_cloud[293]))
                            # print(len(self.distance_cloud[308]))
                            # print(len(self.distance_cloud[323]))
                            # print(len(self.distance_cloud[338]))
                            # print(len(self.distance_cloud[353]))

                            angle_distance_pairs = self.__parse_angle_distance_pairs(self.distance_cloud)
                            self.xs, self.ys = self.__transform_to_coordinates(angle_distance_pairs)
                            self.xs_clean, self.ys_clean = self.__clean_coordinates(self.xs, self.ys)

                            self.points = self.__transform_to_points(self.xs, self.ys)
                            self.points_clean = self.__transform_to_points(self.xs_clean, self.ys_clean)


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

    def init(self):
        if CONFIG_SYSTEM["PLOTTING"]:
            self.plot_app = curr_os.plot_app
            self.plot_win = pg.GraphicsLayoutWidget(show=True, title="M10 Lidar Real-time Plot")
            self.plot_plt = self.plot_win.addPlot(title="M10 Lidar Real-time Plot")
            self.plot_plt.setXRange(-8000, 8000)
            self.plot_plt.setYRange(-8000, 8000)

            self.scatter_dynamic = self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='r', width=1), symbol='o')
            self.scatter_center = self.plot_plt.scatterPlot(size=6, pen=pg.mkPen(color='g', width=6), symbol='o')
            self.scatter_range = [self.plot_plt.plot(pen=pg.mkPen(color='y', width=2)),
                                  self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='y', width=2))]
            self.scatter_calibrate = self.plot_plt.scatterPlot(size=6, pen=pg.mkPen(color='y', width=6), symbol='x')
            self.scatter_center.setData(x=[0], y=[0])
            # self.scatter_range.setData(x=xs, y=ys)
            # self.scatter_range = self.plot_plt.scatterPlot(size=3, pen=pg.mkPen(color='g', width=1), symbol='o')
            # self.scatter_range.setData(x=xs, y=ys)

    def plot_cloud_points(self, xs, ys):
        if CONFIG_SYSTEM["PLOTTING"]:
            self.scatter_dynamic.setData(x=xs, y=ys)
            self.plot_app.processEvents()
            time.sleep(0.05)
        return self

    def plot_boundary(self, boundary_points):
        if CONFIG_SYSTEM["PLOTTING"]:
            xs, ys = Utils.point_to_xs_ys(boundary_points)
            for sr in self.scatter_range:
                sr.setData(x=xs, y=ys)
        return self

    def plot_calibration_point(self, calibration_point):
        x = calibration_point[0]
        y = calibration_point[1]
        if CONFIG_SYSTEM["PLOTTING"]:
            self.scatter_calibrate.setData(x=[x], y=[y])
        return self


class ModbusRTUServer:
    server_thread = None

    identity = None
    store = None
    context = None

    def init(self):
        self.identity = ModbusDeviceIdentification()
        self.identity.VendorName = 'RGT'
        self.identity.ProductCode = 'RGT-LIDAR'
        self.identity.ProductName = 'RGT-LIDAR'

    def loop(self):
        while True:
            self.store = ModbusDeviceContext(
                hr=ModbusSequentialDataBlock(0, [17] * 100),  # start from 40000
            )

            self.context = ModbusServerContext(devices=self.store, single=True)

            print("Starting Modbus RTU Server on COM port...")

            def trace_packet(is_request: bool, packet: bytes) -> bytes:
                direction = ">> TX (Request)" if is_request else "<< RX (Response)"
                # print(f"{direction}: {str(int(packet, 16))}")
                print(f"{direction}: {list(map(int, packet))}")
                return packet  # Return unchanged

            def trace_pdu(is_request: bool, packet: bytes) -> bytes:
                direction = ">> TX (Request)" if is_request else "<< RX (Response)"
                print(f"{direction}: {packet}")
                return packet  # Return unchanged

            StartSerialServer(
                context=self.context,
                identity=self.identity,
                port=curr_os.modbus_rtu_port,
                baudrate=1200,
                bytesize=8,
                parity='N',
                stopbits=1,
                timeout=1,
                # framer=FramerType.RTU,
                # trace_packet=trace_packet
            )

    def start_server_thread(self):
        if self.server_thread is None or not self.server_thread.is_alive():
            self.server_thread = threading.Thread(target=self.loop, daemon=True)
            self.server_thread.start()
            print("[Main] Server thread started.")
        else:
            print("[Main] Server already running.")

    def update_ir(self, address, values):
        if self.store:
            self.store.setValues(4, address, values)  # input registers
