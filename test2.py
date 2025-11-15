import math
import time
import json

import pyqtgraph as pg
import serial
from PyQt5.QtWidgets import QApplication
from gpiozero import LED
import os
from dotenv import load_dotenv

load_dotenv()

CFG_OSS = os.getenv('OSS')
CFG_PLOTTING = os.getenv('PLOTTING') == 'True'
TOP_L_POINT = json.loads(os.getenv('TOP_L_POINT'))
TOP_R_POINT = json.loads(os.getenv('TOP_R_POINT'))
BOTTOM_L_POINT = json.loads(os.getenv('BOTTOM_L_POINT'))
BOTTOM_R_POINT = json.loads(os.getenv('BOTTOM_R_POINT'))

ser = None
ser_to_mcu = None

state = 0b00000000


def connect_to_serial():
    global ser
    global ser_to_mcu

    if ser and ser.is_open:
        ser.close()
        ser = None
    else:
        if CFG_OSS == "WINDOWS":
            ser = serial.Serial("COM25", 460800, timeout=1)
            ser_to_mcu = serial.Serial("COM9", 9600, timeout=1)
        else:
            ser = serial.Serial("/dev/ttyACM0", 460800, timeout=1)
            # ser_to_mcu = serial.Serial("/dev/ttyS0", 9600, timeout=1)


if CFG_OSS == "WINDOWS":
    app = QApplication([])
else:
    if CFG_PLOTTING:
        app = pg.mkQApp("")
    relay = LED(17)

if CFG_PLOTTING:
    win = pg.GraphicsLayoutWidget(show=True, title="M10 Lidar Real-time Plot")
    p = win.addPlot(title="M10 Lidar Real-time Plot")
    p.setXRange(-11000, 11000)
    p.setYRange(-11000, 11000)
    scatter = p.scatterPlot(size=5, pen=pg.mkPen(color='r', width=2), symbol='o')
    scatter_center = p.scatterPlot(size=10, pen=pg.mkPen(color='y', width=3), symbol='o')
    scatter_center.setData(x=[0], y=[0])
    scatter_range = p.scatterPlot(size=10, pen=pg.mkPen(color='g', width=2), symbol='o')
    scatter_range.setData(x=[TOP_L_POINT[0], TOP_R_POINT[0], BOTTOM_L_POINT[0], BOTTOM_R_POINT[0]],
                          y=[TOP_L_POINT[1], TOP_R_POINT[1], BOTTOM_L_POINT[1], BOTTOM_R_POINT[1]])

prevTime = time.time()


def parse_data(data):
    start_angle = (data[0] * 256 + data[1]) / 100.0
    speed = data[2] * 256 + data[3]  # 计算转速
    distances = []

    for x in range(4, 87, 2):
        distance = data[x] * 256 + data[x + 1]
        if distance != 0 and distance != 65535:
            distances.append(distance)
        else:
            distances.append(None)

    return speed, start_angle, distances


def get_angle_distance_pairs_from_cloud():
    res = []
    for angle, distances in distance_cloud.items():
        delta_angle = 0
        for distance in distances:
            if distance is not None:
                res.append([angle + delta_angle, distance])
            delta_angle += 360 / 1008
    return res


def radian_distance_to_coordinate(angle_distance_pairs):
    sin_values = []
    cos_values = []

    for (angle, distance) in angle_distance_pairs:
        radian = angle * math.pi / 180
        sin_theta = math.sin(radian)
        cos_theta = math.cos(radian)
        sin_values.append(distance * sin_theta)
        cos_values.append(distance * cos_theta)

    return sin_values, cos_values


def inbound_detect(sin_values, cos_values):
    length = len(sin_values)
    for i in range(length):
        x = sin_values[i]
        y = cos_values[i]
        if (TOP_L_POINT[0] <= x <= BOTTOM_R_POINT[0]) and (TOP_L_POINT[1] <= y <= BOTTOM_R_POINT[1]):
            print(x, y)
            return True
    return False


def handle_alert():
    global state
    state = state | (1 << 6)
    ser_to_mcu.write(state)


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


def print_data(speed, start_angle, distances, last_angle):
    if last_angle - start_angle > 100:
        print("*******************************")

    print("转速:", speed, end="\t")
    print("起始角度:", start_angle, end="\t")
    print("数据【距离（mm）】*42个点：", end="\t")

    for distance in distances:
        print(distance, end="\t")
    print("\n")


def plot_circle(sin_values, cos_values):
    if CFG_PLOTTING:
        scatter.setData(x=sin_values, y=cos_values)
        app.processEvents()
        time.sleep(0.05)


cnt = 0

curr_err = 0

print("[Program Start]")

if __name__ == '__main__':
    connect_to_serial()

    while True:
        currTime = time.time()
        if currTime - prevTime > 5:
            prevTime = currTime
            # if CFG_OSS == "LINUX":
            # ser_to_mcu.write(b'hrbt')
            if CFG_OSS == "WINDOWS":
                ser_to_mcu.write(b'hrbt')

        if curr_err == 1:
            try:
                connect_to_serial()
                curr_err = 0
            except Exception as e:
                print(e)
                continue

        if curr_err == 0:
            try:
                if ser.in_waiting > 0:
                    data = ser.read(1)
                    if data[0] == 0xA5:
                        data = ser.read(1)
                        if data[0] == 0x5A:
                            data = ser.read(88)
                            speed, start_angle, distances = parse_data(data)
                            distance_cloud[start_angle] = distances
                            angle_distance_pairs = get_angle_distance_pairs_from_cloud()
                            sin_values, cos_values = radian_distance_to_coordinate(angle_distance_pairs)

                            if inbound_detect(sin_values, cos_values):
                                print("detected")
                                if CFG_OSS == "WINDOWS":
                                    relay.off()
                                else:
                                    relay.off()
                            else:
                                if CFG_OSS == "WINDOWS":
                                    relay.on()
                                else:
                                    relay.on()

                            cnt += 1
                            if cnt == 36:
                                cnt = 0
                                plot_circle(sin_values, cos_values)

            except Exception as e:
                print(e)
                curr_err = 1
                continue
