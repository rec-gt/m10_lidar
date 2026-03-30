import json
import math

import numpy as np

from Main import SystemConfig, M10Lidar, PlotLidar
from Utils import Debouncer
import threading


class ProfileSetter:
    polygon = []
    tmp_arr = [[None, None] for i in range(1008)]
    records = []

    @staticmethod
    def shrink_coordinates(points, base_shrink_amount=10):
        centroid = [0, 0]
        new_points = []

        for point in points:
            if point[0] is not None and point[1] is not None:
                # Calculate the angle and distance from the centroid
                angle = math.atan2(point[1] - centroid[1], point[0] - centroid[0])
                current_distance = math.sqrt((point[0] - centroid[0]) ** 2 + (point[1] - centroid[1]) ** 2)

                # Shrink amount is proportional to the distance from the centroid
                shrink_amount = base_shrink_amount * (
                        (current_distance / 6000) / max((current_distance / 6000), 1))  # Avoid divide-by-zero

                # Ensure the new distance is positive
                new_distance = max(current_distance - shrink_amount, 0)

                # Calculate the new coordinates
                new_x = centroid[0] + new_distance * math.cos(angle)
                new_y = centroid[1] + new_distance * math.sin(angle)

                new_points.append([new_x, new_y])

        return new_points

    def set_profile_coordinates(self, coordinates):
        self.polygon = self.shrink_coordinates(coordinates, 500)

        with open("Config_Polygon.json", 'w') as f:
            json.dump(self.polygon, f, indent=4)
        print("Updated polygon profile")
        return self

    def record2(self, coordinates):
        for i in range(len(coordinates)):
            x, y = coordinates[i]
            x_, y_ = self.tmp_arr[i]

            if x_ is None or y_ is None:
                self.tmp_arr[i] = coordinates[i]
                continue

            if x is None or y is None:
                continue

            distance = x ** 2 + y ** 2
            prev_distance = x_ ** 2 + y_ ** 2
            if distance < prev_distance:
                self.tmp_arr[i] = coordinates[i]

        self.records = self.tmp_arr
        return self


systemConfig = SystemConfig()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
profileSetter = ProfileSetter()
debouncer1 = Debouncer()
debouncer2 = Debouncer()

systemConfig.read()
m10Lidar.connect()
plotLidar.init()


def cb():
    (
        profileSetter.
        record2(m10Lidar.points).
        set_profile_coordinates(profileSetter.records)
    )

    (
        plotLidar.
        plot_cloud_points(m10Lidar.points_clean).
        plot_boundary(profileSetter.polygon)
    )


def thd1():
    while True:
        m10Lidar.listen()


thread1 = threading.Thread(target=thd1)

thread1.start()

while True:
    # m10Lidar.listen()
    debouncer1.auto_counter(100, cb)
