import json
import math

import numpy as np

from Main import SystemConfig, M10Lidar, PlotLidar
from Utils import Debouncer


class ProfileSetter:
    polygon = []
    records = []

    @staticmethod
    def shrink_coordinates(points, shrink_amount=10):
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

    def set_profile_coordinates(self, coordinates):
        self.polygon = self.shrink_coordinates(coordinates, 100)

        with open("Config_Polygon.json", 'w') as f:
            json.dump(self.polygon, f, indent=4)
        print("Updated polygon profile")

    def record(self, coordinates):
        self.records.append(coordinates)
        if len(self.records) > 10:
            self.records.pop(0)
        print(self.records)


class ProfileSetter2:
    polygon = []
    tmp_arr = []
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

    def record(self, coordinates, sample_size=10):
        self.tmp_arr.append(coordinates)
        if len(self.tmp_arr) > sample_size:
            self.tmp_arr.pop(0)

        arr_2d = np.array(self.tmp_arr, dtype=object)
        col_avgs = []
        for col in range(arr_2d.shape[1]):
            sum_tuple = (0, 0)
            count = 0
            for row in range(arr_2d.shape[0]):
                current_tuple = arr_2d[row, col]
                if current_tuple[0] is not None and current_tuple[1] is not None:
                    sum_tuple = (sum_tuple[0] + current_tuple[0], sum_tuple[1] + current_tuple[1])
                    count += 1
            if count > 0:
                avg_tuple = (sum_tuple[0] / count, sum_tuple[1] / count)
            else:
                avg_tuple = (None, None)

            col_avgs.append(avg_tuple)
        self.records = col_avgs


systemConfig = SystemConfig()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
profileSetter = ProfileSetter2()
debouncer1 = Debouncer()
debouncer2 = Debouncer()

systemConfig.read()
m10Lidar.connect()
plotLidar.init()


def cb():
    profileSetter.record(m10Lidar.points, sample_size=1)
    profileSetter.set_profile_coordinates(profileSetter.records)
    # profileSetter.set_profile_coordinates(m10Lidar.points)

    (
        plotLidar.
        plot_cloud_points(m10Lidar.xs_clean, m10Lidar.ys_clean).
        plot_boundary(profileSetter.polygon).
        plot_calibration_point(systemConfig.calibration_point)
    )


while True:
    m10Lidar.listen()
    debouncer1.auto_counter(100, cb)
