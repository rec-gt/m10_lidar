import json
import math

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
    def shrink_coordinates(points, shrink_amount=10):
        centroid = [0, 0]
        new_points = []
        for point in points:
            angle = math.atan2(point[1] - centroid[1], point[0] - centroid[0])
            current_distance = math.sqrt((point[0] - centroid[0]) ** 2 + (point[1] - centroid[1]) ** 2)
            new_distance = max(current_distance - shrink_amount, 0)  # 確保新距離為正值
            new_x = centroid[0] + new_distance * math.cos(angle)
            new_y = centroid[1] + new_distance * math.sin(angle)
            new_points.append([new_x, new_y])

        return new_points

    def set_profile_coordinates(self, coordinates):
        self.polygon = self.shrink_coordinates(coordinates, 50)

        with open("Config_Polygon.json", 'w') as f:
            json.dump(self.polygon, f, indent=4)
        print("Updated polygon profile")

    def record(self, coordinates):
        self.tmp_arr.append(coordinates)
        if len(self.tmp_arr) > 10:
            self.tmp_arr.pop(0)
        flat_list = [item for sublist in self.tmp_arr for item in sublist]
        self.records = flat_list


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
    # profileSetter.record(m10Lidar.points)
    # profileSetter.set_profile_coordinates(profileSetter.records)
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
