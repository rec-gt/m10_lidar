import json

from Utils import ConfigSystem, M10Lidar, Debouncer, PlotLidar


class ProfileSetter:
    polygon = []

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
        self.polygon = self.shrink_coordinates(coordinates, 50)

        with open("Polygon.json", 'w') as f:
            json.dump(self.polygon, f, indent=4)
        print("Updated polygon profile")


configSystem = ConfigSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
profileSetter = ProfileSetter()
debouncer1 = Debouncer()
debouncer2 = Debouncer()

configSystem.read()
m10Lidar.connect()
plotLidar.init()


def cb():
    profileSetter.set_profile_coordinates(m10Lidar.points)

    (
        plotLidar.
        plot_cloud_points(m10Lidar.xs, m10Lidar.ys).
        plot_boundary(profileSetter.polygon)
    )


while True:
    m10Lidar.listen()
    debouncer1.auto_counter(100, cb)
