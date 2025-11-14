from Utils import M10Lidar, PlotLidar


class ConfigLidar:
    def init(self):
        pass


m10Lidar = M10Lidar()
plotLidar = PlotLidar()
m10Lidar.connect()
plotLidar.init()

while True:
    m10Lidar.listen()
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    boundary_points = ([-6000, 1000], [0, 1000], [0, 0], [-6000, 0], [-6000, 1000])
    plotLidar.plot_boundary(boundary_points)
