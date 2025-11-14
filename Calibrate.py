from Utils import M10Lidar, PlotLidar, ConfigSystem, Counter

configSystem = ConfigSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()
counter = Counter(100)

configSystem.read()
m10Lidar.connect()
plotLidar.init()


def cb():
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(configSystem.boundary_points)
    plotLidar.plot_calibration_point(configSystem.calibration_point)


while True:
    m10Lidar.listen()
    counter.auto_counter(cb)
