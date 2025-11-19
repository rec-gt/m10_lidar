from Utils import M10Lidar, PlotLidar, ConfigSystem, Debouncer

configSystem = ConfigSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()

configSystem.read()
m10Lidar.connect()
plotLidar.init()

debouncer = Debouncer()


def cb():
    plotLidar.plot_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(configSystem.boundary_points)
    plotLidar.plot_calibration_point(configSystem.calibration_point)


while True:
    m10Lidar.listen()
    debouncer.auto_counter(100, cb)
