import json

from Utils import M10Lidar, PlotLidar, ConfigSystem

with open("Config.json", 'r') as json_file:
    config = json.load(json_file)

configSystem = ConfigSystem()
m10Lidar = M10Lidar()
plotLidar = PlotLidar()

configSystem.read()
m10Lidar.connect()
plotLidar.init()

while True:
    m10Lidar.listen()
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(configSystem.boundary_points)
    plotLidar.plot_calibration_point(configSystem.calibration_point)
