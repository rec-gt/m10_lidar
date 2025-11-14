import json

from Utils import M10Lidar, PlotLidar

with open("Config.json", 'r') as json_file:
    config = json.load(json_file)

boundary_points = (
    (config["TOP_LEFT_POINT"]["x"], config["TOP_LEFT_POINT"]["y"]),
    (config["TOP_RIGHT_POINT"]["x"], config["TOP_RIGHT_POINT"]["y"]),
    (config["BOTTOM_LEFT_POINT"]["x"], config["BOTTOM_LEFT_POINT"]["y"]),
    (config["BOTTOM_RIGHT_POINT"]["x"], config["BOTTOM_RIGHT_POINT"]["y"])
)

m10Lidar = M10Lidar()
plotLidar = PlotLidar()
m10Lidar.connect()
plotLidar.init()


while True:
    m10Lidar.listen()
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(boundary_points)
