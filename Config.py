from Utils import M10Lidar, PlotLidar


m10Lidar = M10Lidar()
plotLidar = PlotLidar()
m10Lidar.connect()
plotLidar.init()

boundary_points = ([-6000, 1000], [0, 1000], [0, 0], [-6000, 0], [-6000, 1000])

while True:
    m10Lidar.listen()
    plotLidar.update_cloud_points(m10Lidar.xs, m10Lidar.ys)
    plotLidar.plot_boundary(boundary_points)
