class Utils:
    @staticmethod
    def point_to_xs_ys(points):
        xs = []
        ys = []
        for point in points:
            xs.append(point[0])
            ys.append(point[1])
        return xs, ys
