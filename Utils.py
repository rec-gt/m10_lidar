import time


class Utils:
    @staticmethod
    def point_to_xs_ys(points):
        xs = []
        ys = []
        for point in points:
            xs.append(point[0])
            ys.append(point[1])
        return xs, ys


class Debouncer:
    prev_time = time.time()
    counter = 0

    def auto_timeout(self, timeout, callback):
        curr_time = time.time()
        if curr_time - self.prev_time >= timeout:
            callback()
            self.prev_time = curr_time

    def auto_counter(self, max_count, callback):
        self.counter += 1
        if self.counter >= max_count:
            callback()
            self.counter = 0
