import math


class Localizer:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    def predict(self, v, yaw_rate, dt):
        self.x += v * math.cos(self.yaw) * dt
        self.y += v * math.sin(self.yaw) * dt
        self.yaw += yaw_rate * dt

    def update_mapping(self, points, grid):
        pass

    def update_racing(self, points, grid):
        pass
