BALL_RADIUS = 0.1
TIME_INTERVAL = 0.01
GRAVITY_ACCELERATION = 5
COLLISION_DAMP = 0.6

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

MAP_SCALE = 1/45 / 1 / 1.5

import numpy as np

# plane class
class planes:
    def __init__(self, p0, p1, p2, index):
        self.p0 = p0
        self.p1 = p1
        self.p2 = p2
        #self.p3 = p2 + p0 - p1
        n = np.cross(p1-p0, p2-p0)
        a = n[0]
        b = n[1]
        c = n[2]
        d = -1*a*p0[0] -1*b*p0[1] -1*c*p0[2]

        self.linEqu = np.array([a, b, c, d]) / np.linalg.norm(np.array([a, b, c]))
        self.linEqu3D = np.array([a, b, c]) / np.linalg.norm(np.array([a, b, c]))
        self.index = index