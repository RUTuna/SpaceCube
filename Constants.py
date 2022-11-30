import numpy as np

AXIS_LENGTH = 0.5
BALL_RADIUS = 0.05
TIME_INTERVAL = 0.01
GRAVITY_ACCELERATION = 5 # gravity acceleration
COLLISION_DAMP = 0.6

WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800

MAP_SCALE = 1/45 / 1 / 1.5

INIT_BALL_POSITION = np.array([0, 0.2/1.5, 0.8]) # initial ball position
INIT_BALL_VELOCITY = np.array([0, 0, 0]) # initial ball velocity
INIT_BALL_ACCERLATION = np.array([0, -1 * GRAVITY_ACCELERATION, 0]) # initial ball accerleration

INIT_COP = np.array([0, 0, 10])
INIT_AT = np.array([0, 0, 0])
INIT_UP = np.array([0, 1, 0])

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