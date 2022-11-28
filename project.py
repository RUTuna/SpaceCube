from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import serial
import sys
import math 
import Constants as const
from Constants import planes # import plane class

# Edit Log: 이승환, 221127 22:19
# Edit Log: 

# instruction
# python project.py -W : Windows OS
# python project.py -L : Linux OS
# python project.py -W -S : Windows OS + Gyroscope
# python project.py -L -S : Linux OS + Gyroscope

# argument setting
argv = []
for i in range(len(sys.argv) - 1):
    argv.append(sys.argv[i+1])
sensor = False
isWindows = True
if '-W' in argv:
    isWindows = True
if '-L' in argv:
    isWindows = False
if '-S' in argv:
    sensor = True

## sensor setting
if sensor:
    if isWindows:
        ser = serial.Serial('COM3', 115200) # Windows
    else:
        ser = serial.Serial('/dev/tty.usbserial-210', 115200) # Linux, MAC ls /dev
    ser.flushInput()

# plane class


class Viewer:
    def __init__(self):
        self.Y = 0 # Yaw
        self.P = 0 # Pitch
        self.R = 0 # Roll
        self.accG = const.GRAVITY_ASSELERATION # gravity acceleration
        self.p = np.array([0, 0.3, 0]) # initial ball position
        self.v = np.array([0, 0, 0]) # initial ball velocity
        self.a = np.array([0, -1 * self.accG, 0]) # initial ball accerleration
        self.dt = const.TIME_INTERVAL # time interval
        self.radius = const.BALL_RADIUS # ball radius
        self.damp = 0.9 # damping coefficient
        self.cameraPosition = np.array([10, 10, 10]) # position of camera 얼짱각도에 카메라 뒀음,,,
        self.cameraLook = np.array([0, 0, 0]) # looAt of camera
        self.cameraUp = np.array([-1, 1, -1]) / (3 ** 0.5) # up vector w.r.t. position vector

        # make map
        self.boundaries = []
        p0 = np.array([0.3, -0.1, 1])
        p1 = np.array([0.3, -0.1, -1])
        p2 = np.array([-0.3, -0.1, -1])
        self.boundaries.append(planes(p0, p1, p2, 0))
        p0 = np.array([0.3, -0.1, 0.3])
        p1 = np.array([1, -0.1, 0.3])
        p2 = np.array([1, -0.1, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 1))
        p0 = np.array([-0.3, -0.1, 0.3])
        p1 = np.array([-0.3, -0.1, -0.3])
        p2 = np.array([-1, -0.1, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 2))
        p0 = np.array([1, -0.1, 0.3])
        p1 = np.array([1, 0.2, 0.3])
        p2 = np.array([1, 0.2, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 3))
        p0 = np.array([-1, -0.1, 0.3])
        p1 = np.array([-1, -0.1, -0.3])
        p2 = np.array([-1, 0.2, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 4))
        p0 = np.array([0.3, -0.1, 1])
        p1 = np.array([-0.3, -0.1, 1])
        p2 = np.array([-0.3, 0.2, 1])
        self.boundaries.append(planes(p0, p1, p2, 5))
        p0 = np.array([0.3, -0.1, -1])
        p1 = np.array([0.3, 0.2, -1])
        p2 = np.array([-0.3, 0.2, -1])
        self.boundaries.append(planes(p0, p1, p2, 6))
        p0 = np.array([0.3, -0.1, 0.3])
        p1 = np.array([0.3, -0.1, 1])
        p2 = np.array([0.3, 0.2, 1])
        self.boundaries.append(planes(p0, p1, p2, 7))
        p0 = np.array([0.3, -0.1, 0.3])
        p1 = np.array([0.3, 0.2, 0.3])
        p2 = np.array([1, 0.2, 0.3])
        self.boundaries.append(planes(p0, p1, p2, 8))
        p0 = np.array([0.3, -0.1, -0.3])
        p1 = np.array([1, -0.1, -0.3])
        p2 = np.array([1, 0.2, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 9))
        p0 = np.array([0.3, -0.1, -0.3])
        p1 = np.array([0.3, 0.2, -0.3])
        p2 = np.array([0.3, 0.2, -1])
        self.boundaries.append(planes(p0, p1, p2, 10))
        p0 = np.array([-0.3, -0.1, 0.3])
        p1 = np.array([-0.3, 0.2, 0.3])
        p2 = np.array([-0.3, 0.2, 1])
        self.boundaries.append(planes(p0, p1, p2, 11))
        p0 = np.array([-0.3, -0.1, 0.3])
        p1 = np.array([-1, -0.1, 0.3])
        p2 = np.array([-1, 0.2, 0.3])
        self.boundaries.append(planes(p0, p1, p2, 12))
        p0 = np.array([-0.3, -0.1, -0.3])
        p1 = np.array([-0.3, 0.2, -0.3])
        p2 = np.array([-1, 0.2, -0.3])
        self.boundaries.append(planes(p0, p1, p2, 13))
        p0 = np.array([-0.3, -0.1, -0.3])
        p1 = np.array([-0.3, -0.1, -1])
        p2 = np.array([-0.3, 0.2, -1])
        self.boundaries.append(planes(p0, p1, p2, 14))
        self.color = (np.random.randn(15, 3) + 1) / 2 # random color

        # 초기에 십자가 돌아가는거, 무시해도 무방, x축 회전
        rot = np.zeros((3,3))
        deg = np.deg2rad(20)
        rot[0, 0] = 1
        rot[1, 1] = np.cos(deg)
        rot[1, 2] = -1 * np.sin(deg)
        rot[2, 1] = np.sin(deg)
        rot[2, 2] = np.cos(deg)
        for i in range(15):
            self.boundaries[i].p0 = rot @ self.boundaries[i].p0
            self.boundaries[i].p1 = rot @ self.boundaries[i].p1
            self.boundaries[i].p2 = rot @ self.boundaries[i].p2
            self.boundaries[i].p3 = rot @ self.boundaries[i].p3
            self.boundaries[i].linEqu3D = rot @ self.boundaries[i].linEqu3D
            self.boundaries[i].linEqu[0:3] = self.boundaries[i].linEqu3D
            
        
        
    # 과제 2랑 같은 method
    def worldToCamera(self, p, a, u):
        # world to camera projection
        z = p - a
        norm = np.linalg.norm(z)
        z = z / norm # normalization
        x = np.cross(u, z)
        norm = np.linalg.norm(x)
        x = x / norm # normalization
        y = np.cross(z, x) # already normalized

        Tscene = np.eye(4) # Tscene matrix is equal to inverse of Tcam
        RT = np.eye(3) # transpose of R
        RT[0, :] = x
        RT[1, :] = y
        RT[2, :] = z
        Tscene[0:3, 0:3] = RT
        Tscene[0:3, 3] = -1 * RT @ p.T
        return Tscene

    def light(self):
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_LIGHTING)
        glEnable(GL_DEPTH_TEST)

        # feel free to adjust light colors
        lightAmbient = [0.5, 0.5, 0.5, 1.0]
        lightDiffuse = [0.5, 0.5, 0.5, 1.0]
        lightSpecular = [0.5, 0.5, 0.5, 1.0]
        lightPosition = [1, 1, -1, 0]    # vector: point at infinity
        glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient)
        glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse)
        glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular)
        glLightfv(GL_LIGHT0, GL_POSITION, lightPosition)
        glEnable(GL_LIGHT0)

    def display(self):
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glColor4f(1, 1, 1, 1)
               
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        glOrtho(-1, 1, -1, 1, 0.1, 300)
        # projection matrix
        # use glOrtho and glFrustum (or gluPerspective) here

        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        
        glMultMatrixf(self.worldToCamera(self.cameraPosition, self.cameraLook, self.cameraUp).T) # camera view
        glColor4f(1, 1, 1, 1)
        glTranslatef(self.p[0], self.p[1], self.p[2]) # translation of ball
        glutSolidSphere(self.radius, 50, 50)
        glTranslatef(-1 * self.p[0], -1 * self.p[1], -1 * self.p[2]) # undo translation for map
        
        # if gyroscope is available
        if sensor:
            glRotatef(self.Y, 0, 1, 0)
            glRotatef(self.P, 0, 0, -1)
            glRotatef(self.R, 1, 0, 0)
        
        # map drawing
        for i, bound in enumerate(self.boundaries):
            glBegin(GL_QUADS)
            glColor4f(self.color[i, 0], self.color[i, 1], self.color[i, 2], 0.5)
            glVertex3f(bound.p0[0], bound.p0[1], bound.p0[2])
            glVertex3f(bound.p1[0], bound.p1[1], bound.p1[2])
            glVertex3f(bound.p2[0], bound.p2[1], bound.p2[2])
            glVertex3f(bound.p3[0], bound.p3[1], bound.p3[2])
            glEnd()
        

        glutSwapBuffers()

    def keyboard(self, key, x, y):
        print(f"keyboard event: key={key}, x={x}, y={y}")
        if glutGetModifiers() & GLUT_ACTIVE_SHIFT:
            print("shift pressed")
        if glutGetModifiers() & GLUT_ACTIVE_ALT:
            print("alt pressed")
        if glutGetModifiers() & GLUT_ACTIVE_CTRL:
            print("ctrl pressed")

        glutPostRedisplay()

    def special(self, key, x, y):
        print(f"special key event: key={key}, x={x}, y={y}")

        glutPostRedisplay()

    def mouse(self, button, state, x, y):
        # button macros: GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, GLUT_RIGHT_BUTTON
        print(f"mouse press event: button={button}, state={state}, x={x}, y={y}")

        glutPostRedisplay()

    def motion(self, x, y):
        print(f"mouse move event: x={x}, y={y}")

        glutPostRedisplay()

    def reshape(self, w, h):
        # implement here
        print(f"window size: {w} x {h}")

        glutPostRedisplay()

    # Physics section, frame 단위 계산
    def timer(self, value):
        # 프레임마다 십자가 돌아가는거, 무시해도 무방, y축 회전
        rot = np.zeros((3,3))
        deg = np.deg2rad(0.1)
        rot[1, 1] = 1
        rot[0, 0] = np.cos(deg)
        rot[0, 2] = np.sin(deg)
        rot[2, 0] = -1 * np.sin(deg)
        rot[2, 2] = np.cos(deg)
        for i in range(15):
            self.boundaries[i].p0 = rot @ self.boundaries[i].p0
            self.boundaries[i].p1 = rot @ self.boundaries[i].p1
            self.boundaries[i].p2 = rot @ self.boundaries[i].p2
            self.boundaries[i].p3 = rot @ self.boundaries[i].p3
            self.boundaries[i].linEqu3D = rot @ self.boundaries[i].linEqu3D
            self.boundaries[i].linEqu[0:3] = self.boundaries[i].linEqu3D

        # 프레임마다 십자가 돌아가는거, 무시해도 무방, x축 회전
        rot = np.zeros((3,3))
        deg = np.deg2rad(+0.1)
        rot[0, 0] = 1
        rot[1, 1] = np.cos(deg)
        rot[1, 2] = -1 * np.sin(deg)
        rot[2, 1] = np.sin(deg)
        rot[2, 2] = np.cos(deg)
        for i in range(15):
            self.boundaries[i].p0 = rot @ self.boundaries[i].p0
            self.boundaries[i].p1 = rot @ self.boundaries[i].p1
            self.boundaries[i].p2 = rot @ self.boundaries[i].p2
            self.boundaries[i].p3 = rot @ self.boundaries[i].p3
            self.boundaries[i].linEqu3D = rot @ self.boundaries[i].linEqu3D
            self.boundaries[i].linEqu[0:3] = self.boundaries[i].linEqu3D


        # update velocity, position
        self.v = self.v + self.a * self.dt
        self.p = self.p + self.v * self.dt

        # list of collisions
        collisionIndex = []

        # 모든 plane에 대해 collision 검사
        for bound in self.boundaries:
            # plane과 ball 중심과의 거리
            d = abs(self.p @ bound.linEqu3D.T + bound.linEqu[3]) / np.linalg.norm(bound.linEqu3D)
            # plane에 ball 중심 projection, 이유는 평면방정식이기 때문에 4개의 점 밖에서 충돌할 수 있음
            estimatedPP = (self.p - d * bound.linEqu3D / np.linalg.norm(bound.linEqu3D))
            Mx = max(bound.p0[0], bound.p1[0], bound.p2[0], bound.p3[0])
            mx = min(bound.p0[0], bound.p1[0], bound.p2[0], bound.p3[0])
            My = max(bound.p0[1], bound.p1[1], bound.p2[1], bound.p3[1])
            my = min(bound.p0[1], bound.p1[1], bound.p2[1], bound.p3[1])
            Mz = max(bound.p0[2], bound.p1[2], bound.p2[2], bound.p3[2])
            mz = min(bound.p0[2], bound.p1[2], bound.p2[2], bound.p3[2])
            inner = (estimatedPP[0] <= Mx) and (estimatedPP[0] >= mx) and (estimatedPP[1] <= My) and (estimatedPP[1] >= my) and (estimatedPP[2] <= Mz) and (estimatedPP[2] >= mz)

            # 충돌 및 plane내부에 충돌하는 두가지 모두 만족할 경우
            isCollision = (d <= self.radius) and inner
            if isCollision: # collision detection
                collisionIndex.append(bound.index)
        
        normalAverage = np.array([0, 0, 0, 0])
        if len(collisionIndex) != 0: # if there is collision
            for i in collisionIndex:
                bound = self.boundaries[i]
                normalAverage = normalAverage + bound.linEqu # 모든 충돌에 대해 반사각 평균 계산

                d = abs(self.p @ bound.linEqu3D.T + bound.linEqu[3]) / np.linalg.norm(bound.linEqu3D) # 각 충돌에 대한 distance 계산
                temp = (self.radius - d) / np.linalg.norm(bound.linEqu3D) # 각 충돌에 대해 어느 시점에서 부딪혔는지 역계산
                self.p = self.p + bound.linEqu3D * temp # 충돌지점으로 시간 되돌림
            
            linEqu = normalAverage / len(collisionIndex) # 모든 충돌에 대해 반사각 평균 계산
            linEqu3D = linEqu[0:3]

            vn = (self.v @ linEqu3D.T) / (linEqu3D @ linEqu3D.T) * linEqu3D # 충돌 방정식
            vt = self.v - vn
            self.v = vt - self.damp * vn
        
        if sensor:
            global ser
            ser.flushInput()
            inp = ser.readline()
            inp = inp.decode().split('\t')
            if len(inp) != 4:
                pass
            else:
                Y = int(float(inp[1]))
                P = int(float(inp[2]))
                R = int(float(inp[3]))
                print(Y, P, R)
                self.Y = Y
                self.P = P
                self.R = R
        glutTimerFunc(int(self.dt * 1000), self.timer, 0)
        
        glutPostRedisplay()

    def run(self):
        glutInit()
        glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH)
        glutInitWindowSize(1000, 1000)
        glutInitWindowPosition(0, 0)
        glutCreateWindow(b"CS471 Computer Graphics #2")

        glutDisplayFunc(self.display)
        glutKeyboardFunc(self.keyboard)
        glutSpecialFunc(self.special)
        glutMouseFunc(self.mouse)
        glutMotionFunc(self.motion)
        glutReshapeFunc(self.reshape)
        glutTimerFunc(int(self.dt * 1000), self.timer, 0)

        self.light()

        glutMainLoop()


if __name__ == "__main__":
    viewer = Viewer()
    viewer.run()
