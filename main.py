from cmath import sqrt
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import *
import numpy as np
import serial
import sys
import math 
import Constants as const
from Constants import planes # import plane class

# r : reset ball position
# v : on/off observer mode (default is False)
# b : on/off ball view mode (default is False)
# ↑↓ : up/down fov

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

# sensor setting
if sensor:
    if isWindows:
        ser = serial.Serial('COM3', 115200) # Windows
    else:
        ser = serial.Serial('/dev/tty.usbserial-210', 115200) # Linux, MAC ls /dev
    ser.flushInput()

class Viewer:
    def __init__(self):
        self.Y = 0 # Yaw
        self.P = 0 # Pitch
        self.R = 0 # Roll
        self.prevY = 0
        self.prevP = 0
        self.prevR = 0
        self.p, self.v, self.a = const.INIT_BALL_POSITION, const.INIT_BALL_VELOCITY, const.INIT_BALL_ACCERLATION
        self.dt = const.TIME_INTERVAL # time interval
        self.radius = const.BALL_RADIUS # ball radius
        self.damp = const.COLLISION_DAMP # damping coefficient

        # transform matrix
        self.rotateMatrix = np.eye(4)

        # projectioin parameter
        self.cop, self.at, self.up = const.INIT_COP, const.INIT_AT, const.INIT_UP
        self.fov = 0

        # window width, height 
        self.width = const.WINDOW_WIDTH
        self.height = const.WINDOW_HEIGHT

        self.click = False
        self.observerMode = False
        self.ballView = False

        # index of last collistion plane (ball's gravity 와 더 가까운)
        self.lastCollistion = -1

        # make map
        f1 = open('map.txt', 'r')
        raw = f1.readlines()
        raw = raw[0].split(']]')
        raw = raw[:-1]
        for index in range(len(raw)):
            if index == 0:
                raw[index] = raw[index][2:]
            else:
                raw[index] = raw[index][4:]
            raw[index] = raw[index].split(',')

            raw[index][1] = raw[index][1][1:]
            raw[index][2] = raw[index][2][1:-1]
            raw[index][3] = raw[index][3][2:]
            raw[index][4] = raw[index][4][1:]
            raw[index][5] = raw[index][5][1:-1]
            raw[index][6] = raw[index][6][2:]
            raw[index][7] = raw[index][7][1:]
            raw[index][8] = raw[index][8][1:]
        self.boundaries = []
        for i in range(len(raw)):
            p2 = np.array([float(raw[i][0]) * const.MAP_SCALE, (float(raw[i][1])-45) * const.MAP_SCALE, float(raw[i][2]) * const.MAP_SCALE])
            p1 = np.array([float(raw[i][3]) * const.MAP_SCALE, (float(raw[i][4])-45) * const.MAP_SCALE, float(raw[i][5]) * const.MAP_SCALE])
            p0 = np.array([float(raw[i][6]) * const.MAP_SCALE, (float(raw[i][7])-45) * const.MAP_SCALE, float(raw[i][8]) * const.MAP_SCALE])
            self.boundaries.append(planes(p2, p1, p0, i))
        
        f2 = open('orientation.txt', 'r')
        for index in range(532):
            raw = f2.readline()
            orientation = int(raw)
            self.boundaries[index].linEqu = self.boundaries[index].linEqu * orientation
            self.boundaries[index].linEqu3D = self.boundaries[index].linEqu3D * orientation
        self.color = (np.random.randn(532, 3) + 1) / 2 # random color
            
            
    # Because of numpy cross, according to https://github.com/microsoft/pylance-release/issues/3277 
    def cross(self, a:np.ndarray,b:np.ndarray)->np.ndarray:
        return np.cross(a,b)

    def world2camera(self, cop, at, up):
        z = cop - at
        z = z / np.linalg.norm(z)
        x = self.cross(up, z)
        x = x / np.linalg.norm(x)
        y = self.cross(z, x)

        rc = [-np.dot(cop, x), -np.dot(cop, y), -np.dot(cop, z), 1]
        x = np.append(x,0)
        y = np.append(y,0)
        z = np.append(z,0)

        return np.array([x,y,z,rc])


    # transform 2D plane to 3D trackball
    def transPos(self, x, y):
        radius = min([self.width, self.height])/2
        z = 0
        d = pow(radius,2) - pow(x,2) - pow(y, 2)
        if d > 0:
            z = sqrt(d).real

        return z

    
    # input angle mustbe degree
    def angleRotate(self, xAngle, yAngle, zAngle):
        xAngle = math.radians(xAngle)
        yAngle = math.radians(yAngle)
        zAngle = math.radians(zAngle)
        cosx = math.cos(xAngle)
        sinx = math.sin(xAngle)

        cosy = math.cos(yAngle)
        siny = math.sin(yAngle)

        cosz = math.cos(zAngle)
        sinz = math.sin(zAngle)

        matrixX = np.array([[cosz,  -sinz,  0,  0], 
                            [sinz,  cosz,   0,  0], 
                            [0,     0,      1,  0], 
                            [0,     0,      0,  1]])

        matrixY = np.array([[cosy,   0,  siny,   0], 
                            [0,     1,  0,      0], 
                            [-siny, 0,  cosy,   0], 
                            [0,     0,  0,      1]])

        matrixZ = np.array([[1,  0,      0,      0], 
                            [0, cosx,   -sinx,  0], 
                            [0, sinx,   cosx,   0], 
                            [0, 0,      0,      1]])


        # ballView 의 observerMode 에선 frame 마다 카메라를 공의 중심으로 옮기기에 변환 행렬을 누적시켜 적용시켜줘야 함
        if self.observerMode and self.ballView:
            self.rotateMatrix =  self.rotateMatrix @ matrixX @ matrixY @ matrixZ 
        else:
            self.rotateMatrix =  matrixX @ matrixY @ matrixZ

    # virtual trackball
    def trackball(self, str, dst):
        # transfrom coordinate(2D plane to 3D trackball), center is (0,0) and y-axis is filp
        str = np.array([str[0] - (self.width/2), (self.height/2) - str[1], 0])/10
        dst = np.array([dst[0] - (self.width/2), (self.height/2) - dst[1], 0])/10
        
        str[2] = self.transPos(str[0], str[1])
        dst[2] = self.transPos(dst[0], dst[1])

        # calculate rotation angle and axis
        axis = self.cross(str, dst)
        axisNorm = np.linalg.norm(axis)
        if axisNorm == 0 :
            angle = 0
        else :
            axis = axis / axisNorm
            angle = axisNorm / (np.linalg.norm(str) + np.linalg.norm(dst))

        self.angleRotate(angle * axis[0], angle * axis[1], angle * axis[2])
       
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
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)

        glColor4f(1, 1, 1, 1)
               
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        # projection matrix
        # use glOrtho and glFrustum (or gluPerspective) here
        if self.fov < 5 : # orhogonal
            top = self.height/800
            right = self.width/800
            factor = 1/min(right, top)
            right = right * factor
            top = top * factor
            glOrtho(-right, right, -top, top, 0.1, 300)
        else:
            top = 0.1 * math.tan(math.radians(self.fov/2));  
            top = top * max(800/self.width, 800/self.height)
            right = (self.width/self.height) * top
            glFrustum(-right, right, -top, top, 0.1, 300)

        glMultMatrixf(self.world2camera(self.cop, self.at, self.up))

        glMatrixMode(GL_MODELVIEW)

        # ball axis
        if not self.ballView:
            glPushMatrix()
            glTranslatef(self.p[0],self.p[1],self.p[2]) # translate to ball position
            glMultMatrixf(self.rotateMatrix) # only rotate

            glLineWidth(3)
            glBegin(GL_LINES)
            glColor4f(0, 0, 1, 1)
            glVertex3f(0, 0, const.AXIS_LENGTH)
            glVertex3f(0, 0, 0)

            glColor4f(0, 1, 0, 1)
            glVertex3f(0, const.AXIS_LENGTH, 0)
            glVertex3f(0, 0, 0)

            glColor4f(1, 0, 0, 1)
            glVertex3f(const.AXIS_LENGTH, 0, 0)
            glVertex3f(0, 0, 0)
            glEnd()
            glPopMatrix()
        
        glLoadIdentity()
        
        glColor4f(1, 1, 1, 1)
        glTranslatef(self.p[0], self.p[1], self.p[2]) # translation of ball

        if not self.ballView: # cube view 에서만 ball 나타냄
            glutSolidSphere(self.radius, 50, 50)

        glTranslatef(-1 * self.p[0], -1 * self.p[1], -1 * self.p[2]) # undo translation for map
        
        # in ball view
        if self.ballView: 
            self.cop = self.p # camera를 ball 의 중심에
            self.at = self.p + (self.boundaries[self.lastCollistion].p0 - self.boundaries[self.lastCollistion].p1)*10 # camera는 lastCollistion 의 평면 벡터와 평행한 방향을 바라보도록
            self.up = -self.a # 중력의 반대 방향을 up 방향으로

        # if gyroscope is available
        if sensor:
            self.angleRotate(-1 * self.R, 1 * self.Y, 1 * self.P)

        rot = np.delete(self.rotateMatrix, 3 , axis = 0)
        rot = np.delete(rot, 3 , axis = 1)

        if self.observerMode: # observer mode 에선 camera 를 회전
            self.cop, self.at, self.up = rot @ self.cop, rot @ self.at, rot @ self.up
        else: # plane 자체를 회전
            for i in range(532):
                self.boundaries[i].p0 = rot @ self.boundaries[i].p0
                self.boundaries[i].p1 = rot @ self.boundaries[i].p1
                self.boundaries[i].p2 = rot @ self.boundaries[i].p2
                self.boundaries[i].linEqu3D = rot @ self.boundaries[i].linEqu3D
                self.boundaries[i].linEqu[0:3] = self.boundaries[i].linEqu3D
            self.p = rot @ self.p
            self.v = rot @ self.v
            
        # map drawing
        for i, bound in enumerate(self.boundaries):
            glBegin(GL_TRIANGLES)
            if i < 508:
                glColor4f(self.color[i, 0], self.color[i, 1], self.color[i, 2], 1)
            else:
                glColor4f(1, 1, 1, 0)
            glVertex3f(bound.p0[0], bound.p0[1], bound.p0[2])
            glVertex3f(bound.p1[0], bound.p1[1], bound.p1[2])
            glVertex3f(bound.p2[0], bound.p2[1], bound.p2[2])
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

        if key == b'b': # on/off ballView
            self.ballView = not self.ballView
            print("Now view mode is" ,"Ball View" if self.ballView else "Cube View")

            if self.ballView: # ball view 에서 기본 fov는 50
                self.fov = 50
            else :
                self.rotateMatrix = np.eye(4) # cube view에선 rotateMatrix를 누적시키지 않기에 초기화
                self.cop, self.at, self.up = const.INIT_COP, const.INIT_AT, const.INIT_UP # camera 위치 초기화
                self.fov = 0

        if key == b'v': # on/off observerMode
            self.observerMode = not self.observerMode            
            print(f"Now observerMode is {self.observerMode}")
            self.rotateMatrix = np.eye(4) # view 전환 시 rotateMatrix 초기화
            if not self.observerMode and not self.ballView: # cube view 에서 observer mode 취소 시 
                self.cop, self.at, self.up = const.INIT_COP, const.INIT_AT, const.INIT_UP # camera 위치 초기화

        if key == b'r': # reset ball position
            self.p, self.v, self.a = const.INIT_BALL_POSITION, const.INIT_BALL_VELOCITY, const.INIT_BALL_ACCERLATION
            
        glutPostRedisplay()

    def special(self, key, x, y):
        print(f"special key event: key={key}, x={x}, y={y}")

        if key == 101 and self.fov < 90: # up arrow
            self.fov += 5.0
            print(f"FoV is {self.fov}")
        elif key == 103 and self.fov > 0: # down arrow
            self.fov -= 5.0
            print(f"FoV is {self.fov}")

        glutPostRedisplay()

    def mouse(self, button, state, x, y):
        # button macros: GLUT_LEFT_BUTTON, GLUT_MIDDLE_BUTTON, GLUT_RIGHT_BUTTON
        print(f"mouse press event: button={button}, state={state}, x={x}, y={y}")

        if state == GLUT_DOWN:
            self.click = True
            self.xStart = x
            self.yStart = y
        elif state == GLUT_UP:
            self.click = False
            if not self.observerMode or not self.ballView:
                self.rotateMatrix = np.eye(4)
            
        glutPostRedisplay()

    def motion(self, x, y):
        print(f"mouse move event: x={x}, y={y}")

        print(f"start position {self.xStart} {self.yStart}")
        self.trackball([self.xStart, self.yStart], [x,y])
        self.xStart = x
        self.yStart = y

        glutPostRedisplay()

    def reshape(self, w, h):
        # implement here
        print(f"window size: {w} x {h}")

        glutPostRedisplay()

    # Physics section, frame 단위 계산
    def timer(self, value):
        # update velocity, position
        self.v = self.v + self.a * self.dt
        self.p = self.p + self.v * self.dt
        # print(self.p)

        # list of collisions
        collisionIndex = []

        # 모든 plane에 대해 collision 검사
        for bound in self.boundaries:
            # plane과 ball 중심과의 거리
            d = abs(self.p @ bound.linEqu3D.T + bound.linEqu[3])# / np.linalg.norm(bound.linEqu3D)
            # plane에 ball 중심 projection, 이유는 평면방정식이기 때문에 3개의 점 밖에서 충돌할 수 있음
            estimatedPP = (self.p - d * bound.linEqu3D)# / np.linalg.norm(bound.linEqu3D))
            # area = np.linalg.norm(np.cross(bound.p1-bound.p0, bound.p2-bound.p0)) / 2
            area = np.cross(bound.p1-bound.p0, bound.p2-bound.p0)
            area = np.sqrt(np.power(area[0], 2) + np.power(area[1], 2) + np.power(area[2], 2)) / 2
            pa = bound.p0 - estimatedPP
            pb = bound.p1 - estimatedPP
            pc = bound.p2 - estimatedPP
            # alpha = np.linalg.norm(np.cross(pb, pc)) / (2*area)
            alpha = np.power(pb[1]*pc[2] - pb[2]*pc[1], 2) + np.power(pb[0] * pc[2] - pb[2] * pc[0], 2) + np.power(pb[0] * pc[1] - pb[1] * pc[0], 2)
            alpha = np.sqrt(alpha) / (2*area)
            if alpha < 0:
                continue
            # beta = np.linalg.norm(np.cross(pc, pa)) / (2*area)
            beta = np.power(pc[1]*pa[2] - pc[2]*pa[1], 2) + np.power(pc[0] * pa[2] - pc[2] * pa[0], 2) + np.power(pc[0] * pa[1] - pc[1] * pa[0], 2)
            beta = np.sqrt(beta) / (2*area)
            if beta < 0:
                continue
            #gamma = 1 - alpha - beta
            gamma = np.power(pa[1]*pb[2] - pa[2]*pb[1], 2) + np.power(pa[0] * pb[2] - pa[2] * pb[0], 2) + np.power(pa[0] * pb[1] - pa[1] * pb[0], 2)
            gamma = np.sqrt(gamma) / (2*area)

            inner = alpha >=0 and beta >=0 and gamma >=0 and abs(alpha+beta+gamma - 1) < 0.2

            # 충돌 및 plane내부에 충돌하는 두가지 모두 만족할 경우
            isCollision = (d <= self.radius) and inner
            if isCollision: # collision detection
                collisionIndex.append(bound.index)

                # lastCollistion 이 아직 없음
                if self.lastCollistion < 0 : 
                    self.lastCollistion = bound.index
                # 이전 lastCollistion 평면과 현재 Collistion 평면 중 ball의 gravity와의 각도가 더 작은 것 선택
                elif np.dot(-self.a, self.boundaries[bound.index].linEqu3D) > np.dot(-self.a, self.boundaries[self.lastCollistion].linEqu3D):
                    self.lastCollistion = bound.index

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
            

            if linEqu3D @ linEqu3D.T == 0:
                vn = (self.v @ linEqu3D.T) / (np.finfo(float).eps) * linEqu3D # 충돌 방정식
            else :
                vn = (self.v @ linEqu3D.T) / (linEqu3D @ linEqu3D.T) * linEqu3D # 충돌 방정식
            vt = self.v - vn
            self.v = vt - self.damp * vn
        
        if sensor:
            global ser
            ser.flushInput()
            inp = ser.readline()
            inp = inp.decode().split('\t')
            if len(inp) != 5:
                pass
            else:
                
                w = (float(inp[1]))
                x = (float(inp[2]))
                y = (float(inp[3]))
                z = (float(inp[4]))
                
                P = np.arctan2(2*y*w - 2*x*z, 1 - 2*y*y - 2*z*z) / np.pi * 180
                R = np.arctan2(2*x*w - 2*y*z, 1 - 2*x*x - 2*z*z) / np.pi * 180
                Y = np.arcsin(2*x*y + 2*z*w) / np.pi * 180
                print(Y, P, R)
                # Y = int(float(inp[1]))
                # P = int(float(inp[2]))
                # R = int(float(inp[3]))
                self.Y = Y - self.prevY
                self.P = P - self.prevP
                self.R = R - self.prevR
                self.prevY = Y
                self.prevP = P
                self.prevR = R
                
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
