import rtmaps.types
import numpy as np
import rtmaps.core as rt
import rtmaps.reading_policy
from rtmaps.base_component import BaseComponent  # base class

import math
import numpy as np
import bisect #Use a binary search algorithm to find which commit in the commit history introduced the error


# Cubic spline planner Atsushi Sakai(@Atsushi_twi)
# https://github.com/AtsushiSakai
class Spline:
    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)
        h = np.diff(x)

        self.a = [iy for iy in y]

        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)

        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                 (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
                 self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calc_d(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calc_dd(self, t):
        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        A = np.zeros((self.nx, self.nx))
        A[0, 0] = 1.0
        for i in range(self.nx - 1):
            if i != (self.nx - 2):
                A[i + 1, i + 1] = 2.0 * (h[i] + h[i + 1])
            A[i + 1, i] = h[i]
            A[i, i + 1] = h[i]

        A[0, 1] = 0.0
        A[self.nx - 1, self.nx - 2] = 0.0
        A[self.nx - 1, self.nx - 1] = 1.0
        #  print(A)
        return A

    def __calc_B(self, h):
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                       h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = [math.sqrt(idx ** 2 + idy ** 2)
                   for (idx, idy) in zip(dx, dy)]
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        dx = self.sx.calc_d(s)
        ddx = self.sx.calc_dd(s)
        dy = self.sy.calc_d(s)
        ddy = self.sy.calc_dd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2) ** (3 / 2))
        return k

    def calc_yaw(self, s):
        dx = self.sx.calc_d(s)
        dy = self.sy.calc_d(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    r_x, r_y, r_yaw, r_k = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        r_x.append(ix)
        r_y.append(iy)
        r_yaw.append(sp.calc_yaw(i_s))
        r_k.append(sp.calc_curvature(i_s))

    return r_x, r_y, r_yaw, r_k, s


class rtmaps_python(BaseComponent):
    def __init__(self):
        BaseComponent.__init__(self)
        self.runtime = 0
        # list contains a reference trajectory
        self.ref_traj = []
        # sample time 0.5s
        self.dt = 0.5 # TODO

    def Dynamic(self):
        # input: list of GPS points (after changing the coordinate system)
        self.add_input("GPS_xy", rtmaps.types.ANY)
        # output: list of X values
        self.add_output("ref_y", rtmaps.types.AUTO, 1000)
        # output: list of Y values
        self.add_output("ref_x", rtmaps.types.AUTO, 1000)
        self.add_output("viewer", rtmaps.types.AUTO, 1000)

    def Birth(self):
        print("Python Birth")

    def Core(self):
        # dimension of the list
        list = self.inputs["GPS_xy"].ioelt.data
        len_list = len(list)
        # x values are the first half of the input
        gps_x = list[0:int(len_list / 2 -1)]
        decalage = list[len_list -1]  # use the value from detect points to decal the trajectory and dodge the obsatcle
        gps_x = np.asarray(gps_x)

        gps_x = gps_x - decalage
        # for j in range(len(gps_x)):        
            # gps_x[j] = gps_x[j] - decalage

        # y values are the second half of the input
        gps_y = list[int(len_list / 2 ):int(len_list - 2)]
        gps_y = np.asarray(gps_y)
        # add [x=0.0, y=0.0] (robot position) each time
        gps_x = np.concatenate((np.zeros(1),gps_x), axis=0)
        gps_y = np.concatenate((np.zeros(1),gps_y), axis=0)
        # when the robot reaches the end, add a point in the middle of robot in order to have better performance
        if gps_y.shape[0] < 2:# TODO
            gps_x = np.concatenate((np.zeros(1),gps_x), axis=0)
            gps_y = np.concatenate((np.array([.5]),gps_y), axis=0)

        
        #calculate a reference trajectory
        ref_x, ref_y, ref_yaw, ref_k, s = calc_spline_course(gps_x, gps_y, ds=self.dt)

        ref_x = ref_x[2:len(ref_x)]  # for AMI, we delete 2 points at begin  TODO
        ref_y = ref_y[2:len(ref_y)]

        #write the reference trajectory
        self.outputs["ref_x"].write(ref_x)
        #print(len(ref_x),ref_x)
        # print('x',ref_x)
        self.outputs["ref_y"].write(ref_y)
        # print('y',ref_y)

        #create an input to display the reference trajectory
        # ref_y : [[y1], [y2], [y3], ...]
        ref_y = np.asarray(ref_y)
        ref_y = ref_y.reshape((ref_y.shape[0],1))
        # ref_x : [[x1], [x2], [x3], ...]
        ref_x = -1 * np.asarray(ref_x)
        ref_x = ref_x.reshape((ref_x.shape[0],1))
        # ref_z : array of 0s
        ref_z = np.zeros((ref_x.shape[0],1))
        
        gps_xy = np.concatenate((ref_y,ref_x,ref_z), axis = 1)
        gps_xy = gps_xy.reshape(-1)
        self.outputs["viewer"].write(gps_xy)

    def Death(self):
        pass
