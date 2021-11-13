"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)

"""
import math
import numpy as np
import bisect
from scipy.integrate import quad


class Spline:
    """
    Cubic Spline class
    """

    def __init__(self, x, y):
        self.b, self.c, self.d, self.w = [], [], [], []

        self.x = x
        self.y = y

        self.nx = len(x)  # dimension of x
        h = np.diff(x)

        # calc coefficient c
        self.a = [iy for iy in y]

        # calc coefficient c
        A = self.__calc_A(h)
        B = self.__calc_B(h)
        self.c = np.linalg.solve(A, B)
        #  print(self.c1)

        # calc spline coefficient b and d
        for i in range(self.nx - 1):
            self.d.append((self.c[i + 1] - self.c[i]) / (3.0 * h[i]))
            tb = (self.a[i + 1] - self.a[i]) / h[i] - h[i] * \
                (self.c[i + 1] + 2.0 * self.c[i]) / 3.0
            self.b.append(tb)

    def calc(self, t):
        """
        Calc position

        if t is outside of the input x, return None

        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.a[i] + self.b[i] * dx + \
            self.c[i] * dx ** 2.0 + self.d[i] * dx ** 3.0

        return result

    def calcd(self, t):
        """
        Calc first derivative

        if t is outside of the input x, return None
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = self.b[i] + 2.0 * self.c[i] * dx + 3.0 * self.d[i] * dx ** 2.0
        return result

    def calcdd(self, t):
        """
        Calc second derivative
        """

        if t < self.x[0]:
            return None
        elif t > self.x[-1]:
            return None

        i = self.__search_index(t)
        dx = t - self.x[i]
        result = 2.0 * self.c[i] + 6.0 * self.d[i] * dx
        return result

    def __search_index(self, x):
        """
        search data segment index
        """
        return bisect.bisect(self.x, x) - 1

    def __calc_A(self, h):
        """
        calc matrix A for spline coefficient c
        """
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
        """
        calc matrix B for spline coefficient c
        """
        B = np.zeros(self.nx)
        for i in range(self.nx - 2):
            B[i + 1] = 3.0 * (self.a[i + 2] - self.a[i + 1]) / \
                h[i + 1] - 3.0 * (self.a[i + 1] - self.a[i]) / h[i]
        return B


class Spline2D:
    """
    2D Cubic Spline class

    """

    def __init__(self, x, y):
        self.s = self.__calc_s(x, y)
        self.sx = Spline(self.s, x)
        self.sy = Spline(self.s, y)

    def __calc_s(self, x, y):
        dx = np.diff(x)
        dy = np.diff(y)
        self.ds = np.hypot(dx, dy)
        s = [0]
        s.extend(np.cumsum(self.ds))
        return s

    def calc_position(self, s):
        """
        calc position
        """
        x = self.sx.calc(s)
        y = self.sy.calc(s)

        return x, y

    def calc_curvature(self, s):
        """
        calc curvature
        """
        dx = self.sx.calcd(s)
        ddx = self.sx.calcdd(s)
        dy = self.sy.calcd(s)
        ddy = self.sy.calcdd(s)
        k = (ddy * dx - ddx * dy) / ((dx ** 2 + dy ** 2)**(3 / 2))
        return k
    
    def calc_path_length(self):
        """
        calc total path length
        """
        max_s_val = self.s[-1]
        self.s_dot = lambda t: max(np.linalg.norm(
            np.hypot(self.sx.calcd(t),self.sy.calcd(t))),1e-6)
        self.total_length = quad(lambda u: self.s_dot(u), 0, max_s_val)
        return self.total_length

    def calc_yaw(self, s):
        """
        calc yaw
        """
        dx = self.sx.calcd(s)
        dy = self.sy.calcd(s)
        yaw = math.atan2(dy, dx)
        return yaw


def calc_spline_course(x, y, ds=0.1):
    sp = Spline2D(x, y)
    s = list(np.arange(0, sp.s[-1], ds))

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    return rx, ry, ryaw, rk, s

class cubic_trajectory:
    def __init__(self,spline_object, v0 = 0.0, a0 = 0.0, max_accel=2.0, max_jerk=5.0):
        #spline object is an instance of SPline2D
        self.max_accel = float(max_accel)
        self.a0 = float(a0)
        self.max_jerk = float(max_jerk)
        self.v0 = v0
        self.total_length = spline_object.calc_path_length()


    def velocity_profile(self):
        r"""                  /~~~~~----------------\
                             /                       \
                            /                         \
                           /                           \
                          /                             \
        (v=v0, a=a0) ~~~~~                               \
                                                          \
                                                           \ ~~~~~ (vf=0, af=0)
                     pos.|pos.|neg.|   cruise at    |neg.| neg. |neg.
                     max |max.|max.|     max.       |max.| max. |max.
                     jerk|acc.|jerk|    velocity    |jerk| acc. |jerk
            index     0    1    2      3 (optional)   4     5     6
        """
        # delta_a: accel change from initialposition to end of maximal jerk section
        delta_a = self.max_accel - self.a0
        # t_s1: time of traversal of maximal jerk section
        t_s1 = delta_a / self.max_jerk
        # v_s1: velocity at the end of the maximal jerk section
        v_s1 = self.v0 + self.a0 * t_s1 + self.max_jerk * t_s1**2 / 2.
        # s_s1: length of the maximal jerk section
        s_s1 = self.v0 * t_s1 + self.a0 * t_s1**2/2 + self.max_jerk * t_s1**3 / 6.
        # t_sf: time of traversal of final section, which is also maximal jerk, but has final velocity 0
        t_sf: self.max_accel / self.max_jerk
        # v_sf: velocity at begnning of final section
        v_sf = self.max_jerk * t_sf**2 / 2.
        # s_sf: length of final section
        s_sf = self.max_jerk * t_sf**3 /6.

        # Solve for the maximum achievable velocity based on the kinematic limits imposed by max_accel and max_jerk
        # this leads to a quadratic euqation in v_max: a*v_max**2 + b*v_max + c = 0
        a = 1 / self.max_accel
        b = 3. * self.max_accel / (2. * self.max_jerk) + v_s1 / self.max_accel - (
            self.max_accel**2 / self.max_jerk + v_s1) / self.max_accel
        c = s_s1 + s_sf - self.total_length - 7. * self.max_accel**3 / (3. * self.max_jerk**2) \
            - v_s1 * (self.max_accel / self.max_jerk + v_s1 / self.max_accel) \
            + (self.max_accel**2 / self.max_jerk + v_s1 /
               self.max_accel)**2 / (2. * self.max_accel)
        v_max = (-b + np.sqrt(b**2 - 4. * a * c)) / (2. * a)

        




def main():  # pragma: no cover
    print("Spline 2D test")
    import matplotlib.pyplot as plt

    with open('way_points.txt') as file:
        lines = file.readlines()
        lines = [[float(str_val) for str_val in line.rstrip().split()] for line in lines]
    # make lines into a numpy array and extend second dimensions with zeros for theta
    lines = np.pad(np.array(lines),[(0,0),(0,1)],mode='constant')
    x = lines[:,0]
    y = lines[:,1]

    # x = [-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0]
    # y = [0.7, -6, 5, 6.5, 0.0, 5.0, -2.0]
    ds = 0.01  # [m] distance of each interpolated points

    sp = Spline2D(x, y)
    s = np.arange(0, sp.s[-1], ds)

    rx, ry, ryaw, rk = [], [], [], []
    for i_s in s:
        ix, iy = sp.calc_position(i_s)
        rx.append(ix)
        ry.append(iy)
        ryaw.append(sp.calc_yaw(i_s))
        rk.append(sp.calc_curvature(i_s))

    plt.subplots(1)
    plt.plot(x, y, "xb", label="input")
    plt.plot(rx, ry, "-r", label="spline")
    plt.grid(True)
    plt.axis("equal")
    plt.xlabel("x[m]")
    plt.ylabel("y[m]")
    plt.legend()

    plt.subplots(1)
    plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("yaw angle[deg]")

    plt.subplots(1)
    plt.plot(s, rk, "-r", label="curvature")
    plt.grid(True)
    plt.legend()
    plt.xlabel("line length[m]")
    plt.ylabel("curvature [1/m]")

    plt.show()


if __name__ == '__main__':
    main()
