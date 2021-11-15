"""
Cubic spline planner

Author: Atsushi Sakai(@Atsushi_twi)

"""
import math
import numpy as np
import bisect
from scipy.integrate import quad
from matplotlib.collections import LineCollection

class MaxVelocityNotReached(Exception):
    def __init__(self, actual_vel, max_vel):
        self.message = 'Actual velocity {} does not equal desired max velocity {}!'.format(
            actual_vel, max_vel)

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
        # try this if there is an error with indexing
        # return min(bisect.bisect(self.x, x) - 1,len(self.x) - 2)
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
        self.s_dot = lambda t: max(
            np.hypot(self.sx.calcd(t),self.sy.calcd(t)),1e-6)
        self.path_len = lambda param: quad(lambda u: self.s_dot(u), 0, param)
        self.total_length = self.path_len(max_s_val)[0]
        return self.total_length

    def xy_derivative_1(self,s):
        """
        return derivatives at s
        """
        return np.array([self.sx.calcd(s),self.sy.calcd(s)])

    def xy_derivative_2(self,s):
        """
        return derivatives at s
        """
        return np.array([self.sx.calcdd(s),self.sy.calcdd(s)])

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
    def __init__(self,spline_object, max_vel, v0 = 0.0, a0 = 0.0, max_accel=2.0, max_jerk=5.0):
        #spline object is an instance of SPline2D
        self.max_accel = float(max_accel)
        self.a0 = float(a0)
        self.max_jerk = float(max_jerk)
        self.v0 = v0
        self.total_length = spline_object.calc_path_length()
        self.spline_object = spline_object
        self.max_vel = float(max_vel)
        # compute velocity profile
        self.velocity_profile()
        self.param_prev = 0


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
        t_sf = self.max_accel / self.max_jerk
        # v_sf: velocity at begnning of final section
        v_sf = self.max_jerk * t_sf**2 / 2.
        # s_sf: length of final section
        s_sf = self.max_jerk * t_sf**3 /6.

        # Solve for the maximum achievable velocity based on the kinematic limits imposed by max_accel and max_jerk
        # this leads to a quadratic euqation in v_max: a*v_max**2 + b*v_max + c = 0
        print(self.max_accel)
        print(self.max_jerk)
        print(v_s1)
        print(self.total_length)
        print(s_s1)
        print(s_sf)

        a = 1 / self.max_accel
        b = 3. * self.max_accel / (2. * self.max_jerk) + v_s1 / self.max_accel - (
            self.max_accel**2 / self.max_jerk + v_s1) / self.max_accel
        c = s_s1 + s_sf - self.total_length - 7. * self.max_accel**3 / (3. * self.max_jerk**2) \
            - v_s1 * (self.max_accel / self.max_jerk + v_s1 / self.max_accel) \
            + (self.max_accel**2 / self.max_jerk + v_s1 /
               self.max_accel)**2 / (2. * self.max_accel)
        v_max = (-b + np.sqrt(b**2 - 4. * a * c)) / (2. * a)

        # v_max represents the maximum velocity that could be attained if there was no cruise period
        # (i.e. driving at constant speed without accelerating or jerking)
        # if this velocity is less than our desired max velocity, the max velocity need to be updated
        if self.max_vel > v_max:
            # when this condition is tripped, there will be no cruise period (s_cruise=0)
            self.max_vel = v_max
        
        # setup arrays to store values at END of trajectory sections
        self.times = np.zeros((7,))
        self.vels = np.zeros((7,))
        self.seg_lengths = np.zeros((7,))

        # Section 0: max jerk up to max acceleration
        self.times[0] = t_s1
        self.vels[0] = v_s1
        self.seg_lengths[0] = s_s1

        # Section 1: accelerate at max_accel
        index = 1
        # compute change in velocity over the section
        delta_v = (self.max_vel - self.max_jerk * (self.max_accel /
                                                   self.max_jerk)**2 / 2.) - self.vels[index - 1]
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] + \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] + self.max_accel * self.times[index]**2 / 2.

        # Section 2: decrease acceleration (down to 0) until max speed is hit
        index = 2
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.vels[index - 1] + self.max_accel * self.times[index] \
            - self.max_jerk * self.times[index]**2 / 2.
        
        # as a check, the velocity at the end of the section should be self.max_vel
        if not np.isclose(self.vels[index], self.max_vel):
            raise MaxVelocityNotReached(self.vela[index], self.max_vel)
        
        self.seg_lengths[index] = self.vels[index - 1] * self.times[index] + self.max_accel * self.times[index]**2 / 2. \
            - self.max_jerk * self.times[index]**3 / 6.
        # Section 3: will be done last
        
        # Section 4: Apply min jerk until min accn is hit
        index = 4
        self.times[index] = self.max_accel / self.max_jerk
        self.vels[index] = self.max_vel - \
            self.max_jerk * self.times[index]**2 / 2.
        self.seg_lengths[index] = self.max_vel * self.times[index] - \
            self.max_jerk * self.times[index]**3 / 6.
        
        # Section 5: continue deceleration at max rate
        index = 5
        # Compute velocity change over sections
        delta_v = self.vels[index - 1] - v_sf
        self.times[index] = delta_v / self.max_accel
        self.vels[index] = self.vels[index - 1] - \
            self.max_accel * self.times[index]
        self.seg_lengths[index] = self.vels[index - 1] * \
            self.times[index] - self.max_accel * self.times[index]**2 / 2.

        # Section 6(final): max jerk to get to zero velocity and zero acceleration simultaneously
        index = 6
        self.times[index] = t_sf
        self.vels[index] = self.vels[index - 1] - self.max_jerk * t_sf**2 /2.
        print(self.vels)
        print(self.times)
        print(self.seg_lengths)
        try:
            assert np.isclose(self.vels[index], 0)
        except AssertionError as e:
            print('The final veloity {} is not zero'.format(self.vels[index]))
            raise e
        
        self.seg_lengths[index] = s_sf

        if self.seg_lengths.sum() < self.total_length:
            index = 3
            # the length of the crusie section is whatever length hasn't already been accounted for
            # NOTE: the total array self.seg_lengths is summed because the entry for the cruise segment is
            # initialized to 0!
            self.seg_lengths[index] = self.total_length - \
                self.seg_lengths.sum()
            self.vels[index] = self.max_vel
            self.times[index] = self.seg_lengths[index] / self.max_vel  

        # Make sure that all of the times are positive, otherwise the kinematic limits
        # chosen cannot be enforces on the path.
        assert(np.all(self.times >= 0))
        self.total_time = self.times.sum()

    def calc_traj_point(self, time):
        # Compute velocity at time
        if time <= self.times[0]:
            linear_velocity = self.v0 + self.max_jerk * time**2 / 2.
            s = self.v0 * time + self.max_jerk * time**3 / 6
            linear_accel = self.max_jerk * time
        elif time <= self.times[:2].sum():
            delta_t = time - self.times[0]
            linear_velocity = self.vels[0] + self.max_accel * delta_t
            s = self.seg_lengths[0] + self.vels[0] * \
                delta_t + self.max_accel * delta_t**2 / 2.
            linear_accel = self.max_accel
        elif time <= self.times[:3].sum():
            delta_t = time - self.times[:2].sum()
            linear_velocity = self.vels[1] + self.max_accel * \
                delta_t - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:2].sum() + self.vels[1] * delta_t + self.max_accel * delta_t**2 / 2. \
                - self.max_jerk * delta_t**3 / 6.
            linear_accel = self.max_accel - self.max_jerk * delta_t
        elif time <= self.times[:4].sum():
            delta_t = time - self.times[:3].sum()
            linear_velocity = self.vels[3]
            s = self.seg_lengths[:3].sum() + self.vels[3] * delta_t
            linear_accel = 0.
        elif time <= self.times[:5].sum():
            delta_t = time - self.times[:4].sum()
            linear_velocity = self.vels[3] - self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:4].sum() + self.vels[3] * \
                delta_t - self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_jerk * delta_t
        elif time <= self.times[:-1].sum():
            delta_t = time - self.times[:5].sum()
            linear_velocity = self.vels[4] - self.max_accel * delta_t
            s = self.seg_lengths[:5].sum() + self.vels[4] * \
                delta_t - self.max_accel * delta_t**2 / 2.
            linear_accel = -self.max_accel
        elif time < self.times.sum():
            delta_t = time - self.times[:-1].sum()
            linear_velocity = self.vels[5] - self.max_accel * \
                delta_t + self.max_jerk * delta_t**2 / 2.
            s = self.seg_lengths[:-1].sum() + self.vels[5] * delta_t - self.max_accel * delta_t**2 / 2. \
                + self.max_jerk * delta_t**3 / 6.
            linear_accel = -self.max_accel + self.max_jerk * delta_t
        else:
            linear_velocity = 0.
            s = self.total_length
            linear_accel = 0.
        curr_length = s
        interpol_param = self.get_interp_param(
            s = curr_length, prev_param = self.param_prev)
        # print(interpol_param)
        self.param_prev = interpol_param
        # compute anglular veclocity of current point= (ydd*xd - xdd*yd)/9xd**2 + yd**2)
        d = self.spline_object.xy_derivative_1(interpol_param)
        dd = self.spline_object.xy_derivative_2(interpol_param)
        # su - the rate of change of arclength wrt u
        su = self.spline_object.s_dot(interpol_param)
        if not np.isclose(su, 0.) and not np.isclose(linear_velocity, 0.):
            # ut - time derivative of interpolation paramter u
            ut = linear_velocity / su
            # utt - time-derivative of ut
            utt = linear_accel / su - \
                (d[0] * dd[0] + d[1] * dd[1]) / su**2 * ut
            xt = d[0] * ut
            yt = d[1] * ut
            xtt = dd[0] * ut**2 + d[0] * utt
            ytt = dd[1] * ut**2 + d[1] * utt
            angular_velocity = (ytt * xt - xtt * yt) / linear_velocity**2
        else:
            angular_velocity = 0.
        
        # combine path point with orienation and velocities
        pos = self.spline_object.calc_position(interpol_param)
        state = np.array([pos[0], pos[1], np.arctan2(
            d[1], d[0]), linear_velocity, angular_velocity])
        return state

    def get_interp_param(self, s, prev_param, tol=0.001):
        def f(u):
            return self.spline_object.path_len(prev_param)[0] - s
        
        def fprime(u):
            return self.spline_object.s_dot(prev_param)
        
        while (0 <= prev_param <= self.spline_object.s[-1]) and abs(f(prev_param)) > tol:
            prev_param -= f(prev_param) / fprime(prev_param)
        
        new_param = max(0, min(prev_param, self.spline_object.s[-1]))
        #print("new_param: {}".format(new_param))
        return new_param


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

    cub_traj = cubic_trajectory(sp, max_vel=0.15, max_accel=0.05, max_jerk=0.1)

    # interpolate at several points along the path
    print("total time for execution is {}s".format(cub_traj.total_time))
    times = np.linspace(0, cub_traj.total_time, 501)
    state = np.empty((5, times.size))
    for i, t in enumerate(times):
        state[:, i] = cub_traj.calc_traj_point(t)

    fig, ax = plt.subplots()
    x_T, y_T = state[0, :], state[1, :]
    points = np.array([x_T, y_T]).T.reshape(-1, 1, 2)
    segs = np.concatenate([points[:-1], points[1:]], axis=1)
    lc = LineCollection(segs, cmap=plt.get_cmap('inferno'))
    ax.set_xlim(np.min(x_T) - 1, np.max(x_T) + 1)
    ax.set_ylim(np.min(y_T) - 1, np.max(y_T) + 1)
    lc.set_array(state[3, :])
    lc.set_linewidth(3)
    ax.add_collection(lc)
    axcb = fig.colorbar(lc)
    axcb.set_label('velocity(m/s)')
    ax.set_title('Trajectory')
    plt.xlabel('x')
    plt.ylabel('y')
    plt.pause(1.0)

    cmap_copy = lc.get_color()
    # print(cmap_copy)

    fig1, ax1 = plt.subplots()
    ax1.plot(times, state[3, :], 'b-')
    ax1.set_xlabel('time(s)')
    ax1.set_ylabel('velocity(m/s)', color='b')
    ax1.tick_params('y', colors='b')
    ax1.set_title('Control')
    ax2 = ax1.twinx()
    ax2.plot(times, state[4, :], 'r-')
    ax2.set_ylabel('angular velocity(rad/s)', color='r')
    ax2.tick_params('y', colors='r')
    fig.tight_layout()
    plt.show()

    # plt.subplots(1)
    # plt.plot(x, y, "xb", label="input")
    # plt.plot(rx, ry, "-r", label="spline")
    # plt.grid(True)
    # plt.axis("equal")
    # plt.xlabel("x[m]")
    # plt.ylabel("y[m]")
    # plt.legend()

    # plt.subplots(1)
    # plt.plot(s, [np.rad2deg(iyaw) for iyaw in ryaw], "-r", label="yaw")
    # plt.grid(True)
    # plt.legend()
    # plt.xlabel("line length[m]")
    # plt.ylabel("yaw angle[deg]")

    # plt.subplots(1)
    # plt.plot(s, rk, "-r", label="curvature")
    # plt.grid(True)
    # plt.legend()
    # plt.xlabel("line length[m]")
    # plt.ylabel("curvature [1/m]")

    # plt.show()


if __name__ == '__main__':
    main()
