# cython: profile=True

from libc.math cimport tan, sin, sqrt, exp, pow
from libc.math cimport M_PI as PI

cdef struct ProjectionResult:
    double length
    double x
    double y


cdef ProjectionResult projection_on_ellipse(double a, double b, double x_origin, double angle):
    """Computes the length of a line starting in (x_origin, 0) in a direction "angle" until it reaches the border
     of the ellipse.

    :param a: Half length of the ellipse
    :param b: Half width of the ellipse
    :param x_origin: X-coordinate in [-a,a] of a point on the main axis of the ellipse, (0, 0) being the center of
     the ellipse. Y-coordinate is always 0.
    :param angle: Angle at which to project the point
    :return: (length, x_inter, y_inter) where "length" is the length of the line (x_orig, 0) -- (x_inter, y_inter)
     and (x_inter, y_inter) is the coordinates of the projection of (x_orig, 0) on the ellipse in the direction of
     the angle
    """
    cdef double x, y, alpha, beta, f2, f1, f0, x1, x2, y1, y2, length

    if PI/2 -0.0001 < angle % (2*PI) < PI/2 + 0.0001:
        # angle is around PI/2, intersection is at the top of the ellipse
        # some margin is taken to avoid overflows as tan(x) tends to infinity when x tends to PI/2
        x, y = 0, b
    elif PI*3/2 -0.0001 < angle % (2*PI) < PI*3/2 + 0.0001:
        # angle is around PI/2, intersection is at the top of the ellipse
        # some margin is taken to avoid overflows as tan(x) tends to -infinity when x tends to -PI/2
        x, y = 0, -b
    else:
        alpha = tan(angle)  # coefficient directeur
        beta = - alpha * x_origin
        # We need to solve the equation system
        # x**2/a**2 + y**2/b**2 = 1   (ellipse equation)
        # y = alpha * x + beta  (line equation of the direction)
        # sign(y) = sign(sin(angle))  (sens of the direction)
        # The two first equations reduce to f2*x**2 + f1*x + f0 = 0 where
        f2 = b**2 + a**2 * alpha**2
        f1 = 2 * a**2 * beta * alpha
        f0 = beta**2 * a**2 - a**2 * b**2

        # quadratic formula gives the two roots:
        x1 = (-f1 + sqrt(f1 ** 2 - 4 * f2 * f0)) / (2 * f2)
        x2 = (-f1 - sqrt(f1 ** 2 - 4 * f2 * f0)) / (2 * f2)
        y1 = alpha * x1 + beta
        y2 = alpha * x2 + beta

        if y1 * sin(angle) >= 0:  # same sign
            x, y = x1, y1
        else:
            assert y2 * sin(angle) >= 0
            x, y = x2, y2

    assert 0.99999 <= (x / a) ** 2 + (y / b) ** 2 <= 1.00001 # The computed intersection is not on the ellipse
    length = sqrt((x - x_origin)**2 + y**2)
    cdef ProjectionResult res
    res.length = length
    res.x = x
    res.y = y
    return res


# For increased efficiency, we could merge the two distinct propagation shapes into one.
cdef class PropagationShape:
    def speed(self, angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        return

    def dist_ignition_to_center(self):
        """Returns the distance from the ignition point to the center of the ellipse."""
        return


cdef class DoubleEllipsePropagationShape(PropagationShape):
    """Double ellipse fire shape [Anderson 83]"""
    cdef double effective_wind_angle  # main spread direction [rad]
    cdef double ros  # rate of spread [m/s]
    cdef double a_back  # half length of the back ellipse
    cdef double a_front  # half length of the front ellipse
    cdef double b  # half width of both ellipses
    cdef double x_ignition_position  # position of the ignition point on the x-axis ((0,0) being the center)

    def __init__(self, double effective_wind_speed, double effective_wind_angle, double ros):
        """Creates a double ellipse shape from wind and rate of spread

        :param effective_wind_speed: Normalized effect of wind and slope [km/h]
        :param effective_wind_angle: Combined normalized direction of wind and slope [radians]
        :param ros: Rate of Spread along the main direction [m/s]
        """
        self.effective_wind_angle = effective_wind_angle
        self.ros = ros
        cdef double u = 0.621371192 * effective_wind_speed  # wind speed in miles/h
        # dimension less characterisation of back and front ellipses
        cdef double c_dl = 0.492 * exp(-0.1845 * u)  # back-propagation speed
        cdef double a_back_dl = 2.502 * pow(88*u, -0.3)  # half length of back ellipse
        cdef double a_front_dl = 1 + c_dl - a_back_dl  # half length of front ellipse
        cdef double b_dl = 0.534 * exp(-0.1147 * u)  # half width of both ellipses

        # we have ros = a1 + a2 -c, compute the factor to apply to all dimension less values
        cdef double f = ros / (a_back_dl + a_front_dl - c_dl)
        self.a_back = f * a_back_dl
        self.a_front = f * a_front_dl
        self.b = f * b_dl
        self.x_ignition_position = f*c_dl - self.a_back

    def speed(self, double angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        cdef double rel_angle = angle - self.effective_wind_angle
        cdef ProjectionResult res = <ProjectionResult>projection_on_ellipse(self.a_back, self.b, self.x_ignition_position, rel_angle)
        if res.x <= 0:
            # we are indeed on the back ellipse
            return res.length
        else:
            res = <ProjectionResult>projection_on_ellipse(self.a_front, self.b, self.x_ignition_position, rel_angle)
            assert res.x >= 0, "Seems that we can't find an intersection on either ellipses"
            return res.length

    def dist_ignition_to_center(self):
        """Returns the distance from the ignition point to the center of the ellipse."""
        return self.x_ignition_position


cdef class SingleEllipsePropagationShape(PropagationShape):
    """Single ellipse propagation shape.

    This model is proposed by [Alexander 85] and we use the adaptation of [Rios 14] to get circular shape when
    there is no wind."""
    cdef double effective_wind_angle
    cdef double ros
    cdef double a  # half length of the ellipse
    cdef double b  # half width of the ellipse
    cdef double c  # dist from ignition point to center (focal point)

    def __init__(self, double effective_wind_speed, double effective_wind_angle, double ros):
        """Creates a double ellipse shape from wind and rate of spread

        :param effective_wind_speed: Normalized effect of wind and slope [km/h]
        :param effective_wind_angle: Combined normalized direction of wind and slope [radians]
        :param ros: Rate of Spread along the main direction [m/s]
        """
        self.effective_wind_angle = effective_wind_angle
        self.ros = ros
        cdef double u = effective_wind_speed * 0.2777778
        # length to breadth ratio of the ellipse, given by the effective wind speed
        cdef double LB = 0.936 * exp(0.2566*u) + 0.461 * exp(-0.1548*u) - 0.397

        # a: half of length
        # b: half of breadth
        # LB = a/b
        # c: focal point (i.e. ignition point) to center
        # ros = a + c
        self.a = ros / (1 + sqrt(LB*LB-1)/LB)
        self.b = self.a / LB
        self.c = self.b * sqrt(LB*LB - 1)  # dist from ignition point to center

    def speed(self, double angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        cdef double rel_angle = angle - self.effective_wind_angle
        cdef ProjectionResult res = <ProjectionResult>projection_on_ellipse(self.a, self.b, -self.c, rel_angle)
        return res.length

    def dist_ignition_to_center(self):
        """Returns the distance from the ignition point to the center of the ellipse."""
        return self.c


def get_fire_shape(effective_wind_speed, effective_wind_angle, ros, kind='auto'):
    """Build a fire shape from the effective wind and the rate of spread.

    :param effective_wind_speed: Effective wind speed [km/h], combining the wind and slope
    :param effective_wind_angle: Effective wind angle [radians], combining the wind and slope
    :param ros: Rate of Spread [m/s]
    :param kind: Possible values are 'single' for a single ellipse shape, 'double' for a double ellipse shape and
     'auto' (default). For 'auto', the single ellipse shape will be selected for wind speed below 2km/H and the double ellipse
     for all other situations.
    :return: A propagation shape.
    """
    assert kind in ['auto', 'single', 'double'], "Unknown fire shape"
    if kind == 'single' or kind == 'auto' and effective_wind_speed < 2:
        return SingleEllipsePropagationShape(effective_wind_speed, effective_wind_angle, ros)
    else:
        return DoubleEllipsePropagationShape(effective_wind_speed, effective_wind_angle, ros)

