# cython: profile=True

import numpy as np


class Ellipse:
    """Internal class used to get the projection of points on the ellipse. It used internally by single and double
    ellipse propagation shape to compute the rate of spread in a given direction.
    """
    def __init__(self, a, b):
        """Initialize an ellipse.

        :param a: Half length of the ellipse
        :param b: Half width of the ellipse
        """
        self.a = a
        self.b = b

    def projection(self, x_origin, angle):
        """Computes the length of a line starting in (x_origin, 0) in a direction "angle" until it reaches the border
         of the ellipse.

        :param x_origin: X-coordinate in [-a,a] of a point on the main axis of the ellipse, (0, 0) being the center of
         the ellipse. Y-coordinate is always 0.
        :param angle: Angle at which to project the point
        :return: (length, x_inter, y_inter) where "length" is the length of the line (x_orig, 0) -- (x_inter, y_inter)
         and (x_inter, y_inter) is the coordinates of the projection of (x_orig, 0) on the ellipse in the direction of
         the angle
        """
        x = 0.
        y = 0.
        if np.pi/2 -0.0001 < angle % (2*np.pi) < np.pi/2 + 0.0001:
            # angle is around PI/2, intersection is at the top of the ellipse
            # some margin is taken to avoid overflows as tan(x) tends to infinity when x tends to PI/2
            x, y = 0, self.b
        elif np.pi*3/2 -0.0001 < angle % (2*np.pi) < np.pi*3/2 + 0.0001:
            # angle is around PI/2, intersection is at the top of the ellipse
            # some margin is taken to avoid overflows as tan(x) tends to -infinity when x tends to -PI/2
            x, y = 0, -self.b
        else:
            alpha = np.tan(angle)  # coefficient directeur
            beta = - alpha * x_origin
            # We need to solve the equation system
            # x**2/a**2 + y**2/b**2 = 1   (ellipse equation)
            # y = alpha * x + beta  (line equation of the direction)
            # sign(y) = sign(sin(angle))  (sens of the direction)
            # The two first equations reduce to f2*x**2 + f1*x + f0 = 0 where
            f2 = self.b**2 + self.a**2 * alpha**2
            f1 = 2 * self.a**2 * beta * alpha
            f0 = beta**2 * self.a**2 - self.a**2 * self.b**2

            # quadratic formula gives the two roots:
            x1 = (-f1 + np.sqrt(f1 ** 2 - 4 * f2 * f0)) / (2 * f2)
            x2 = (-f1 - np.sqrt(f1 ** 2 - 4 * f2 * f0)) / (2 * f2)
            y1 = alpha * x1 + beta
            y2 = alpha * x2 + beta

            if np.sign(y1) == np.sign(np.sin(angle)):
                x, y = x1, y1
            else:
                assert np.sign(y2) == np.sign(np.sin(angle))
                x, y = x2, y2

        assert 0.99999 <= (x / self.a) ** 2 + (y / self.b) ** 2 <= 1.00001,\
            "The computed intersection is not on the ellipse: {}".format(x ** 2 / self.a ** 2 + y ** 2 / self.b ** 2)
        length = np.sqrt((x - x_origin)**2 + y**2)
        return length, x, y


class PropagationShape:
    def speed(self, angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        return

    def dist_ignition_to_center(self):
        """Returns the distance from the ignition point to the center of the ellipse."""
        return


class DoubleEllipsePropagationShape(PropagationShape):
    """Double ellipse fire shape [Anderson 83]"""

    def __init__(self, effective_wind_speed, effective_wind_angle, ros):
        """Creates a double ellipse shape from wind and rate of spread

        :param effective_wind_speed: Normalized effect of wind and slope [km/h]
        :param effective_wind_angle: Combined normalized direction of wind and slope [radians]
        :param ros: Rate of Spread along the main direction [m/s]
        """
        self.effective_wind_angle = effective_wind_angle
        self.ros = ros
        u = 0.621371192 * effective_wind_speed  # wind speed in miles/h
        # dimension less characterisation of back and front ellipses
        c_dl = 0.492 * np.exp(-0.1845 * u)  # back-propagation speed
        a1_dl = 2.502 * np.power(88*u, -0.3)  # half length of back ellipse
        a2_dl = 1 + c_dl - a1_dl  # half length of front ellipse
        b_dl = 0.534 * np.exp(-0.1147 * u)  # half width of both ellipses

        # we have ros = a1 + a2 -c, compute the factor to apply to all dimension less values
        f = ros / (a1_dl + a2_dl - c_dl)
        self.a1 = f * a1_dl
        self.a2 = f * a2_dl
        self.b = f * b_dl
        self.c = f * c_dl

        self.back_ellipse = Ellipse(self.a1, self.b)
        self.front_ellipse = Ellipse(self.a2, self.b)

    def speed(self, angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        rel_angle = angle - self.effective_wind_angle
        length, x, y = self.back_ellipse.projection(self.c - self.a1, rel_angle)
        if x <= 0:
            # we are indeed on the back ellipse
            return length
        else:
            length, x, y = self.front_ellipse.projection(self.c - self.a1, rel_angle)
            assert x >= 0, "Seems that we can't find an intersection on either ellipses"
            return length

    def dist_ignition_to_center(self):
        """Returns the distance from the ignition point to the center of the ellipse."""
        return self.c - self.a1


class SingleEllipsePropagationShape(PropagationShape):
    """Single ellipse propagation shape.

    This model is proposed by [Alexander 85] and we use the adaptation of [Rios 14] to get circular shape when
    there is no wind."""
    def __init__(self, effective_wind_speed, effective_wind_angle, ros):
        """Creates a double ellipse shape from wind and rate of spread

        :param effective_wind_speed: Normalized effect of wind and slope [km/h]
        :param effective_wind_angle: Combined normalized direction of wind and slope [radians]
        :param ros: Rate of Spread along the main direction [m/s]
        """
        self.effective_wind_angle = effective_wind_angle
        self.ros = ros
        u = effective_wind_speed * 0.2777778
        # length to breadth ratio of the ellipse, given by the effective wind speed
        LB = 0.936 * np.exp(0.2566*u) + 0.461 * np.exp(-0.1548*u) - 0.397

        # a: half of length
        # b: half of breadth
        # LB = a/b
        # c: focal point (i.e. ignition point) to center
        # ros = a + c
        a = ros / (1 + np.sqrt(LB*LB-1)/LB)
        b = a / LB
        self.c = b * np.sqrt(LB*LB - 1)  # dist from ignition point to center
        self.ellipse = Ellipse(a, b)

    def speed(self, angle):
        """Returns the rate of spread along the absolute direction given.

        :param angle: Angle at which to compute the rate of spread [radians]
        :return: Rate of Spread in the given direction [m/s]
        """
        rel_angle = angle - self.effective_wind_angle
        length, x, y = self.ellipse.projection(-self.c, rel_angle)
        return length

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


if __name__ == '__main__':
    e = Ellipse(5, 2)
    print(e.projection(-1, np.pi))

