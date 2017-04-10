import numpy as np


class UAV:
    """A very simple UAV model defined by its speed limits (airspeed, climb and descent)."""

    def __init__(self, min_airspeed, max_airspeed, max_rate_of_climb, max_rate_of_descent):
        """All speeds are in m/s"""
        self.min_airspeed = min_airspeed
        self.max_airspeed = max_airspeed
        self.max_rate_of_climb = max_rate_of_climb
        self.max_rate_of_descent = max_rate_of_descent

    def travel_time(self, origin, destination):
        """Returns the travel time (s) to go from a a point 'origin' to a point 'destination'.
    
        Both origin and destination are of the form (x, y) or (x, y, z) 
        """
        assert 2 <= len(origin) <= 3, "Origin should by (x, y) or (x, y, z)"
        assert 2 <= len(destination) <= 3, "Origin should by (x, y) or (x, y, z)"
        assert len(origin) == len(destination), "Elevation should be present in origin and destination or absent in both"
        if len(origin) == 2:
            xo, yo = origin
            xd, yd = destination
            zo = zd = 0
        else:
            assert len(origin) == 3
            xo, yo, zo = origin
            xd, yd, zd = destination

        ground_distance = np.sqrt((xd-xo)**2 + (yd-yo)**2)
        elevation_diff = zd - zo
        if elevation_diff >= 0:
            return max(ground_distance / self.max_airspeed, elevation_diff / self.max_rate_of_climb)
        else:
            return max(ground_distance / self.max_airspeed, -elevation_diff / self.max_rate_of_descent)

    def trajectory_length(self, trajectory):
        duration = 0
        for i in range(1, len(trajectory)):
            duration += self.travel_time(trajectory[i - 1], trajectory[i])
        return duration


# the X8 Skywalker UAV from PORTO
skywalker = UAV(10, 20, 2, 3)
