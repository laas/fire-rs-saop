import unittest
import fire_rs.uav_planning as up


def plot_dubins(waypoints):
    traj = up.Trajectory(up.UAV(1, 1))
    for wp in waypoints:
        traj = traj.with_waypoint_at_end(wp)
    plot_trajectory(traj)


def plot_trajectory(traj):
    import matplotlib.pyplot as plt
    sampled_waypoints = traj.as_waypoints(step_size=0.1)
    x = [wp.x for wp in sampled_waypoints]
    y = [wp.y for wp in sampled_waypoints]
    plt.scatter(x, y, s=0.1, c='b')

    waypoints = traj.as_waypoints()
    x = [wp.x for wp in waypoints]
    y = [wp.y for wp in waypoints]
    plt.scatter(x, y, c='r')
    plt.show(block=False)


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.uav = up.UAV(1, 1)
        self.wp1 = up.Waypoint(0, 0, 0)
        self.wp2 = up.Waypoint(4, 0, 0)

    def test_waypoint(self):
        wp1 = up.Waypoint(4, 3, 2)
        self.assertEquals(wp1.x, 4)
        self.assertEquals(wp1.y, 3)
        self.assertEquals(wp1.dir, 2)

    def test_straight_line_dubins(self):
        self.assertAlmostEquals(4, self.uav.travel_distance(self.wp1, self.wp2))
        traj = up.Trajectory(self.uav)
        for wp in [self.wp1, self.wp2]:
            traj = traj.with_waypoint_at_end(wp)
            print(traj)
        self.assertAlmostEquals(4, traj.length())

    def test_trajectory_plotting(self):
        plot_dubins([up.Waypoint(0, 0, 0), up.Waypoint(1, 1, 0), up.Waypoint(10, 10, 3.14), up.Waypoint(6, 0, 0),
                     up.Waypoint(7, 1, 0)])

