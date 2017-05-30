import unittest
import fire_rs.uav_planning as up
import numpy as np

# X8 UAS from Porto University
uav = up.UAV(18., 32.*np.pi/180)


def get_trajectory(waypoints):
    traj = up.Trajectory(uav)
    for wp in waypoints:
        traj = traj.with_waypoint_at_end(up.Waypoint(wp.x * uav.min_turn_radius, wp.y * uav.min_turn_radius, wp.dir))
    return traj


def plot_trajectory(traj, blocking=False):
    import matplotlib.pyplot as plt
    ax = plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")
    sampled_waypoints = traj.as_waypoints(step_size=2)
    x = [wp.x for wp in sampled_waypoints]
    y = [wp.y for wp in sampled_waypoints]
    ax.scatter(x, y, s=0.1, c='b')

    waypoints = traj.as_waypoints()
    x = [wp.x for wp in waypoints]
    y = [wp.y for wp in waypoints]
    ax.scatter(x, y, c='r')
    plt.show(block=blocking)


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.wp1 = up.Waypoint(0, 0, 0)
        self.wp2 = up.Waypoint(4, 0, 0)

    def test_waypoint(self):
        wp1 = up.Waypoint(4, 3, 2)
        self.assertEquals(wp1.x, 4)
        self.assertEquals(wp1.y, 3)
        self.assertEquals(wp1.dir, 2)

    def test_straight_line_dubins(self):
        self.assertAlmostEquals(4, uav.travel_distance(self.wp1, self.wp2))
        traj = up.Trajectory(uav)
        for wp in [self.wp1, self.wp2]:
            traj = traj.with_waypoint_at_end(wp)
            print(traj)
        self.assertAlmostEquals(4, traj.length())

    def test_trajectory_plotting(self):
        traj = get_trajectory([up.Waypoint(0, 0, 0), up.Waypoint(1, 1, 0), up.Waypoint(10, 10, 3.14), up.Waypoint(6, 0, 0),
                     up.Waypoint(7, 1, 0)])
        plot_trajectory(traj)

    def test_trajectory_mutations(self):
        traj = get_trajectory([up.Waypoint(0, 0, 2), up.Waypoint(1.4, 4.4, 0), up.Waypoint(10, 10, 3.14),
                               up.Waypoint(5, 0, 0), up.Waypoint(7, 1, 0), up.Waypoint(15, 9, 2), up.Waypoint(15, 5, 0),
                               up.Waypoint(0, 10, 3.14), up.Waypoint(1, 9, 3.14)])
        print(traj.length())
        plot_trajectory(traj, blocking=False)
        traj = up.improve(traj)
        print(traj.length())
        plot_trajectory(traj, blocking=True)
        print(traj)

        print("end")




