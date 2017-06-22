import unittest
import fire_rs.uav_planning as up
import numpy as np

# X8 UAS from Porto University
from fire_rs.geodata.geo_data import GeoData

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
        # plot_trajectory(traj)

    def test_trajectory_mutations(self):
        traj = get_trajectory([up.Waypoint(0, 0, 4.5), up.Waypoint(1.4, 4.4, 0), up.Waypoint(10, 10, 3.14),
                               up.Waypoint(5, 0, 0), up.Waypoint(7, 1, 0), up.Waypoint(15, 9, 2), up.Waypoint(15, 5, 0),
                               up.Waypoint(0, 10, 3.14), up.Waypoint(1, 9, 3.14)])
        print(traj.length())
        # plot_trajectory(traj, blocking=False)
        traj = up.improve(traj)
        print(traj.length())
        # plot_trajectory(traj, blocking=False)
        print(traj)

    def test_geodata_conversion(self):
        from fire_rs.geodata.environment import World
        world = World()
        gd = world.get_elevation([[475060.0,485060], [6200074.0, 6210074]])
        raster = gd.as_cpp_raster()
        print(raster)
        gd2 = GeoData.from_cpp_raster(raster, "elevation_cpp")
        print(gd2[10,10][0])
        self.assertAlmostEquals(gd[10,10][0], gd2[10,10][0])
        self.assertAlmostEquals(gd.x_offset, gd2.x_offset)
        self.assertAlmostEquals(gd.y_offset, gd2.y_offset)
        self.assertEquals(gd.data.shape, gd2.data.shape)
        print(gd2)

    def test_visibility(self):
        from fire_rs.firemodel import propagation
        env = propagation.Environment([[475060.0, 477060.0], [6200074.0, 6202074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20, horizon=3600)
        ignitions = prop.ignitions()
        v = up.Visibility(ignitions.as_cpp_raster(), 1800, 2700)
        bl = up.Segment(up.Waypoint(475160.0, 6200174, np.pi/2), 300)  # bottom left segment overlapping the interesting fire area
        tr = up.Segment(up.Waypoint(476500, 6201500, 0), 100)  # top right segment non-overlapping the fire zone

        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            GeoData.from_cpp_long_raster(cpp_raster, "xx").plot(blocking=blocking)

        # pr(v.interest, blocking=False)
        c1 = v.cost()
        pr(v.visibility)
        v.add_segment(uav, bl)
        # pr(v.visibility, blocking=False)
        self.assertLess(v.cost(), c1, "cost should have decreased")
        c2 = v.cost()
        v.add_segment(uav, tr)
        self.assertEquals(c2, v.cost(), "Cost should have been identical")
        v.remove_segment(uav, bl)
        self.assertAlmostEqual(c1, v.cost(), msg="Cost should have came back to its original value")
        pr(v.visibility)
        # prop.plot(blocking=False)

    def test_smart_insertion(self):
        from fire_rs.firemodel import propagation
        env = propagation.Environment([[475060.0, 477060.0], [6200074.0, 6202074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20, horizon=3600)
        ignitions = prop.ignitions()
        v = up.Visibility(ignitions.as_cpp_raster(), 1800, 2700)
        bl = up.Segment(up.Waypoint(475060.0, 6200074, np.pi/2), 300)  # bottom left segment overlapping the interesting fire area
        tr = up.Segment(up.Waypoint(476500, 6201500, 0), 100)  # top right segment non-overlapping the fire zone
        segments = [
            up.Segment(up.Waypoint(475060.0, 6200074, np.pi/2), 300),
            up.Segment(up.Waypoint(476500, 6201500, 0), 100),
            up.Segment(up.Waypoint(475160.0, 6201074, np.pi/2), 300),
            up.Segment(up.Waypoint(476700, 6201400, 0), 100)
        ]
        p = up.make_plan(uav, segments, v)
        # plot_trajectory(p.trajectories[0], blocking=False)

    def test_vns(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[475060.0, 477060.0], [6200074.0, 6202074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20, horizon=3600)
        prop.plot()
        ignitions = prop.ignitions()
        ignitions.plot(blocking=False)
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), 0, 2700)

        print(res.final_plan())
        print(res.final_plan().trajectories)




