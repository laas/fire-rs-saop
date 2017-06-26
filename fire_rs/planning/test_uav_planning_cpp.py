import unittest
import fire_rs.uav_planning as up
import numpy as np

# X8 UAS from Porto University
from fire_rs.geodata.geo_data import GeoData

uav = up.UAV(18., 32.*np.pi/180)
uav = up.UAV(7., 10.*np.pi/180)

def plot_trajectory(traj, axes=None, blocking=False):
    import matplotlib.pyplot as plt
    if not axes:
        axes = plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")
    sampled_waypoints = traj.as_waypoints(step_size=2)
    x = [wp.x for wp in sampled_waypoints]
    y = [wp.y for wp in sampled_waypoints]
    axes.scatter(x, y, s=0.1, c='b')

    waypoints = traj.as_waypoints()
    x = [wp.x for wp in waypoints]
    y = [wp.y for wp in waypoints]
    axes.scatter(x, y, c='r')
    plt.show(block=blocking)
    return axes


class TestUAV(unittest.TestCase):

    def setUp(self):
        self.wp1 = up.Waypoint(0, 0, 0)
        self.wp2 = up.Waypoint(4, 0, 0)
        self.test_area = [[480060.0, 490060.0], [6210074.0, 6220074.0]]

    def test_waypoint(self):
        wp1 = up.Waypoint(4, 3, 2)
        self.assertEquals(wp1.x, 4)
        self.assertEquals(wp1.y, 3)
        self.assertEquals(wp1.dir, 2)

    def test_straight_line_dubins(self):
        self.assertAlmostEquals(4, uav.travel_distance(self.wp1, self.wp2))
        traj = up.Trajectory(up.TrajectoryConfig.build(uav))
        for wp in [self.wp1, self.wp2]:
            traj = traj.with_waypoint_at_end(wp)
            print(traj)
        self.assertAlmostEquals(4, traj.length())

    def test_trajectory_mutations(self):
        # builds a trajectory, makeing sure points are sufficiently spaced to allow interesting solutions
        def get_trajectory(waypoints):
            traj = up.Trajectory(up.TrajectoryConfig.build(uav))
            for wp in waypoints:
                traj = traj.with_waypoint_at_end(
                    up.Waypoint(wp.x * uav.min_turn_radius, wp.y * uav.min_turn_radius, wp.dir))
            return traj
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
        gd = world.get_elevation([[475060.0, 485060], [6200074.0, 6210074]])
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
        env = propagation.Environment(self.test_area, wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 10, 20, horizon=3600)
        ignitions = prop.ignitions()
        v = up.Visibility(ignitions.as_cpp_raster(), 0, 2700)
        bl = up.Segment(up.Waypoint(self.test_area[0][0] + 10*25, self.test_area[1][0] + 20*25, np.pi/2), 300)  # bottom left segment overlapping the interesting fire area
        tr = up.Segment(up.Waypoint(self.test_area[0][1], self.test_area[1][1], 0), 100)  # top right segment non-overlapping the fire zone

        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            GeoData.from_cpp_long_raster(cpp_raster, "xx").plot(blocking=blocking)

        # pr(v.interest, blocking=False)
        c1 = v.cost()
        # pr(v.visibility)
        v.add_segment(uav, bl)
        # pr(v.visibility, blocking=False)
        self.assertLess(v.cost(), c1, "cost should have decreased")
        c2 = v.cost()
        v.add_segment(uav, tr)
        self.assertEquals(c2, v.cost(), "Cost should have been identical")
        v.remove_segment(uav, bl)
        self.assertAlmostEqual(c1, v.cost(), msg="Cost should have came back to its original value")
        # pr(v.visibility)
        # prop.plot(blocking=False)

    def test_vns(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            return GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[480060.0, 485060.0], [6210074.0, 6214074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate(env, 100, 100, horizon=14000)
        ax = prop.plot()
        ignitions = prop.ignitions()
        # ax = ignitions.plot(blocking=False)
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), 11500, 13500)

        plan = res.final_plan()
        plot_trajectory(plan.trajectories[0], axes=ax, blocking=True)

        # plot_trajectory(plan.trajectories[0], axes=ax, blocking=False)
        # plot_trajectory(plan.trajectories[0], axes=ax, blocking=True)

        print("durations: ")
        for traj in plan.trajectories:
            print(traj.duration())






