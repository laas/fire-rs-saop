import unittest
import fire_rs.uav_planning as up
import numpy as np

# X8 UAS from Porto University
from fire_rs.geodata.geo_data import GeoData, TimedPoint
from fire_rs.planning.benchmark import plot_trajectory, plot_plan

uav = up.UAV(18., 32.*np.pi/180)
uav = up.UAV(9., 10.*np.pi/180)


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
        prop = propagation.propagate_from_points(env, [TimedPoint(self.test_area[0][0]+250, self.test_area[1][0]+500, 0)], horizon=3600)
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

    def test_vns_small(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            return GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[480060.0, 481060.0], [6210074.0, 6211074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, TimedPoint(480460, 6210374, 0), horizon=3000)

        ignitions = prop.ignitions()
        # ax = ignitions.plot(blocking=False)
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), 2500, 3000, 150, save_every=5)

        # ax = prop.plot()
        # plan = res.final_plan()
        # plot_trajectory(plan.trajectories[0], axes=ax, blocking=False)
        self.plot_search_result(prop, res)

        print("durations: ")
        for traj in res.final_plan().trajectories:
            print(traj.duration())

    def test_vns(self):
        def pr(cpp_raster, blocking=False):  # plots a cpp raster
            return GeoData.from_cpp_raster(cpp_raster, "xx").plot(blocking=blocking)

        from fire_rs.firemodel import propagation
        env = propagation.Environment([[480060.0, 485060.0], [6210074.0, 6214074.0]], wind_speed=4.11, wind_dir=0)
        prop = propagation.propagate_from_points(env, TimedPoint(482060, 6211074, 0), horizon=14000)

        ignitions = prop.ignitions()
        # ax = ignitions.plot(blocking=False)
        res = up.make_plan_vns(uav, ignitions.as_cpp_raster(), 9500, 12000, 1500, save_every=50)

        ax = prop.plot()
        plan = res.final_plan()
        plot_trajectory(plan.trajectories[0], axes=ax, blocking=False)

        self.plot_search_result(prop, res, blocking=False)

        print("durations: ")
        for traj in plan.trajectories:
            print(traj.duration())

    def plot_search_result(self, propagation, result, blocking=False):
        import matplotlib.pyplot as plt

        ax = propagation.plot()
        plot_plan(result.initial_plan(), ax)
        plt.show(block=False)
        plt.pause(0.0001)
        patches = []
        for intermediate_plan in result.intermediate_plans:
            # propagation.plot(axes=ax)
            for patch in patches:
                patch.set_visible(False)
            patches = plot_plan(intermediate_plan, ax)
            plt.show(block=False)
            plt.pause(0.0001)

        for patch in patches:
            patch.set_visible(False)
        propagation.plot(axes=ax)
        plot_plan(result.final_plan(), ax, blocking=blocking)





