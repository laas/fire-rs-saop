import random
from collections import namedtuple

import fire_rs.uav_planning as up
import numpy as np
import time

from fire_rs.geodata.geo_data import TimedPoint, Area


class UAV:

    def __init__(self, max_air_speed: float, max_angular_velocity: float, base_waypoint: (float, float, float)):
        self.max_air_speed = max_air_speed
        self.max_angular_velocity = max_angular_velocity
        self.base_waypoint = base_waypoint

    def as_cpp(self):
        return up.UAV(self.max_air_speed, self.max_angular_velocity)


class Flight:

    def __init__(self, uav: UAV, start_time: float, max_flight_time: float):
        assert max_flight_time > 0
        assert start_time > 0
        self.uav = uav
        self.start_time = start_time
        self.max_flight_time = max_flight_time

    def as_trajectory_config(self):
        x = self.uav.base_waypoint
        wp = up.Waypoint(x[0], x[1], x[2])
        return up.TrajectoryConfig(self.uav.as_cpp(), wp, wp, self.start_time, self.max_flight_time)


class Scenario:

    def __init__(self, area: ((float, float), (float, float)),
                 wind_speed: float,
                 wind_direction: float,
                 ignitions: [TimedPoint],
                 flights: [Flight]):
        self.area = area
        self.wind_speed = wind_speed
        self.wind_direction = wind_direction
        assert len(ignitions) > 0
        self.ignitions = ignitions
        assert len(flights) > 0
        self.flights = flights

        self.time_window_start = np.inf
        self.time_window_end = -np.inf
        for flight in flights:
            self.time_window_start = min(self.time_window_start, flight.start_time - 180)
            self.time_window_end = max(self.time_window_end, flight.start_time+flight.max_flight_time + 180)


def plot_trajectory(traj, axes=None, blocking=False, display=True, color='r'):
    import matplotlib.pyplot as plt
    if not axes:
        axes = plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")
    sampled_waypoints = traj.as_waypoints(step_size=2)
    x = [wp.x for wp in sampled_waypoints]
    y = [wp.y for wp in sampled_waypoints]
    path_patch = axes.scatter(x, y, s=0.1, c=color, edgecolor=color)

    waypoints = traj.as_waypoints()
    x = [wp.x for wp in waypoints]
    y = [wp.y for wp in waypoints]
    waypoints_patch = axes.scatter(x, y, c=color)
    if display:
        plt.show(block=blocking)
    return axes, [waypoints_patch, path_patch]


def plot_plan(plan, axes, blocking=False, display=True):
    colors = ['r', 'b', 'g', 'c', 'w']
    import matplotlib.pyplot as plt
    patches_of_plan = []
    print(len(plan.trajectories))
    for i in range(len(plan.trajectories)):
        traj = plan.trajectories[i]
        _, traj_patches = plot_trajectory(traj, axes=axes, blocking=False, display=display, color=colors[i])
        for tp in traj_patches:
            patches_of_plan.append(tp)
    if display:
        plt.show(block=blocking)
    return patches_of_plan


def run_benchmark(scenario, save_directory, instance_name, plot=False):
    import os
    from fire_rs.firemodel import propagation
    env = propagation.Environment(scenario.area, wind_speed=scenario.wind_speed, wind_dir=scenario.wind_direction)
    prop = propagation.propagate_from_points(env, scenario.ignitions, horizon=scenario.time_window_end)

    ignitions = prop.ignitions()
    flight = scenario.flights[0]
    flights = [f.as_trajectory_config() for f in scenario.flights]
    # ax = ignitions.plot(blocking=False)
    res = up.plan_vns(flights, ignitions.as_cpp_raster(), scenario.time_window_start, scenario.time_window_end, save_every=0)
    plan = res.final_plan()

    axis = prop.plot(display=plot)
    plot_plan(res.final_plan(), axes=axis, blocking=True, display=plot)
    print("saving as: " + str(os.path.join(save_directory, instance_name + ".png")))
    axis.get_figure().set_size_inches(20, 15)
    axis.get_figure().savefig(os.path.join(save_directory, instance_name + ".png"), bbox_inches='tight')

    summary_file = os.path.join(save_directory, "summary.csv")
    if not os.path.exists(summary_file):
         with open(summary_file, "a") as summary:
             summary.write("instance,planning-time,preprocessing-time,cost,duration\n")

    with open(summary_file, "a") as summary:
        summary.write("{},{},{},{},{}\n".format(instance_name, res.planning_time, res.preprocessing_time, plan.cost(), plan.duration()))


Waypoint = namedtuple('Waypoint', 'x, y, dir')


def generate_scenario():
    # 9 by 7 km area
    area = Area(480060.0, 489060.0, 6210074.0, 6217074.0)
    uav_speed = 18.  # m/s
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin +100, area.ymin+100, 0),
        Waypoint(area.xmin+100, area.ymax-100, 0),
        Waypoint(area.xmax-100, area.ymin+100, 0),
        Waypoint(area.xmax-100, area.ymax-100, 0)
    ]

    wind_speed = 15.  #random.uniform(10., 20.)  # wind speed in [10,20] km/h
    wind_dir = 0.  #random.random() * 2 * np.pi
    num_ignitions = random.randint(1, 3)
    ignitions = [TimedPoint(random.uniform(area.xmin, area.xmax),
                            random.uniform(area.ymin, area.ymax),
                            random.uniform(0, 3000))
                 for i in range(num_ignitions)]

    # start once all fires are ignited
    start = max([igni.time for igni in ignitions])

    num_flights = random.randint(1, 3)
    flights = []
    for i in range(num_flights):
        uav = UAV(uav_speed, uav_max_turn_rate, random.choice(uav_bases))
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1500)
        flights.append(Flight(uav, uav_start, max_flight_time))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario


def main():
    import pickle
    import os
    from fire_rs.geodata.environment import DEFAULT_FIRERS_DATA_FOLDER

    run_id = time.strftime("%Y-%m-%d--%H:%M:%S")

    benchmark_dir = os.path.join(DEFAULT_FIRERS_DATA_FOLDER, "benchmark")
    if not os.path.exists(benchmark_dir):
        os.makedirs(benchmark_dir)

    scenarios_file = os.path.join(benchmark_dir, "scenarios.dump")
    if not os.path.exists(scenarios_file):
        scenarios = [generate_scenario() for i in range(40)]
        pickle.dump(scenarios, open(scenarios_file, "wb"))
    else:
        scenarios = pickle.load(open(scenarios_file, "rb"))

    run_dir = os.path.join(benchmark_dir, run_id)
    if not os.path.exists(run_dir):
        os.makedirs(run_dir)

    i=0
    for scenario in scenarios:
        print(scenario)
        run_benchmark(scenario, run_dir, str(i))
        i += 1


if __name__=='__main__':
    main()
