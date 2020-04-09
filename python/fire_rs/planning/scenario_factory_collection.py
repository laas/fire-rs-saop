"""Demonstration of a custom collection of scenario factories for benchmark.py"""
import random

import numpy as np

from fire_rs.planning.benchmark import FlightConf, Scenario
from fire_rs.geodata.geo_data import TimedPoint, Area
from fire_rs.planning.new_planning import UAV, Waypoint

def scenario_a():
    # 9 by 7 km area
    area = Area(532000.0, 540000.0, 4567000.0, 4575000.0)
    uav_speed = 18.  # m/s
    uav_max_pitch_angle = 6. / 180. * np.pi
    uav_max_turn_rate = 32. * np.pi / 180
    uav_bases = [  # four corners of the map
        Waypoint(area.xmin + 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmin + 100, area.ymax - 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymin + 100, 0, 0),
        Waypoint(area.xmax - 100, area.ymax - 100, 0, 0)
    ]

    wind_speed = 15.  # random.uniform(10., 20.)  # wind speed in [10,20] km/h
    wind_dir = 0.  # random.random() * 2 * np.pi
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
        uav_start = random.uniform(start, start + 4000.)
        max_flight_time = random.uniform(500, 1200)
        name = " ".join(("UAV", str(i)))
        uav = UAV("x8-06", uav_speed, uav_max_turn_rate, uav_max_pitch_angle)
        flights.append(FlightConf(name, uav, uav_start, max_flight_time, random.choice(uav_bases)))

    scenario = Scenario(((area.xmin, area.xmax), (area.ymin, area.ymax)),
                        wind_speed, wind_dir, ignitions, flights)
    return scenario
