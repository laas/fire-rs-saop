# Copyright (c) 2018, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import typing as ty
import datetime
import numpy as np
import fire_rs.geodata.geo_data as geo_data
import fire_rs.planning.planning as planning
import fire_rs.firemodel.propagation as propagation

supersaop_start_time = datetime.datetime.now()

Alarm = ty.Tuple[datetime.datetime, ty.Sequence[geo_data.TimedPoint]]

Area2D = ty.Tuple[ty.Tuple[float, float], ty.Tuple[float, float]]


def uav_confs_planning() -> ty.Dict[str, planning.UAVConf]:
    """Get planning UAVConf objects for every available UAV."""
    uav_x8 = planning.UAVConf.X8()
    return {'x8-02': uav_x8, 'x8-06': uav_x8}


def create_base(area: Area2D):
    """Generate a UAV base near the boundary of 'area'"""
    return [planning.Waypoint(area[0][1] - 150., area[1][1] - 150., 0., 0.)]


def bases():
    """Get UAV bases"""
    return [planning.Waypoint(483500.0 - 150., 6215000.0 - 150., 0., 0.)]


def poll_alarm() -> Alarm:
    """Poll an incoming alarm.
    An alarm is a tuple:
        0: datetime of discovering a wildfire.
        1: a sequence with the ignition points of the fire rising the alarm
        They would be far in the past to consider that the fire is already
        developed when it is discovered.
    """
    ignitions = [geo_data.TimedPoint(
        478500.0 + 2500.0, 6210000.0 + 2100.0,
        supersaop_start_time - datetime.timedelta(minutes=30))]
    return datetime.datetime.now(), ignitions


def get_area_wind(position_time: ty.Tuple[tuple, tuple, datetime]) -> ty.Tuple[float, float]:
    return 10., np.pi / 4


def compute_area(igntions: ty.Sequence[geo_data.TimedPoint],
                 bases: ty.Optional[ty.Sequence[planning.Waypoint]] = None) -> Area2D:
    """Given some sequences of ignition points and bases, compute reasonable
    bounds for a PlanningEnvironment around them"""

    def extend_area(area: Area2D, point, clearance):
        max_px, min_px, max_py, min_py = area[0][1], area[0][0], area[0][1], area[0][0]

        min_px = min(area[0][0], point[0] - clearance)
        max_px = max(area[0][1], point[0] + clearance)
        min_py = min(area[1][0], point[1] - clearance)
        max_py = max(area[1][1], point[1] + clearance)

        return (min_px, max_px), (min_py, max_py)

    ignition_clear = (2500.0, 2500.0)

    area = ((igntions[0], igntions[0]), (igntions[1], igntions[1]))
    for ig in igntions:
        area = extend_area(area, ig, ignition_clear)
    if bases is not None:
        base_clear = (150.0, 150.0)
        for b in bases:
            area = extend_area(area, b, ignition_clear)

    return area


def get_environment(alarm: Alarm) -> planning.PlanningEnvironment:
    """Get a PlanningEnvironment around an alarm"""
    wind = get_area_wind(alarm[1][0])
    area = compute_area(alarm, bases)

    return planning.PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                                        planning_elevation_mode='flat', flat_altitude=0)


def compute_fire_propagation(env: planning.PlanningEnvironment,
                             alarm: Alarm,
                             until: datetime.datetime) -> propagation.FirePropagation:
    return propagation.propagate_from_points(env, alarm[1], until.timestamp())


if __name__ == "__main__":
    alarma = poll_alarm()
    p_env = get_environment(alarma)
    fire_prop = compute_fire_propagation(p_env, alarma, alarma[0] + datetime.timedelta(minutes=60))

    takeoff_delay = datetime.timedelta(minutes=10)
    f_conf = planning.FlightConf(uav_confs_planning()["x8-06"],
                                 (alarma[0] + takeoff_delay).timestamp(), bases()[0])
    fl_window = (f_conf.start_time, f_conf.start_time + f_conf.max_flight_time)
    saop_conf = planning.SAOPPlannerConf(fl_window, planning.VNSConfDB.demo_db(), 10.0, False, 0)
    planner = planning.Planner(p_env, fire_prop.prop_data, f_conf, saop_conf)
