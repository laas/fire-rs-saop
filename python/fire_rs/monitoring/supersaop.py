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

import abc
import itertools
import functools
import logging
import queue
import threading
import typing as ty
import datetime
import uuid

import collections

from collections import namedtuple
from enum import Enum

import numpy as np
# import matplotlib.pyplot as plt

import fire_rs.firemodel.propagation

import fire_rs.geodata.geo_data as geo_data
# import fire_rs.geodata.display as gdisplay
import fire_rs.planning.planning as planning
import fire_rs.firemodel.propagation as propagation
import fire_rs.neptus_interface as nifc

# import fire_rs.planning.display as pdisplay
# import fire_rs.monitoring.ui as ui

supersaop_start_time = datetime.datetime.now()

Alarm = ty.Tuple[datetime.datetime, ty.Sequence[geo_data.TimedPoint]]

Area2D = ty.Tuple[ty.Tuple[float, float], ty.Tuple[float, float]]

DEFAULT_HORIZON = datetime.timedelta(minutes=60)


#
# class Hangar:
#     """Store UAV related information"""
#
#     def __init__(self, available: ty.Optional[ty.Sequence[str]] = None):
#         """Init a UAV Hangar
#
#         :param available: Optional sequence of available UAVs.
#             If None, all uavs are available.
#         """
#
#         base_default = planning.Waypoint(483500.0 - 150., 6215000.0 - 150., 0., 0.)
#         uav_x8 = planning.UAVConf.x8()
#
#         self._uav_keys = ['x8-02', 'x8-06']
#         if available is not None:
#             self._uav_keys = list(filter(lambda x: x in available, self._uav_keys))
#         self._bases = {}
#         self._uavconfs = {}
#         self._colors = {}
#
#         for uav_k, color in zip(self._uav_keys, itertools.cycle(pdisplay.TRAJECTORY_COLORS)):
#             self._bases[uav_k] = base_default
#             self._uavconfs[uav_k] = uav_x8
#             self._colors[uav_k] = color
#
#     def uavconfs(self) -> ty.Dict[str, planning.UAVConf]:
#         """Get planning UAVConf objects for every available UAV."""
#         return self._uavconfs
#
#     def move_uav_to_area(self, uav: str, area: Area2D):
#         """Generate a UAV base near the boundary of 'area' and move 'uav' to it"""
#         self._bases[uav] = planning.Waypoint(area[0][1] - 150., area[1][1] - 150., 0., 0.)
#
#     @property
#     def vehicles(self) -> ty.Sequence[str]:
#         """Get a sequence of all available vehicles"""
#         return self._uav_keys
#
#     @property
#     def bases(self) -> ty.Dict[str, planning.Waypoint]:
#         """Get UAV bases"""
#         return self._bases
#
#     @property
#     def colors(self) -> ty.Dict[str, str]:
#         """Mapping of UAVs and associated color"""
#         return self._colors
#
#
# def draw_situation(gdd: gdisplay.GeoDataDisplay,
#                    alarm: Alarm,
#                    environment: planning.Environment,
#                    firepropagation: propagation.FirePropagation,
#                    hangar: Hangar, ):
#     # Show wildfire Situation Assessment
#     gdd.draw_elevation_shade()
#     gdd.draw_ignition_contour(geodata=firepropagation.prop_data, layer='ignition')
#     for uav, base, color in zip(hangar.bases.keys(), hangar.bases.values(), hangar.colors.values()):
#         gdd.draw_base(base, color=color, label=uav, s=50)
#     gdd.draw_ignition_points(alarm[1], zorder=1000)
#     gdd.figure.legend()
#     gdd.figure.suptitle(
#         "Situation assessment of wildfire alarm.\n Valid from " + str(alarm[0]) + " to " + str(
#             alarm[0] + datetime.timedelta(seconds=firepropagation.until_time)))
#
#
# def poll_alarm() -> Alarm:
#     """Poll an incoming alarm.
#     An alarm is a tuple:
#         0: datetime of discovering a wildfire.
#         1: a sequence with the ignition points of the fire rising the alarm
#         They would be far in the past to consider that the fire is already
#         developed when it is discovered.
#     """
#     ignitions = [geo_data.TimedPoint(
#         478500.0 + 2500.0, 6210000.0 + 2100.0,
#         supersaop_start_time - datetime.timedelta(minutes=30))]
#     return datetime.datetime.now(), ignitions
#
#
# class MonitoredList(collections.MutableSequence):
#
#     def __init__(self, *args):
#         self._list = list(*args)
#         self.dirty = True
#
#     def __getitem__(self, item):
#         return self._list.__getitem__(item)
#
#     def __setitem__(self, key, value):
#         self._list.__setitem__(key, value)
#         self.dirty = True
#
#     def __delitem__(self, key):
#         self._list.__delitem__(key)
#         self.dirty = True
#
#     def __len__(self):
#         return self._list.__len__()
#
#     def insert(self, index, obj):
#         self._list.insert(index, obj)
#         self.dirty = True
#
#
# class RegisteredAlarms(MonitoredList):
#
#     def __init__(self):
#         super().__init__()
#
#     def __getitem__(self, item):
#         return super().__getitem__(item)
#
#     def __setitem__(self, key, value):
#         super().__setitem__(key, value)
#
#     def __delitem__(self, key):
#         super().__delitem__(key)
#
#     def __len__(self):
#         return super().__len__()
#
#     def insert(self, index, obj):
#         super().insert(index, obj)
#
#
# class AlarmResponse:
#     """All the information regarding a wildfire alarm monitoring mission"""
#
#     def __init__(self, alarm: Alarm, p_env: planning.PlanningEnvironment,
#                  fire_prop: propagation.FirePropagation, uav_allocation: ty.Dict[int, str],
#                  f_conf, saop_conf, search_result):
#         self.alarm = alarm
#         self.planning_env = p_env
#         self.fire_prop = fire_prop
#         self.uav_allocation = uav_allocation
#         self.flight_conf = f_conf
#         self.saop_conf = saop_conf
#         self.search_result = search_result
#
#     @property
#     def plan(self):
#         """Get the monitoring plan"""
#         return self.search_result.final_plan()
#

class AreaGenerator:

    def __init__(self, keypoints: ty.Sequence[float], clearance: ty.Sequence[float]):
        self.area = ((.0, .0), (.0, .0))

    # def recompute_area(self):
    #     """Given some sequences of ignition points and bases, compute reasonable
    #     bounds for a PlanningEnvironment around them"""
    #
    #     def extend_area(area: Area2D, point, clearance):
    #         min_px = min(area[0][0], point[0] - clearance[0])
    #         max_px = max(area[0][1], point[0] + clearance[0])
    #         min_py = min(area[1][0], point[1] - clearance[1])
    #         max_py = max(area[1][1], point[1] + clearance[1])
    #
    #         return (min_px, max_px), (min_py, max_py)
    #
    #     if not self._ignition_pts:
    #         # No alarms, then no area
    #         return
    #
    #     # first alarm, ignited seq, first ignited, x or y
    #     area_temp = self.area
    #
    #     ignition_clear = (2500.0, 2500.0)
    #     for ig in self._ignition_pts:
    #         area = extend_area(area_temp, ig, ignition_clear)
    #
    #     base_clear = (150.0, 150.0)
    #     for base_wp in self.hangar.bases.values():
    #         area = extend_area(area_temp, base_wp, ignition_clear)
    #
    #     self._area = area_temp


class SituationAssessment:
    """Evaluate the current state of a wildfire and provide fire perimeter forecasts"""

    def __init__(self, area, logger: logging.Logger):
        super().__init__()
        self.logger = logger

        self._empty_area = ((np.inf, -np.inf), (np.inf, -np.inf))  # type: Area2D
        self._area = area  # type: Area2D

        self._surface_wind = (.0, .0)  # (speed, orientation)

        self._environment = fire_rs.firemodel.propagation.Environment(
            self.area, wind_speed=self._surface_wind[0], wind_dir=self._surface_wind[
                1])  # type: ty.Optional[fire_rs.firemodel.propagation.Environment]

        self._elevation_id = str(uuid.uuid4())  # Id of the elevation map (inside self._environment)

        self._fire_propagation = fire_rs.firemodel.propagation.FirePropagation(
            self._environment)  # type: ty.Optional[fire_rs.firemodel.propagation.FirePropagation]

        self._wildfire = self._environment.raster.clone(
            fill_value=np.finfo(np.float64).max, dtype=[('ignition', 'float64')])

        self._predicted_wildfire = self._environment.raster.clone(
            fill_value=np.finfo(np.float64).max, dtype=[('ignition', 'float64')])
        self._predicted_wildfire_id = str(uuid.uuid4())

        self._cells_on_fire = {}  # type: ty.MutableMapping[ty.Tuple[int, int],ty.Tuple[int, int, float]]

    @property
    def surface_wind(self):
        return self._surface_wind

    def set_surface_wind(self, value: ty.Tuple[float, float]):
        """ Set mean surface wind.

        :param value: as (speed, direction)
        """
        self._environment.update_area_wind(value[0], value[1])
        self.logger.debug("Surface wind has been updated to %s", value)

    @property
    def area(self):
        return self._area

    # @property
    # def environment(self):
    #     return self._environment

    @property
    def wildfire(self):
        """Copy of the expected fire propagation"""
        return self._wildfire.clone()

    @property
    def predicted_wildfire(self):
        """Expected fire propagation"""
        return self._predicted_wildfire

    @property
    def predicted_wildfire_id(self):
        """Expected wildfire map id"""
        return self._predicted_wildfire_id

    @property
    def elevation(self):
        """Elevation map"""
        return self._environment.raster.slice("elevation")

    @property
    def elevation_id(self):
        """Elevation map id"""
        return self._elevation_id

    # def set_wildfire_map(self, to=None):
    #     """Replace or reset the observed wildfire map.
    #
    #     The current wildfire propagator is also reset.
    #     """
    #     if to is None:
    #         self._wildfire = self._environment.raster.clone(fill_value=np.finfo(np.float64).max,
    #                                                         dtype=[('ignition', 'float64')])
    #     else:
    #         raise NotImplementedError()
    #         # self._wildfire = map
    #     self._fire_propagation = fire_rs.firemodel.propagation.FirePropagation(self._environment)

    def set_point_onfire(self, ig_pt: geo_data.TimedPoint):
        """Set some position as on fire.

        The current wildfire propagator is not reset.
        """
        c = self._wildfire.array_index((ig_pt[0], ig_pt[1]))
        self.set_cell_onfire((c[0], c[1], ig_pt[2]))

    def set_cell_onfire(self, ig_cell: ty.Tuple[int, int, float]):
        """Set some cell on fire
        :param ig_cell: (x_cell, y_cell, time)
        """
        c = (ig_cell[0], ig_cell[1])
        self._cells_on_fire[c] = ig_cell
        self._wildfire['ignition'][c] = ig_cell[2]

    def clear_cell_onfire(self, cell: ty.Tuple[int, int]):
        """Clear some cell that was previously set on fire
        :param cell: (x_cell, y_cell)
        """
        if cell in self._cells_on_fire:
            del self._cells_on_fire[cell]
        self._wildfire['ignition'][cell] = np.inf

    def assess_until(self, until: datetime.datetime):
        """Compute an expected wildfire simulation from initial observations."""
        self.logger.info("Start fire propagation from until %s", until)
        self._fire_propagation = fire_rs.firemodel.propagation.FirePropagation(self._environment)
        for cf in self._cells_on_fire.values():
            self._fire_propagation.set_ignition_cell(cf)
        self._fire_propagation.propagate(until.timestamp())
        self._predicted_wildfire = self._fire_propagation.ignitions()
        self._predicted_wildfire_id = str(uuid.uuid4())
        self.logger.info("Propagation ended")


class ObservationPlanning:
    """Create and manage plans to observe wildfires"""

    TrajectoryConf = namedtuple("TrajectoryConf", ["name", "uav_model", "start_wp", "end_wp",
                                                   "start_time", "max_duration", "wind"])
    DEFAULT_UAV_MODELS = {'x8-06': fire_rs.planning.planning.UAVConf.x8("06"),
                          'x8-02': fire_rs.planning.planning.UAVConf.x8("02")}

    DEFAULT_VNS_CONFS = fire_rs.planning.planning.VNSConfDB.demo_db()

    def __init__(self, logger: logging.Logger,
                 uav_models: ty.Mapping[str, fire_rs.planning.planning.UAVConf],
                 vns_confs):
        self.logger = logger
        self.uav_models = uav_models

        # TODO: Support more VNS confs
        self._vns_conf_db = vns_confs

        self._tagged_elevation = None  # type: ty.Tuple[str, fire_rs.geodata.geo_data.GeoData]
        self._tagged_wildfire_map = None  # type: ty.Tuple[str, fire_rs.geodata.geo_data.GeoData]
        self._current_planner = None  # type: fire_rs.planning.planning.SimplePlanner
        self.current_plan = None

    def set_elevation(self, id_hash: str, elevation: fire_rs.geodata.geo_data.GeoData):
        """Replace the current elevation map by a new one"""
        assert "elevation" in elevation.layers
        self._tagged_elevation = (id_hash, elevation)

    def set_wildfire_map(self, id_hash: str, wildfire_map: fire_rs.geodata.geo_data.GeoData):
        """Replace the current wildfire map by a new one"""
        assert "ignition" in wildfire_map.layers
        self._tagged_wildfire_map = (id_hash, wildfire_map)

    def initial_plan(self, name: str, wildfire_id: str, vns_conf_name: str,
                     traj_conf: ty.Sequence[TrajectoryConf], flight_window: ty.Tuple[float, float]):
        """Set up the planner for an initial plan"""
        elevation_map = self._tagged_elevation[1]
        wildfire_map = None
        if self._tagged_wildfire_map[0] == wildfire_id:
            wildfire_map = self._tagged_wildfire_map[1]
        elif wildfire_id == "":
            # When no id is requested, use the most recent wildfire map
            wildfire_map = self._tagged_wildfire_map[1]

        if wildfire_map is None:
            raise ValueError(
                "Wildfire id {} is not known by Observation Planning".format(wildfire_id))
        vns_conf = self._vns_conf_db[vns_conf_name]

        vns_planner_flight_confs = [fire_rs.planning.planning.FlightConf(
            self.uav_models[c.uav_model], c.start_time,
            fire_rs.planning.planning.Waypoint(*c.start_wp),
            fire_rs.planning.planning.Waypoint(*c.end_wp), c.wind,
            c.max_duration) for c in traj_conf]
        self._current_planner = planning.SimplePlanner(name, self._tagged_elevation[1],
                                                       self._tagged_wildfire_map[1],
                                                       vns_planner_flight_confs, vns_conf,
                                                       flight_window)

    def compute_plan(self, planning_duration):
        """Execute the current planner and return the final plan"""
        self.current_plan = self._current_planner.compute_plan(planning_duration).final_plan()
        return self.current_plan


# class SuperSAOP:
#     """Supervise a wildifre monitoring mission"""
#
#     class State(Enum):
#         """SuperSAOP operations"""
#         SA = 0,  # Situation Assessment
#         OP = 1,  # Observation Planning
#         MO = 2  # Stop the mission
#
#     def __init__(self, hangar: Hangar, logger: logging.Logger, saop_ui=ui.NoUI()):
#         """"""
#         self.logger = logger
#         self.ui = saop_ui
#         self.hangar = hangar
#
#         self.monitoring = True
#
#     def main(self, alarm: Alarm):
#         execution_monitor = ExecutionMonitor(self.logger.getChild("ExecutionMonitor"))
#         while self.monitoring:
#             situation_assessment = SituationAssessment(self.hangar,
#                                                        self.logger.getChild("SituationAssessment"))
#
#             observation_planning = ObservationPlanning(self.hangar,
#                                                        self.logger.getChild("ObservationPlanning"))
#
#             environment, firepropagation = situation_assessment.expected_situation(alarm)
#
#             # Show wildfire Situation Assessment
#             alarm_gdd = gdisplay.GeoDataDisplay(*gdisplay.get_pyplot_figure_and_axis(),
#                                                 environment.raster, frame=(0., 0.))
#             alarm_gdd.add_extension(pdisplay.TrajectoryDisplayExtension, (None,))
#             draw_situation(alarm_gdd, alarm, environment, firepropagation, self.hangar)
#             alarm_gdd.figure.show()
#             plt.pause(0.001)  # Needed to let matplotlib a chance of showing figures
#
#             # Observation Planning
#             response = observation_planning.respond_to_alarm(
#                 alarm, expected_situation=(environment, firepropagation))
#
#             draw_response(alarm_gdd, response)
#             plt.pause(0.001)  # Needed to let matplotlib a chance of showing figures
#
#             while not execution_monitor.gcs.is_ready():
#                 plt.pause(0.1)  # Use the matplotlib pause. So at least figures remain responsive
#
#             command_outcome = execution_monitor.start_response(response.plan,
#                                                                response.uav_allocation)
#
#             if functools.reduce(lambda a, b: a and b, command_outcome) or True:
#                 # state_monitor = execution_monitor.monitor_uav_state(self.hangar.vehicles)
#                 traj_vec, state_vec, res = self.do_monitoring(response, execution_monitor)
#                 if res == SuperSAOP.MonitoringAction.UNDECIDED:
#                     self.monitoring = True
#                 elif res == SuperSAOP.MonitoringAction.REPLAN:
#                     self.logger.info("Replan triggered")
#                     self.logger.info("TODO: Change bases of UAVs to their actual position")
#                     # TODO: Change base of UAV
#                 elif res == SuperSAOP.MonitoringAction.EXIT:
#                     self.logger.info("SuperSAOP exit")
#                     self.monitoring = False
#             else:
#                 if not self.ui.question_dialog("The plan couldn't be started. Restart?"):
#                     self.monitoring = False
#
#         self.logger.info("End of monitoring mission for alarm %s", str(alarm))
#
#     def do_monitoring(self, response, execution_monitor) -> ty.Tuple[
#         ty.Tuple[str, nifc.TrajectoryExecutionReport],
#         ty.Tuple[ty.Tuple[str, int], nifc.UAVStateReport], MonitoringAction]:
#         # FIXME: monitor all the trajectories not just one
#         plan_name, traj = response.plan.name(), 0
#         trajectory_monitor = execution_monitor.monitor_trajectory(plan_name, traj)
#         uav_state_monitor = execution_monitor.monitor_uav_state(response.uav_allocation.values())
#
#         while True:
#             traj_state_vector = next(trajectory_monitor, None)
#             uav_state_vector = next(uav_state_monitor, None)
#             if traj_state_vector is not None:
#                 state, decision = self.plan_state_and_action(traj_state_vector)
#                 if decision == SuperSAOP.MonitoringAction.EXIT:
#                     return traj_state_vector, uav_state_vector, decision
#                 elif decision == SuperSAOP.MonitoringAction.REPLAN:
#                     return traj_state_vector, uav_state_vector, decision
#                 elif decision == SuperSAOP.MonitoringAction.UNDECIDED:
#                     if state == SuperSAOP.MissionState.FAILED:
#                         self.logger.warning(
#                             "Monitoring mission failed. User interaction needed")
#                         if self.ui.question_dialog("Monitoring mission failed. Recover?"):
#                             return traj_state_vector, uav_state_vector, SuperSAOP.MonitoringAction.REPLAN
#                         else:
#                             return traj_state_vector, uav_state_vector, SuperSAOP.MonitoringAction.EXIT
#                     elif state == SuperSAOP.MissionState.EXECUTING:
#                         pass  # SuperSAOP.MonitoringAction.UNDECIDED
#                     elif state == SuperSAOP.MissionState.ENDED:
#                         # TODO: If execution ended, REPLAN or EXIT?
#                         return traj_state_vector, uav_state_vector, SuperSAOP.MonitoringAction.EXIT
#                 current_mans, decision = self.meneuver_state_and_action(traj_state_vector,
#                                                                         response.plan)
#                 if decision != SuperSAOP.MonitoringAction.UNDECIDED:
#                     return traj_state_vector, uav_state_vector, decision
#
#     def meneuver_state_and_action(self, state_dict: ty.Dict[
#         ty.Tuple[str, int], nifc.TrajectoryExecutionReport], saop_plan) \
#             -> ty.Tuple[ty.Dict[ty.Tuple[str, int], int], MonitoringAction]:
#         """Determine the execution state of trajectory maneuvers regarding the original plan
#
#         If the execution is lagging with respect to the plan, the mission should be replanned.
#
#         Let `m` and `n` two consecutive maneuvers in a trajectory `T`.
#         Then `t_m` is the start time of maneuver `m` and `t_n` the start time of maneuver `n`.
#         A UAV is reporting the execution of maneuver `n` at time `t_x`
#
#         The plan is being followed if `t_m` < `t_x` <= `t_n` with a 3-minute tolerance
#
#         :returns A mapping of the current maneuvers being executed,
#             and a MonitoringAction.
#         """
#         slack = datetime.timedelta(minutes=3)
#         current_maneuvers = {}
#         actions = {}
#
#         # TODO: Tolerance to be defined
#         for plan_id, ter in state_dict.items():
#
#             if ter.timestamp < datetime.datetime.now().timestamp() - datetime.timedelta(
#                     seconds=30).seconds:
#                 # Ignore old TER messages
#                 continue
#
#             action = SuperSAOP.MonitoringAction.UNDECIDED
#             T = saop_plan.trajectories()[plan_id[1]]
#             n = int(ter.maneuver)
#             m = int(ter.maneuver) - 1
#             t_x = ter.timestamp
#             t_n = T.start_time(n) if n < len(T) else None
#             t_m = T.start_time(m) if m >= 0 else None
#
#             if t_n is not None and t_x > t_n + slack.seconds:
#                 # Going slower than expected
#                 action = SuperSAOP.MonitoringAction.REPLAN
#                 self.logger.info(
#                     "Trajectory %s is running slower than expected man(%s) t(%s) > t_%s(%s)",
#                     str(plan_id), str(n), datetime.datetime.fromtimestamp(t_x), str(n),
#                     datetime.datetime.fromtimestamp(t_n) + slack)
#             if t_m is not None and t_x < t_m - slack.seconds:
#                 # Going faster than expected
#                 action = SuperSAOP.MonitoringAction.REPLAN
#                 self.logger.info(
#                     "Trajectory %s is running faster than expected man(%s) t_x(%s) < t_%s(%s)",
#                     str(plan_id), str(n), datetime.datetime.fromtimestamp(t_x), str(m),
#                     datetime.datetime.fromtimestamp(t_m) - slack)
#
#             current_maneuvers[plan_id] = n
#             actions[plan_id] = action
#
#         if next(filter(lambda a: a == SuperSAOP.MonitoringAction.REPLAN, actions.values()),
#                 None) is None:
#             return current_maneuvers, SuperSAOP.MonitoringAction.UNDECIDED
#         else:
#             return current_maneuvers, SuperSAOP.MonitoringAction.REPLAN
#
#     def plan_state_and_action(self, state_dict: ty.Dict[
#         ty.Tuple[str, int], nifc.TrajectoryExecutionReport]) \
#             -> ty.Tuple[MissionState, MonitoringAction]:
#         """Determine the state of the mission and decide wether a mission should continue.
#
#         A mission is considered
#             - FAILED if any trajectory state is Blocked or its outcome is Failure
#             - ENDED: if all trajectory outcomes are Success
#             - EXECUTING: if all trajectory states are Executing
#
#         The next action should be
#             - UNDECIDED: No decision has been made
#             - REPLAN: trigger a replan
#             - EXIT: Stop the mission
#         """
#
#         mission_s_v = {}
#         for plan_id, ter in state_dict.items():
#             if ter.state == nifc.TrajectoryExecutionState.Executing:
#                 mission_s_v[plan_id] = SuperSAOP.MissionState.EXECUTING
#             elif ter.state == nifc.TrajectoryExecutionState.Ready:
#                 if ter.last_outcome == nifc.TrajectoryExecutionOutcome.Failure or \
#                         ter.last_outcome == nifc.TrajectoryExecutionOutcome.Nothing:
#                     mission_s_v[plan_id] = SuperSAOP.MissionState.FAILED
#                 if ter.last_outcome == nifc.TrajectoryExecutionOutcome.Success:
#                     mission_s_v[plan_id] = SuperSAOP.MissionState.ENDED
#             else:  # nifc.TrajectoryExecutionState.Blocked
#                 self.logger.critical("UAV %s is blocked. This is a failure")
#                 return SuperSAOP.MissionState.FAILED, SuperSAOP.MonitoringAction.EXIT
#
#         for m_s in mission_s_v.values():
#             if m_s == SuperSAOP.MissionState.FAILED:
#                 # If one trajectory failed, the plan is a failure. Leave action undecided
#                 return SuperSAOP.MissionState.FAILED, SuperSAOP.MonitoringAction.UNDECIDED
#             elif m_s == SuperSAOP.MissionState.EXECUTING:
#                 # If one trajectory is still running then the plan is not over yet
#                 return SuperSAOP.MissionState.EXECUTING, SuperSAOP.MonitoringAction.UNDECIDED
#         else:
#             # All the trajectories are over
#             return SuperSAOP.MissionState.ENDED, SuperSAOP.MonitoringAction.UNDECIDED
#
#
class NeptusBridge:
    """Communicate with the UAV ground control software"""

    def __init__(self, logger: logging.Logger):

        self.logger = logger

        self.imccomm = nifc.IMCComm()
        self.gcs = None

        self.t_imc = threading.Thread(target=self.imccomm.run, daemon=False)
        self.t_gcs = threading.Thread(target=self._create_gcs, daemon=False)
        self.t_imc.run()
        self.t_gcs.run()

        self.monitoring = True  # Determine wether we are monitoring

        self.uav_state = {}
        self.traj_state = {}

        self.uav_state_cb = None
        self.traj_state_cb = None

    def _create_gcs(self):
        """Create GCS object of this class.
        To be runned in a different thread."""
        self.gcs = nifc.GCS(self.imccomm, self.on_trajectory_execution_report,
                            self._on_uav_state_report)

    @staticmethod
    def saop_plan_and_traj(neptus_plan_id: str) -> ty.Tuple[str, int]:
        splited = neptus_plan_id.split(sep=".")
        return ".".join(splited[:-1]), int(splited[-1])

    @staticmethod
    def neptus_plan_id(plan_name: str, traj_id: int) -> str:
        return ".".join((plan_name, str(traj_id)))

    def send_home(self, uav):
        """ Tell a UAV to go home"""
        raise NotImplementedError

    def start_trajectory(self, plan, traj_i: int, uav: str) -> bool:
        """Execute a SAOP Plan 'a_plan' trajectory 'trajectory' with using the vehicle 'uav'"""
        # self.stop_uav(uav)

        plan_name = NeptusBridge.neptus_plan_id(plan.name(), traj_i)

        # Start the mission
        # FIXME: Mission start seems to fail always
        command_r = self.gcs.start(plan, traj_i, plan_name, uav)
        if command_r:
            self.logger.info("Mission %s for %s started", plan_name, uav)
            return True
        else:
            self.logger.error("Start of mission %s failed for %s failed", plan_name, uav)
            return False

    def stop_uav(self, uav):
        # Stop previous trajectory (if any)
        command_r = self.gcs.stop("", uav)
        if command_r:
            self.logger.info("Mission of %s stopped", str(uav))
            return True
        else:
            self.logger.warning("Stop %s failed", str(uav))
            return False

    def set_trajectory_state_callback(self, fn):
        """Function to be called each time a new Estimated State is received
        :param fn: a Callable(**kwargs)
        """
        self.traj_state_cb = fn

    def on_trajectory_execution_report(self, ter: nifc.TrajectoryExecutionReport):
        """Method called by the GCS to report about the state of the missions"""
        if self.traj_state_cb:
            self.traj_state_cb(time=ter.timestamp, plan_id=ter.plan_id, vehicle=ter.vehicle,
                               maneuver=ter.maneuver, manuever_eta=ter.manuever_eta,
                               state=ter.state, last_outcome=ter.last_outcome)

    def set_uav_state_callback(self, fn):
        """Function to be called each time a new Estimated State is received
        :param fn: a Callable(**kwargs) (time: float, uav: str,
                                         x: float, y: float, z: float,
                                         phi: float, theta: float, psi: float,
                                         vx: float, vy: float, vz: float)
        x, y and z in ENU frame"""
        self.uav_state_cb = fn

    def _on_uav_state_report(self, usr: nifc.UAVStateReport):
        """Method called by the GCS to report about the state of the UAVs"""
        self.uav_state[usr.uav] = usr
        # TODO: Transform position from LatLonHeight to XYZ
        if self.uav_state_cb:
            self.uav_state_cb(time=usr.timestamp, uav=usr.uav,
                              x=usr.lat, y=usr.lon, z=usr.height,
                              phi=usr.phi, theta=usr.theta, psi=usr.psi,
                              vx=usr.vx, vy=usr.vy, vz=usr.vz)

        # if self.monitoring:
        #     try:
        #         if usr.uav not in self.uav_state:
        #             self.uav_state[usr.uav] = queue.Queue()
        #         self.uav_state[usr.uav].put(usr, block=True, timeout=1)
        #     except queue.Full:
        #         self.logger.exception("UAVStateReport queue timeout reached")
#
#
# def draw_response(gdd: gdisplay.GeoDataDisplay,
#                   response: AlarmResponse):
#     gdd.draw_ignition_contour(response.fire_prop.prop_data, with_labels=True)
#     pdisplay.plot_plan_trajectories(response.plan, gdd, layers=['trajectory_solid', 'arrows'])
#     gdd.figure.legend()
#     gdd.figure.suptitle(
#         "Observation Plan of wildfire alarm.\n Valid from " + str(response.alarm[0]) + " to " + str(
#             response.alarm[0] + datetime.timedelta(seconds=response.fire_prop.until_time)))
