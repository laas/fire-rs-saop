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

import itertools
import functools
import logging
import queue
import threading
import typing as ty
import datetime

from enum import Enum

import numpy as np
import matplotlib.pyplot as plt

import fire_rs.geodata.geo_data as geo_data
import fire_rs.geodata.display as gdisplay
import fire_rs.planning.planning as planning
import fire_rs.firemodel.propagation as propagation
import fire_rs.neptus_interface as nifc
import fire_rs.planning.display as pdisplay
import fire_rs.monitoring.ui as ui

supersaop_start_time = datetime.datetime.now()

Alarm = ty.Tuple[datetime.datetime, ty.Sequence[geo_data.TimedPoint]]

Area2D = ty.Tuple[ty.Tuple[float, float], ty.Tuple[float, float]]

DEFAULT_HORIZON = datetime.timedelta(minutes=60)


class Hangar:
    """Store UAV related information"""

    def __init__(self, available: ty.Optional[ty.Sequence[str]] = None):
        """Init a UAV Hangar

        :param available: Optional sequence of available UAVs.
            If None, all uavs are available.
        """

        base_default = planning.Waypoint(483500.0 - 150., 6215000.0 - 150., 0., 0.)
        uav_x8 = planning.UAVConf.X8()

        self._uav_keys = ['x8-02', 'x8-06']
        if available is not None:
            self._uav_keys = list(filter(lambda x: x in available, self._uav_keys))
        self._bases = {}
        self._uavconfs = {}
        self._colors = {}

        for uav_k, color in zip(self._uav_keys, itertools.cycle(pdisplay.TRAJECTORY_COLORS)):
            self._bases[uav_k] = base_default
            self._uavconfs[uav_k] = uav_x8
            self._colors[uav_k] = color

    def uavconfs(self) -> ty.Dict[str, planning.UAVConf]:
        """Get planning UAVConf objects for every available UAV."""
        return self._uavconfs

    def move_uav_to_area(self, uav: str, area: Area2D):
        """Generate a UAV base near the boundary of 'area' and move 'uav' to it"""
        self._bases[uav] = planning.Waypoint(area[0][1] - 150., area[1][1] - 150., 0., 0.)

    @property
    def vehicles(self) -> ty.Sequence[str]:
        """Get a sequence of all available vehicles"""
        return self._uav_keys

    @property
    def bases(self) -> ty.Dict[str, planning.Waypoint]:
        """Get UAV bases"""
        return self._bases

    @property
    def colors(self) -> ty.Dict[str, str]:
        """Mapping of UAVs and associated color"""
        return self._colors


def draw_situation(gdd: gdisplay.GeoDataDisplay,
                   alarm: Alarm,
                   environment: planning.Environment,
                   firepropagation: propagation.FirePropagation,
                   hangar: Hangar, ):
    # Show wildfire Situation Assessment
    gdd.draw_elevation_shade()
    gdd.draw_ignition_contour(geodata=firepropagation.prop_data, layer='ignition')
    for uav, base, color in zip(hangar.bases.keys(), hangar.bases.values(), hangar.colors.values()):
        gdd.draw_base(base, color=color, label=uav, s=50)
    gdd.draw_ignition_points(alarm[1], zorder=1000)
    gdd.figure.legend()
    gdd.figure.suptitle(
        "Situation assessment of wildfire alarm.\n Valid from " + str(alarm[0]) + " to " + str(
            alarm[0] + datetime.timedelta(seconds=firepropagation.until_time)))


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


class AlarmResponse:
    """All the information regarding a wildfire alarm monitoring mission"""

    def __init__(self, alarm: Alarm, p_env: planning.PlanningEnvironment,
                 fire_prop: propagation.FirePropagation, uav_allocation: ty.Dict[int, str],
                 f_conf, saop_conf, search_result):
        self.alarm = alarm
        self.planning_env = p_env
        self.fire_prop = fire_prop
        self.uav_allocation = uav_allocation
        self.flight_conf = f_conf
        self.saop_conf = saop_conf
        self.search_result = search_result

    @property
    def plan(self):
        """Get the monitoring plan"""
        return self.search_result.final_plan()


class SituationAssessment:
    """Evaluate the current state of a wildifre and provide fire perimeter forecasts"""

    def __init__(self, hangar: Hangar, logger: logging.Logger):
        self.logger = logger
        self.hangar = hangar

    @staticmethod
    def _get_area_wind(position_time: ty.Tuple[float, float, datetime.datetime]) \
            -> ty.Tuple[float, float]:
        return 10., np.pi / 4

    @staticmethod
    def _compute_area(igntions: ty.Sequence[geo_data.TimedPoint],
                      bases: ty.Optional[ty.Sequence[planning.Waypoint]] = None) -> Area2D:
        """Given some sequences of ignition points and bases, compute reasonable
        bounds for a PlanningEnvironment around them"""

        def extend_area(area: Area2D, point, clearance):
            min_px = min(area[0][0], point[0] - clearance[0])
            max_px = max(area[0][1], point[0] + clearance[0])
            min_py = min(area[1][0], point[1] - clearance[1])
            max_py = max(area[1][1], point[1] + clearance[1])

            return (min_px, max_px), (min_py, max_py)

        ignition_clear = (2500.0, 2500.0)

        area = ((igntions[0][0], igntions[0][0]), (igntions[0][1], igntions[0][1]))
        for ig in igntions:
            area = extend_area(area, ig, ignition_clear)
        if bases is not None:
            base_clear = (150.0, 150.0)
            for b in bases:
                area = extend_area(area, b, ignition_clear)

        return area

    def expected_situation(self, alarm: Alarm,
                           horizon: datetime.timedelta = DEFAULT_HORIZON) -> \
            ty.Tuple[planning.PlanningEnvironment, propagation.FirePropagation]:
        """Precompute an expected wildfire simulation in the future since a fire alarm.
        The result can be passed to the alarm response planning method"""

        planning_env = SituationAssessment._get_environment(alarm, self.hangar.bases.values())
        return planning_env, self._compute_fire_propagation(planning_env, alarm, alarm[0] + horizon)

    @staticmethod
    def _get_environment(alarm: Alarm, bases) -> planning.PlanningEnvironment:
        """Get a PlanningEnvironment around an alarm"""
        wind = SituationAssessment._get_area_wind(alarm[1][0])
        area = SituationAssessment._compute_area(alarm[1], bases)

        return planning.PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                                            planning_elevation_mode='flat', flat_altitude=0)

    def _compute_fire_propagation(self, env: planning.PlanningEnvironment,
                                  alarm: Alarm,
                                  until: datetime.datetime) -> propagation.FirePropagation:
        self.logger.info("Start fire propagation from %s until %s", alarm[1], until)
        pt_float = [(x, y, time.timestamp()) for x, y, time in alarm[1]]
        prop = propagation.propagate_from_points(env, pt_float, until.timestamp())
        self.logger.info("Propagation ended")
        return prop


class ObservationPlanning:
    """Create monitoring plans for a wildfire alarm"""

    def __init__(self, hangar: Hangar, logger: logging.Logger):
        self.hangar = hangar
        self._registered_alarms = {}
        self.logger = logger

    def respond_to_alarm(self, alarm: Alarm,
                         expected_situation: ty.Optional[ty.Tuple[
                             planning.PlanningEnvironment, propagation.FirePropagation]] = None,
                         takeoff_delay: datetime.timedelta = datetime.timedelta(minutes=10),
                         horizon: datetime.timedelta = DEFAULT_HORIZON) \
            -> AlarmResponse:
        """Compute an AlarmResponse to an alarm.

        Arguments:
            alarm: wildfire alarm
            expected_situation: A precomputed pair of PlanningEnvironment and FirePropagation
                of the alarm.
            takeoff_delay: time when the UAVs are expected to take off
            horizon: duration of the monitoring mission
        """
        # Get the area including the alarm ignition points and the UAV bases
        planning_env = expected_situation[0]
        fire_prop = expected_situation[1]

        # Do vehicle allocation before planning
        # Assign 1 vehicle per ignition point in the alarm
        # More inteligent resource allocation can be done in the future
        # For instance measure fire area and precalculate how many UAVs we would need to cover it
        f_confs = []
        uav_allocation = {}
        v = self.hangar.vehicles
        fl_window = (np.inf, -np.inf)

        for ignition, (traj_i, vehicle) in zip(alarm[1], enumerate(v)):
            f_conf = planning.FlightConf(uav=self.hangar.uavconfs()[vehicle],
                                         start_time=(alarm[0] + takeoff_delay).timestamp(),
                                         base_waypoint=self.hangar.bases[vehicle],
                                         wind=planning_env.area_wind)
            fl_window = (min(fl_window[0], f_conf.start_time),
                         max(fl_window[1], f_conf.start_time + f_conf.max_flight_time))
            uav_allocation[traj_i] = vehicle
            f_confs.append(f_conf)

        # The VNS planning strategy is fixed
        # Planning time is proportional to the number of UAV trajectories to be planned
        # Do not save intermediate plans
        saop_conf = planning.SAOPPlannerConf(fl_window, planning.VNSConfDB.demo_db()["demo"],
                                             max_planning_time=10.0 * len(alarm[1]),
                                             save_improvements=False, save_every=0)

        # Plan considering all the previous configuration and data
        planner = planning.Planner(planning_env, fire_prop.prop_data, f_confs, saop_conf)
        sr = planner.compute_plan()

        self._registered_alarms[str(alarm[0])] = AlarmResponse(alarm, planning_env, fire_prop,
                                                               uav_allocation, f_confs,
                                                               saop_conf, sr)
        return self._registered_alarms[str(alarm[0])]


class SuperSAOP:
    """Supervise a wildifre monitoring mission"""

    class MissionState(Enum):
        EXECUTING = 0,
        ENDED = 1,
        FAILED = 2

    class MonitoringAction(Enum):
        """Decision about the future of the mission"""
        REPLAN = 0,  # No decision has been made
        UNDECIDED = 1,  # trigger a replan
        EXIT = 2  # Stop the mission

    def __init__(self, hangar: Hangar, logger: logging.Logger, saop_ui=ui.NoUI()):
        """"""
        self.logger = logger
        self.ui = saop_ui
        self.hangar = hangar

        self.monitoring = True

    def main(self, alarm: Alarm):
        execution_monitor = ExecutionMonitor(self.logger.getChild("ExecutionMonitor"))
        while self.monitoring:
            situation_assessment = SituationAssessment(self.hangar,
                                                       self.logger.getChild("SituationAssessment"))

            observation_planning = ObservationPlanning(self.hangar,
                                                       self.logger.getChild("ObservationPlanning"))

            environment, firepropagation = situation_assessment.expected_situation(alarm)

            # Show wildfire Situation Assessment
            alarm_gdd = gdisplay.GeoDataDisplay(*gdisplay.get_pyplot_figure_and_axis(),
                                                environment.raster, frame=(0., 0.))
            alarm_gdd.add_extension(pdisplay.TrajectoryDisplayExtension, (None,))
            draw_situation(alarm_gdd, alarm, environment, firepropagation, self.hangar)
            alarm_gdd.figure.show()
            plt.pause(0.001)  # Needed to let matplotlib a chance of showing figures

            # Observation Planning
            response = observation_planning.respond_to_alarm(
                alarm, expected_situation=(environment, firepropagation))

            draw_response(alarm_gdd, response)
            plt.pause(0.001)  # Needed to let matplotlib a chance of showing figures

            while not execution_monitor.gcs.is_ready():
                plt.pause(0.1)  # Use the matplotlib pause. So at least figures remain responsive

            command_outcome = execution_monitor.start_response(response.plan,
                                                               response.uav_allocation)

            if functools.reduce(lambda a, b: a and b, command_outcome) or True:
                # state_monitor = execution_monitor.monitor_uav_state(self.hangar.vehicles)
                res = self.do_monitoring(response, execution_monitor)
                if res == SuperSAOP.MonitoringAction.UNDECIDED:
                    self.monitoring = True
                elif res == SuperSAOP.MonitoringAction.REPLAN:
                    self.logger.info("Replan triggered")
                elif res == SuperSAOP.MonitoringAction.EXIT:
                    self.logger.info("SuperSAOP exit")
                    self.monitoring = False
            else:
                if not self.ui.question_dialog("The plan couldn't be started. Restart?"):
                    self.monitoring = False

        self.logger.info("End of monitoring mission for alarm %s", str(alarm))

    def do_monitoring(self, response, execution_monitor) -> MonitoringAction:
        # FIXME: monitor all the trajectories not just one
        plan_name, traj = response.plan.name(), 0
        trajectory_monitor = execution_monitor.monitor_trajectory(plan_name, traj)

        for state_vector in trajectory_monitor:
            state, decision = self.plan_state_and_action(state_vector)
            if decision == SuperSAOP.MonitoringAction.EXIT:
                return decision
            elif decision == SuperSAOP.MonitoringAction.REPLAN:
                return decision
            elif decision == SuperSAOP.MonitoringAction.UNDECIDED:
                if state == SuperSAOP.MissionState.FAILED:
                    self.logger.warning(
                        "Monitoring mission failed. User interaction needed")
                    if self.ui.question_dialog("Monitoring mission failed. Recover?"):
                        return SuperSAOP.MonitoringAction.REPLAN
                    else:
                        return SuperSAOP.MonitoringAction.EXIT
                elif state == SuperSAOP.MissionState.EXECUTING:
                    pass  # SuperSAOP.MonitoringAction.UNDECIDED
                elif state == SuperSAOP.MissionState.ENDED:
                    # TODO: If execution ended, REPLAN or EXIT?
                    return SuperSAOP.MonitoringAction.EXIT

    def plan_state_and_action(self, state_dict: ty.Dict[
        ty.Tuple[str, int], nifc.TrajectoryExecutionReport]) \
            -> ty.Tuple[MissionState, MonitoringAction]:
        """Determine the state of the mission and decide wether a mission should continue.

        A mission is considered
            - FAILED if any trajectory state is Blocked or its outcome is Failure
            - ENDED: if all trajectory outcomes are Success
            - EXECUTING: if all trajectory states are Executing

        The next action should be
            - UNDECIDED: No decision has been made
            - REPLAN: trigger a replan
            - EXIT: Stop the mission
        """

        mission_s_v = {}
        for plan_id, ter in state_dict.items():
            if ter.state == nifc.TrajectoryExecutionState.Executing:
                mission_s_v[plan_id] = SuperSAOP.MissionState.EXECUTING
            elif ter.state == nifc.TrajectoryExecutionState.Ready:
                if ter.last_outcome == nifc.TrajectoryExecutionOutcome.Failure or \
                        ter.last_outcome == nifc.TrajectoryExecutionOutcome.Nothing:
                    mission_s_v[plan_id] = SuperSAOP.MissionState.FAILED
                if ter.last_outcome == nifc.TrajectoryExecutionOutcome.Success:
                    mission_s_v[plan_id] = SuperSAOP.MissionState.ENDED
            else:  # nifc.TrajectoryExecutionState.Blocked
                self.logger.critical("UAV %s is blocked. This is a failure")
                return SuperSAOP.MissionState.FAILED, SuperSAOP.MonitoringAction.EXIT

        for m_s in mission_s_v.values():
            if m_s == SuperSAOP.MissionState.FAILED:
                # If one trajectory failed, the plan is a failure. Leave action undecided
                return SuperSAOP.MissionState.FAILED, SuperSAOP.MonitoringAction.UNDECIDED
            elif m_s == SuperSAOP.MissionState.EXECUTING:
                # If one trajectory is still running then the plan is not over yet
                return SuperSAOP.MissionState.EXECUTING, SuperSAOP.MonitoringAction.UNDECIDED
        else:
            # All the trajectories are over
            return SuperSAOP.MissionState.ENDED, SuperSAOP.MonitoringAction.UNDECIDED


class ExecutionMonitor:
    """Communicate with the UAV ground control software"""

    def __init__(self, logger: logging.Logger):

        self.logger = logger

        self.imccomm = nifc.IMCComm()
        self.gcs = None

        self.t_imc = threading.Thread(target=self.imccomm.run, daemon=False)
        self.t_gcs = threading.Thread(target=self._create_gcs, daemon=False)
        self.t_imc.run()
        self.t_gcs.run()

        self.monitoring = False  # Determine if we're doing monitoring of a plan or not
        self.traj_report_q = queue.Queue()

        self.message_q = {'TrajectoryExecutionReport': queue.Queue(),
                          'UAVStateReport': queue.Queue()}

    def _create_gcs(self):
        """Create GCS object of this class.
        To be runned in a different thread."""
        self.gcs = nifc.GCS(self.imccomm, self.on_trajectory_execution_report,
                            self.on_uav_state_report)

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

    def start_response(self, plan, uav_allocation: ty.Dict[int, str]) -> ty.Sequence[bool]:
        """Execute a SAOP Plan 'a_plan' with the AUV to trajectory allocation of 'uav_allocation'"""
        # Store results (success/failure) trajectory starts
        results = {t: True for t in uav_allocation.keys()}

        for traj_i, uav in uav_allocation.items():
            plan_name = ExecutionMonitor.neptus_plan_id(plan.name(), traj_i)

            # Load the trajectory
            command_r = nifc.GCSCommandOutcome.Unknown
            retries = 0
            max_retries = 2
            while retries < max_retries and command_r != nifc.GCSCommandOutcome.Success:
                command_r = self.gcs.load(plan, traj_i, plan_name, uav)
                if command_r != nifc.GCSCommandOutcome.Success:
                    self.logger.warning("Loading of trajectory %s failed. Retrying...", plan_name)
                    retries += 1
                else:
                    self.logger.info("Trajectory %s loaded", plan_name)

            if command_r != nifc.GCSCommandOutcome.Success:
                self.logger.error("Loading of trajectory %s failed. Giving up after %d trials",
                                  plan_name, max_retries)
                results[traj_i] = False

            # Start the mission
            # FIXME: Mission start seems to fail always
            command_r = self.gcs.start("saop_" + plan_name, uav)
            if command_r != nifc.GCSCommandOutcome.Success:
                self.logger.error("Start of trajectory %s failed", plan_name)
                results[traj_i] = False
            else:
                self.logger.error("Start trajectory %s", plan_name)

        return results

    def _monitor_queue(self, q, id_field: str, monitored_ids: ty.Sequence[str],
                       discard_older: bool = False, timeout: int = 5):
        state_vector = {}
        yield_start = datetime.datetime.now()

        while True:
            report = None
            try:
                report = q.get(block=True, timeout=timeout)
            except queue.Empty:
                self.logger.warning("Nothing is being reported")
                return

            if report is None:
                continue

            # Discard messages created before monitoring started
            if discard_older and report.timestamp < yield_start.timestamp():
                continue

            if getattr(report, id_field) in monitored_ids:
                state_vector[report.plan_id] = report
            if len(state_vector) == len(monitored_ids):
                # When all the reports are gathered, yield a state vector
                new_state = state_vector.copy()
                state_vector = {}
                yield new_state

    def monitor_uav_state(self, uavs: ty.Sequence[str]):
        for m in self._monitor_queue(self.message_q['UAVStateReport'], "uav", uavs, True, 3):
            yield m

    def monitor_trajectory(self, saop_plan_name: str, saop_trajectory: int):
        for m in self._monitor_queue(self.message_q['TrajectoryExecutionReport'], "plan_id",
                                     [ExecutionMonitor.neptus_plan_id(saop_plan_name,
                                                                      saop_trajectory)],
                                     False, 10):
            # Replace neptus plan_id keys by (plan.name, traj) pairs
            yield {ExecutionMonitor.saop_plan_and_traj(k): v for k, v in m.items()}

    def on_trajectory_execution_report(self, ter: nifc.TrajectoryExecutionReport):
        """Method called by the GCS to report about the state of the missions"""
        # FIXME: Do something if the queue is full
        try:
            self.message_q['TrajectoryExecutionReport'].put(ter, block=True, timeout=1)
        except queue.Full:
            self.logger.exception("TrajectoryExecutionReport queue timeout reached")

    def on_uav_state_report(self, usr: nifc.UAVStateReport):
        """Method called by the GCS to report about the state of the UAVs"""
        try:
            self.message_q['UAVStateReport'].put(usr, block=True, timeout=1)
        except queue.Full:
            self.logger.exception("UAVStateReport queue timeout reached")


def draw_response(gdd: gdisplay.GeoDataDisplay,
                  response: AlarmResponse):
    gdd.draw_ignition_contour(response.fire_prop.prop_data, with_labels=True)
    pdisplay.plot_plan_trajectories(response.plan, gdd, layers=['trajectory_solid', 'arrows'])
    gdd.figure.legend()
    gdd.figure.suptitle(
        "Observation Plan of wildfire alarm.\n Valid from " + str(response.alarm[0]) + " to " + str(
            response.alarm[0] + datetime.timedelta(seconds=response.fire_prop.until_time)))
