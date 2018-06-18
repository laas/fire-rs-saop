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

import logging
import queue
import threading
import typing as ty
import datetime
import numpy as np
import fire_rs.geodata.geo_data as geo_data
import fire_rs.planning.planning as planning
import fire_rs.firemodel.propagation as propagation
import fire_rs.neptus_interface as nifc

supersaop_start_time = datetime.datetime.now()

Alarm = ty.Tuple[datetime.datetime, ty.Sequence[geo_data.TimedPoint]]

Area2D = ty.Tuple[ty.Tuple[float, float], ty.Tuple[float, float]]


class Hangar:
    def __init__(self):
        base_default = planning.Waypoint(483500.0 - 150., 6215000.0 - 150., 0., 0.)
        uav_x8 = planning.UAVConf.X8()

        self._uav_keys = ['x8-02', 'x8-06']
        self._bases = {}
        self._uavconfs = {}

        for uav_k in self._uav_keys:
            self._bases[uav_k] = base_default
            self._uavconfs[uav_k] = uav_x8

    def uavconfs(self) -> ty.Dict[str, planning.UAVConf]:
        """Get planning UAVConf objects for every available UAV."""
        return self._uavconfs

    def move_uav_to_area(self, uav: str, area: Area2D):
        """Generate a UAV base near the boundary of 'area' and move 'uav' to it"""
        self._bases[uav] = planning.Waypoint(area[0][1] - 150., area[1][1] - 150., 0., 0.)

    @property
    def vehicles(self) -> ty.Sequence[str]:
        return self._uav_keys

    @property
    def bases(self) -> ty.Dict[str, planning.Waypoint]:
        """Get UAV bases"""
        return self._bases


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


class ObservationPlanning:
    def __init__(self, hangar: Hangar, logger: logging.Logger):
        self.hangar = hangar
        self._registered_alarms = {}
        self.logger = logger

    def respond_to_alarm(self, alarm: Alarm,
                         takeoff_delay: datetime.timedelta = datetime.timedelta(minutes=10),
                         horizon: datetime.timedelta = datetime.timedelta(minutes=60)) \
            -> AlarmResponse:
        """Get a Plan that responds to an alarm"""
        # Get the area including the alarm ignition points and the UAV bases
        p_env = self._get_environment(alarm, self.hangar.bases.values())
        self.logger.info("Got environment: %s", p_env)

        fire_prop = self._compute_fire_propagation(p_env, alarma, alarma[0] + horizon)

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
                                         start_time=(alarma[0] + takeoff_delay).timestamp(),
                                         base_waypoint=self.hangar.bases[vehicle],
                                         wind=p_env.area_wind)
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
        planner = planning.Planner(p_env, fire_prop.prop_data, f_confs, saop_conf)
        sr = planner.compute_plan()

        self._registered_alarms[str(alarm[0])] = AlarmResponse(alarm, p_env, fire_prop,
                                                               uav_allocation, f_confs,
                                                               saop_conf, sr)
        return self._registered_alarms[str(alarm[0])]

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

    @staticmethod
    def _get_environment(alarm: Alarm, bases) -> planning.PlanningEnvironment:
        """Get a PlanningEnvironment around an alarm"""
        wind = ObservationPlanning._get_area_wind(alarm[1][0])
        area = ObservationPlanning._compute_area(alarm[1], bases)

        return planning.PlanningEnvironment(area, wind_speed=wind[0], wind_dir=wind[1],
                                            planning_elevation_mode='flat', flat_altitude=0)

    def _compute_fire_propagation(self, env: planning.PlanningEnvironment,
                                  alarm: Alarm,
                                  until: datetime.datetime) -> propagation.FirePropagation:
        self.logger.info("Start fire propagation from %s until %s", alarma[1], until)
        pt_float = [(x, y, time.timestamp()) for x, y, time in alarm[1]]
        prop = propagation.propagate_from_points(env, pt_float, until.timestamp())
        self.logger.info("Propagation ended")
        return prop


class SuperSAOP:
    pass


class MonitoringReport:
    """Report of the monitoring mission including the observed firemap"""

    def __init__(self, response: AlarmResponse, firemap):
        pass


class ExecutionMonitor:

    def __init__(self, logger: logging.Logger):

        self.logger = logger

        self.imccomm = nifc.IMCComm()
        self.gcs = None

        self.t_imc = threading.Thread(target=self.imccomm.run, daemon=True)
        self.t_gcs = threading.Thread(target=self._create_gcs, daemon=True)
        self.t_imc.run()
        self.t_gcs.run()

        self.monitoring = False  # Determine if we're doing monitoring of a plan or not
        self.traj_report_q = queue.Queue()

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

    def start_response(self, response: AlarmResponse) -> bool:
        """Execute a SAOP Plan 'a_plan' with the AUV to trajectory allocation of 'uav_allocation'"""
        # Store results (success/failure) trajectory starts
        results = [nifc.GCSCommandOutcome.Unknown for _ in range(len(response.uav_allocation))]

        for traj_i, uav in response.uav_allocation.items():
            plan_name = ExecutionMonitor.neptus_plan_id(response.plan.name(), traj_i)

            # Load the trajectory
            results[traj_i] = nifc.GCSCommandOutcome.Unknown
            retries = 0
            max_retries = 2
            while retries < max_retries and results[traj_i] != nifc.GCSCommandOutcome.Success:
                results[traj_i] = self.gcs.load(response.plan, traj_i, plan_name, uav)
                if results[traj_i] != nifc.GCSCommandOutcome.Success:
                    logger.warning("Loading of trajectory %s failed. Retrying...", plan_name)
                    retries -= 1
                else:
                    logger.info("Trajectory %s loaded", plan_name)

            if results[traj_i] != nifc.GCSCommandOutcome.Success:
                logger.error("Loading of trajectory %s failed. Giving up after %d trials",
                             plan_name, max_retries)
                return False

            # Start the mission
            results[traj_i] = self.gcs.start("saop_" + plan_name, uav)
            if results[traj_i] != nifc.GCSCommandOutcome.Success:
                logger.error("Start of trajectory %s failed", plan_name)
                return False
            else:
                logger.error("Start trajectory %s", plan_name)

        # return [True if r is nifc.GCSCommandOutcome.Success else False for r in results]
        return True

    def monitor(self, response: AlarmResponse):
        last_yield_report = datetime.datetime.now()  # Time of the last TER that triggered a PlanReport
        last_recv_report = datetime.datetime.now()  # time of the last received TER

        last_yield_uav = datetime.datetime.now()
        last_recv_uav = datetime.datetime.now()

        self.monitoring = True
        monitored_planids = [ExecutionMonitor.neptus_plan_id(response.plan.name(), i) for i in
                             range(len(response.plan.trajectories()))]

        # Store recent trajectory reports to join them
        last_tr = {}
        last_uav = {}

        while self.monitoring:
            report = None
            try:
                report = self.traj_report_q.get(block=True, timeout=5)
            except queue.Empty:
                logger.warning("Nothing is being reported")

            if report is None:
                continue

            if isinstance(report, nifc.TrajectoryExecutionReport):
                # Discard old messages
                if report.timestamp < last_yield_report.timestamp():
                    continue
                if report.plan_id in monitored_planids:
                    last_tr[report.plan_id] = report
                    last_recv_report = datetime.datetime.fromtimestamp(report.timestamp)
                if len(last_tr) == len(monitored_planids):
                    # When all the trajectory reports are gathered, report the state of the plan
                    plan_report = last_tr.copy()
                    last_tr = {}
                    last_yield_report = last_recv_report
                    yield plan_report
            elif isinstance(report, nifc.UAVStateReport):
                # Discard old messages
                if report.timestamp < last_yield_uav.timestamp():
                    continue
                if report.uav in response.uav_allocation.values():
                    last_uav[report.uav] = report
                    last_recv_uav = datetime.datetime.fromtimestamp(report.timestamp)
                if len(last_uav) == len(monitored_planids):
                    # When all the trajectory reports are gathered, report the state of the plan
                    all_uav_report = last_uav.copy()
                    last_uav = {}
                    last_yield_uav = last_recv_uav
                    yield all_uav_report
            else:
                raise TypeError("Wrong report type")

    def on_trajectory_execution_report(self, ter: nifc.TrajectoryExecutionReport):
        """Method called by the GCS to report about the state of the missions"""
        # FIXME: Do something if the queue is full
        if self.monitoring:
            try:
                self.traj_report_q.put(ter, block=True, timeout=1)
            except queue.Full:
                logger.exception("report queue is full")

    def on_uav_state_report(self, usr: nifc.UAVStateReport):
        """Method called by the GCS to report about the state of the UAVs"""
        if self.monitoring:
            try:
                self.traj_report_q.put(usr, block=True, timeout=1)
            except queue.Full:
                logger.exception("report queue is full")


if __name__ == "__main__":
    import logging
    import fire_rs.geodata.display as gdisplay

    # Set up logging
    FORMAT = '%(asctime)-23s %(levelname)-8s [%(name)s]: %(message)s'
    logging.basicConfig(format=FORMAT)

    logger = logging.getLogger("__main__")
    logger.setLevel(logging.DEBUG)

    logger_up = logger.getChild("uav_planning")
    planning.up.set_logger(logger_up)

    # Start Situation Assessment
    alarma = poll_alarm()
    logger.info("Alarm raised: %s", alarma)

    observation_planning = ObservationPlanning(Hangar(), logger.getChild("ObservationPlanning"))
    response = observation_planning.respond_to_alarm(alarma)

    alarm_figure = gdisplay.get_pyplot_figure_and_axis()
    alarm_gdd = gdisplay.GeoDataDisplay(*gdisplay.get_pyplot_figure_and_axis(),
                                        response.planning_env.raster,
                                        frame=(0., 0.))

    alarm_gdd.draw_elevation_shade()
    alarm_gdd.draw_ignition_contour(geodata=response.fire_prop.prop_data, layer='ignition')
    alarm_gdd.draw_base(observation_planning.hangar.bases[response.uav_allocation[0]], color='b', s=50)
    alarm_gdd.draw_ignition_points(response.alarm[1], zorder=1000)
    alarm_gdd.figure.show()

    execution_monitor = ExecutionMonitor(logger.getChild("ExecutionMonitor"))
    if execution_monitor.start_response(response) or True:
        for report in execution_monitor.monitor(response):
            print(str(report))
    else:
        logger.error("Execution of plan failed")
