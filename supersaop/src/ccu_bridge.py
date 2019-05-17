#!/usr/bin/env python3

# import pandas to workaround issue when pandas is imported after some other libraries
# https://github.com/pandas-dev/pandas/issues/23040
import pandas

import datetime
import logging
import re
import threading
import typing as ty

import numpy as np
import matplotlib

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point

from supersaop.msg import ElevationMap, Euler, PoseEuler, PredictedWildfireMap, PropagateCmd, \
    Raster, RasterMetaData, WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped, \
    Plan, VehicleState, Velocity, TrajectoryState, StopCmd, HomeCmd

from fire_rs.geodata.display import GeoDataDisplay
import fire_rs.geodata.geo_data as geo_data
from fire_rs.geodata.wildfire import _compute_perimeter
from fire_rs.monitoring.supersaop import NeptusBridge

import serialization

TEST_AREA = ((480060.0, 489060.0), (6210074.0, 6217074.0))


class CCUBridgeNode:

    def __init__(self, coordinate_system=None):
        rospy.init_node('ccu_bridge')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        if coordinate_system is not None:
            if coordinate_system == geo_data.EPSG_ETRS89_LAEA:
                # self.ccu.set_coordinate_system(geo_data.EPSG_ETRS89_LAEA, geo_data.EPSG_ETRS89)
                pass
            elif coordinate_system == geo_data.EPSG_RGF93_LAMBERT93:
                # self.ccu.set_coordinate_system(geo_data.EPSG_RGF93_LAMBERT93, geo_data.EPSG_RGF93)
                pass
            elif coordinate_system == geo_data.EPSG_WGS84_UTM29N:
                # self.ccu.set_coordinate_system(geo_data.EPSG_WGS84_UTM29N, geo_data.EPSG_WGS84)
                pass
            else:
                rospy.logfatal("Unknown coordinate system code '%s'", str(coordinate_system))
                raise rospy.ROSInterruptException(
                    "Unknown coordinate system code '{}'".format(str(coordinate_system)))
        else:
            rospy.logwarn("No coordinate system has been set. Using default Lambert93/RGF93")

        self.ccu = NeptusBridge(logging.getLogger(__name__), coordinate_system)
        self.ccu.start()
        self.ccu.set_uav_state_callback(self.on_uav_state_from_neptus)
        self.ccu.set_trajectory_state_callback(self.on_trajectory_state_from_neptus)
        self.ccu.set_firemap_callback(self.on_firemap_from_neptus)

        self._known_uavs = set()
        self._known_uavs.add('x8-06')
        self._known_uavs.add('x8-02')

        self.dict_state_lock = threading.Lock()
        self.dict_state_msg = {uav: None for uav in self._known_uavs}

        # type: ty.Mapping[str, rospy.Publisher]
        self.pub_dict_state = {
            uav: rospy.Publisher("/".join(("uavs", serialization.ros_name_for(uav), 'state')),
                                 VehicleState,
                                 queue_size=10) for uav in self._known_uavs}

        self.dict_traj_lock = threading.Lock()
        self.dict_traj_msg = {uav: None for uav in self._known_uavs}

        # type: ty.Mapping[str, rospy.Publisher]
        self.pub_dict_traj = {
            uav: rospy.Publisher("/".join(("uavs", serialization.ros_name_for(uav), 'trajectory')),
                                 TrajectoryState, queue_size=10) for uav in self._known_uavs}

        self.dict_firemap_lock = threading.Lock()
        self.dict_firemap_msg = {uav: None for uav in self._known_uavs}

        # type: ty.Mapping[str, rospy.Publisher]
        self.pub_dict_firemap = {
            uav: rospy.Publisher(
                "/".join(("uavs", serialization.ros_name_for(uav), 'wildfire_observed')),
                WildfireMap, queue_size=10) for uav in self._known_uavs}

        self.current_saop_plan = None
        self.sub_saop_plan = rospy.Subscriber("plan", Plan, self.on_saop_plan, queue_size=10)
        self.sub_saop_plan = rospy.Subscriber("stop", StopCmd, self.on_stop_cmd, queue_size=10)
        self.sub_saop_plan = rospy.Subscriber("go_home", HomeCmd, self.on_home_cmd, queue_size=10)

        self.sub_wildfire_pred = rospy.Subscriber('wildfire_prediction', PredictedWildfireMap,
                                                  self.on_wildifre_prediction, queue_size=10)
        self.sub_wildfire_current = rospy.Subscriber('wildfire', WildfireMap,
                                                     self.on_wildfire_current, queue_size=10)

        self.sub_windspeed_dict = {
            uav: rospy.Subscriber(
                "/".join(("uavs", serialization.ros_name_for(uav), 'windspeed')),
                MeanWindStamped, callback=self.on_wind_speed, callback_args=uav,
                queue_size=10) for uav in self._known_uavs}

    def on_saop_plan(self, msg: Plan):
        t = serialization.saop_trajectories_from_plan_msg(msg)
        for traj in t:
            if traj.length() > 0.:
                self.ccu.load_trajectory(traj, traj.conf.uav.name)

    def on_stop_cmd(self, msg: StopCmd):
        self.ccu.stop_uav(msg.uav)

    def on_home_cmd(self, msg: HomeCmd):
        rospy.logwarn(msg)
        uav = msg.uav.replace("_", "-")
        self.ccu.loiter(uav, msg.plan_name, msg.loiter.id,
                        (msg.loiter.center.position.x, msg.loiter.center.position.y,
                         msg.loiter.center.position.z),
                        msg.loiter.radius, msg.loiter.direction, msg.loiter.duration.to_sec())

    def on_wildifre_prediction(self, msg: PredictedWildfireMap):
        def create_drawable_contour(f_map: geo_data.GeoData, time_sec: float,
                                    color: ty.Tuple[int, int, int], one_out_of:int=1):
            cur_perimeter = _compute_perimeter(firemap, time_sec)
            count = 0
            if cur_perimeter[1]:  # cells
                coor_tran = geo_data.CoordinateTransformation(firemap.projection_epsg,
                                                              geo_data.EPSG_WGS84)
                polygon = []
                for cell in cur_perimeter[2][0]:
                    if count % one_out_of:
                        continue
                    if not np.isnan(cell[0]) and not np.isnan(cell[1]):
                        pcs_point = firemap.coordinates(cell)
                        gcs_point = coor_tran.transform(*pcs_point)
                        polygon.append({"lat": gcs_point[1], "lon": gcs_point[0]})
            if polygon:
                return {"time": time_sec, "color": [255, 0, 0], "polygon": polygon}

        firemap = serialization.geodata_from_raster_msg(msg.raster, "ignition")
        oof = 2
        rospy.loginfo("Sending to Neptus %s out of %s point in contours", str(1), str(oof))
        # Colors from bright red to white, varying saturation: (255, x, x)
        desired_contours = [(rospy.Time.now().to_sec(), (255, 0, 0)),
                            (rospy.Time.now().to_sec() + 1800, (255, 170, 170)),
                            (rospy.Time.now().to_sec() + 3600, (255, 85, 85))]
        drawable_conts = []
        for d in desired_contours:
            d_cont = create_drawable_contour(firemap, d[0], d[1], oof)
            if d_cont:
                drawable_conts.append(d_cont)

        self.ccu.send_wildfire_contours(drawable_conts)

    def on_wildfire_current(self, msg: WildfireMap):
        pass

    def on_wind_speed(self, msg: MeanWindStamped, uav: str):
        pass
        # self.ccu.set_wind(msg.speed, msg.direction, uav)

    def on_uav_state_from_neptus(self, **kwargs):
        if kwargs['uav'] in self._known_uavs:
            with self.dict_state_lock:
                position_msg = Point(kwargs['x'], kwargs['y'], kwargs['z'])
                orientation_msg = Euler(phi=kwargs['phi'], theta=kwargs['theta'],
                                        psi=kwargs['psi'])
                velocity_msg = Velocity(kwargs['vx'], kwargs['vy'], kwargs['vz'])
                pose_msg = VehicleState(header=Header(stamp=rospy.Time.from_sec(kwargs['time'])),
                                        name=kwargs['uav'], position=position_msg,
                                        orientation=orientation_msg, ground_velocity=velocity_msg)
                self.dict_state_msg[kwargs['uav']] = pose_msg

    def on_trajectory_state_from_neptus(self, **kwargs):
        if kwargs['uav'] in self._known_uavs:
            with self.dict_traj_lock:
                traj_state_msg = TrajectoryState(
                    header=Header(stamp=rospy.Time.from_sec(kwargs['time'])),
                    name=kwargs['plan_id'], uav=kwargs['uav'],
                    maneuver=kwargs['maneuver'],
                    maneuver_eta=rospy.Duration(kwargs['maneuver_eta']),
                    state=int(kwargs['state']), last_outcome=int(kwargs['last_outcome']))
                self.dict_traj_msg[kwargs['uav']] = traj_state_msg

    def on_firemap_from_neptus(self, **kwargs):
        if kwargs['uav'] in self._known_uavs:
            with self.dict_firemap_lock:
                kwargs["firemap"].data["ignition"] = kwargs["firemap"].data["ignition"][..., ::-1]
                firemap_msg = WildfireMap(
                    header=Header(stamp=rospy.Time.from_sec(kwargs['time'])),
                    raster=serialization.raster_msg_from_geodata(kwargs["firemap"], 'ignition'))
                self.dict_firemap_msg[kwargs['uav']] = firemap_msg

    def publish_state(self):
        """ Publish retrieved information from Neptus into ROS topics"""
        for uav, state in self.dict_state_msg.items():
            with self.dict_state_lock:
                if state is not None:
                    # rospy.logdebug("%s: %s", str(uav), str(state))
                    self.pub_dict_state[uav].publish(state)

        for uav, traj_state in self.dict_traj_msg.items():
            with self.dict_traj_lock:
                if traj_state is not None:
                    # rospy.logdebug("%s: %s", str(uav), str(traj_state))
                    self.pub_dict_traj[uav].publish(traj_state)

        for uav, firemap in self.dict_firemap_msg.items():
            with self.dict_firemap_lock:
                if firemap is not None:
                    # rospy.logdebug("%s: %s", str(uav), str(firemap))
                    self.pub_dict_firemap[uav].publish(firemap)


if __name__ == '__main__':
    try:
        coordinate_s = rospy.get_param("coordinate_system", None)
        ccu = CCUBridgeNode(coordinate_system=coordinate_s)
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            ccu.publish_state()
            r.sleep()
    except rospy.ROSInterruptException as e:
        print(e)
