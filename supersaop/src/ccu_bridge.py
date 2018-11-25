#!/usr/bin/env python3
import datetime
import logging
import re
import threading
import typing as ty

import matplotlib

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String, Header
from geometry_msgs.msg import Point

from supersaop.msg import ElevationMap, Euler, PoseEuler, PredictedWildfireMap, PropagateCmd, \
    Raster, RasterMetaData, WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped, \
    Plan, VehicleState, Velocity, TrajectoryState

from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.monitoring.supersaop import NeptusBridge

import serialization

TEST_AREA = ((480060.0, 489060.0), (6210074.0, 6217074.0))


class CCUBridgeNode:

    def __init__(self):
        rospy.init_node('ccu_bridge')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.ccu = NeptusBridge(logging.getLogger(__name__))
        self.ccu.set_uav_state_callback(self.on_uav_state_from_neptus)
        self.ccu.set_trajectory_state_callback(self.on_trajectory_state_from_neptus)

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

        self.current_saop_plan = None
        self.sub_saop_plan = rospy.Subscriber("plan", Plan, self.on_saop_plan, queue_size=10)

    def on_saop_plan(self, msg: Plan):
        t = serialization.saop_trajectories_from_plan_msg(msg)
        for traj in t:
            if traj.length() > 0.:
                self.ccu.start_trajectory(traj, traj.conf.uav.name)

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


if __name__ == '__main__':
    try:
        ccu = CCUBridgeNode()
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            ccu.publish_state()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
