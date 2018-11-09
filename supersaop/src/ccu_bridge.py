#!/usr/bin/env python3
import datetime
import logging
import re
import threading
import typing as ty
import uuid

import matplotlib

matplotlib.use('Gtk3Agg')

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

from supersaop.msg import ElevationMap, Euler, PoseEuler, PredictedWildfireMap, PropagateCmd, \
    Raster, RasterMetaData, WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped, Plan

from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.monitoring.supersaop import NeptusBridge

import serialization

TEST_AREA = ((480060.0, 489060.0), (6210074.0, 6217074.0))


def ros_name_for(s: str):
    """Obtain a ROS-allowed name"""
    return re.sub('[^a-zA-Z0-9_]', '_', s)


class CCUBridgeNode:

    def __init__(self):
        rospy.init_node('ccu_bridge')
        rospy.loginfo("Starting {}".format(self.__class__.__name__))

        self.ccu = NeptusBridge(logging.getLogger(__name__))
        self.ccu.set_uav_state_callback(self.on_uav_state_from_neptus)

        self._known_uavs = set()
        self._known_uavs.add('x8-06')

        self.dict_state_lock = threading.Lock()
        self.dict_state_msg = {uav: None for uav in self._known_uavs}

        self.pub_dict_state = {uav: rospy.Publisher("/".join((ros_name_for(uav), 'state')),
                                                    PoseEuler, queue_size=10) for uav in
                               self._known_uavs}  # type: ty.Mapping[rospy.Publisher]

        self.current_saop_plan = None
        self.sub_saop_plan = rospy.Subscriber("plan", Plan, queue_size=10)

    def on_uav_state_from_neptus(self, **kwargs):
        if kwargs['uav'] in self._known_uavs:
            with self.dict_state_lock:
                # kwargs['uav'] = rospy.Publisher("/".join((ros_name_for(kwargs['uav']), 'pose')),
                #                                 PoseEuler, queue_size=10)
                pose_msg = PoseEuler(position=Point(kwargs['x'], kwargs['y'], kwargs['z']),
                                     orientation=Euler(phi=kwargs['phi'], theta=kwargs['theta'],
                                                       psi=kwargs['psi']))
                self.dict_state_msg[kwargs['uav']] = pose_msg

    def publish_state(self):
        """ Publish retrieved information from Neptus into ROS topics"""
        for uav, state in self.dict_state_msg.items():
            with self.dict_state_lock:
                if state is not None:
                    rospy.loginfo("%s: %s", str(uav), str(state))
                    self.pub_dict_state[uav].publish(state)


if __name__ == '__main__':
    try:
        ccu = CCUBridgeNode()
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            ccu.publish_state()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
