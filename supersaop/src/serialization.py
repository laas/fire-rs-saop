import re
import typing as ty

import numpy as np
import rospy

from fire_rs.geodata.geo_data import GeoData
import fire_rs.planning.new_planning as planning
from supersaop.msg import Raster, RasterMetaData, Plan


def ros_name_for(s: str):
    """Obtain a ROS-allowed name"""
    return re.sub('[^a-zA-Z0-9_]', '_', s)


def raster_msg_from_geodata(geodata: GeoData, layer: str, invert=False):
    array = geodata.data[layer]
    if invert:
        array = array[..., ::-1]
    return Raster(metadata=RasterMetaData(x_offset=geodata.x_offset, y_offset=geodata.y_offset,
                                          x_width=geodata.max_x, y_height=geodata.max_y,
                                          cell_width=geodata.cell_width,
                                          epsg=geodata.projection_epsg),
                  data=array.ravel())


def geodata_from_raster_msg(msg: Raster, layer: str, invert=False):
    """Deserialize a Raster msg into a GeoData object.

    if layer is "elevation", the raster is inverted
    """
    array = np.fromiter(zip(msg.data), dtype=[(layer, 'float64')])
    array.resize((msg.metadata.x_width, msg.metadata.y_height))
    # FIXME (Workaround) invert elevation rasters as they are shown inverted in the GUI
    if layer == "elevation":
        array = array[..., ::-1]
    if invert:
        array = array[..., ::-1]
    return GeoData(array, msg.metadata.x_offset, msg.metadata.y_offset, msg.metadata.cell_width,
                   msg.metadata.cell_width, projection=msg.metadata.epsg)


def saop_trajectories_from_plan_msg(msg: Plan) -> ty.Sequence[planning.Trajectory]:
    # msg_tc.conf.uav_model
    return [planning.Trajectory(planning.TrajectoryConfig(
        msg_tc.conf.name,
        planning.UAVModels.get(msg_tc.conf.uav_model),
        planning.Waypoint(msg_tc.conf.start_wp.position.x, msg_tc.conf.start_wp.position.y,
                          msg_tc.conf.start_wp.position.z, msg_tc.conf.start_wp.orientation.psi),
        planning.Waypoint(msg_tc.conf.end_wp.position.x, msg_tc.conf.end_wp.position.y,
                          msg_tc.conf.end_wp.position.z, msg_tc.conf.end_wp.orientation.psi),
        rospy.Time.to_sec(msg_tc.conf.start_time),
        msg_tc.conf.max_duration,
        planning.WindVector(
            msg_tc.conf.wind.speed * np.cos(msg_tc.conf.wind.direction),
            msg_tc.conf.wind.speed * np.sin(msg_tc.conf.wind.direction))),
        [planning.TrajectoryManeuver(
            planning.Waypoint(m.waypoint.position.x, m.waypoint.position.y, m.waypoint.position.z,
                              m.waypoint.orientation.psi),
            rospy.Time.to_sec(m.time), m.id) for m in msg_tc.maneuvers]) for msg_tc in
        msg.trajectories]
