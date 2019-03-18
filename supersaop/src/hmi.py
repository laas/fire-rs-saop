#!/usr/bin/env python3

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

# import pandas to workaround issue when pandas is imported after some other libraries
# https://github.com/pandas-dev/pandas/issues/23040
import pandas

import collections
import datetime
import threading
import typing as ty
from itertools import cycle

import numpy as np

import matplotlib.figure

import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gio, GLib, Gtk, Pango
from matplotlib.backends.backend_gtk3cairo import FigureCanvasGTK3Cairo as FigureCanvas
from matplotlib.backends.backend_gtk3 import NavigationToolbar2GTK3 as NavigationToolbar

import rospy
from rosgraph_msgs.msg import Log
from std_msgs.msg import Header
from geometry_msgs.msg import Point
from supersaop.msg import ElevationMap, PredictedWildfireMap, PropagateCmd, Raster, RasterMetaData, \
    WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped, VehicleState, PlanCmd, Plan, \
    Trajectory, TrajectoryConf, PlanConf, PoseEuler, Euler, MeanWind, StopCmd

from fire_rs.geodata.geo_data import GeoData, Area
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_trajectory, TRAJECTORY_COLORS

import serialization


class FlaggedData:
    """Variable data with special tags to flag update and drawing enabled"""

    def __init__(self, name: str, data: ty.Any, updatable: bool = True, drawable: bool = True):
        self._name = name
        self._data = data
        self.updatable = updatable
        self.drawable = drawable
        self._update_callback = lambda drawableobj: None

    def __bool__(self):
        return bool(self._data)

    def _on_update(self):
        if self.data:
            self._update_callback(self)

    def updated(self):
        """Tell this the object that the data has changed"""
        if self.updatable:
            self._on_update()

    @property
    def update_callback(self) -> ty.Callable[[ty.Any], None]:
        return self._update_callback

    @update_callback.setter
    def update_callback(self, value: ty.Callable[[ty.Any], None]):
        self._update_callback = value

    @property
    def name(self) -> str:
        return self._name

    @property
    def data(self) -> ty.Any:
        return self._data

    @data.setter
    def data(self, value: ty.Any):
        if self.updatable:
            self._data = value
            self._on_update()


class HMIModel:
    def __init__(self):
        self.gui = None
        self.node = None

        self.gdd = None
        self.gdd_drawable = True

        # Model data
        # _updatable controls whether new values are stored when updates arrive
        # _drawable controls whether the field is displayed over the figure

        elevation_map = None  # type: ty.Optional[GeoData]
        self.elevation_map = FlaggedData("Elevation", elevation_map, updatable=True, drawable=True)

        wildfire_map = None  # type: ty.Optional[GeoData]
        self.wildfire_map = FlaggedData("Fire front", wildfire_map, updatable=True, drawable=True)

        wildfire_map_predicted = None  # type: ty.Optional[GeoData]
        self.wildfire_map_predicted = FlaggedData("Predicted wildfire", wildfire_map_predicted,
                                                  updatable=True, drawable=True)

        wildfire_map_observed = None  # type: ty.Optional[GeoData]
        self.wildfire_map_observed = FlaggedData("Observed wildfire", wildfire_map_observed,
                                                 updatable=True, drawable=True)

        self._uav_state_dict = {}
        self._last_uav_state_draw = datetime.datetime.min
        self.uav_state = FlaggedData("UAVs", self._uav_state_dict, updatable=True, drawable=True)

        uav_trail_dict = {}  # type: ty.MutableMapping[str, collections.deque]
        self._uav_trail_count = 100
        self.uav_trail = FlaggedData("UAV trails", uav_trail_dict, updatable=True, drawable=True)

        trajectories = []
        self.trajectories = FlaggedData("Plan", trajectories, updatable=True, drawable=True)

        self.uav_colors = TRAJECTORY_COLORS

    def known_uavs(self):
        return list(self._uav_state_dict.keys())

    def set_controller(self, saopcontrolwindow):
        """Set the GUI that is controlling stuff"""
        self.gui = saopcontrolwindow

        def gui_callback(*args):
            cb_list = args

            def callback(fdata):
                if self.gui:
                    for cb in cb_list:
                        GLib.idle_add(cb)

            return callback

        the_cb = gui_callback(
            self.draw_geodatadisplay, self.gui.update_figure)

        self.elevation_map.update_callback = the_cb
        self.wildfire_map_predicted.update_callback = the_cb
        self.wildfire_map_observed.update_callback = the_cb
        self.uav_state.update_callback = the_cb

    def set_model_provider(self, hmi_node):
        """Set the HMI node retrieving information from ROS topics"""
        self.node = hmi_node

    def on_log(self, msg: Log):
        if self.gui is not None:
            GLib.idle_add(self.gui.append_log,
                          datetime.datetime.fromtimestamp(msg.header.stamp.to_sec()), msg.name,
                          msg.msg)

    def on_elevation_map(self, elevation_map_msg: ElevationMap):
        self.elevation_map.data = serialization.geodata_from_raster_msg(
            elevation_map_msg.raster, "elevation")

    def on_wildfire_map_predicted(self, wildfire_map_msg: PredictedWildfireMap):
        self.wildfire_map_predicted.data = serialization.geodata_from_raster_msg(
            wildfire_map_msg.raster, "ignition")

    def on_wildfire_map(self, wildfire_map_msg: WildfireMap):
        self.wildfire_map.data = serialization.geodata_from_raster_msg(
            wildfire_map_msg.raster, "ignition")

    def on_wildfire_map_observed(self, uav: str, wildfire_map_observed_msg: WildfireMap):
        gd = serialization.geodata_from_raster_msg(
            wildfire_map_observed_msg.raster, "ignition", invert=True)
        self.wildfire_map_observed.data = gd

    def on_uav_state(self, uav: str, vehicle_state_msg: VehicleState):
        if self.uav_state.updatable:
            self.uav_state.data[uav] = {"position": (vehicle_state_msg.position.x,
                                                     vehicle_state_msg.position.y,
                                                     vehicle_state_msg.position.z),
                                        "orientation": (vehicle_state_msg.orientation.phi,
                                                        vehicle_state_msg.orientation.theta,
                                                        vehicle_state_msg.orientation.psi)}
            self.uav_state.updated()
            if datetime.datetime.now() - self._last_uav_state_draw > datetime.timedelta(seconds=2.):
                if uav not in self.uav_trail.data:
                    self.uav_trail.data[uav] = collections.deque([], self._uav_trail_count)
                self.uav_trail.data[uav].appendleft(
                    (vehicle_state_msg.position.x, vehicle_state_msg.position.y))

    def on_plan(self, msg: Plan):
        if self.trajectories.updatable:
            self.trajectories.data = serialization.saop_trajectories_from_plan_msg(msg)
            self.trajectories.updated()

    def draw_geodatadisplay(self):
        if self.gdd_drawable:
            if self.gdd is None:
                if self.elevation_map is not None:
                    self.gdd = GeoDataDisplay.pyplot_figure(self.elevation_map.data, frame=(0., 0.))
                    self.gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
            else:
                self.gdd.clear_axis()
                if self.elevation_map and self.elevation_map.drawable:
                    self.gdd.draw_elevation_shade(self.elevation_map.data)
                if self.wildfire_map_observed and self.wildfire_map_observed.drawable:
                    self.gdd.draw_ignition_shade(self.wildfire_map_observed.data,
                                                 with_colorbar=False)
                if self.wildfire_map_predicted and self.wildfire_map_predicted.drawable:
                    self.gdd.draw_ignition_contour(self.wildfire_map_predicted.data,
                                                   with_labels=True)
                if self.wildfire_map and self.wildfire_map.drawable:
                    self.gdd.draw_ignition_contour(
                        self.wildfire_map.data, cmap="plasma", with_labels=True,
                        n_fronts=1,
                        time_range=(rospy.Time.now().to_sec() - 60, rospy.Time.now().to_sec() + 60))
                if self.uav_state and self.uav_state.drawable:
                    for uav_state, color in zip(self.uav_state.data.values(),
                                                cycle(self.uav_colors)):
                        self.gdd.draw_uav(uav_state["position"][0:2],
                                          uav_state["orientation"][2], facecolor=color)
                if self.uav_trail and self.uav_trail.drawable:
                    for trail, color in zip(self.uav_trail.data.values(), cycle(self.uav_colors)):
                        self.gdd.TrajectoryDisplayExtension.draw_waypoint_trail(trail, color=color)
                if self.trajectories and self.trajectories.drawable:
                    for traj, color in zip(self.trajectories.data, cycle(self.uav_colors)):
                        plot_trajectory(self.gdd, traj, ["bases", "trajectory_solid", "arrows"],
                                        color=color)

    def propagate_command(self):
        if self.node is not None:
            self.node.propagate()

    def loiter_command(self, uav: str):
        if self.node is not None:
            self.node.loiter(uav)

    def plan_command(self, vns_conf: str, planning_duration: float, mission_duration: float,
                     uavs: ty.Sequence[str], trajectory_duration: ty.Mapping[str, float],
                     start_delay: float):
        if self.node is not None:
            uav_stuff = {uav: {"start": self.uav_state.data[uav],
                               "end": self.uav_state.data[uav]} for uav in uavs}
            self.node.publish_plan_request(vns_conf, planning_duration, uav_stuff,
                                           mission_duration=mission_duration,
                                           uav_trajectory_duration=trajectory_duration,
                                           start_delay=start_delay)

    def stop_command(self, uav: str):
        if self.node is not None:
            self.node.stop(uav)


class HMINode:

    def __init__(self, model: HMIModel):
        rospy.init_node("hmi")
        self.elevation_map_cb = lambda x: None

        self.hmi_model = model
        self.hmi_model.set_model_provider(self)

        self.uavs = ('x8-02', 'x8-06')

        self.sub_rosout = rospy.Subscriber("rosout", Log, self._on_log, queue_size=10)
        self.sub_elevation_map = rospy.Subscriber("elevation", ElevationMap, self._on_elevation_map,
                                                  queue_size=10)
        self.sub_wildfire_map = rospy.Subscriber("wildfire_prediction", PredictedWildfireMap,
                                                 self._on_wildfire_map_predicted, queue_size=10)
        self.sub_wildfire_map = rospy.Subscriber("wildfire", WildfireMap,
                                                 self._on_wildfire_map, queue_size=10)

        self.sub_uav_state_dict = {uav: rospy.Subscriber(
            "/".join(("uavs", serialization.ros_name_for(uav), 'state')), VehicleState,
            callback=self._on_vehicle_state, callback_args=uav, queue_size=10) for uav in self.uavs}

        self.sub_firemap_obs_dict = {
            uav: rospy.Subscriber(
                "/".join(("uavs", serialization.ros_name_for(uav), 'wildfire_observed')),
                WildfireMap, callback=self._on_wildfire_map_observed, callback_args=uav,
                queue_size=10) for uav in self.uavs}

        self.sub_plan = rospy.Subscriber("plan", Plan, callback=self._on_plan, queue_size=10)

        self.pub_propagate = rospy.Publisher("propagate", PropagateCmd, queue_size=10)
        self.pub_initial_plan = rospy.Publisher("initial_plan", PlanCmd, queue_size=10)
        self.pub_stop = rospy.Publisher("stop", StopCmd, queue_size=10)

    def _on_log(self, msg: Log):
        self.hmi_model.on_log(msg)

    def _on_elevation_map(self, msg: ElevationMap):
        if self.hmi_model:
            self.hmi_model.on_elevation_map(msg)

    def _on_wildfire_map_predicted(self, msg: PredictedWildfireMap):
        if self.hmi_model:
            self.hmi_model.on_wildfire_map_predicted(msg)

    def _on_wildfire_map(self, msg: WildfireMap):
        if self.hmi_model:
            self.hmi_model.on_wildfire_map(msg)

    def _on_wildfire_map_observed(self, msg: WildfireMap, uav: str):
        if self.hmi_model:
            self.hmi_model.on_wildfire_map_observed(uav, msg)

    def _on_vehicle_state(self, msg: VehicleState, uav: str):
        if self.hmi_model:
            self.hmi_model.on_uav_state(uav, msg)

    def _on_plan(self, msg: Plan):
        if self.hmi_model:
            self.hmi_model.on_plan(msg)

    def propagate(self):
        self.pub_propagate.publish()

    def loiter(self, uav: str):
        raise NotImplementedError()

    def stop(self, uav: str):
        self.pub_stop.publish(StopCmd(uav))

    def publish_plan_request(self, vns_conf: str, planning_duration: float,
                             uavs: ty.Mapping[str, ty.Any], mission_duration: float = 10 * 60,
                             uav_trajectory_duration=None, start_delay: float = 0):
        if not uav_trajectory_duration:
            uav_trajectory_duration = {}
        rospy_now = rospy.Time.now()
        p_conf = PlanConf(
            name="A manual plan",
            flight_window=(rospy_now - rospy.Duration(5 * 10) + rospy.Duration(int(start_delay)),
                           rospy_now + rospy.Duration(int(mission_duration))))
        trajs = []
        for u_name, u_val in uavs.items():
            start_wp = PoseEuler(position=Point(*u_val["start"]["position"]),
                                 orientation=Euler(*u_val["start"]["orientation"]))
            end_wp = PoseEuler(position=Point(*u_val["end"]["position"]),
                               orientation=Euler(*u_val["end"]["orientation"]))

            max_duration = uav_trajectory_duration.get(u_name, 6000)

            t_conf = TrajectoryConf(name="-".join(("traj", u_name)), uav_model=u_name,
                                    start_wp=start_wp, end_wp=end_wp,
                                    start_time=rospy.Time.now() + rospy.Duration.from_sec(
                                        planning_duration) + rospy.Duration(int(start_delay)),
                                    max_duration=max_duration, wind=MeanWind(.0, .0))
            trajs.append(Trajectory(conf=t_conf, maneuvers=[]))
        p = Plan(header=Header(stamp=rospy.Time.now()), conf=p_conf, trajectories=trajs)

        plan_command = PlanCmd(vns_conf=vns_conf, planning_duration=planning_duration,
                               plan_prototype=p)
        self.pub_initial_plan.publish(plan_command)
        rospy.loginfo(plan_command)


class PlanCreationWindow(Gtk.Dialog):
    def __init__(self, parent, uavs: ty.Sequence[str]):
        Gtk.Dialog.__init__(self, title="New plan", transient_for=parent, modal=True,
                            destroy_with_parent=True, use_header_bar=True)

        self.set_default_size(150, 100)
        self.set_resizable(False)
        self.add_button(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)
        self.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)

        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, border_width=12)

        conf_grid = Gtk.Grid(orientation=Gtk.Orientation.HORIZONTAL, column_spacing=12,
                             row_spacing=6)

        # VNS conf
        vnsconf_label = Gtk.Label.new_with_mnemonic("VNS configuration:")
        vnsconf_label.set_halign(Gtk.Align.END)
        self.vnsconf_entry = Gtk.Entry(text="demo")
        self.vnsconf_entry.set_hexpand(True)

        # Planning time
        planning_duration_adjustment = Gtk.Adjustment(value=10, lower=5, upper=300,
                                                      step_increment=1, page_increment=10,
                                                      page_size=0)
        planning_duration_label = Gtk.Label.new_with_mnemonic("Planning duration (s):")
        planning_duration_label.set_halign(Gtk.Align.END)
        self.planning_duration_entry = Gtk.SpinButton()
        self.planning_duration_entry.set_hexpand(True)
        self.planning_duration_entry.set_adjustment(planning_duration_adjustment)
        self.planning_duration_entry.set_numeric(True)

        # Mission duration
        mission_duration_adjustment = Gtk.Adjustment(value=600, lower=60, upper=3600,
                                                     step_increment=30, page_increment=60,
                                                     page_size=0)
        mission_duration_label = Gtk.Label.new_with_mnemonic("Mission duration (s):")
        mission_duration_label.set_halign(Gtk.Align.END)
        self.mission_duration_entry = Gtk.SpinButton()
        self.mission_duration_entry.set_hexpand(True)
        self.mission_duration_entry.set_adjustment(mission_duration_adjustment)
        self.mission_duration_entry.set_numeric(True)

        # Mission start delay
        mission_delay_adjustment = Gtk.Adjustment(value=0, lower=0, upper=3600,
                                                  step_increment=10, page_increment=60,
                                                  page_size=0)
        mission_delay_label = Gtk.Label.new_with_mnemonic("Additional start delay (s):")
        mission_delay_label.set_halign(Gtk.Align.END)
        self.mission_delay_entry = Gtk.SpinButton()
        self.mission_delay_entry.set_hexpand(True)
        self.mission_delay_entry.set_adjustment(mission_delay_adjustment)
        self.mission_delay_entry.set_numeric(True)

        # UAV Checkbutton row
        uav_enabled_label = Gtk.Label.new_with_mnemonic("UAVs enabled:")
        uav_enabled_label.set_halign(Gtk.Align.END)
        self.uav_selection = {}
        uav_checkbutton_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL)
        uav_checkbutton_box.set_hexpand(True)
        Gtk.StyleContext.add_class(uav_checkbutton_box.get_style_context(), "linked")
        for uav in uavs:
            b = Gtk.CheckButton.new_with_label(uav)
            b.props.draw_indicator = False  # Display as a button
            b.connect("toggled", self._toggled_cb, uav)
            b.set_active(True)
            self.uav_selection[uav] = True
            uav_checkbutton_box.pack_start(b, True, True, 0)

        # UAV option notebook
        uav_conf_notebook = Gtk.Notebook()
        uav_conf_notebook.set_scrollable(True)
        self.uav_flight_duration_entry = {}
        for uav in uavs:
            page, wd = PlanCreationWindow.uav_conf_tab_content(uav)
            self.uav_flight_duration_entry[uav] = wd["duration_entry"]
            uav_conf_notebook.append_page(page, Gtk.Label.new_with_mnemonic(uav))

        # Placement of widgets
        conf_grid.attach(vnsconf_label, 0, 0, 1, 1)
        conf_grid.attach_next_to(self.vnsconf_entry, vnsconf_label,
                                 Gtk.PositionType.RIGHT, 1, 1)

        conf_grid.attach_next_to(planning_duration_label, vnsconf_label,
                                 Gtk.PositionType.BOTTOM, 1, 1)
        conf_grid.attach_next_to(self.planning_duration_entry, planning_duration_label,
                                 Gtk.PositionType.RIGHT, 1, 1)

        conf_grid.attach_next_to(mission_duration_label, planning_duration_label,
                                 Gtk.PositionType.BOTTOM, 1, 1)
        conf_grid.attach_next_to(self.mission_duration_entry, mission_duration_label,
                                 Gtk.PositionType.RIGHT, 1, 1)

        conf_grid.attach_next_to(mission_delay_label, mission_duration_label,
                                 Gtk.PositionType.BOTTOM, 1, 1)
        conf_grid.attach_next_to(self.mission_delay_entry, mission_delay_label,
                                 Gtk.PositionType.RIGHT, 1, 1)
        if uavs:
            conf_grid.attach_next_to(uav_enabled_label, mission_delay_label,
                                     Gtk.PositionType.BOTTOM, 1, 1)
            conf_grid.attach_next_to(uav_checkbutton_box, uav_enabled_label,
                                     Gtk.PositionType.RIGHT, 1, 1)

        content.pack_start(conf_grid, False, False, 0)
        if uavs:
            content.pack_start(uav_conf_notebook, False, False, 0)

        box = self.get_content_area()
        box.add(content)
        self.show_all()

    def _toggled_cb(self, button, uav_name):
        self.uav_selection[uav_name] = button.props.active

    @property
    def selected_uavs(self):
        return list(filter(lambda x: self.uav_selection[x], self.uav_selection.keys()))

    @property
    def trajectory_duration(self):
        return {k: self.uav_flight_duration_entry[k].get_value() for k in self.selected_uavs}

    @property
    def vns_conf(self):
        return self.vnsconf_entry.get_text()

    @property
    def planning_duration(self):
        return self.planning_duration_entry.get_value()

    @property
    def mission_duration(self):
        return self.mission_duration_entry.get_value()

    @property
    def mission_start_delay(self):
        return self.mission_delay_entry.get_value()

    @staticmethod
    def uav_conf_tab_content(uav: str):
        grid = Gtk.Grid(orientation=Gtk.Orientation.HORIZONTAL, column_spacing=12, row_spacing=6)
        grid.set_border_width(12)
        # Max flight duration
        max_flight_duration_adjustement = Gtk.Adjustment(value=600, lower=60, upper=3600,
                                                         step_increment=30, page_increment=60,
                                                         page_size=0)
        max_flight_duration_label = Gtk.Label.new_with_mnemonic("Max. flight duration (s):")
        max_flight_duration_label.set_halign(Gtk.Align.END)
        max_flight_duration_entry = Gtk.SpinButton()
        max_flight_duration_entry.set_hexpand(True)
        max_flight_duration_entry.set_adjustment(max_flight_duration_adjustement)
        max_flight_duration_entry.set_numeric(True)

        grid.attach(max_flight_duration_label, 0, 0, 1, 1)
        grid.attach_next_to(max_flight_duration_entry, max_flight_duration_label,
                            Gtk.PositionType.RIGHT, 1, 1)

        return grid, {"duration_entry": max_flight_duration_entry}


class SAOPControlWindow(Gtk.Window):

    def __init__(self, model: HMIModel):
        super(SAOPControlWindow, self).__init__(
            default_width=500, default_height=400, title="SAOP HMI")

        self.hmi_model = model
        self.hmi_model.set_controller(self)

        ## Actions
        self.plan_button = Gtk.Button(label="Create")
        self.plan_button.connect("clicked", self.on_plan_clicked)

        self.cancel_button = Gtk.Button(label="Stop")
        self.cancel_button.connect("clicked", self.on_cancel_clicked)

        self.propagate_button = Gtk.Button(label="Propagate")
        self.propagate_button.connect("clicked", self.on_propagate_clicked)

        self.action_bar = Gtk.ActionBar()
        self.action_bar.pack_start(Gtk.Label("Plan:"))
        self.action_bar.pack_start(self.plan_button)
        self.action_bar.pack_start(self.cancel_button)
        self.action_bar.pack_start(Gtk.Separator(orientation=Gtk.Orientation.VERTICAL))
        self.action_bar.pack_start(Gtk.Label("Wildfire:"))
        self.action_bar.pack_start(self.propagate_button)

        ## Visualization control
        # Empty drawing area that will be replaced by a matplotlib figure
        default_fig = matplotlib.figure.Figure()
        self.canvas = FigureCanvas(default_fig)
        self.navigation_toolbar = NavigationToolbar(self.canvas, self)

        # Visualization Checkboxes
        scrolled_flowbox = Gtk.ScrolledWindow()
        scrolled_flowbox.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)

        flowbox = Gtk.FlowBox()
        flowbox.set_valign(Gtk.Align.START)
        flowbox.set_max_children_per_line(30)
        flowbox.set_selection_mode(Gtk.SelectionMode.NONE)

        # Checkboxes
        def toggle_pause(button):
            self.set_update_plot(not button.props.active)

        pause_toggle = Gtk.ToggleButton("Pause")
        pause_toggle.props.active = False
        pause_toggle.connect("toggled", toggle_pause)
        pause_toggle.toggled()
        flowbox.add(pause_toggle)

        display_label = Gtk.Label("Display:")
        flowbox.add(display_label)

        def toggle_elevation(button):
            self.hmi_model.elevation_map.drawable = button.get_active()

        elevation_toggle = SAOPControlWindow._check_button_with_action("Elevation",
                                                                       toggle_elevation)
        flowbox.add(elevation_toggle)

        def toggle_wildfire(button):
            self.hmi_model.wildfire_map.drawable = button.get_active()

        wildfire_toggle = SAOPControlWindow._check_button_with_action("Wildfire", toggle_wildfire)
        flowbox.add(wildfire_toggle)

        def toggle_wildfire_observed(button):
            self.hmi_model.wildfire_map_observed.drawable = button.get_active()

        wildfire_observed_toggle = SAOPControlWindow._check_button_with_action(
            "Observed wildfire", toggle_wildfire_observed)
        flowbox.add(wildfire_observed_toggle)

        def toggle_wildfire_predicted(button):
            self.hmi_model.wildfire_map_predicted.drawable = button.get_active()

        wildfire_predicted_toggle = SAOPControlWindow._check_button_with_action(
            "Predicted wildfire", toggle_wildfire_predicted)
        flowbox.add(wildfire_predicted_toggle)

        def toggle_uav(button):
            self.hmi_model.uav_state.drawable = button.get_active()

        uav_toggle = SAOPControlWindow._check_button_with_action("UAV", toggle_uav)
        flowbox.add(uav_toggle)

        def toggle_trails(button):
            self.hmi_model.uav_trail.drawable = button.get_active()

        trail_toggle = SAOPControlWindow._check_button_with_action("UAV trails", toggle_trails)
        flowbox.add(trail_toggle)

        def toggle_plan(button):
            self.hmi_model.trajectories.drawable = button.get_active()

        plan_toggle = SAOPControlWindow._check_button_with_action("Plan", toggle_plan)
        flowbox.add(plan_toggle)

        scrolled_flowbox.add(flowbox)

        self.figure_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0, border_width=0)
        self.figure_box.pack_start(self.navigation_toolbar, False, True, 0)
        self.figure_box.pack_start(self.canvas, True, True, 0)
        self.figure_box.pack_start(scrolled_flowbox, False, False, 0)

        ## Side panel
        right_panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        # View of ROS logs
        log_textview = Gtk.TextView()
        log_textview.set_editable(False)
        log_textview.set_monospace(True)
        log_textview.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.log_textbuffer = log_textview.get_buffer()
        self.tag_bold = self.log_textbuffer.create_tag("bold", weight=Pango.Weight.BOLD)
        log_scrolled = Gtk.ScrolledWindow()
        log_scrolled.add(log_textview)
        right_panel.pack_start(log_scrolled, True, True, 0)

        # UAV controls

        # self.content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6,
        #                            border_width=12)
        self.content_panned = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        self.content_panned.pack1(self.figure_box, True, True)
        self.content_panned.pack2(right_panel, False, True)

        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0, border_width=0)
        box.pack_start(self.content_panned, True, True, 0)
        box.pack_start(self.action_bar, False, False, 0)
        self.add(box)

    @staticmethod
    def _check_button_with_action(name, action):
        toggle = Gtk.CheckButton(name)
        toggle.props.active = True
        toggle.connect("toggled", action)
        toggle.toggled()
        return toggle

    def set_update_plot(self, state: bool):
        self.hmi_model.gdd_drawable = state

    def append_log(self, t: datetime.datetime, name: str, msg: str):
        self._append_text_with_markup(
            "<b>{}</b> <i>{}</i>: <tt>{}</tt>\n".format(datetime.datetime.strftime(t, "%x %X"),
                                                        str(name), str(msg)))

    def _append_text_with_markup(self, text):
        end_iter = self.log_textbuffer.get_end_iter()
        self.log_textbuffer.insert_markup(end_iter, text, -1)

    def set_uav_controls(self):
        """Update UAV control panel"""
        pass

    def update_figure(self):
        self._update_figure()

    def _update_figure(self):
        """Redraw the matplotlib figure"""
        if self.hmi_model.gdd:
            self.canvas.figure = self.hmi_model.gdd.figure  # a Gtk.DrawingArea
            self.canvas.draw()  # TODO: verify whether we need it or not

    def on_propagate_clicked(self, button):
        self.hmi_model.propagate_command()

    def on_plan_clicked(self, button):
        known_uavs = self.hmi_model.known_uavs()
        if known_uavs:
            pdiag = PlanCreationWindow(self, known_uavs)
            if pdiag.run() == Gtk.ResponseType.OK:
                GLib.idle_add(self.hmi_model.plan_command, pdiag.vns_conf, pdiag.planning_duration,
                              pdiag.mission_duration, pdiag.uav_selection,
                              pdiag.trajectory_duration, pdiag.mission_start_delay)
            pdiag.destroy()
        else:
            dialog = Gtk.MessageDialog(self, 0, Gtk.MessageType.INFO,
                                       Gtk.ButtonsType.OK, "No UAVs available for planning")
            dialog.format_secondary_text(
                "Wait for UAVs to be detected or check Neptus and Dune are running")
            dialog.set_transient_for(self)
            dialog.run()
            dialog.destroy()

    def on_cancel_clicked(self, button):
        for uav in self.hmi_model.known_uavs():
            self.hmi_model.stop_command(uav)


if __name__ == "__main__":

    def ros_loop():
        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            pass


    # Model
    themodel = HMIModel()

    # View
    win = SAOPControlWindow(themodel)

    # Provider
    node = HMINode(themodel)

    win.show_all()
    win.connect("delete-event", Gtk.main_quit)

    th = threading.Thread(None, ros_loop, daemon=True)
    th.start()

    Gtk.main()
