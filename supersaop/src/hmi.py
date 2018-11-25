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

import datetime
import logging
import queue
import time
import threading
import typing as ty

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
    Trajectory, TrajectoryConf, PlanConf, PoseEuler, Euler, MeanWind

from fire_rs.geodata.geo_data import GeoData
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.planning.display import TrajectoryDisplayExtension, plot_trajectory

import serialization


class HMIModel:
    def __init__(self):
        self.gui = None
        self.node = None

        self.gdd = None
        self.gdd_drawable = True

        # Model data
        # _updatable controls whether new values are stored when updates arrive
        # _drawable controls whether the field is displayed over the figure

        self.elevation_map = None  # type: ty.Optional[GeoData]
        self.elevation_map_updatable = True
        self.elevation_map_drawable = True

        self.wildfire_map = None  # type: ty.Optional[GeoData]
        self.wildfire_map_updatable = True
        self.wildfire_map_drawable = True

        self.uav_state_dict = {}
        self.last_uav_state_draw = datetime.datetime.min
        self.uav_state_updatable = True
        self.uav_state_drawable = True

        self.trajectories = []
        self.trajectories_updatable = True
        self.trajectories_drawable = True

    def set_controller(self, saopcontrolwindow):
        """Set the GUI that is controlling stuff"""
        self.gui = saopcontrolwindow

    def set_model_provider(self, hmi_node):
        """Set the HMI node retrieving information from ROS topics"""
        self.node = hmi_node

    def on_log(self, msg: Log):
        if self.gui is not None:
            GLib.idle_add(self.gui.append_log,
                          datetime.datetime.fromtimestamp(msg.header.stamp.to_sec()), msg.name,
                          msg.msg)

    def on_elevation_map(self, elevation_map_msg: ElevationMap):
        if self.elevation_map_updatable:
            self.elevation_map = serialization.geodata_from_raster_msg(elevation_map_msg.raster,
                                                                       "elevation")
            if self.gui is not None:
                GLib.idle_add(self._draw_geodatadisplay)
                GLib.idle_add(self.gui.update_elevation_map)

    def on_wildfire_map(self, wildfire_map_msg: PredictedWildfireMap):
        if self.wildfire_map_updatable:
            self.wildfire_map = serialization.geodata_from_raster_msg(wildfire_map_msg.raster,
                                                                      "ignition")
            if self.gui is not None:
                GLib.idle_add(self._draw_geodatadisplay)
                GLib.idle_add(self.gui.update_wildfire_map)

    def on_uav_state(self, uav: str, vehicle_state_msg: VehicleState):
        if self.uav_state_updatable:
            self.uav_state_dict[uav] = {"position": (vehicle_state_msg.position.x,
                                                     vehicle_state_msg.position.y,
                                                     vehicle_state_msg.position.z),
                                        "orientation": (vehicle_state_msg.orientation.phi,
                                                        vehicle_state_msg.orientation.theta,
                                                        vehicle_state_msg.orientation.psi)}
            if datetime.datetime.now() - self.last_uav_state_draw > datetime.timedelta(seconds=2.):
                if self.gui is not None:
                    GLib.idle_add(self._draw_geodatadisplay)
                    GLib.idle_add(self.gui.update_uav_state, uav)

    def on_plan(self, msg: Plan):
        if self.trajectories_updatable:
            self.trajectories = serialization.saop_trajectories_from_plan_msg(msg)

    def _draw_geodatadisplay(self):
        if self.gdd_drawable:
            if self.gdd is None:
                if self.elevation_map is not None:
                    self.gdd = GeoDataDisplay.pyplot_figure(self.elevation_map, frame=(0, 0))
                    self.gdd.add_extension(TrajectoryDisplayExtension, (None,), {})
            else:
                self.gdd.axes.cla()
                if self.elevation_map_drawable and self.elevation_map is not None:
                    self.gdd.draw_elevation_shade(self.elevation_map)
                if self.wildfire_map_drawable and self.wildfire_map is not None:
                    self.gdd.draw_ignition_contour(self.wildfire_map, with_labels=True)
                    # self.gdd.draw_ignition_shade(self.wildfire_map, with_colorbar=True)
                if self.uav_state_drawable:
                    for uav_state in self.uav_state_dict.values():
                        self.gdd.draw_uav(uav_state["position"][0:2],
                                          uav_state["orientation"][2])
                if self.trajectories_drawable:
                    for traj in self.trajectories:
                        plot_trajectory(self.gdd, traj, ["bases", "trajectory_solid", "arrows"])

    def propagate_command(self):
        if self.node is not None:
            self.node.propagate()

    def loiter_command(self, uav: str):
        if self.node is not None:
            self.node.loiter(uav)

    def plan_command(self, vns_conf: str, planning_duration: float, uavs: ty.Sequence[str]):
        if self.node is not None:
            uav_stuff = {uav: {"start": self.uav_state_dict[uav],
                               "end": self.uav_state_dict[uav]} for uav in uavs}
            self.node.publish_plan_request(vns_conf, planning_duration, uav_stuff)


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
                                                 self._on_wildfire_map, queue_size=10)
        self.sub_wildfire_map = rospy.Subscriber("wildfire_prediction", PredictedWildfireMap,
                                                 self._on_wildfire_map, queue_size=10)

        self.sub_uav_state_dict = {uav: rospy.Subscriber(
            "/".join(("uavs", serialization.ros_name_for(uav), 'state')), VehicleState,
            callback=self._on_vehicle_state, callback_args=uav, queue_size=10) for uav in self.uavs}

        self.sub_plan = rospy.Subscriber("plan", Plan, callback=self._on_plan, queue_size=10)

        self.pub_propagate = rospy.Publisher("propagate", PropagateCmd, queue_size=10)
        self.pub_initial_plan = rospy.Publisher("initial_plan", PlanCmd, queue_size=10)

    def _on_log(self, msg: Log):
        self.hmi_model.on_log(msg)

    def _on_elevation_map(self, msg: ElevationMap):
        if self.hmi_model:
            self.hmi_model.on_elevation_map(msg)

    def _on_wildfire_map(self, msg: PredictedWildfireMap):
        if self.hmi_model:
            self.hmi_model.on_wildfire_map(msg)

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

    def publish_plan_request(self, vns_conf: str, planning_duration: float,
                             uavs: ty.Mapping[str, ty.Any]):

        p_conf = PlanConf(name="A manual plan",
                          flight_window=(rospy.Time.from_sec(.0), rospy.Time.from_sec(4294967295)))
        trajs = []
        for u_name, u_val in uavs.items():
            start_wp = PoseEuler(position=Point(*u_val["start"]["position"]),
                                 orientation=Euler(*u_val["start"]["orientation"]))
            end_wp = PoseEuler(position=Point(*u_val["end"]["position"]),
                               orientation=Euler(*u_val["end"]["orientation"]))

            t_conf = TrajectoryConf(name="-".join(("traj", u_name)), uav_model=u_name,
                                    start_wp=start_wp, end_wp=end_wp,
                                    start_time=rospy.Time.now() + rospy.Duration.from_sec(
                                        planning_duration),
                                    max_duration=6000, wind=MeanWind(.0, .0))
            trajs.append(Trajectory(conf=t_conf, maneuvers=[]))
        p = Plan(header=Header(stamp=rospy.Time.now()), conf=p_conf, trajectories=trajs)

        plan_command = PlanCmd(vns_conf=vns_conf, planning_duration=planning_duration,
                               plan_prototype=p)
        self.pub_initial_plan.publish(plan_command)
        rospy.loginfo(plan_command)


class PlanCreationWindow(Gtk.Dialog):
    def __init__(self, parent, uavs: ty.Sequence[str]):
        Gtk.Dialog.__init__(self, title="New plan", parent=parent, modal=True,
                            destroy_with_parent=True, use_header_bar=True)

        self.set_default_size(150, 100)
        self.add_button(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)
        self.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)

        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6, border_width=12)

        vnsconf_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6, border_width=0)
        vnsconf_label = Gtk.Label("VNS configuration")
        self.vnsconf_entry = Gtk.Entry(text="demo")
        vnsconf_box.pack_start(vnsconf_label, False, False, 0)
        vnsconf_box.pack_start(self.vnsconf_entry, True, True, 0)

        duration_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6, border_width=0)
        duration_adjustment = Gtk.Adjustment(30, 5, 300, 1, 10, 0)
        duration_label = Gtk.Label("Planning duration")
        self.duration_entry = Gtk.SpinButton(text="demo")
        self.duration_entry.set_adjustment(duration_adjustment)
        duration_box.pack_start(duration_label, False, False, 0)
        duration_box.pack_start(self.duration_entry, True, True, 0)

        flowbox = Gtk.FlowBox()
        flowbox.set_valign(Gtk.Align.START)
        flowbox.set_max_children_per_line(3)
        flowbox.set_selection_mode(Gtk.SelectionMode.NONE)

        # UAV Checkboxes
        self.uav_selection = {}
        for uav in uavs:
            b = Gtk.CheckButton(uav)
            b.props.active = True
            self.uav_selection[uav] = True
            b.connect("toggled", self._toggled_cb, uav)
            flowbox.add(b)

        content.pack_start(vnsconf_box, False, False, 0)
        content.pack_start(duration_box, False, False, 0)
        content.pack_start(flowbox, False, False, 0)

        box = self.get_content_area()
        box.add(content)
        self.show_all()

    def _toggled_cb(self, button, uav_name):
        self.uav_selection[uav_name] = button.props.active

    @property
    def selected_uavs(self):
        return [filter(lambda x: self.uav_selection[x], self.uav_selection.keys())]

    @property
    def vns_conf(self):
        return self.vnsconf_entry.get_text()

    @property
    def planning_duration(self):
        return self.duration_entry.get_value()


class SAOPControlWindow(Gtk.Window):

    def __init__(self, model: HMIModel):
        super(SAOPControlWindow, self).__init__(
            default_width=500, default_height=400, title="SAOP HMI")

        self.hmi_model = model
        self.hmi_model.set_controller(self)

        ## Actions
        self.cancel_button = Gtk.Button(label="Plan")
        self.cancel_button.connect("clicked", self.on_plan_clicked)

        self.propagate_button = Gtk.Button(label="Propagate")
        self.propagate_button.connect("clicked", self.on_propagate_clicked)

        self.propagate_button = Gtk.Button(label="Loiter")
        self.propagate_button.connect("clicked", self.on_propagate_clicked)

        self.action_bar = Gtk.ActionBar()
        self.action_bar.pack_start(self.propagate_button)
        self.action_bar.pack_start(self.cancel_button)

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
        elevation_toggle = Gtk.CheckButton("Elevation")
        elevation_toggle.props.active = True

        def toggle_elevation(button):
            self.hmi_model.elevation_map_drawable = button.get_active()

        elevation_toggle.connect("toggled", toggle_elevation)
        elevation_toggle.toggled()
        flowbox.add(elevation_toggle)

        wildfire_toggle = Gtk.CheckButton("Wildfire")
        wildfire_toggle.props.active = True

        def toggle_wildfire(button):
            self.hmi_model.wildfire_map_drawable = button.get_active()

        wildfire_toggle.connect("toggled", toggle_wildfire)
        wildfire_toggle.toggled()
        flowbox.add(wildfire_toggle)

        uav_toggle = Gtk.CheckButton("UAVs")
        uav_toggle.props.active = True

        def toggle_uav(button):
            self.hmi_model.uav_state_drawable = button.get_active()

        uav_toggle.connect("toggled", toggle_uav)
        uav_toggle.toggled()
        flowbox.add(uav_toggle)

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

    def append_log(self, t: datetime.datetime, name: str, msg: str):
        self._append_text_with_markup(
            "<b>{}</b> <i>{}</i>: <tt>{}</tt>\n".format(datetime.datetime.strftime(t, "%x %X"),
                                                        str(name), str(msg)))

    def _append_text_with_markup(self, text):
        end_iter = self.log_textbuffer.get_end_iter()
        self.log_textbuffer.insert_markup(end_iter, text, -1)

    def set_uav_controls(self):
        """Update UAV control panel"""

    def update_uav_state(self, uav: str):
        self._update_figure()

    def update_elevation_map(self):
        self._update_figure()

    def update_wildfire_map(self):
        self._update_figure()

    def _update_figure(self):
        """Redraw the matplotlib figure"""
        if self.hmi_model.gdd:
            self.canvas.figure = self.hmi_model.gdd.figure  # a Gtk.DrawingArea
            self.canvas.draw()  # TODO: verify whether we need it or not

    def on_propagate_clicked(self, button):
        self.hmi_model.propagate_command()

    def on_plan_clicked(self, button):
        pdiag = PlanCreationWindow(self, tuple(self.hmi_model.uav_state_dict.keys()))
        if pdiag.run() == Gtk.ResponseType.OK:
            GLib.idle_add(self.hmi_model.plan_command, pdiag.vns_conf, pdiag.planning_duration,
                          pdiag.uav_selection)
        pdiag.destroy()


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
