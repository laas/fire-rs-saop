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
from supersaop.msg import ElevationMap, PredictedWildfireMap, PropagateCmd, Raster, RasterMetaData, \
    WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped, VehicleState

from fire_rs.geodata.geo_data import GeoData
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.monitoring.supersaop import SituationAssessment

import serialization


class HMIModel:
    def __init__(self):
        self.gui = None
        self.node = None

        self.gdd = None
        self.gdd_update_enabled = True

        self.elevation_map = None  # type: ty.Optional[GeoData]
        self.elevation_map_enabled = True
        self.wildfire_map = None  # type: ty.Optional[GeoData]
        self.wildfire_map_enabled = True

        self.uav_state_dict = {}
        self.last_uav_state_draw = datetime.datetime.min
        self.uav_state_enabled = True

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
        self.elevation_map = serialization.geodata_from_raster_msg(elevation_map_msg.raster,
                                                                   "elevation")
        GLib.idle_add(self._update_geodatadisplay)
        if self.gui is not None:
            GLib.idle_add(self.gui.update_elevation_map)

    def on_wildfire_map(self, wildfire_map_msg: PredictedWildfireMap):
        self.wildfire_map = serialization.geodata_from_raster_msg(wildfire_map_msg.raster,
                                                                  "ignition")
        GLib.idle_add(self._update_geodatadisplay)
        if self.gui is not None:
            GLib.idle_add(self.gui.update_wildfire_map)

    def on_uav_state(self, uav: str, vehicle_state_msg: VehicleState):
        self.uav_state_dict[uav] = {"position": (vehicle_state_msg.position.x,
                                                 vehicle_state_msg.position.y,
                                                 vehicle_state_msg.position.z),
                                    "orientation": (vehicle_state_msg.orientation.phi,
                                                    vehicle_state_msg.orientation.theta,
                                                    vehicle_state_msg.orientation.psi)}
        if datetime.datetime.now() - self.last_uav_state_draw > datetime.timedelta(seconds=2.):
            GLib.idle_add(self._update_geodatadisplay)
            if self.gui is not None:
                GLib.idle_add(self.gui.update_uav_state, uav)

    def _update_geodatadisplay(self):
        if self.gdd_update_enabled:
            if self.gdd is None:
                if self.elevation_map is not None:
                    self.gdd = GeoDataDisplay.pyplot_figure(self.elevation_map, frame=(0, 0))
            else:
                self.gdd.axes.cla()
                if self.elevation_map_enabled and self.elevation_map is not None:
                    self.gdd.draw_elevation_shade(self.elevation_map)
                if self.wildfire_map_enabled and self.wildfire_map is not None:
                    self.gdd.draw_ignition_contour(self.wildfire_map, with_labels=True)
                    # self.gdd.draw_ignition_shade(self.wildfire_map, with_colorbar=True)
                if self.uav_state_enabled:
                    for uav_state in self.uav_state_dict.values():
                        self.gdd.draw_uav(uav_state["position"][0:2],
                                          uav_state["orientation"][2])

    def propagate_command(self):
        if self.node is not None:
            self.node.propagate()


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

        self.pub_propagate = rospy.Publisher("propagate", PropagateCmd, queue_size=10)

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

    def propagate(self):
        self.pub_propagate.publish()


class PlanCreationWindow(Gtk.Dialog):
    def __init__(self, parent):
        Gtk.Dialog.__init__(self, title="New plan", parent=parent, modal=True,
                            destroy_with_parent=True, use_header_bar=True)

        self.set_default_size(150, 100)
        self.add_button(Gtk.STOCK_CANCEL, Gtk.ResponseType.CANCEL)
        self.add_button(Gtk.STOCK_OK, Gtk.ResponseType.OK)

        label = Gtk.Label("This is a dialog to display additional information")

        box = self.get_content_area()
        box.add(label)
        self.show_all()


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

        self.action_bar = Gtk.ActionBar()
        self.action_bar.pack_start(self.propagate_button)
        self.action_bar.pack_start(self.cancel_button)

        ## Visualization
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
            self.hmi_model.elevation_map_enabled = button.get_active()

        elevation_toggle.connect("toggled", toggle_elevation)
        elevation_toggle.toggled()
        flowbox.add(elevation_toggle)

        wildfire_toggle = Gtk.CheckButton("Wildfire")
        wildfire_toggle.props.active = True

        def toggle_wildfire(button):
            self.hmi_model.wildfire_map_enabled = button.get_active()

        wildfire_toggle.connect("toggled", toggle_wildfire)
        wildfire_toggle.toggled()
        flowbox.add(wildfire_toggle)

        uav_toggle = Gtk.CheckButton("UAVs")
        uav_toggle.props.active = True

        def toggle_uav(button):
            self.hmi_model.uav_state_enabled = button.get_active()

        uav_toggle.connect("toggled", toggle_uav)
        uav_toggle.toggled()
        flowbox.add(uav_toggle)

        scrolled_flowbox.add(flowbox)

        self.figure_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0, border_width=0)
        self.figure_box.pack_start(self.navigation_toolbar, False, True, 0)
        self.figure_box.pack_start(self.canvas, True, True, 0)
        self.figure_box.pack_start(scrolled_flowbox, False, False, 0)

        # View of ROS logs
        log_textview = Gtk.TextView()
        log_textview.set_editable(False)
        log_textview.set_monospace(True)
        log_textview.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.log_textbuffer = log_textview.get_buffer()
        self.tag_bold = self.log_textbuffer.create_tag("bold", weight=Pango.Weight.BOLD)
        log_scrolled = Gtk.ScrolledWindow()
        log_scrolled.add(log_textview)

        # UAV controls

        # self.content_box = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6,
        #                            border_width=12)
        self.content_panned = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        self.content_panned.pack1(self.figure_box, True, True)
        self.content_panned.pack2(log_scrolled, False, True)

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

    def update_uav_state(self, uav: str):
        self._redraw_figure()

    def update_elevation_map(self):
        self._redraw_figure()

    def update_wildfire_map(self):
        self._redraw_figure()

    def _redraw_figure(self):
        """Redraw the matplotlib figure"""
        if self.hmi_model.gdd:
            self.canvas.figure = self.hmi_model.gdd.figure  # a Gtk.DrawingArea
            self.canvas.draw()  # TODO: verify whether we need it or not

    def on_propagate_clicked(self, button):
        self.hmi_model.propagate_command()

    def on_plan_clicked(self, button):
        pdiag = PlanCreationWindow(self)
        if pdiag.run() == Gtk.ResponseType.OK:
            print("hello")
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
