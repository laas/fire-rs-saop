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

import matplotlib.figure

import gi

gi.require_version('Gtk', '3.0')
from gi.repository import Gio, GLib, Gtk, Pango
from matplotlib.backends.backend_gtk3cairo import FigureCanvasGTK3Cairo as FigureCanvas
from matplotlib.backends.backend_gtk3 import NavigationToolbar2GTK3 as NavigationToolbar

import rospy
from rosgraph_msgs.msg import Log
from supersaop.msg import ElevationMap, PredictedWildfireMap, PropagateCmd, Raster, RasterMetaData, \
    WildfireMap, MeanWindStamped, SurfaceWindMap, Timed2DPointStamped

from fire_rs.geodata.geo_data import GeoData
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.monitoring.supersaop import SituationAssessment

import serialization


class HMIModel:
    def __init__(self):
        self.gui = None
        self.node = None

        self.elevation_map = None  # type: ty.Optional[GeoData]
        self.wildfire_map = None  # type: ty.Optional[GeoData]

        self.uav_state = {}

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
        if self.gui is not None:
            self.elevation_map = serialization.geodata_from_raster_msg(elevation_map_msg.raster,
                                                                       "elevation")
            GLib.idle_add(self.gui.update_elevation_map)

    def on_wildfire_map(self, wildfire_map_msg: PredictedWildfireMap):
        if self.gui is not None:
            self.wildfire_map = serialization.geodata_from_raster_msg(wildfire_map_msg.raster,
                                                                      "ignition")
            GLib.idle_add(self.gui.update_wildfire_map)

    def propagate_command(self):
        if self.node is not None:
            self.node.propagate()


class HMINode:

    def __init__(self, model: HMIModel):
        rospy.init_node("hmi")
        self.elevation_map_cb = lambda x: None

        self.hmi_model = model
        self.hmi_model.set_model_provider(self)

        self.sub_rosout = rospy.Subscriber("rosout", Log, self._on_log, queue_size=10)
        self.sub_elevation_map = rospy.Subscriber("elevation", ElevationMap, self._on_elevation_map,
                                                  queue_size=10)
        self.sub_wildfire_map = rospy.Subscriber("wildfire_prediction", PredictedWildfireMap,
                                                 self._on_wildfire_map, queue_size=10)

        self.pub_propagate = rospy.Publisher("propagate", PropagateCmd, queue_size=10)

    def _on_log(self, msg: Log):
        self.hmi_model.on_log(msg)

    def _on_elevation_map(self, msg: ElevationMap):
        if self.elevation_map_cb:
            self.hmi_model.on_elevation_map(msg)

    def _on_wildfire_map(self, msg: PredictedWildfireMap):
        if self.elevation_map_cb:
            self.hmi_model.on_wildfire_map(msg)

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

        self.figure_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0, border_width=0)
        self.figure_box.pack_start(self.navigation_toolbar, False, True, 0)
        self.figure_box.pack_start(self.canvas, True, True, 0)

        # View of ROS logs
        log_textview = Gtk.TextView()
        log_textview.set_editable(False)
        log_textview.set_monospace(True)
        log_textview.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self.log_textbuffer = log_textview.get_buffer()
        self.tag_bold = self.log_textbuffer.create_tag("bold", weight=Pango.Weight.BOLD)
        log_scrolled = Gtk.ScrolledWindow()
        log_scrolled.add(log_textview)

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

    def update_uav_state(self):
        self._redraw_figure()

    def update_elevation_map(self):
        self._redraw_figure()

    def update_wildfire_map(self):
        self._redraw_figure()

    def _redraw_figure(self):
        """Redraw the matplotlib figure"""
        # Redraw
        gdd = None
        if self.model.elevation_map is not None:
            gdd = GeoDataDisplay.pyplot_figure(self.model.elevation_map, frame=(0, 0))
            gdd.draw_elevation_shade()
        if self.model.wildfire_map is not None:
            if gdd is None:
                gdd = GeoDataDisplay.pyplot_figure(self.model.wildfire_map, frame=(0, 0))
            gdd.draw_ignition_contour(self.model.wildfire_map, with_labels=True)

        # Update the Gtk object
        # self.figure_box.remove(self.canvas)
        self.canvas.figure = gdd.figure  # a Gtk.DrawingArea
        # self.figure_box.pack_end(self.canvas, True, True, 0)
        # self.navigation_toolbar.canvas = self.canvas
        # self.canvas.show()
        self.canvas.queue_draw()  # TODO: verifiy whether we need it or not

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
    model = HMIModel()

    # View
    win = SAOPControlWindow(model)

    # Provider
    node = HMINode(model)

    win.show_all()
    win.connect("delete-event", Gtk.main_quit)

    th = threading.Thread(None, ros_loop, daemon=True)
    th.start()

    Gtk.main()