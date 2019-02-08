# Copyright (c) 2017, CNRS-LAAS
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

"""Simple demo fro LSTS showing a sample of what is expected be be displayed on neptus"""
import datetime

import skimage.measure
import skimage.draw

import numpy as np

import matplotlib.cm
import matplotlib.pyplot as plt

from fire_rs.firemodel import propagation
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.geodata.geo_data import TimedPoint, GeoData

if __name__ == "__main__":
    output_format = "svg"

    # Uncomment below to use latex for typesetting
    # matplotlib.rcParams['font.family'] = 'sans-serif'
    # matplotlib.rcParams['text.usetex'] = True
    # output_format="pdf"

    area = ((481000.0, 484000.0), (6211000.0, 6213500.0))

    expected_env = propagation.Environment(area, wind_speed=5., wind_dir=0.)
    now = datetime.datetime.now().timestamp()
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 1000.0, now)
    expected_fire = propagation.propagate_from_points(
        expected_env, ignition_point, now + datetime.timedelta(hours=2).total_seconds())

    env = propagation.Environment(area, wind_speed=6., wind_dir=0.)
    fire = propagation.propagate_from_points(
        env, ignition_point, now + datetime.timedelta(hours=2).total_seconds())

    # Create fire perimeter geodata
    fire_perimeter = GeoData.full_like(fire.prop_data.slice("ignition"), np.nan)
    # Create contour with scikit-image
    contours = skimage.measure.find_contours(fire.prop_data.data["ignition"],
                                             now + datetime.timedelta(hours=1).total_seconds())
    # Draw contour in the geodata
    for contour in contours:
        for pt_i in range(len(contour)):
            if pt_i < len(contour) / 3:
                continue
            rr, cc = skimage.draw.line(*np.asarray(contour[pt_i - 1], dtype=int),
                                       *np.asarray(contour[pt_i], dtype=int))
            fire_perimeter.data[rr, cc] = fire.prop_data.data["ignition"][rr, cc]

    contours = skimage.measure.find_contours(fire.prop_data.data["ignition"],
                                             now + datetime.timedelta(minutes=30).total_seconds())
    # Draw contour in the geodata
    for contour in contours:
        for pt_i in range(len(contour)):
            rr, cc = skimage.draw.line(*np.asarray(contour[pt_i - 1], dtype=int),
                                       *np.asarray(contour[pt_i], dtype=int))
            fire_perimeter.data[rr, cc] = fire.prop_data.data["ignition"][rr, cc]

    # Figure terrain + ignition contour + ignition point
    gdd = GeoDataDisplay.pyplot_figure(expected_env.raster, frame=(0., 0.))
    # gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
    # # gdd.draw_wind_quiver()
    expected_fire.prop_data.data["ignition"][expected_fire.prop_data.data["ignition"] < now + datetime.timedelta(hours=1).total_seconds()] = now + datetime.timedelta(hours=1).total_seconds()
    gdd.draw_ignition_contour(geodata=fire.prop_data, with_labels=True,
                              cmap=matplotlib.cm.plasma)
    gdd.draw_ignition_shade(geodata=fire_perimeter, with_colorbar=True, cmap=matplotlib.cm.Reds)

    # gdd.draw_ignition_points(ignition_point)

    gdd.figure.show()

    gdd.figure.savefig(".".join(("demo_propagation", output_format)), dpi=400, bbox_inches='tight')
