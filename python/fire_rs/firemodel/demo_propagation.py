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

"""Simple demo showcasing how to use the fire propagation module and how to display the result"""
import matplotlib.cm
import matplotlib.pyplot as plt

from fire_rs.firemodel import propagation
from fire_rs.geodata.display import GeoDataDisplay
from fire_rs.geodata.geo_data import TimedPoint


if __name__ == "__main__":
    output_format = "svg"

    # Uncomment below to use latex for typesetting
    # matplotlib.rcParams['font.family'] = 'sans-serif'
    # matplotlib.rcParams['text.usetex'] = True
    # output_format="pdf"

    area = ((480000.0, 485000.0), (6210000.0, 6215000.0))
    env = propagation.Environment(area, wind_speed=5., wind_dir=0.)
    ignition_point = TimedPoint(area[0][0] + 1000.0, area[1][0] + 2000.0, 0)
    fire = propagation.propagate_from_points(env, ignition_point, 90000)

    # Figure terrain + ignition contour + ignition point
    gdd = GeoDataDisplay.pyplot_figure(env.raster.combine(fire.ignitions().slice(["ignition"])),
                                       frame=(0., 0.))
    gdd.draw_elevation_shade(with_colorbar=False, cmap=matplotlib.cm.terrain)
    gdd.draw_wind_quiver()
    gdd.draw_ignition_contour(with_labels=True, cmap=matplotlib.cm.plasma)
    gdd.draw_ignition_points(ignition_point)

    gdd.figure.show()

    gdd.figure.savefig(".".join(("demo_propagation", output_format)), dpi=150, bbox_inches='tight')
