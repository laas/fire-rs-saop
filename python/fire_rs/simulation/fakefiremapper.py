#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import typing as ty

import fire_rs.firemapping
import fire_rs.planning.new_planning

from fire_rs.geodata.geo_data import GeoData


class FakeFireMapper:
    """Simulate observed wildfire map creation from a UAV"""

    def __init__(self, elevation: GeoData, wildfire: GeoData, elevation_layer='elevation',
                 wildfire_layer='ignition', observed_fire_layer='ignition',
                 observed_layer='observed'):

        self.observed_fire_layer = observed_fire_layer
        self.observed_layer = observed_layer

        self._gfmapper = fire_rs.firemapping.GhostFireMapper(
            fire_rs.planning.new_planning.make_fire_data(wildfire, elevation,
                                                         wildfire_layer, elevation_layer))

    @property
    def firemap(self) -> GeoData:
        return GeoData.from_cpp_raster(self._gfmapper.firemap, self.observed_fire_layer)

    @property
    def observed(self) -> GeoData:
        return GeoData.from_cpp_raster(self._gfmapper.observed, self.observed_fire_layer)

    def observe(self, executed_path, uav: fire_rs.planning.new_planning.UAV):
        self._gfmapper.observe(executed_path[0], executed_path[1], uav)
