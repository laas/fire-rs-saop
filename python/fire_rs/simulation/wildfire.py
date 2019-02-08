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

import datetime
import typing as ty

import numpy as np

import fire_rs.firemodel.propagation
import fire_rs.geodata.environment
import fire_rs.geodata.geo_data
import fire_rs.geodata.wildfire


class RealWildfire:
    """Generate wildfire maps by combining ignition, propagation and weather changes operations"""

    def __init__(self, start_timestamp: datetime.datetime,
                 environment: fire_rs.firemodel.propagation.Environment):
        self._environment = environment
        self.start_timestamp = start_timestamp
        self._current_time = self.start_timestamp

        # Pending ignitions
        # type: ty.MutableMapping[datetime.datetime, ty.Tuple[int, int]]
        self._pending_ignitions = {}

        self._perimeter = None
        self._fire_map = fire_rs.firemodel.propagation.empty_firemap(self._environment.raster)

        self._action_log = []

    def ignite(self, position: ty.Union[ty.Tuple[float, float], fire_rs.geodata.geo_data.Point]):
        """Set some location on fire"""
        c = self._environment.raster.array_index(position)
        self._pending_ignitions[c] = self._current_time.timestamp()
        self._fire_map["ignition"][c] = self._pending_ignitions[c]

        self._action_log.append(
            (self._current_time, "{} position {} ".format("Ignite", str(position))))

    def change_wind(self, speed, direction):
        self._environment.update_area_wind(speed, direction)
        self._action_log.append(
            (self._current_time,
             "{} to {} km/h {} °".format("Set Wind", str(speed), str(direction / np.pi * 180))))

    def propagate(self, duration: datetime.timedelta):

        if self._perimeter:
            self._pending_ignitions = {**self._pending_ignitions, **self._perimeter.cells}

        old_fire_map = self._fire_map.clone()

        # First propagation
        fireprop = fire_rs.firemodel.propagation.FirePropagation(self._environment)

        # Mark burnt cells, so fire do not propagate over them again
        mask = np.where(
            (old_fire_map.data["ignition"] > 0) & (old_fire_map.data["ignition"] < np.inf))
        if self._perimeter:
            mask = np.where(self._perimeter.area_array | np.isfinite(self._perimeter.array))
        fireprop.prop_data.data["ignition"][mask] = np.NaN

        for k, v in self._pending_ignitions.items():
            fireprop.set_ignition_cell((k[0], k[1], v))

        fireprop.propagate((self._current_time + duration).timestamp())

        # remove pending ignitions
        self._pending_ignitions = {}

        # Store firemap
        self._fire_map = fireprop.ignitions()

        # Fuse the old firemap wih the new one
        self._fire_map.data["ignition"][mask] = old_fire_map["ignition"][mask]

        # Advance time
        self._current_time += duration

        # Calculate perimeter
        self._perimeter = fire_rs.geodata.wildfire.Perimeter(self._fire_map,
                                                             self.current_time.timestamp())

        self._action_log.append(
            (self._current_time,
             "{} for {}".format("Propagate", str(duration))))

    @property
    def action_log(self) -> ty.Sequence[ty.Tuple[datetime.datetime, str]]:
        return self._action_log

    @property
    def current_time(self) -> datetime.datetime:
        return self._current_time

    @property
    def current_perimeter(self) -> ty.Optional[fire_rs.geodata.wildfire.Perimeter]:
        return self._perimeter

    @property
    def fire_map(self) -> fire_rs.geodata.geo_data.GeoData:
        return self._fire_map


if __name__ == "__main__":

    import matplotlib.pyplot as plt
    import fire_rs.geodata.display

    # Test realwildfire
    time = datetime.datetime.now()
    area = [[480060.0, 485060.0], [6210074.0, 6215074.0]]
    speed = 1.
    direction = 0.
    world = fire_rs.geodata.environment.World()
    env = fire_rs.firemodel.propagation.Environment(area, speed, direction, world=world)
    rw = RealWildfire(time, env)

    actions = [(rw.ignite, ((area[0][0] + 1000.0, area[1][0] + 1000.),)),
               (rw.propagate, (datetime.timedelta(minutes=30.),)),
               (rw.change_wind, (3, np.pi / 4)),
               (rw.propagate, (datetime.timedelta(minutes=31.),)),
               (rw.change_wind, (3, np.pi / 2)),
               (rw.ignite, ((area[0][0] + 3000.0, area[1][0] + 3000.),)),
               (rw.propagate, (datetime.timedelta(minutes=32.),)),
               (rw.change_wind, (3, 0.)),
               (rw.propagate, (datetime.timedelta(minutes=33.),)),
               (rw.change_wind, (3, np.pi / 4)),
               (rw.propagate, (datetime.timedelta(minutes=34.),)),
               (rw.change_wind, (3, np.pi / 2)),
               (rw.propagate, (datetime.timedelta(minutes=35.),))
               ]

    fig = plt.figure()
    ax = fig.gca()

    for action in actions:
        fig.clear()
        ax = fig.gca()

        if len(action[1]) == 0:
            action[0]()
        else:
            action[0](*action[1])

        v_min = np.nanmin(rw.fire_map.data["ignition"][np.isfinite(rw.fire_map.data["ignition"])])
        v_max = np.nanmax(rw.fire_map.data["ignition"][np.isfinite(rw.fire_map.data["ignition"])])
        fig.colorbar(ax.matshow(rw.fire_map.data["ignition"], vmin=v_min, vmax=v_max),
                     format=fire_rs.geodata.display.SecondDateFormatter('%d/%m/%y %H:%M'), )

        if rw.current_perimeter:
            # if rw.current_perimeter.area_array is not None:
            #     ax.matshow(rw.current_perimeter.area_array)
            fig.colorbar(
                ax.matshow(rw.current_perimeter.array, cmap="Reds", vmin=v_min, vmax=v_max),
                format=fire_rs.geodata.display.SecondDateFormatter('%d/%m/%y %H:%M'))
        fig.show()
    print("bye")
