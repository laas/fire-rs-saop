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
import numpy as np
import scipy.spatial
import skimage.draw
import itertools
import matplotlib.pyplot as plt


class UtilityMap:

    def __init__(self, base_utility: np.ndarray):
        self.raster = base_utility

        self.observations = set()  # type: ty.Set[ty.Tuple[int, int]]
        self.observations_rect = set()

    def add_observation(self, obs: ty.Tuple[int, int]):
        if obs not in self.observations:
            self.observations.add(obs)
            # self.observations.append(np.array([obs[0], obs[1]]))

    def add_rect(self, center: ty.Tuple[int, int], length: int, orient: float):
        if (center, length, orient) not in self.observations_rect:
            self.observations_rect.add((center, length, orient))
            # self.observations.append(np.array([obs[0], obs[1]]))

    def utility(self):

        def rbf(dist):
            return np.clip(1 - 0.1 * (dist - 10), 0., 1.)

        def find_closest(cell, obs_list):
            dist = scipy.spatial.distance.cdist(np.array([cell]), obs_list)
            return obs_list[np.argmin(dist)], np.nanmin(dist)

        utility_map = np.copy(self.raster)
        obs_list = list(self.observations)
        min_dist = np.full(len(obs_list), np.inf)
        #
        # with np.nditer(utility_map, flags=['multi_index'], op_flags=["writeonly"]) as it:
        #     for i in it:
        #         if np.isnan(i):
        #             continue
        #         obs, obs_dist = find_closest(it.multi_index, obs_list)
        #         i[...] = np.clip(
        #             self.raster[it.multi_index] - self.raster[obs] * rbf(obs_dist), 0., 1.)

        obs_rect_list = list(self.observations_rect)
        w = width = 3

        for (c, l, angle) in obs_rect_list:
            front = (c[0] + l / 2 * np.cos(angle), c[1] + l / 2 * np.sin(angle))
            back = (c[0] + l / 2 * np.cos(angle + np.pi), c[1] + l / 2 * np.sin(angle + np.pi))

            front_left = (front[0] + w / 2 * np.cos(angle + np.pi / 2),
                          front[1] + w / 2 * np.sin(angle + np.pi / 2))
            front_right = (front[0] + w / 2 * np.cos(angle - np.pi / 2),
                           front[1] + w / 2 * np.sin(angle - np.pi / 2))

            back_right = (back[0] + w / 2 * np.cos(angle + np.pi / 2),
                          back[1] + w / 2 * np.sin(angle + np.pi / 2))
            back_left = (back[0] + w / 2 * np.cos(angle + 3 * np.pi / 2),
                         back[1] + w / 2 * np.sin(angle + 3 * np.pi / 2))

            coords = skimage.draw.polygon(
                [front_left[0], front_right[0], back_left[0], back_right[0]],
                [front_left[1], front_right[1], back_left[1], back_right[1]],
                shape=utility_map.shape)

            utility_map[coords] = 0.

        return utility_map
        #
        # for i, other_i in itertools.permutations(range(len(obs_list)), 2):
        #     min_dist[i] = min(min_dist[i],
        #                       np.min(
        #                           scipy.spatial.distance.euclidean(obs_list[i], obs_list[other_i])))
        #
        # print(min_dist)
        # max_min_dist = max(min_dist)
        # min_min_dist = min(min_dist)
        # print(max_min_dist)
        # print(min_min_dist)
        #
        # min_dist -= min_min_dist
        # min_dist /= max_min_dist - min_min_dist
        #
        # for i, o in enumerate(obs_list):
        #     utility_map.raster[o] = min_dist[i]


if __name__ == "__main__":
    import os
    import fire_rs.geodata.environment
    import fire_rs.geodata.geo_data
    import fire_rs.firemodel.propagation

    FIRERS_DATA_FOLDER = '/home/rbailonr/src/fire-rs-data-private/porto_laea'
    FIRERS_DEM_DATA = os.path.join(FIRERS_DATA_FOLDER, 'dem')
    FIRERS_WIND_DATA = os.path.join(FIRERS_DATA_FOLDER, 'wind')
    FIRERS_LANDCOVER_DATA = os.path.join(FIRERS_DATA_FOLDER, 'landcover')

    the_world = fire_rs.geodata.environment.World(elevation_path=FIRERS_DEM_DATA,
                                                  wind_path=FIRERS_WIND_DATA,
                                                  landcover_path=FIRERS_LANDCOVER_DATA,
                                                  wind_mesh_resolution='fine',
                                                  landcover_to_fuel_remap=fire_rs.geodata.environment.SLOW_FUELMODEL_REMAP)
    the_world.dem_wind_tile_split = 1

    area = ((2776825.0 - 1000, 2776825.0 + 1000), (2212175.0 - 1000, 2212175.0 + 1000))
    ignition = fire_rs.geodata.geo_data.TimedPoint(2776825.0, 2212175.0, 0)

    ################################################################################
    # Reference fire propagation
    fire_env = fire_rs.firemodel.propagation.Environment(area, 5., np.pi / 2, the_world)

    fire_prop = fire_rs.firemodel.propagation.FirePropagation(fire_env)
    fire_prop.set_ignition_point(ignition)

    propagation_end_time = 60 * 60 * 60
    # propagation_end_time = np.inf

    fire_prop.propagate(propagation_end_time)

    firemap = fire_prop.ignitions()["ignition"]
    gradient_firemap = np.linalg.norm(np.gradient(firemap), axis=0)

    utility = 1 - (gradient_firemap - np.nanmin(gradient_firemap)) / (
            np.nanmax(gradient_firemap) - np.nanmin(gradient_firemap))

    # utility_map[firemap < 10 * 60 * 60] = 0.
    # utility_map[firemap > 20 * 60 * 60] = 0.

    # utility_map[...] = 1
    # utility_map[0:int(utility_map.shape[0]/2):1, ...] = 0.5
    import random

    umap = UtilityMap(utility)
    print(np.sum(umap.utility()))
    for i in range(10):
        umap.add_rect(
            (random.randint(0, utility.shape[0] - 1), random.randint(0, utility.shape[1] - 1)),
            random.random()*100., random.random() * 2 * np.pi)
    print(np.sum(umap.utility()))

    f = plt.figure()
    a = f.gca()
    m = a.matshow(umap.utility())
    f.colorbar(m)
    f.show()
    print("adios")

    ################################################################################
    # World initialization

    f = plt.figure()
    a = f.gca()
    m = a.matshow(utility)
    f.colorbar(m)
    f.show()
    print("adiuos")
