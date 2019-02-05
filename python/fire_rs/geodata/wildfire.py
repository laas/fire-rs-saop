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
import scipy.interpolate
import skimage.draw
import skimage.measure

import fire_rs.geodata.geo_data


class Perimeter:

    def __init__(self, wildfire: fire_rs.geodata.geo_data.GeoData, threshold: float,
                 layer: str = 'ignition', perimeter_background=np.inf):
        self._wildfire = wildfire
        self._threshold = threshold
        self._layer = layer
        self._empty_val = perimeter_background

        self._perimeter_array, self._cells, self._n_contours = compute_perimeter(
            self._wildfire, self._threshold, layer=self._layer, empty_val=self._empty_val)

        self._perimeter_geodata = None

    @property
    def array(self) -> np.ndarray:
        return self._perimeter_array

    @property
    def cells(self) -> ty.MutableMapping[ty.Tuple[int, int], np.float64]:
        return self._cells

    @property
    def count(self) -> int:
        """Number of indepedent perimeters"""
        return self._n_contours

    @property
    def geodata(self) -> fire_rs.geodata.geo_data.GeoData:
        if self._perimeter_geodata is None:
            self._perimeter_geodata = self._wildfire.clone(data_array=self._perimeter_array,
                                                           dtype=self._wildfire.data.dtype)
        return self._perimeter_geodata


_compute_perimeter_output_type = ty.Tuple[
    np.ndarray, ty.MutableMapping[ty.Tuple[int, int], np.float64], int]


def compute_perimeter(wildfire: fire_rs.geodata.geo_data.GeoData, threshold: float,
                      layer: str = 'ignition', empty_val=np.inf) -> _compute_perimeter_output_type:
    """Extract the perimeter given by threshold from a fire map."""

    array = np.ones(wildfire.data.shape, dtype=np.float64) * empty_val
    cells = {}

    contours = skimage.measure.find_contours(wildfire.data[layer], threshold)
    n_contours = len(contours)

    for contour in contours:
        prev_edge = contour[0]
        for edge in contour[1:]:
            if np.isnan(prev_edge).any() or np.isnan(edge).any():
                continue
            rr, cc = skimage.draw.line(*np.asarray(prev_edge, dtype=int),
                                       *np.asarray(edge, dtype=int))
            # Set perimeter in array format
            array[rr, cc] = wildfire.data[layer][rr, cc]
            # Set perimeter in dict format
            for r, c in zip(rr, cc):
                cells[r, c] = wildfire.data[layer][r, c]

            prev_edge = edge

    return array, cells, n_contours


def interpolate(x, y, z, shape, function='thin_plate') -> np.ndarray:
    """RBF interpolation"""
    # Wildland fire modeling with an Eulerian level set method and automated calibration
    # might give a clue of which kind of kernel function to use

    # default smooth=0 for interpolation
    interpolator = scipy.interpolate.Rbf(x, y, z, function=function, smooth=0)

    xi = np.linspace(0, shape[0] - 1, shape[0])
    yi = np.linspace(0, shape[1] - 1, shape[1])
    meshgrid = np.meshgrid(xi, yi, indexing="ij")

    dense_array = interpolator(*[x.flatten() for x in meshgrid])

    return dense_array.reshape(shape[0], shape[1])
