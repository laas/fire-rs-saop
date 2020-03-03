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

import cv2
import numpy as np
import skimage.draw
import skimage.measure

import fire_rs.geodata.geo_data
import fire_rs.rbf


class Perimeter:

    def __init__(self, wildfire: fire_rs.geodata.geo_data.GeoData, threshold: float,
                 layer: str = 'ignition', perimeter_background=np.inf):
        self._wildfire = wildfire
        self._threshold = threshold
        self._layer = layer
        self._empty_val = perimeter_background

        self._perimeter_array, self._cells, self._contour = _compute_perimeter(
            self._wildfire, self._threshold, layer=self._layer, empty_val=self._empty_val)

        if self._contour:
            for cont in self._contour:
                skimage.measure.grid_points_in_poly(self._wildfire.data.shape, cont)

        self._perimeter_geodata = None
        self._area_array = None

    @property
    def array(self) -> np.ndarray:
        return self._perimeter_array

    @property
    def cells(self) -> ty.MutableMapping[ty.Tuple[int, int], np.float64]:
        return self._cells

    @property
    def count(self) -> int:
        """Number of indepedent perimeters"""
        return len(self._contour)

    @property
    def geodata(self) -> fire_rs.geodata.geo_data.GeoData:
        if self._perimeter_geodata is None:
            self._perimeter_geodata = self._wildfire.clone(data_array=self._perimeter_array,
                                                           dtype=self._wildfire.data.dtype)
        return self._perimeter_geodata

    @property
    def area_array(self):
        """Mask of cells inside the perimeter"""
        if self._area_array is None:
            if self._contour:
                self._area_array = skimage.measure.grid_points_in_poly(self._wildfire.data.shape,
                                                                       self._contour[0])
                if len(self._contour) > 1:
                    for cont in self._contour[1:]:
                        self._area_array += skimage.measure.grid_points_in_poly(
                            self._wildfire.data.shape, cont)
        return self._area_array


_compute_perimeter_output_type = ty.Tuple[
    np.ndarray, ty.MutableMapping[ty.Tuple[int, int], np.float64], ty.List[np.ndarray]]


def _compute_perimeter(wildfire: fire_rs.geodata.geo_data.GeoData, threshold: float,
                       layer: str = 'ignition', empty_val=np.inf) -> _compute_perimeter_output_type:
    """Extract the perimeter given by threshold from a fire map."""

    array = np.ones(wildfire.data.shape, dtype=np.float64) * empty_val
    cells = {}

    contours = skimage.measure.find_contours(wildfire.data[layer], threshold)

    for contour in contours:
        try:
            rr, cc = skimage.draw.polygon_perimeter(contour[..., 0], contour[..., 1],
                                                    shape=wildfire.data.shape, clip=True)
            # Set perimeter in array format
            array[rr, cc] = wildfire.data[layer][rr, cc]
            # Set perimeter in dict format
            for r, c in zip(rr, cc):
                cells[r, c] = wildfire.data[layer][r, c]
        except IndexError as e:
            pass # Ignore contour if it contains NaN (polygon_perimeter throws IndexError)


    # for contour in contours:
    #     prev_edge = contour[0]
    #     for edge in contour[1:]:
    #         if np.isnan(prev_edge).any() or np.isnan(edge).any():
    #             continue
    #         rr, cc = skimage.draw.line(*np.asarray(prev_edge, dtype=int),
    #                                    *np.asarray(edge, dtype=int))
    #         # Set perimeter in array format
    #         array[rr, cc] = wildfire.data[layer][rr, cc]
    #         # Set perimeter in dict format
    #         for r, c in zip(rr, cc):
    #             cells[r, c] = wildfire.data[layer][r, c]
    #
    #         prev_edge = edge

    return array, cells, contours


def interpolate(x, y, z, shape, function='thin_plate') -> np.ndarray:
    """RBF interpolation"""
    # Wildland fire modeling with an Eulerian level set method and automated calibration
    # might give a clue of which kind of kernel function to use

    # default smooth=0 for interpolation
    interpolator = fire_rs.rbf.Rbf(x, y, z, function=function, smooth=0, cond=10 ** -5)

    xi = np.linspace(0, shape[0] - 1, shape[0])
    yi = np.linspace(0, shape[1] - 1, shape[1])
    meshgrid = np.meshgrid(xi, yi, indexing="ij")

    dense_array = interpolator(*[x.flatten() for x in meshgrid])

    return dense_array.reshape(shape[0], shape[1])


def rate_of_spread_map(firemap: fire_rs.geodata.geo_data.GeoData, layer="ignition",
                       output_layer="ros") -> fire_rs.geodata.geo_data.GeoData:
    """Compute the Rate of Spread from a wildfire_map"""
    gradient = firemap.clone(data_array=np.linalg.norm(np.gradient(firemap[layer]), axis=0),
                             dtype=[(output_layer, 'float64')])
    return gradient


class WildfireGraph:
    """Computation of the propagation graph and end of ignition from a wildfire map"""
    IGNITION_END = 'ignition_end'
    PROP_X = 'prop_x'
    PROP_Y = 'prop_y'
    PROP_DIR = 'prop_dir'

    def __init__(self, firemap: fire_rs.geodata.geo_data.GeoData, ignition_layer: str = 'ignition',
                 ignition_end_layer: str = IGNITION_END, prop_x_layer: str = PROP_X,
                 prop_y_layer: str = PROP_Y, prop_dir_layer: str = PROP_DIR):
        self._ignition_layer = ignition_layer
        self._ignition_end_layer = ignition_end_layer
        self._prop_x_layer = prop_x_layer
        self._prop_y_layer = prop_y_layer
        self._prop_dir_layer = prop_dir_layer
        self.geodata = firemap.clone(fill_value=0,
                                     dtype=[(ignition_layer, 'float64'),
                                            ('ignition_end', 'float64'),
                                            ('prop_x', 'int8'),
                                            ('prop_y', 'int8'),
                                            ('prop_dir', 'float64')])
        self.geodata.data[ignition_layer] = firemap.data[ignition_layer]
        grad = np.gradient(self.geodata.data[ignition_layer])
        self.geodata.data[prop_dir_layer] = np.arctan2(grad[1], grad[0])
        self.geodata.data[prop_x_layer] = np.array(
            np.round(np.cos(self.geodata.data[prop_dir_layer])), np.int)
        self.geodata.data[prop_y_layer] = np.array(
            np.round(np.sin(self.geodata.data[prop_dir_layer])), np.int)
        self.geodata.data[ignition_end_layer] = WildfireGraph._compute_traversal_end(
            self.geodata.data[ignition_layer])

    @staticmethod
    def _compute_propagation_direction(fire_array: np.array):
        def default_ignition(x, y, dx, dy):
            if x == 0 and dx < 0:
                return fire_array[x, y]
            if x + dx >= fire_array.shape[0]:
                return fire_array[x, y]
            if y == 0 and dy < 0:
                return fire_array[x, y]
            if y + dy >= fire_array.shape[1]:
                return fire_array[x, y]
            if fire_array[x + dx, y + dy] < np.inf:
                return fire_array[x + dx, y + dy]
            else:
                return fire_array[x, y]

        prop_delta_x = np.zeros_like(fire_array, np.int)
        prop_delta_y = np.zeros_like(fire_array, np.int)
        prop_dir = np.zeros_like(fire_array)

        for x in range(0, fire_array.shape[0]):
            for y in range(0, fire_array.shape[1]):
                if fire_array[x, y] < np.inf:
                    ign = lambda dx, dy: default_ignition(x, y, dx, dy)
                    prop_delta_x[x, y] = ign(1, -1) + 2 * ign(1, 0) + ign(1, 1) - ign(-1,
                                                                                      -1) - 2 * ign(
                        -1, 0) - ign(-1, 1)
                    prop_delta_y[x, y] = ign(1, 1) + 2 * ign(0, 1) + ign(-1, 1) - ign(1,
                                                                                      -1) - 2 * ign(
                        0, -1) - ign(-1, -1)
                    prop_dir = np.arctan2(prop_delta_y[x, y], prop_delta_x[x, y])

        return prop_delta_x, prop_delta_y, prop_dir

    @staticmethod
    def _compute_traversal_end(fire_array: np.ndarray):
        """Compute the ignition end time for each cell.

        The ignition end time is the latest ignition time among all neighbor cells."""
        end = np.zeros_like(fire_array) * np.inf

        for x in range(0, fire_array.shape[0]):
            for y in range(0, fire_array.shape[1]):
                if fire_array[x, y] < np.inf:
                    max_neighbor = 0.0
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            if (dx == 0 and dy == 0) or (x == 0 and dx < 0) or (
                                    x + dx >= fire_array.shape[0]) or (y == 0 and dy < 0) or (
                                    y + dy >= fire_array.shape[1]):
                                continue
                            if fire_array[x + dx, y + dy] < np.inf:
                                max_neighbor = max(max_neighbor, fire_array[x + dx, y + dy])
                    if max_neighbor <= fire_array[x, y]:
                        # propagation_border
                        end[x, y] = fire_array[x, y] + 180  # assume 3 minutes
                    else:
                        end[x, y] = max_neighbor
                else:
                    end[x, y] = fire_array[x, y]
        return end

    def find_parent_or_child_of_time(self, start_cell: fire_rs.geodata.geo_data.Cell, time: float):
        """Find an ignited cell at 'time' in the propagation graph starting from 'start_cell'

        This function similar to the c++ planning 'project on firefront' algorithm.
        """

        coord = start_cell
        found = False
        while not found:
            if self.geodata.data[self._ignition_layer][coord] <= time and time < \
                    self.geodata.data[self._ignition_end_layer][coord]:
                found = True
            else:
                uphill_cell = (coord[0] + self.geodata.data[self._prop_x_layer][coord],
                               coord[1] + self.geodata.data[self._prop_y_layer][coord])
                downhill_cell = (coord[0] - self.geodata.data[self._prop_x_layer][coord],
                                 coord[1] - self.geodata.data[self._prop_y_layer][coord])
                if uphill_cell[0] == downhill_cell[0] and uphill_cell[1] == downhill_cell[1]:
                    # Extrema reached
                    coord = uphill_cell
                    found = True
                next_cell = downhill_cell if self.geodata.data[self._ignition_layer][
                                                 coord] > time else uphill_cell

                if (next_cell[0] < 0 or next_cell[0] >
                    self.geodata.data[self._ignition_layer].shape[0] - 1) and (
                        next_cell[1] < 0 or next_cell[1] >
                        self.geodata.data[self._ignition_layer].shape[1] - 1):
                    found = True
                if (next_cell == downhill_cell) and (
                        self.geodata.data[self._ignition_layer][next_cell] >
                        self.geodata.data[self._ignition_layer][coord]) or (
                        next_cell == uphill_cell) and (
                        self.geodata.data[self._ignition_layer][next_cell] <
                        self.geodata.data[self._ignition_layer][coord]):
                    # Extrema reached
                    found = True
                if self.geodata.data[self._ignition_layer][next_cell] < np.inf:
                    coord = next_cell
        return coord


def warp_firemap(gd: fire_rs.geodata.geo_data.GeoData, orig: ty.Sequence[ty.Tuple[int, int]],
                 dest: ty.Sequence[ty.Tuple[int, int]],
                 layer: str = "ignition"):
    array = _warp_image(gd[layer], orig, dest)
    new_gd = fire_rs.firemodel.propagation.empty_firemap(gd, layer=layer)
    new_gd.data["ignition"] = array
    return new_gd


def _warp_image(array: np.ndarray, orig: ty.Sequence[ty.Tuple[int, int]],
                dest: ty.Sequence[ty.Tuple[int, int]]) -> np.ndarray:
    newarray = array.copy()
    newarray = newarray.T  # opencv convention on columns and rows is inverted
    newarray_min = newarray.min()
    newarray_max = newarray.max()
    newarray = (newarray - newarray_min) / (newarray_max - newarray_min)
    orig_p = np.array(orig, np.int32)
    orig_p = orig_p.reshape(1, -1, 2)  # Don't ask why. https://stackoverflow.com/a/47114049
    dest_p = np.array(dest, np.int32)
    dest_p = dest_p.reshape(1, -1, 2)  # Don't ask why. https://stackoverflow.com/a/47114049
    good_matches = [cv2.DMatch(p, p, 0) for p in range(len(orig))]
    tps = cv2.createThinPlateSplineShapeTransformer(60.0)  # Relax strict interpolation condition
    tps.estimateTransformation(dest_p, orig_p, good_matches)
    warped = None
    warped = tps.warpImage(newarray, warped, cv2.INTER_LINEAR, cv2.BORDER_CONSTANT, np.inf)
    warped = warped * (newarray_max - newarray_min) + newarray_min
    warped = warped.T
    return warped
