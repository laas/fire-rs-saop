import gdal
import numpy as np
from typing import List, Tuple
from osgeo import osr
import matplotlib
import matplotlib.pyplot as plt


class GeoData:
    """Container for geo-referenced raster data stored in a structured numpy array."""

    def __init__(self, array, x_offset, y_offset, cell_width, cell_height):
        assert cell_width > 0 and cell_height > 0, 'Origin must be on left-bottom'
        self.data = array
        self.x_offset = x_offset
        self.y_offset = y_offset
        self.cell_width = cell_width
        self.cell_height = cell_height
        projection = osr.SpatialReference()
        projection.ImportFromEPSG(2154)  # EPSG code for RGF93 / Lambert-93 projection
        self.projection = projection

    def __contains__(self, coordinates):
        (x, y) = coordinates
        x_lim_low = self.x_offset - self.cell_width/2
        x_lim_up = self.x_offset + (self.data.shape[0] + .5) * self.cell_width
        y_lim_low = self.y_offset - self.cell_height/2
        y_lim_up = self.y_offset + (self.data.shape[1] + .5) * self.cell_height

        return x_lim_low <= x <= x_lim_up and y_lim_low <= y <= y_lim_up

    def __repr__(self):
        return self.data.__repr__()

    def __getitem__(self, item):
        return self.data

    @property
    def data_display(self):
        """Data array in display form."""
        return self.data.T[::-1,...]

    def subset(self, area: Tuple[Tuple[float, float], Tuple[float, float]]) -> 'GeoData':
        ((x_min, x_max), (y_min, y_max)) = area
        (xi_min, yi_min) = self.array_index((x_min, y_min))
        (xi_max, yi_max) = self.array_index((x_max, y_max))
        ary = self.data[xi_min:xi_max+1, yi_min:yi_max+1]
        return GeoData(ary, *self.coordinates((xi_min, yi_min)), self.cell_width, self.cell_height)

    def array_index(self, coordinates: Tuple[float, float]) -> Tuple[int, int]:
        (x, y) = coordinates
        xi = int(round((x - self.x_offset) / self.cell_width))
        yi = int(round((y - self.y_offset) / self.cell_height))
        return xi, yi

    def coordinates(self, indices: Tuple[int, int]) -> Tuple[float, float]:
        (xi, yi) = indices
        x = self.x_offset + xi * self.cell_width
        y = self.y_offset + yi * self.cell_height
        return x, y

    def append_right(self, other: 'GeoData') -> 'GeoData':
        assert self.data.dtype == other.data.dtype
        assert self.cell_width == other.cell_width and self.cell_height == other.cell_height
        assert self.y_offset == other.y_offset
        assert self.x_offset + self.data.shape[0] * self.cell_width == other.x_offset
        assert self.data.shape[1] == other.data.shape[1]
        combined_array = np.concatenate([self.data, other.data], axis=0)
        return GeoData(combined_array, self.x_offset, self.y_offset,
                       self.cell_width, self.cell_height)

    def append_bottom(self, other: 'GeoData') -> 'GeoData':
        assert self.data.dtype == other.data.dtype
        assert self.cell_width == other.cell_width and self.cell_height == other.cell_height
        assert self.x_offset == other.x_offset
        assert self.y_offset + self.data.shape[1] * self.cell_height == other.y_offset
        assert self.data.shape[0] == other.data.shape[0]
        combined_array = np.concatenate([self.data, other.data], axis=1)
        return GeoData(combined_array, self.x_offset, self.y_offset,
                       self.cell_width, self.cell_height)

    def combine(self, other: 'GeoData') -> 'GeoData':
        assert self.data.shape == other.data.shape
        assert self.cell_width == other.cell_width and self.cell_height == other.cell_height
        combined_array = join_structured_arrays([self.data, other.data])
        return GeoData(combined_array, self.x_offset, self.y_offset,
                       self.cell_width, self.cell_height)

    def split(self, x_splits: int, y_splits: int) -> List['GeoData']:
        if x_splits == 1:
            splet = np.split(self.data, y_splits, axis=1)
            curr_y_offset = self.y_offset
            res = []
            for ary in splet:
                res.append(GeoData(ary, self.x_offset, curr_y_offset,
                                   self.cell_width, self.cell_height))
                curr_y_offset += res[-1].data.shape[1] * self.cell_height
            assert len(res) == x_splits * y_splits
            return res
        else:
            splet_on_y = self.split(x_splits=1, y_splits=y_splits)
            res = []
            for gd in splet_on_y:
                splet_on_x_y = np.split(gd.data, x_splits, axis=0)
                curr_x_offset = gd.x_offset
                for ary in splet_on_x_y:
                    res.append(GeoData(ary, curr_x_offset, gd.y_offset,
                                       self.cell_width, self.cell_height))
                    curr_x_offset += res[-1].data.shape[0] * self.cell_width
            assert len(res) == x_splits * y_splits
            return res

    def clone(self, data_array=None, fill_value=None, dtype=None):
        """Returns a clone of this GeoData.

        If data_array != None, the array is used as the internal data structure.
        If dtype and fill_value are not None, a new array of the same shape and the given dtype is created
        and filled with fill_value.
        Otherwise, a copy of self.data is used.
        """
        if data_array is None and fill_value is None:
            assert dtype is None
            return GeoData(self.data.copy(), self.x_offset, self.y_offset, self.cell_width, self.cell_height)
        elif data_array is not None:
            assert fill_value is None
            # impose data-type if provided
            data_array = data_array if dtype is None else np.array(data_array, dtype=dtype)
            assert data_array.shape == self.data.shape, 'The passed array as a different shape'
            return GeoData(data_array, self.x_offset, self.y_offset, self.cell_width, self.cell_height)
        else:
            assert fill_value is not None and dtype is not None
            d = np.full(self.data.shape, fill_value, dtype=dtype)
            return GeoData(d, self.x_offset, self.y_offset, self.cell_width, self.cell_height)

    def _get_plot_data(self, downscale):
        # axes with labels
        ax = plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")
        ax_formatter = matplotlib.ticker.ScalarFormatter(useOffset=False)
        ax.yaxis.set_major_formatter(ax_formatter)
        ax.xaxis.set_major_formatter(ax_formatter)

        # inverted and downscaled raster
        data = self.data.T[::-1, ...][::downscale, ::downscale]

        # downscaled and geo-transformed ticks
        x = (np.arange(data.shape[0]) * self.cell_width + self.x_offset)
        y = (np.arange(data.shape[1]) * self.cell_height + self.y_offset)

        # image geographic bounds
        image_scale = (x[0], x[-1], y[0], y[-1])
        return ax, data, x, y, image_scale

    def plot_vector(self, dir_layer, length_layer, downscale=1, blocking=False):
        ax, data, x, y, image_scale = self._get_plot_data(downscale)

        vel = data[length_layer]
        ang = 180 - data[dir_layer]
        wx = (vel * np.cos(ang / 180 * np.pi))
        wy = (vel * np.sin(ang / 180 * np.pi))

        ax.quiver(*np.meshgrid(x, y), wx, wy, pivot='middle', color='dimgrey')
        plt.show(block=blocking)

    def plot(self, layer=None, downscale=1, blocking=False):
        ax, data, x, y, image_scale = self._get_plot_data(downscale)
        if layer is None:
            assert len(self.data.dtype) == 1
            layer = self.data.dtype.names[0]

        z = data[layer]

        ax.imshow(z, extent=image_scale, vmin=z.min(), vmax=z.max(),
                  cmap=matplotlib.cm.terrain, interpolation='none')
        plt.show(block=blocking)

    def write_to_separate_files(self, parameterized_filename: str):
        """Writes each layer to a distinct GeoTiff file.

         The filename should contain %s which will replaced by name of each layer."""
        assert '%s' in parameterized_filename, 'File name should contain %s which will be replaced by the layer name'
        for layer in self.data.dtype.names:
            file = parameterized_filename % layer
            self.write_to_file(file, layer)

    def write_to_file(self, filename: str, layer_name=None):
        """Writes to GeoTiff file.

        If a layer_name is provided, then only the corresponding layer will be written, otherwise the GeoTiff file
        will contain all layers."""
        layers = self.data.dtype.names if layer_name is None else [layer_name]

        if self.cell_height < 0:
            data = self.data.transpose()  # in "image" files, rows and columns are inverted
            cell_height = self.cell_height
            origin_y = self.y_offset - self.cell_height / 2  # + array.shape[1] * pixelHeight
        else:
            # workaround a wind ninja bug that does not work with non-negative cell height
            # hence, we invert our matrix on the y axis to have a negative cell_height
            data = self.data.transpose()[..., ::-1]
            cell_height = - self.cell_height
            origin_y = self.y_offset + self.data.shape[1] * self.cell_height - self.cell_height/2

        cols = data.shape[1]
        rows = data.shape[0]
        origin_x = self.x_offset - self.cell_width / 2

        driver = gdal.GetDriverByName('GTiff')
        out_raster = driver.Create(filename, cols, rows, len(layers), gdal.GDT_Float32)
        out_raster.SetGeoTransform((origin_x, self.cell_width, 0, origin_y, 0, cell_height))
        for i, layer in enumerate(layers):
            outband = out_raster.GetRasterBand(i+1)
            outband.WriteArray(data[layer])
            outband.SetDescription(layer)  # apparently not visible in QGIS, maybe there is a better alternative
            outband.FlushCache()
        out_raster.SetProjection(self.projection.ExportToWkt())


def join_structured_arrays(arrays):
    """Efficient method to combine several structured arrays into a single one.

    This is based on the implementation of
    http://stackoverflow.com/questions/5355744/numpy-joining-structured-arrays
    with modifications to account for n-dimensional arrays.

    It is equivalent (but much faster) to the pure numpy function:
         rfn.merge_arrays(arrays, flatten=False, usemask=False).reshape(arrays[0].shape)
    """
    assert len(arrays) > 0
    assert all([array.shape == arrays[0].shape for array in arrays]), "Arrays have different shapes"
    sizes = np.array([a.itemsize for a in arrays])
    offsets = np.r_[0, sizes.cumsum()]
    n = arrays[0].size  # total number of cell
    joint = np.empty((n, offsets[-1]), dtype=np.uint8)
    # copy each array into joint at the proper offsets
    for a, size, offset in zip(arrays, sizes, offsets):
        # reshape as a C-contiguous array of bytes
        tmp = np.ascontiguousarray(a).view(np.uint8).reshape(n, size)
        joint[:, offset:offset+size] = tmp
    dtype = sum((a.dtype.descr for a in arrays), [])
    return joint.ravel().view(dtype).reshape(arrays[0].shape)
