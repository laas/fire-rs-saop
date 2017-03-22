import gdal
import numpy as np
from typing import List, Tuple
from osgeo import osr

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

    def clone(self, data_array=None):
        """Returns a clone of this GeoData.

        If the data_array is None, a copy of self.data is used.
        Otherwise the array is used as the internal data structure.
        """
        if data_array is None:
            return GeoData(self.data.copy(), self.x_offset, self.y_offset, self.cell_width, self.cell_height)
        else:
            assert data_array.shape == self.data.shape, 'The passed array as a different shape'
            return GeoData(data_array, self.x_offset, self.y_offset, self.cell_width, self.cell_height)

    def write_to_file(self, filename: str):
        assert len(self.data.dtype) == 1, \
            "Writing to file data with multiple layers is not supported yet"

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
        origin_x = self.x_offset - self.cell_width / 2  # + array.shape[0] * pixelWidth

        driver = gdal.GetDriverByName('GTiff')
        out_raster = driver.Create(filename, cols, rows, 1, gdal.GDT_Float32)
        out_raster.SetGeoTransform((origin_x, self.cell_width, 0, origin_y, 0, cell_height))
        outband = out_raster.GetRasterBand(1)
        outband.WriteArray(data)
        out_raster.SetProjection(self.projection.exportToWkt())
        outband.FlushCache()


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
