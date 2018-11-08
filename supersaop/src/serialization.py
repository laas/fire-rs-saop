import numpy as np

from fire_rs.geodata.geo_data import GeoData
from supersaop.msg import Raster, RasterMetaData


def raster_msg_from_geodata(geodata: GeoData, layer: str):
    return Raster(metadata=RasterMetaData(x_offset=geodata.x_offset, y_offset=geodata.y_offset,
                                          x_width=geodata.max_x, y_height=geodata.max_y,
                                          cell_width=geodata.cell_width),
                  data=geodata.data[layer].ravel())


def geodata_from_raster_msg(msg: Raster, layer: str):
    array = np.fromiter(zip(msg.data), dtype=[(layer, 'float64')])
    array.resize((msg.metadata.x_width, msg.metadata.y_height))
    return GeoData(array, msg.metadata.x_offset, msg.metadata.y_offset, msg.metadata.cell_width, msg.metadata.cell_width)
