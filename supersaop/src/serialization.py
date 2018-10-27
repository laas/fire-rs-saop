from supersaop.msg import Raster, RasterMetaData


def raster_msg_from_geodata(geodata, layer: str):
    return Raster(metadata=RasterMetaData(x_offset=geodata.x_offset, y_offset=geodata.y_offset,
                                          x_width=geodata.max_x, y_height=geodata.max_y,
                                          cell_width=geodata.cell_width),
                  data=geodata.data[layer].ravel())


def geodata_from_raster_msg(geodata, layer: str):
    raise NotImplemented
