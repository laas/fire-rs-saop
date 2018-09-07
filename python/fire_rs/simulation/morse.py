import numpy as np
import skimage.io
import skimage.draw
import skimage.measure
import tempfile

import pymorse

from datetime import datetime
from typing import Tuple, Optional

from fire_rs.geodata.geo_data import GeoData


class MorseWildfire:

    def __init__(self, address: 'Tuple[str, int]', terrain_object: 'str', fire_gd: 'GeoData',
                 layer='ignition'):
        self.address = address
        self.terrain_object = terrain_object
        self.fire_map = fire_gd
        self.layer = layer

        self.color = (255, 255, 255)

        self.fire_image = np.zeros((*self.fire_map.data.shape, 3), dtype=np.uint8)

        self.morse_conn = None  # type: Optional[pymorse.Morse]

        self._morse_timeout = 5.

    def close(self, *args):
        if self.morse_conn is not None:
            self.morse_conn.close(*args)

    def _update_fire_image(self, time: 'float'):
        fire_image = np.zeros((*self.fire_map.data.shape, 3), dtype=np.uint8)
        # Create contour with scikit-image
        contours = skimage.measure.find_contours(self.fire_map.data[self.layer], time)
        # Print contour as binary image
        for contour in contours:
            for pt_i in range(len(contour)):
                rr, cc = skimage.draw.line(*np.asarray(contour[pt_i - 1], dtype=int),
                                           *np.asarray(contour[pt_i], dtype=int))
                fire_image[rr, cc] = self.color

        self.fire_image = fire_image

    def update(self, time: 'Optional[float]' = None):
        """Update wildfire at current time in morse"""
        if time is None:
            time = datetime.now().timestamp()

        self._update_fire_image(time)

        with tempfile.NamedTemporaryFile(suffix=".png", delete=True) as t_file:
            skimage.io.imsave(t_file, self.fire_image)

            if self.morse_conn is None or not self.morse_conn.is_up():
                self.morse_conn = pymorse.Morse(*self.address)

            self.morse_conn.rpc_t(self._morse_timeout, "simulation", "set_texture",
                                  str(self.terrain_object), str(t_file.name))

    def __enter__(self):
        if self.morse_conn is None or not self.morse_conn.is_up():
            self.morse_conn = pymorse.Morse(*self.address)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if not exc_type:
            self.close()
        else:
            self.close(True)
            return False  # re-raise exception
