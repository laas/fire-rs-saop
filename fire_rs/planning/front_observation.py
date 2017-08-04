import logging
import numpy as np
import matplotlib.pyplot as plt
from fire_rs.geodata import environment
import fire_rs.firemodel.propagation as propagation
import fire_rs.geodata.display
import fire_rs.planning.observation_path_search
import fire_rs.geodata.geo_data
import os


def burn(area_bounds, ignition_point, wind):
    """Burn some area from an ignition point with given wind conditions"""
    env = propagation.Environment(area_bounds, wind_speed=wind[0], wind_dir=wind[1])
    ignition_map = propagation.propagate(env, *ignition_point)
    return ignition_map


def main():
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    ignition_point = (100, 100)
    area_wind = (10, np.pi)
    ignition_times = None
    if not os.path.exists('/tmp/ignition.tmp'):
        ignition_times = burn(area, ignition_point, area_wind)
        ignition_times.write_to_file('/tmp/ignition.tmp')
    else:
        ignition_times = fire_rs.geodata.geo_data.GeoData.load_from_file('/tmp/ignition.tmp')
    ignition_times.plot('ignition')


    world = environment.World()
    some_area = world.get_elevation(area)

    x = np.arange(some_area.data.shape[0])
    x = (x * some_area.cell_width) + some_area.x_offset

    y = np.arange(some_area.data.shape[1])
    y = (y * some_area.cell_height) + some_area.y_offset

    X, Y = np.meshgrid(x, y)

    figure, axis = fire_rs.geodata.display.get_pyplot_figure_and_axis()
    fire_rs.geodata.display.plot_ignition_contour(axis, X, Y, ignition_times.data['ignition']/60, nfronts=50)
    print("")

    for low, high in [[60, 61], [70, 71], [100, 101]]:
        ignition_array = ignition_times.data['ignition']/60
        low_bound = ignition_array>low
        up_bound = ignition_array<high
        valid_points = low_bound & up_bound
        selected_points = []
        for hor in range(len(x)):
            for ver in range(len(y)):
                if valid_points[hor, ver]:
                    selected_points.append([hor, ver])
        selected_points = np.array(selected_points)
        try:
            fire_rs.planning.observation_path_search.process_points(selected_points, (5,5), eps=5)
        except ValueError:
            logging.exception("")


if __name__ == '__main__':
    main()
    print("stop")



