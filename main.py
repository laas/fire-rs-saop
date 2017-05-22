"""Main client of fire_rs library"""


from fire_rs.geodata import environment
import fire_rs.firemodel.propagation as propagation
from mpl_toolkits.mplot3d import Axes3D
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import LightSource
from matplotlib.ticker import FuncFormatter
from matplotlib import cm, pyplot
import numpy as np


def display():

    area = [[535000.0, 540000.0], [6235000.0, 6240000.0]]
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]

    # area = [[525000.0, 530000.0], [6225000.0, 6230000.0]]  # Produces error!
    # area = [[525000.0, 530000.0], [6225000.0, 6250000.0]]
    ignition_point = (100, 100)
    area_wind = (10, np.pi)

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    prop = propagation.propagate(env, *ignition_point)
    prop.plot(blocking=True)


def step_by_step():
    area = [[530000.0, 535000.0], [6230000.0, 6235000.0]]
    ignition_point = (100, 100)
    ignition_time = 0
    area_wind = (10, np.pi)

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    propag = propagation.FirePropagation(env)
    propag.set_ignition_point(*ignition_point, ignition_time)

    plt.figure().gca(aspect='equal', xlabel="X position [m]", ylabel="Y position [m]")

    # todo: axes and directions are wrong when using imshow to display an array (see workarounds in GeoData)
    p = plt.imshow(propag.ignitions().data.view('float32'))
    plt.title("Wildfire propagation")

    next_step = ignition_time
    while not propag.propagation_finished:
        propag.propagate(next_step)
        next_step += 300
        p.set_data(propag.ignitions().data.view('float32'))
        plt.pause(0.1)

if __name__ == "__main__":
    # step_by_step()
    display()
