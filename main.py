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
import random

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


def multiple_ignitions():

    area = [[480060.0, 489060.0], [6210074.0, 6217074.0]]

    ignitions = [propagation.TimedPoint(486000.0, 6214000.0, 0.),
                 propagation.TimedPoint(486000.0, 6214100.0, 0.),
                 propagation.TimedPoint(486000.0, 6214150.0, 0.),
                 propagation.TimedPoint(486000.0, 6214200.0, 0.),
                 propagation.TimedPoint(486000.0, 6214250.0, 0.),
                 propagation.TimedPoint(486000.0, 6214300.0, 0.),
                 propagation.TimedPoint(486000.0, 6214350.0, 0.),
                 propagation.TimedPoint(486000.0, 6214430.0, 0.)]
    area_wind = (10, np.pi)

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    fireprop = propagation.FirePropagation(env)
    for ignition in ignitions:
        fireprop.set_ignition_point(ignition)
    fireprop.propagate(1200+9600+5000)
    fireprop.plot(blocking=False)

    low  = 159.5
    high = 160.5
    ignitions2 = []
    ignition_array = fireprop.prop_data['ignition'] / 60
    low_bound = ignition_array > low
    up_bound = ignition_array < high
    valid_points = low_bound & up_bound
    selected_points = []
    for hor in range(ignition_array.shape[0]):
        for ver in range(ignition_array.shape[1]):
            if valid_points[hor, ver]:
                ignitions2.append(propagation.TimedPoint(*fireprop.prop_data.coordinates((hor, ver)), 80.))

    print(ignitions2)
    ignitions = random.sample(ignitions2, 50)

    # Light a fire and propagate it over the whole area
    env = propagation.Environment(area, wind_speed=area_wind[0], wind_dir=area_wind[1])
    fireprop = propagation.FirePropagation(env)
    for ignition in ignitions:
        fireprop.set_ignition_point(ignition)
    fireprop.propagate(1200+5000)
    fireprop.plot(blocking=False)
    print(ignitions)

if __name__ == "__main__":
    # step_by_step()
    multiple_ignitions()
