"""Sampling fire spread (ROS, wind angle and force)
"""
import fire_rs.firemodel.propagation as propagation
import numpy as np
import itertools
import os
import errno
import time

FIRERS_DATA = os.environ['FIRERS_DATA']

def mkdir_p_file(path):
    fpath = os.path.abspath(path)
    dpath = os.path.dirname(fpath)
    try:
        os.makedirs(dpath)
    except OSError as exc:  # Python >2.5
        if exc.errno == errno.EEXIST and os.path.isdir(dpath):
            pass
        else:
            raise exc
    return fpath


def sample_spread_params(**kwargs):
    """Gaussian error sampling of fire spread params 
    """
    area  = kwargs["area"]
    wstru = kwargs["wind_speed_true"]
    wsran = kwargs["wind_speed_range"]
    wdtru = kwargs["wind_dir_true"]
    wdran = kwargs["wind_dir_range"]

    wssam = np.arange(wstru-wsran, wstru+wsran+1, step=1)
    wdsam = np.arange(wdtru-wdran, wdtru+wdran+1, step=1)
    samples = itertools.product(wssam,wdsam)
    nsamp = len(wssam)*len(wdsam)
    
    #for the result filenames
    time_string = time.strftime('%y_%m_%d_%Hh%Mm%Ss')

    print("Building true env") # for the shape but commputation is not lost (cached)
    env_true = propagation.Environment(area, wind_speed=wstru, wind_dir=wdtru/180.0*np.pi)

    shape = env_true.raster.data.shape[:2]
    out = np.ndarray((nsamp,) + shape + (3,), dtype='double')
    for i, w in enumerate(samples):
        print("Building env #"+str(i))
        ws, wd = w
        env = propagation.Environment(area, wind_speed=ws, wind_dir=wd/180.0*np.pi)
        print("Computing spread parameters #"+str(i))
        out[i] = compute_spread_parameters(env)

    pid_string = "{}".format(os.getpid())
    filename = FIRERS_DATA+"/results/sample_spread_params_"+time_string+"_"+pid_string
    filename = mkdir_p_file(filename)
    np.savez_compressed(filename, params=kwargs,
                        samples=np.array([s for s in samples]), out=out)
    return out
    
def compute_spread_parameters(env, out=None):
    shape = env.raster.data.shape[0:2]
    if out is None:
        out = np.ndarray(shape + (3,))

    assert(shape == out.shape[0:2] and out.shape[2] == 3)

    for x in range(shape[0]):
        for y in range(shape[1]):
            out[x,y] = env.get_spread_parameters(x,y)

    return out


if __name__ == "__main__":
    area =  [[480060.0, 490060.0], [6210074.0, 6220074.0]]
    params = {
        "area":area,
        "wind_speed_true": 11,
        "wind_speed_range": 2,
        "wind_dir_true": 73,
        "wind_dir_range": 2,
    }
    out = sample_spread_params(**params)
