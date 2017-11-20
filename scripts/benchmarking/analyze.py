import pandas as pd
import json
from pprint import pprint
import copy
import glob
from functools import reduce
import sys
import numpy as np
import matplotlib.pyplot as plt


def sample(x_space, step_func):
    """Sample a piecewise constant function at each point in x_space.
       This is used to define all utility histories on the same x point so we can manipulate them together."""
    cur_step = 0
    sampled = []
    for x in x_space:
        while cur_step < len(step_func)-1 and step_func[cur_step+1][0] <= x:
            cur_step += 1

        sampled.append(step_func[cur_step][1])
    return sampled

# sampled times on which to plot utility history in [0.001, 10] (seconds)
x = np.logspace(-3, 1, num=10000)

bench_name = "default" if len(sys.argv) <= 1 else sys.argv[1]

# set benchmark dir to the most recent sub folder(i.e. last by name since date is in the name)
subdirs = list(glob.glob("*/"))
subdirs.sort()
bench_dir = subdirs[-1]

result_files = glob.glob(bench_dir + "data/benchmark/*/*.json")

# array that will be used to store one dictionnary per (benchmark-instance, vns-configuration) instance
all_lines = []

print("Benchmark dir: " + str(bench_dir))
#print("Files: " + str(result_files))



for f in result_files:
    print("Processing: " + str(f))
    j = {}
    # read result file
    with open(f) as raw:
        j = json.load(raw)

    # extract meaningful information to a dictionnary
    res = {}
    res["configuration_name"] = j["configuration"]["vns"]["configuration_name"]
    res["instance"] = j["benchmark_id"]
    res["utility"] = j["plan"]["utility"]
    res["neighborhoods"] = reduce(lambda x, y: str(x)+":"+str(y),
                              map(lambda x: x["name"],
                                  j["configuration"]["vns"]["neighborhoods"]))

    res["planning_time"] = j["planning_time"]
    hist = [(pt[0], pt[1]) for pt in j["utility_history"]]
    y = np.array(sample(x, hist))
    res["utility_history"] = y


    for n in j["neighborhoods"]:
        res[n['name']+'-runtime'] = n['runtime']

    for k in ["duration", "num_segments", "max_duration"]:
        res[k] =  sum([t[k] for t in j["plan"]["trajectories"]])

    # save dictionnary
    all_lines.append(res)
    #print(res)

# create a dataframe from all benchmark results
df = pd.DataFrame.from_records(all_lines)


def best_util(instance):
    """Returns the best utility found for a given benchmark instance accross all configurations."""
    return df[df['instance']==instance]['utility'].min()

# Adds a column giving the best utility for each problem instance
df["utility_star"] = df.apply(lambda row: best_util(row['instance']), axis=1)

for conf in df["configuration_name"].unique():
    # create a new dataframe restricted to the current vns configuration
    df_of_conf = df[df["configuration_name"]==conf]

    # extract all utility histories and normalize them (a normalized utility of 1 means the best utility for the given instance)
    normalized_utility_histories = [ hist / u_star for (_, (hist, u_star))
                                     in df_of_conf[['utility_history', 'utility_star']].iterrows()]
    # mean of all utility histories for the current configuration
    mean_utility_history = np.mean(normalized_utility_histories, axis=0)

    plt.loglog(x, mean_utility_history, label=conf)

plt.legend(loc="upper right")
plt.show()

# remove the utility history column because it is very large (each cell is an array)
# and it gets is the way when visualizing data
del df["utility_history"]


# df["utility_star"] = df.apply(lambda row: best_util(row['instance']), axis=1)
df["utility_norm"] = (df["utility"]+0.001) / (df["utility_star"]+0.001)

# superseeded by "utility_norm"
del df["utility"]
del df["utility_star"]

# print summary data for each configuration
for conf in df["configuration_name"].unique():
    print("\n\n------- "+conf+" --------")
    print(df[df['configuration_name']==conf].dropna(axis=1,how='all').describe())
