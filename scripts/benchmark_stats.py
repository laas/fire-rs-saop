import pandas as pd
import json
from pprint import pprint
import copy
import glob
from functools import reduce
import sys

bench_name = "default" if len(sys.argv) <= 1 else sys.argv[1]

result_files = glob.glob("../../data/benchmark_"+bench_name+"/*/*.json")

all_lines = []

for f in result_files:
    j = {}
    with open(f) as raw:
        j = json.load(raw)

    res = {}
    res["configuration_name"] = j["configuration"]["vns"]["configuration_name"]
    res["instance"] = j["benchmark_id"]
    res["utility"] = j["plan"]["utility"]
    res["neighborhoods"] = reduce(lambda x, y: str(x)+":"+str(y),
                              map(lambda x: x["name"],
                                  j["configuration"]["vns"]["neighborhoods"]))

    res["planning_time"] = j["planning_time"]

    for n in j["neighborhoods"]:
        res[n['name']+'-runtime'] = n['runtime']


        pprint(res)

    def extract_traj_to(dic, traj):
        dic["duration"] = traj["duration"]
        dic["max_duration"] = traj["max_duration"]
        return dict(dic)

    one_per_traj = list(map(lambda x: extract_traj_to(copy.deepcopy(res), x), j["plan"]["trajectories"]))

    pprint(one_per_traj)
    all_lines.extend(one_per_traj)

df = pd.DataFrame.from_records(all_lines)
df2 = df.groupby(['instance','configuration_name']).agg('mean')
df2 = df2.reset_index()

print("Full:")
print(df2[df2['configuration_name']=='full'].describe())

print("Base:")
print(df2[df2['configuration_name']=='base'].describe())
