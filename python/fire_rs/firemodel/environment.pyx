# cython: profile=True

# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import pandas as pd
from collections import namedtuple
import sys
if sys.version_info[0] < 3: 
    from StringIO import StringIO
else:
    from io import StringIO


fuel_models_csv = StringIO(""","Fuel_Model_Type","Load_1h","Load_10h","Load_100h","Load_Live_Herb","Load_Live_Woody","SA/V_1h","SA/V_10h","SA/V_100h","SA/V_Live_Herb","SA/V_Live_Woody","Fuel_Bed_Depth","Mx_dead","Heat_1h","Heat_10h","Heat_100h","Heat_Live_Herb","Heat_Live_Woody"
"A1","S",1.66,0,0,0,0,11483,0,0,0,0,30,12,18622,18622,18622,18622,18622
"A2","S",4.49,2.25,1.12,1.12,0,9843,358,98,4921,0,30,15,18622,18622,18622,18622,18622
"A3","S",6.74,0,0,0,0,4921,0,0,0,0,76,25,18622,18622,18622,18622,18622
"A4","S",11.23,8.98,4.49,0,11.23,6562,358,98,0,4921,183,20,18622,18622,18622,18622,18622
"A5","S",2.25,1.12,0,0,5.79,6562,358,98,0,4921,61,20,18622,18622,18622,18622,18622
"A6","S",3.7,6.18,4.94,0,0,5741,358,98,0,0,76,25,18622,18622,18622,18622,18622
"A7","S",2.72,4.69,3.71,0,0.91,5741,358,98,0,4921,76,40,18622,18622,18622,18622,18622
"A8","S",3.37,2.25,5.61,0,0,6562,358,98,0,0,6,30,18622,18622,18622,18622,18622
"A9","S",6.54,0.93,0.34,0,0,8202,358,98,0,0,6,25,18622,18622,18622,18622,18622
"A10","S",6.74,4.49,11.23,0,4.49,6562,358,98,0,4921,30,25,18622,18622,18622,18622,18622
"A11","S",3.37,10.1,12.35,0,0,4921,358,98,0,0,30,15,18622,18622,18622,18622,18622
"A12","S",8.98,21.44,37.06,0,0,4921,358,98,0,0,70,20,18622,18622,18622,18622,18622
"A13","S",15.72,51.66,62.89,0,0,4921,358,98,0,0,91,25,18622,18622,18622,18622,18622
"GR1","D",0.22,0,0,0.67,0,7218,358,98,6562,0,12,15,18622,18622,18622,18622,18622
"GR2","D",0.22,0,0,2.25,0,6562,358,98,5906,0,30,15,18622,18622,18622,18622,18622
"GR3","D",0.22,0.9,0,3.37,0,4921,358,98,4265,0,61,30,18622,18622,18622,18622,18622
"GR4","D",0.56,0,0,4.27,0,6562,358,98,5906,0,61,15,18622,18622,18622,18622,18622
"GR5","D",0.9,0,0,5.62,0,5906,358,98,5249,0,46,40,18622,18622,18622,18622,18622
"GR6","D",0.22,0,0,7.64,0,7218,358,98,6562,0,46,40,20934,20934,20934,20934,20934
"GR7","D",2.25,0,0,12.13,0,6562,358,98,5906,0,91,15,18622,18622,18622,18622,18622
"GR8","D",1.12,2.25,0,16.4,0,4921,358,98,4265,0,122,30,18622,18622,18622,18622,18622
"GR9","D",2.25,2.25,0,20.22,0,5906,358,98,5249,0,152,40,18622,18622,18622,18622,18622
"GS1","D",0.45,0,0,1.12,1.46,6562,358,98,5906,5906,27,15,18622,18622,18622,18622,18622
"GS2","D",1.12,1.12,0,1.35,2.25,6562,358,98,5906,5906,46,15,18622,18622,18622,18622,18622
"GS3","D",0.67,0.56,0,3.26,2.81,5906,358,98,5249,5249,55,40,18622,18622,18622,18622,18622
"GS4","D",4.27,0.67,0.22,7.64,15.96,5906,358,98,5249,5249,64,40,18622,18622,18622,18622,18622
"SH1","D",0.56,0.56,0,0.34,2.92,6562,358,98,5906,5249,30,15,18622,18622,18622,18622,18622
"SH2","S",3.03,5.39,1.69,0,8.65,6562,358,98,0,5249,30,15,18622,18622,18622,18622,18622
"SH3","S",1.01,6.74,0,0,13.93,5249,358,98,0,4593,73,40,18622,18622,18622,18622,18622
"SH4","S",1.91,2.58,0.45,0,5.73,6562,358,98,5906,5249,91,30,18622,18622,18622,18622,18622
"SH5","S",8.09,4.72,0,0,6.52,2461,358,98,0,5249,183,15,18622,18622,18622,18622,18622
"SH6","S",6.52,3.26,0,0,3.15,2461,358,98,0,5249,61,30,18622,18622,18622,18622,18622
"SH7","S",7.87,11.91,4.94,0,7.64,2461,358,98,0,5249,183,15,18622,18622,18622,18622,18622
"SH8","S",4.61,7.64,1.91,0,9.78,2461,358,98,0,5249,91,40,18622,18622,18622,18622,18622
"SH9","D",10.11,5.51,0,3.48,15.73,2461,358,98,5906,4921,134,40,18622,18622,18622,18622,18622
"TU1","D",0.45,2.02,3.37,0.45,2.02,6562,358,98,5906,5249,18,20,18622,18622,18622,18622,18622
"TU2","S",2.13,4.04,2.81,0,0.45,6562,358,98,0,5249,30,30,18622,18622,18622,18622,18622
"TU3","D",2.47,0.34,0.56,1.46,2.47,5906,358,98,5249,4593,40,30,18622,18622,18622,18622,18622
"TU4","S",10.11,0,0,0,4.49,7546,358,98,0,6562,15,12,18622,18622,18622,18622,18622
"TU5","S",8.99,8.99,6.74,0,6.74,4921,358,98,0,2461,30,25,18622,18622,18622,18622,18622
"TL1","S",2.25,4.94,8.09,0,0,6562,358,98,0,0,6,30,18622,18622,18622,18622,18622
"TL2","S",3.15,5.17,4.94,0,0,6562,358,98,0,0,6,25,18622,18622,18622,18622,18622
"TL3","S",1.12,4.94,6.29,0,0,6562,358,98,0,0,9,20,18622,18622,18622,18622,18622
"TL4","S",1.12,3.37,9.44,0,0,6562,358,98,0,0,12,25,18622,18622,18622,18622,18622
"TL5","S",2.58,5.62,9.89,0,0,6562,358,98,0,5249,18,25,18622,18622,18622,18622,18622
"TL6","S",5.39,2.7,2.7,0,0,6562,358,98,0,0,9,25,18622,18622,18622,18622,18622
"TL7","S",0.67,3.15,18.2,0,0,6562,358,98,0,0,12,25,18622,18622,18622,18622,18622
"TL8","S",13.03,3.15,2.47,0,0,5906,358,98,0,0,9,35,18622,18622,18622,18622,18622
"TL9","S",14.94,7.42,9.33,0,0,5906,358,98,0,5249,18,35,18622,18622,18622,18622,18622
"SB1","S",3.37,6.74,24.72,0,0,6562,358,98,0,0,30,25,18622,18622,18622,18622,18622
"SB2","S",10.11,9.55,8.99,0,0,6562,358,98,0,0,30,25,18622,18622,18622,18622,18622
"SB3","S",12.36,6.18,6.74,0,0,6562,358,98,0,0,37,25,18622,18622,18622,18622,18622
"SB4","S",11.8,7.87,11.8,0,0,6562,358,98,0,0,82,25,18622,18622,18622,18622,18622
"NB1","S",1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16
"NB2","S",1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16
"NB3","S",1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16
"NB8","S",1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16
"NB9","S",1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16,1e-16""")



cdef class FuelModel:
    def __init__(self, name, is_dynamic, loads, savs, depth, mx_dead, heats):
        self.is_dynamic = is_dynamic
        self.depth = depth
        self.mx_dead = mx_dead
        for i in range(5):
            self.loads[i] = loads[i]
            self.savs[i] = savs[i]
            self.heat_contents[i] = heats[i]
        

fuel_models = {}
for f in pd.read_csv(fuel_models_csv, index_col=0).itertuples():
    kind = STATIC_FUEL_MODEL if f[1] == 'S' else DYNAMIC_FUEL_MODEL
    fm = FuelModel(f[0], kind, f[2:7], f[7:12], f[12], f[13], f[13:18])
    fuel_models[f[0]] = fm

# An index of fuel model names
fuel_models_names = sorted(list(fuel_models.keys()))

def get_fuel_model_name(fuel_model_id):
    return fuel_models_names[fuel_model_id]

def get_fuel_model_id(fuel_model_name):
    return fuel_models_names.index(fuel_model_name)

moisture_scenarios_csv = StringIO(""""","Moist_1h","Moist_10h","Moist_100h","Moist_Live_Herb","Moist_Live_Woody","Description"
"D1L1",3,4,5,30,60,"Very dry dead FM, fully cured herb"
"D2L2",6,7,8,60,90,"Dry dead FM, 2/3 cured herb"
"D3L3",9,10,11,90,120,"Moderate dead FM 1/3 cured herb"
"D4L4",12,13,14,120,150,"High dead FM, uncured herb"
"D1L2",3,4,5,60,90,"Very dry dead FM, 2/3 cured herb"
"D1L3",3,4,5,90,120,"Very dry dead FM, 1/3 cured herb"
"D1L4",3,4,5,120,150,"Very dry dead FM, uncured herb"
"D2L1",6,7,8,30,60,"Dry dead FM, fully cured herb"
"D2L3",6,7,8,90,120,"Dry dead FM, 1/3 cured herb"
"D2L4",6,7,8,120,150,"Dry dead FM, uncured herb"
"D3L1",9,10,11,30,60,"Moderate dead FM, fully cured herb"
"D3L2",9,10,11,60,90,"Moderate dead FM, 2/3 cured herb"
"D3L4",9,10,11,120,150,"Moderate dead FM, uncured herb"
"D4L1",12,13,15,30,60,"High dead FM, fully cured herb"
"D4L2",12,13,15,60,90,"High dead FM, 2/3 cured herb"
"D4L3",12,13,15,90,120,"High dead FM, 1/3 cured herb"
""")


cdef class MoistureScenario:
    def __init__(self, name, moistures, description):
        for i in range(5):
            self.moistures[i] = moistures[i]
        

moisture_scenarios = {}
for row in pd.read_csv(moisture_scenarios_csv, index_col=0).itertuples():
    ms = MoistureScenario(row[0], row[1:6], row[6])
    moisture_scenarios[row[0]] = ms 

# An index of moisture scenarios names
moisture_scenarios_names = sorted(list(moisture_scenarios.keys()))

def get_moisture_scenario_name(moisture_scenario_id):
    return moisture_scenarios_names[moisture_scenario_id]

def get_moisture_scenario_id(moisture_scenario_name):
    return moisture_scenarios_names.index(moisture_scenario_name)

if __name__ == '__main__':
    pass
