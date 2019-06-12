#! /usr/bin/env python3
#  Copyright (c) 2019, CNRS-LAAS
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#  list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#  this list of conditions and the following disclaimer in the documentation
#  and/or other materials provided with the distribution.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import argparse
import datetime
import logging
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import pytz
import os
import os.path

if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description='Obtain UAV pose list at picture timestamps')
    parser.add_argument(
        'estimated_state', nargs=1, type=argparse.FileType('r'),
        help='EstimatedState as csv file')
    parser.add_argument(
        'photo_paths', nargs='+', type=str,
        help='Photo files to process')
    parser.add_argument(
        'result_file', nargs=1, type=argparse.FileType('w'),
        help='CSV file where poses are stored to')
    args = parser.parse_args()

    logger = logging.getLogger(__name__)
    logger.info("Loading EstimatedState from: " % args.estimated_state)

    dateparser = lambda dates: [pd.datetime.fromtimestamp(float(d), tz=pytz.utc) for d in dates]
    es = pd.read_csv(args.estimated_state[0], index_col=0, skipinitialspace=True,
                     parse_dates=["timestamp (seconds since 01/01/1970)"], date_parser=dateparser)

    logger.info("Analyze %s photos" % len(args.photo_paths))
    new_ts = []
    new_ts_float = []
    for photo_path in args.photo_paths:
        print(photo_path)
        new_ts.append(pd.Timestamp(datetime.datetime.fromtimestamp(float(os.path.splitext(os.path.basename(photo_path))[0]), tz=pytz.utc)))
        new_ts_float.append(float(os.path.splitext(os.path.basename(photo_path))[0]))

    es2 = es.reindex(es.index.append(pd.Index(new_ts)).sort_values())
    es3 = es2.interpolate(method="time")
    es3 = es3[~es3.index.duplicated()]
    es4 = es3.reindex(pd.Index(new_ts))
    logger.info("Writing UAV poses to %s" % args.result_file[0])
    es4.index = new_ts_float
    es4.to_csv(args.result_file[0], index_label="timestamp", index=True, columns=["lat (rad)", "lon (rad)", "height (m)", "phi (rad)", "theta (rad)", "psi (rad)"])
    logger.info("End")




