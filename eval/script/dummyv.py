#!/bin/bash

""" 
    This file allows to visualize the single trajectory of the ground truth and the estimated trajectory.
    It is not related to the evaluation but it's only for visualization purpose.
"""

import os
import sys
import numpy as np

import io_stream as ios

global __path_gt__
global __path_pred__


def main() -> None:
    # load the data except the first row
    opti_poss, opti_vec_rots, _, _ = io_stream.process_opti_camera_data(__path_gt__,
                                                                        None, None)



if __name__ == "__main__":
    # get module path of the python script

    __path_gt__ = "/home/as/TSlam/eval/util/dummydata/gt.csv"
    __path_pred__ = "/home/as/TSlam/eval/util/dummydata/pred.csv"
    main()