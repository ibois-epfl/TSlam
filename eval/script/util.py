"""
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
"""

import numpy as np
import os
import itertools

def verify_trajectories_distortion(trajectory1 : np.ndarray,
                                   trajectory2 : np.ndarray,
                                   dec_tolerance : int =6,
                                   verbose : bool =False):
    """
        Verify if two trajectories are the same by checking distances from each point to origin.

        Args:
            trajectory1 (np.ndarray): trajectory1 in positions
            trajectory2 (np.ndarray): trajectory2 in postions
            dec_tolerance (int, optional): decimal tolerance. Defaults to 6.
            verbose (bool, optional): verbose. Defaults to False.
    """
    if len(trajectory1) != len(trajectory2):
        return False
    
    origin1 = trajectory1[0][:3]
    origin2 = trajectory2[0][:3]

    for point1, point2 in zip(trajectory1, trajectory2):
        position1 = point1[:3]
        position2 = point2[:3]

        dist_2_origin1 = np.linalg.norm(position1 - origin1).round(dec_tolerance)
        dist_2_origin2 = np.linalg.norm(position2 - origin2).round(dec_tolerance)

        if verbose:
            print(f"[INFO]: origin1: {origin1}")
            print(f"[INFO]: origin2: {origin2}")
            print(f"[INFO]: position1: {position1}")
            print(f"[INFO]: position2: {position2}")
            print(f"[INFO]: dist_2_origin1: {dist_2_origin1}")
            print(f"[INFO]: dist_2_origin2: {dist_2_origin2}")

        if dist_2_origin1 != dist_2_origin2:
            return False
    return True

def pop_nans(list : np.array) -> np.array:
    list2 = []
    for idx, item in enumerate(list):
        if item != "nan":
            list2.append(item)
    return np.array(list2)

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)