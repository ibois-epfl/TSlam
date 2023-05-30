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