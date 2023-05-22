
from datetime import datetime
import transformations as tfm
import math
import util

import numpy as np
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt  # FIXME: TEMP
import argparse  # FIXME: TEMP


def __select_best_idx(_tag_threshold,
                      _ts_tags : np.array,
                      _ts_coverages : np.array) -> np.array:
    """
        It selects the idxs of poses with the highest score of detected tags
        
        Args:
            _tag_threshold (int, optional): the threshold of number of detected tags.
            _ts_tags (np.array): the array of detected tags.
            _ts_coverages (np.array): the array of coverage of the signal.

        Returns:
            np.array: the array of idxs of the poses with the highest score of detected tags.
    """
    ts_idx_candidates = []

    for idx, tag in enumerate(_ts_tags):
        if tag >= _tag_threshold and _ts_coverages[idx] == 1:
            ts_idx_candidates.append(idx)
    if len(ts_idx_candidates) < 2:
        ts_idx_candidates = []

    return np.array(ts_idx_candidates)

def align_trajectories(src_poss : np.array,
                       tgt_poss : np.array,
                       src_rot_vec : np.array,
                       tgt_rot_vec : np.array,

                       est_coverage : np.array,
                       coverage_threshold : float,
                       tag_threshold : int,
                       est_tags : np.array,
                       ) -> np.array:
    """
        The main function responsible for register the tslam trajectory to the gt trajectory.
        It is based on the following steps:
            1. select the best, farest 2 idxs for the alignment (only valid poses)
            2. apply a rotation to the src_poss to align it with the tgt_poss
            3. apply a translation to the src_poss to align it with the tgt_poss
            4. apply a rotation yaw-only to allign the rotation vectors of the highest best pose

        Args:
            np.array: src_poss, the positions of the camera
            np.array: tgt_poss, the positions of the camera
            np.array: src_rot_vec, the rotations of the camera as rotation vectors
            np.array: tgt_rot_vec, the rotations of the camera as rotation vectors
            np.array: est_coverage, the coverage of the tracking system (0: good, 1: corrupted(lost/drifted/undetected))
            int: coverage_threshold, the minimum coverage percentage to consider the data valid.
            int: tag_threshold, the threshold of number of detected tags.
            np.array: est_tags, the array of detected tags.

        Returns:
            np.array: the positions of the camera
            np.array: the rotations of the camera as rotation vectors
    """
    ##############################################################
    # check conditions for allignement

    # check coverage
    coverage_perc : float = (np.sum(est_coverage == 1) / len(est_coverage) * 100)
    print(f">>>>>>>>>>Coverage: {coverage_perc.round(1)} %")
    if coverage_perc < coverage_threshold:
        print("\033[93m[WARNING]: Low coverage detected, skipping alignment\n\033[0m")
        return src_poss, src_rot_vec

    # select the best idxs for the alignment (only valid poses)
    est_idx_candidates = __select_best_idx(tag_threshold, est_tags, est_coverage)
    print(f"est_idx_candidates: {est_idx_candidates}")
    if len(est_idx_candidates) < 2:
        print("\033[93m[WARNING]: less than 2 candidates, not possible to define alignement vector \n\033[0m")
        return src_poss, src_rot_vec

    
    # # select the two highest, and farest away idx based on the tags values and the coverage
    # est_idx_candidates = est_idx_candidates[np.argsort(est_tags[est_idx_candidates])][::-1][:2]
    # est_idx_candidates = est_idx_candidates[np.argsort(src_poss[est_idx_candidates, 0])][::-1][:2]


    ##############################################################
    if len(est_idx_candidates) != 0:
        src_poss_candidate = src_poss.copy()[est_idx_candidates]
        tgt_poss_candidate = tgt_poss.copy()[est_idx_candidates]
        est_tags_candidate = est_tags.copy()[est_idx_candidates]
        src_rot_vec_candidate = src_rot_vec.copy()[est_idx_candidates]
        tgt_rot_vec_candidate = tgt_rot_vec.copy()[est_idx_candidates]
    else:
        src_poss_candidate = src_poss.copy()
        tgt_poss_candidate = tgt_poss.copy()
        est_tags_candidate = est_tags.copy()
        src_rot_vec_candidate = src_rot_vec.copy()
        tgt_rot_vec_candidate = tgt_rot_vec.copy()
    
    ##############################################################
    # WORKING-SEMI METHOD: trans + rot
    # rotation A
    vec_A = tgt_poss_candidate[-1] - tgt_poss_candidate[0]
    vec_B = src_poss_candidate[-1] - src_poss_candidate[0]
    rot = tfm.rotation_matrix_from_vectors(vec_B, vec_A)

    aligned_src_poss_r = np.dot(rot.as_matrix(), src_poss.copy().T).T
    src_rot_vec_r = np.dot(rot.as_matrix(), src_rot_vec.copy().T).T
    src_poss_candidate_r = np.dot(rot.as_matrix(), src_poss_candidate.copy().T).T
    # apply the rotation to the vector rotations
    src_rot_vec_candidate_r = np.dot(rot.as_matrix(), src_rot_vec_candidate.copy().T).T

    # translation
    point_start = src_poss_candidate_r[0]
    point_end = tgt_poss_candidate[0]
    trans = point_end - point_start
    aligned_src_poss_r_t = aligned_src_poss_r.copy() + trans
    src_poss_candidate_r_t = src_poss_candidate_r.copy() + trans

    # >>>>>>>>>>>>>

    # rotation B

    # rotation axis
    axis_rot = src_poss_candidate_r_t[-1] - src_poss_candidate_r_t[0]

    # rotation angle
    tgt_poss_axis = tgt_rot_vec_candidate[0]
    src_poss_axis = src_rot_vec_candidate_r[0]
    plane_pp_to_axis = tfm.get_perpendicular_plane_to_vector(axis_rot)
    angle_rot = tfm.angle_between_vectors(src_poss_axis, tgt_poss_axis, plane_pp_to_axis)

    rot = tfm.rotate_around_axis(axis_rot, angle_rot)
    rot_neg = tfm.rotate_around_axis(axis_rot, -angle_rot)

    temp_src_rot = np.dot(rot.as_matrix(), src_rot_vec_candidate_r.copy().T).T
    temp_src_rot_neg = np.dot(rot_neg.as_matrix(), src_rot_vec_candidate_r.copy().T).T

    # FIXME: this is not clear
    # check which rotation is the best
    if np.linalg.norm(temp_src_rot - tgt_rot_vec_candidate) < np.linalg.norm(temp_src_rot_neg - tgt_rot_vec_candidate):
        rot = rot
    else:
        rot = rot_neg

    aligned_src_poss_r_t_r = np.dot(rot.as_matrix(), aligned_src_poss_r_t.copy().T).T
    src_rot_vec_r_r = np.dot(rot.as_matrix(), src_rot_vec_r.copy().T).T
    src_poss_candidate_r_t_r = np.dot(rot.as_matrix(), src_poss_candidate_r_t.copy().T).T
    src_rot_vec_candidate_r_r = np.dot(rot.as_matrix(), src_rot_vec_candidate_r.copy().T).T


    # translation B
    point_start = src_poss_candidate_r_t_r[0]
    point_end = tgt_poss_candidate[0]
    trans = point_end - point_start
    aligned_src_poss_r_t_r_t = aligned_src_poss_r_t_r.copy() + trans
    src_poss_candidate_r_t_r_t = src_poss_candidate_r_t_r.copy() + trans



    ##############################################################
    # # # >> CHECKS for deformation
    assert util.verify_trajectories_distortion(src_poss, aligned_src_poss_r, verbose=False), "[ERROR]: The trajectories are distorted r"
    assert util.verify_trajectories_distortion(src_poss, aligned_src_poss_r_t, verbose=False), "[ERROR]: The trajectories are distorted rxt"
    assert util.verify_trajectories_distortion(src_poss, aligned_src_poss_r_t_r, verbose=False), "[ERROR]: The trajectories are distorted rxtxr"
    assert util.verify_trajectories_distortion(src_poss, aligned_src_poss_r_t_r_t, verbose=False), "[ERROR]: The trajectories are distorted rxtxrt"

    assert len(aligned_src_poss_r_t_r_t) == len(src_rot_vec_r_r), "[ERROR]: not same length"

    return aligned_src_poss_r_t_r_t, src_rot_vec_r_r, est_idx_candidates