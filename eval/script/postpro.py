
from datetime import datetime
import transformations as tfm
import math
import util

import numpy as np
from scipy.spatial.transform import Rotation


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
                       est_tags : np.array,
                       coverage_threshold : float = 20,
                       tag_threshold : int = 3,

                       is_rescale : bool = False,
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
            int: coverage_threshold, the minimum coverage percentage to consider the data valid in pourcentage e.g. 20 (%).
            int: tag_threshold, the threshold of number of detected tags.
            np.array: est_tags, the array of detected tags.
            bool: is_rescale, if True, the registration will rescale the trajectory.

        Returns:
            np.array: the positions of the camera
            np.array: the rotations of the camera as rotation vectors
    """
    ##############################################################
    # check conditions for allignement

    # select the best idxs for the alignment (only valid poses)
    est_idx_candidates = __select_best_idx(tag_threshold, est_tags, est_coverage)
    if len(est_idx_candidates) < 3:
        print("\033[93m[WARNING]: less than 3 candidates, not possible to define alignement vector \n\033[0m")
        ts_idx_candidates = []
        return src_poss, src_rot_vec, ts_idx_candidates

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
    
    trans_mat = tfm.get_rigid_trans_mat_umeyama(src_poss_candidate, tgt_poss_candidate,
                                                use_scale=is_rescale)
    aligned_src_poss = tfm.transform(trans_mat, src_poss)

    trans_mat = tfm.get_rigid_trans_mat_umeyama(src_rot_vec_candidate, tgt_rot_vec_candidate,
                                                use_scale=is_rescale)
    aligned_src_rot_vec_candidate = tfm.transform(trans_mat, src_rot_vec)

    return aligned_src_poss, aligned_src_rot_vec_candidate, est_idx_candidates