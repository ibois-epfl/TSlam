import metrics

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
                       tag_threshold : int = 3,

                       is_rescale : bool = False,
                       ) -> np.array:
    """
        The main function responsible for register the tslam trajectory to the gt trajectory based
        on umeyama. We only take the poses with a good coverage and a minimum number of a given
        threshold of detected tags. If the candidates are less than 3, we consider the transformation
        not to be possible.

        Args:
            np.array: src_poss, the positions of the camera
            np.array: tgt_poss, the positions of the camera
            np.array: src_rot_vec, the rotations of the camera as rotation vectors
            np.array: tgt_rot_vec, the rotations of the camera as rotation vectors
            np.array: est_coverage, the coverage of the tracking system (0: good, 1: corrupted(lost/drifted/undetected))
            int: tag_threshold, the threshold of number of detected tags.
            np.array: est_tags, the array of detected tags.
            bool: is_rescale, if True, the registration will rescale the trajectory.

        Returns:
            np.array: the positions of the camera
            np.array: the rotations of the camera as rotation vectors
            np.array: the idxs of the poses which have been used for the registration (this value is used only in this function)
    """
    # select the best idxs for the alignment (only valid poses)
    est_idx_candidates = __select_best_idx(tag_threshold, est_tags, est_coverage)
    if len(est_idx_candidates) < 3:
        print(F"\033[93m[WARNING]: less than 3 candidates, not possible to define alignement vector \n\033[0m")
        ts_idx_candidates = []
        return src_poss, src_rot_vec, ts_idx_candidates

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

def align_and_benchmark(src_poss : np.array,
                        tgt_poss : np.array,
                        src_rot_vec : np.array,
                        tgt_rot_vec : np.array,
                        est_coverage : np.array,
                        est_tags : np.array,
                        tag_threshold : int,
                        is_rescale : bool,
                        distances : np.array) -> dict:
    # #######################################################
    # # Trajectory registration
    # #######################################################
    """ 
        At this point we found and apply the rigid transformation that alligns positions and rotations.
        Given the fact that the tslam and optitrack are registered in two different coordinate systems,
        we need to apply a transformation to allign them (only possible because we are doing operation
         per operation). To find the transformation for the alignement we use in both cases, but separately,
         the umeyama algorithm. But first we select the candidate poses to find the transformations. The 
         candidatess are defined by the following criteria:
            - the coverage of the tslam is good, =1 (not corrupted, lost, drifted, undetected)
            - the number of detected tags is above a threshold (e.g.,3). This allows us to have  a better 
              chance of having the best closest to gt positions and rotations in the tslam feed.
        Once the umeyama transformation is found, we apply it to the entire positions and rotations of the tslam
        , not only the candidate positions and rotation.
    """
    src_poss, src_rot_vec, __ts_idx_candidates = align_trajectories(src_poss,
                                                                           tgt_poss,
                                                                           src_rot_vec,
                                                                           tgt_rot_vec,
                                                                           est_coverage,
                                                                           est_tags,
                                                                           tag_threshold,
                                                                           is_rescale
                                                                           )
    # #######################################################
    # # Analytics
    # #######################################################
    """ 
        We calculare here the metrics for the tslam. For the benchmark we use only the values with a true value
        for coverage, the rest are not used in the evaluation. On the other hand a pourcentage of the coverage
        is provided. The metrics are the following:
        - coverage_perc: the percentage of coverage of the tslam (see coverage definition above)
        - tags_mean: the mean of the number of detected tags for each pose
        - drift_poss_mean: the mean of the drift of the position of the tslam with its ground truth
        - drift_rots_mean: the mean of the drift of the rotation of the tslam with its ground truth
        - drift_poss_xyz: the drift of the position of the tslam with its ground truth for each pose in meters.
        - drift_rots_xyz: the drift of the rotation of the tslam with its ground truth for each pose in degrees.
    """
    # if the alignement fails return empty results
    if len(__ts_idx_candidates) == 0:
        return None

    # IMPORTANT: for the metrics we use only the values with a true value for coverage, the rest are not used in the evaluation
    # NB: the coverages values are passed as is to the results output
    src_poss_4_metrics = np.array([src_poss[i] for i in range(len(src_poss)) if est_coverage[i] == True])
    src_rot_vec_4_metrics = np.array([src_rot_vec[i] for i in range(len(src_rot_vec)) if est_coverage[i] == True])
    est_tags_4_metrics = np.array([est_tags[i] for i in range(len(est_tags)) if est_coverage[i] == True])
    tgt_poss_4_metrics = np.array([tgt_poss[i] for i in range(len(tgt_poss)) if est_coverage[i] == True])
    tgt_rot_vec_4_metrics = np.array([tgt_rot_vec[i] for i in range(len(tgt_rot_vec)) if est_coverage[i] == True])
    distances_4_metrics = np.array([distances[i] for i in range(len(distances)) if est_coverage[i] == True])

    mean_coverage_perc = metrics.compute_coverage(est_coverage)
    tags_mean, tags = metrics.compute_tags_nbr_mean(est_tags_4_metrics)
    drift_poss_mean, drift_poss_xyz, drift_poss_mean_xyz = metrics.compute_position_drift(tgt_poss_4_metrics,
                                                               src_poss_4_metrics)
    drift_rots_mean, drift_rots_xyz, drift_rots_mean_xyz = metrics.compute_rotation_drift(tgt_rot_vec_4_metrics,
                                                               src_rot_vec_4_metrics)

    results = {
        "ts_position": src_poss,                     # the transformed positions
        "ts_rotation_vec": src_rot_vec,              # the transformed rotations as rotation vectors
        "ts_idx_candidates": __ts_idx_candidates,    # the idxs of the poses used for the registration
        "mean_coverage_perc": mean_coverage_perc,    # the percentage of coverage of the tslam
        "tags_mean": tags_mean,                      # the mean of the number of detected tags for each pose
        "tags" : tags,                               # the number of detected tags for each pose
        "drift_position_mean": drift_poss_mean,      # the mean of the drift of the position of the tslam with its ground truth
        "drift_rotation_mean": drift_rots_mean,      # the mean of the drift of the rotation of the tslam with its ground truth
        "drift_poss_xyz": drift_poss_xyz,            # the drift of the position of the tslam with its ground truth for each pose in meters and per axe.
        "drift_rots_xyz": drift_rots_xyz,            # the drift of the rotation of the tslam with its ground truth for each pose in degrees and per axe.
        "drift_poss_mean_xyz": drift_poss_mean_xyz,  # the mean of the drift of the position of the tslam with its ground truth for each pose in meters.
        "drift_rots_mean_xyz": drift_rots_mean_xyz,  # the mean of the drift of the rotation of the tslam with its ground truth for each pose in degrees.
        "distances_4_metrics": distances_4_metrics   # the distances at each pose from the start of the trajectory
    }

    return results




