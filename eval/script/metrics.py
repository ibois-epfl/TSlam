import numpy as np
import transformations as tfm

def compute_coverage(ts_coverages : np.array) -> float:
    """
        The function computes the percentage of coverage of the tslam.
        Coverage is defined as the percentage of poses which are not:
        - corrupted
        - lost
        - drifted
        - undetected
        - with a minimum threshold of detected tags

        Args:
            ts_coverages (np.array): the coverage of the tracking system (0: good, 1: corrupted(lost/drifted/undetected))
    """
    return (np.sum(ts_coverages == 1) / len(ts_coverages) * 100).round(2)

def compute_tags_nbr_mean(tags : np.array(int)):
    """
        The function computes the mean of the number of tags detected by the tslam during the trajectory.
    """
    return np.mean(tags).round(1)

def compute_position_drift(gt_poss : np.array,
                           est_poss : np.array) -> tuple[float, np.array(float)]:
    """
        The function computes the drift of the position of the tslam with its ground truth.
        Drift is defined as the average distance between the position of the tslam and its ground truth.

        Args:
            gt_poss (np.array): the positions of the camera
            est_poss (np.array): the positions of the camera

        Returns:
            float: the averaged drift of the position of the tslam with its ground truth in meters
            np.array(float): the drift of the position of the tslam with its ground truth for each pose in meters.
            np.array(float): the drift value for each pose and each axis (x,y,z) in meters.
    """
    # compute the drift of the position of the tslam with its ground truth
    drifts = np.linalg.norm(gt_poss - est_poss, axis=1)
    drift_mean = np.mean(drifts)

    # get the drift seperately for each axis and for each pose
    drifts_x = gt_poss[:, 0] - est_poss[:, 0]
    drifts_y = gt_poss[:, 1] - est_poss[:, 1]
    drifts_z = gt_poss[:, 2] - est_poss[:, 2]
    drifts_xyz = np.array([drifts_x, drifts_y, drifts_z]).T

    return drift_mean, drifts_xyz

def compute_rotation_drift(gt_rots : np.array,
                           est_rots : np.array) -> tuple[float, np.array(float)]:
    """
        The function computes the drift of the rotation of the tslam with its ground truth.
        Drift is defined as the average distance between the rotation of the tslam and its ground truth.

        Args:
            gt_rots (np.array): the rotation of the camera in form of a rotation vector
            est_rots (np.array): the rotation of the camera in form of a rotation vector

        Returns:
            float: the averaged drift of the rotation of the tslam with its ground truth in degrees
            np.array(float): the drift of the rotation of the tslam with its ground truth for each pose in degrees.
            np.array(float): the drift value for each pose and each axis (x,y,z) in degrees.
    """
    drifts = []
    for idx, gt_pos in enumerate(gt_rots):
        gt_rot = tfm.rotation_vector_to_matrix(tfm.rotation_vector_to_matrix(gt_rots[idx]))
        est_rot = tfm.rotation_vector_to_matrix(tfm.rotation_vector_to_matrix(est_rots[idx]))
        drifts.append(np.linalg.norm(gt_rot - est_rot))
    
    drift_mean = np.mean(drifts)

    drifts_x = gt_rots[:, 0] - est_rots[:, 0]
    drifts_y = gt_rots[:, 1] - est_rots[:, 1]
    drifts_z = gt_rots[:, 2] - est_rots[:, 2]
    drifts_xyz = np.array([drifts_x, drifts_y, drifts_z]).T

    return drift_mean, drifts_xyz