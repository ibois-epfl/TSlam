import os
import sys
import numpy as np
import transformations as tfm
import util
from dataclasses import dataclass
import io_stream
import csv
from tqdm import tqdm

# ================================================================================================
# ================================= sub-sequence metrics =========================================
# ================================================================================================

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
            ts_coverage_perc (float): a single value representing the percentage coverage value per sub-sequence
    """
    mean_coverage_perc = (np.sum(ts_coverages == 1) / len(ts_coverages) * 100).round(2)
    return mean_coverage_perc

def compute_tags_nbr_mean(tags : np.array(int)) -> float:
    """
        The function computes the mean of the number of tags detected by the tslam during the trajectory.
    
        Args:
            tags (np.array(int)): the number of tags detected by the tslam for each pose

        Returns:
            float: the number of tags per pose detected
            np.array(int): the tags per detected pose
    """
    mean = np.mean(tags).round(1)
    tags = np.array(tags)
    return mean, tags

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
            np.array(float): the drift value for each pose and each axis (x,y,z) in meters.
            np.array(float): the mean drift value for each pose across all axis in meters.
    """
    drifts = np.linalg.norm(gt_poss - est_poss, axis=1)
    drift_mean = np.mean(drifts)

    drifts_x = gt_poss[:, 0] - est_poss[:, 0]
    drifts_y = gt_poss[:, 1] - est_poss[:, 1]
    drifts_z = gt_poss[:, 2] - est_poss[:, 2]
    drifts_xyz = np.array([drifts_x, drifts_y, drifts_z]).T

    drifts_mean_xyz = np.mean(drifts_xyz, axis=1)

    return drift_mean, drifts_xyz, drifts_mean_xyz

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
            np.array(float): the drift mean value for each pose in degrees.
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

    drifts_mean_xyz = np.mean(drifts_xyz, axis=1)

    return drift_mean, drifts_xyz, drifts_mean_xyz

# ================================================================================================
# ================================== sequence metrics ============================================
# ================================================================================================

# @dataclass
# class ToolResults():
#     # wether or not the tool is present in the video fabrication recording
#     is_present : bool = False
#     # the number of times the tool is used in the video fabrication recording
#     nbr_occurences : int = 0
#     # the mean coverage percentage of the tool
#     mean_coverage_percentage : float = 0.0
#     # the mean number of tags detected by the tslam
#     mean_tags_mean : float = 0.0
#     # the mean drift of the position of the tslam with its ground truth
#     mean_drift_position_mean : float = 0.0
#     # the mean drift of the rotation of the tslam with its ground truth
#     mean_drift_rotation_mean : float = 0.0
#     # the number of times where reallignement was not possible
#     number_reallignement_not_possible : int = 0

class DefaultList(list):
    def default_factory(self):
        return []
@dataclass
class SequenceResults():
    def __init__(self):
        self.drift_position_mean = DefaultList()
        self.drift_rotation_mean = DefaultList()
        self.tags = DefaultList()
        self.coverage_percentage = DefaultList()
        self.frames = DefaultList()
    # mean_drift_position_mean : list[float] = []
    # mean_drift_rotation_mean : list[float] = []
    # tags_mean : list[int] = []
    # coverage_percentage : list[float] = []
    # frames : list[int] = []


TOOLS = {
    "circular_sawblade_140": SequenceResults(),
    "saber_sawblade_t1": SequenceResults(),
    "drill_hinge_cutter_bit_50": SequenceResults(),
    "drill_auger_bit_20_200": SequenceResults(),
    "drill_auger_bit_25_500": SequenceResults(),
    "drill_oblique_hole_bit_40": SequenceResults(),
    "st_screw_120": SequenceResults(),
    "st_screw_100": SequenceResults(),
    "st_screw_80": SequenceResults(),
    "st_screw_45": SequenceResults()
}

def _load_tools(csv_paths : str) -> None:
    """ Loads the csv sub-sequence results into memory of tools dict """
    for csv_path in tqdm(csv_paths, total=len(csv_paths)):
        with open(csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            next(csv_reader)
            drift_poss_mean_IDX = 6
            drift_rots_mean_IDX = 10
            tags_IDX = 2
            coverage_percentage_IDX = 1
            frames_IDX = 0

            TEMP_seqres = SequenceResults()

            for key, tool in TOOLS.items():
                if os.path.basename(csv_path) == f"{key}.csv":
                    for row in csv_reader:
                        row = row[0].split(";")
                        if row[coverage_percentage_IDX] == "True":
                            TEMP_seqres.drift_position_mean.append(float(row[drift_poss_mean_IDX]))
                            TEMP_seqres.drift_rotation_mean.append(float(row[drift_rots_mean_IDX]))
                            TEMP_seqres.tags.append(int(row[tags_IDX]))
                        else:
                            TEMP_seqres.drift_position_mean.append(row[drift_poss_mean_IDX])
                            TEMP_seqres.drift_rotation_mean.append(row[drift_rots_mean_IDX])
                            TEMP_seqres.tags.append(row[tags_IDX])
                        TEMP_seqres.coverage_percentage.append(row[coverage_percentage_IDX])
                        TEMP_seqres.frames.append(row[frames_IDX])
                    TOOLS[key] = TEMP_seqres

def compute_fab_results(out_dir : str) -> None:
    """
        This function outputs the overview of the entire sequence based on the results computed

        Args:
            out_dir (str): the directory where the results are saved.
            sess_name (str): the name of the session
    """
    csv_paths = io_stream.get_subseq_metrics_csv(out_dir)

    _load_tools(csv_paths)

    # # print all the tools keys and items
    # for key, item in TOOLS.items():
    #     print(key, "driftpos", item.drift_position_mean)
    #     print(key, "driftrot", item.drift_rotation_mean)
    #     print(key, "tags", item.tags)
    #     print(key, "coverage", item.coverage_percentage)
    #     print(key, "fames", item.frames)


