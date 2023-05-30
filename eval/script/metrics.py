import os
import sys
import numpy as np
import transformations as tfm
import util
from dataclasses import dataclass
import io_stream
import csv
from tqdm import tqdm
import itertools

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

class DefaultList(list):
    def default_factory(self):
        return numpy.array([])

@dataclass
class SequenceRes():
    # how many operations are present with this tool in the video
    nbr_operations : int = 0

    mean_drift_position_NONAN = DefaultList()
    mean_drift_position_m : float = 0.0
    mean_drift_rotation_q1 : float = 0.0
    mean_drift_rotation_q3 : float = 0.0
    mean_drift_rotation_min : float = 0.0
    mean_drift_rotation_max : float = 0.0
    mean_drift_rotation_outliers = DefaultList()

    mean_drift_rotation_NONAN = DefaultList()
    mean_drift_rotation_m : float = 0.0
    mean_drift_rotation_q1 : float = 0.0
    mean_drift_rotation_q3 : float = 0.0
    mean_drift_rotation_min : float = 0.0
    mean_drift_rotation_max : float = 0.0
    mean_drift_rotation_outliers = DefaultList()

    tags_NONAN = DefaultList()
    tags_m : float = 0.0
    tags_q1 : float = 0.0
    tags_q3 : float = 0.0
    tags_min : float = 0.0
    tags_max : float = 0.0
    tags_outliers = DefaultList()
    
    coverages = DefaultList()
    coverage_m : float = 0.0
    # TODO: here we need to observe the coverage related to the temporal
    # sequence of the fabrication
    coverage_temporal = DefaultList()  # ??

@dataclass
class SubSequenceRes():
    def __init__(self):
        self.nbr_operations = 0
        self.drift_position_mean = DefaultList()
        self.drift_rotation_mean = DefaultList()
        self.tags = DefaultList()
        self.coverage = DefaultList()
        self.frames = DefaultList()

TOOLS = {
    "circular_sawblade_140": SubSequenceRes(),
    "saber_sawblade_t1": SubSequenceRes(),
    "drill_hinge_cutter_bit_50": SubSequenceRes(),
    "drill_auger_bit_20_200": SubSequenceRes(),
    "drill_auger_bit_25_500": SubSequenceRes(),
    "drill_oblique_hole_bit_40": SubSequenceRes(),
    "st_screw_120": SubSequenceRes(),
    "st_screw_100": SubSequenceRes(),
    "st_screw_80": SubSequenceRes(),
    "st_screw_45": SubSequenceRes()
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
            coverage_IDX = 1
            frames_IDX = 0

            TEMP_seqres = SubSequenceRes()

            for key, tool in TOOLS.items():
                if os.path.basename(csv_path) == f"{key}.csv":
                    for current_rowR, next_rowR in util.pairwise(csv_reader):
                        row = current_rowR[0].split(";")

                        try:
                            row_next = next_rowR[0].split(";")
                            if abs(int(row[frames_IDX]) - int(row_next[frames_IDX])) >= 2:
                                TEMP_seqres.nbr_operations += 1
                        except StopIteration:
                            pass

                        if row[coverage_IDX] == "True":
                            TEMP_seqres.drift_position_mean.append(float(row[drift_poss_mean_IDX]))
                            TEMP_seqres.drift_rotation_mean.append(float(row[drift_rots_mean_IDX]))
                            TEMP_seqres.tags.append(int(row[tags_IDX]))
                        else:
                            TEMP_seqres.drift_position_mean.append(row[drift_poss_mean_IDX])
                            TEMP_seqres.drift_rotation_mean.append(row[drift_rots_mean_IDX])
                            TEMP_seqres.tags.append(row[tags_IDX])
                        TEMP_seqres.coverage.append(row[coverage_IDX])
                        TEMP_seqres.frames.append(row[frames_IDX])
                    
                    TOOLS[key] = TEMP_seqres

def _get_distribution_data(data : np.array) -> tuple[float, float, float, float, list[float], list[float]]:
    """
        This function computes the 5 values for the box plot:
        - mean drift position
        - the 2 quartiles (q1, q3)
        - the min and ma
        - the outliers
        - the cleaned out data without "nans"

        Args:
            data (np.array): the data to compute the distribution

        Returns:
            tuple[float, float, float, float, list[float]]: the 5 values for the box plot + the clean out data
    """
    # excluse all the "nan" values from the analysis
    data_NONAN = util.pop_nans(data)

    data_m = np.mean(data_NONAN)
    data_q1 = np.quantile(data_NONAN, 0.25)
    data_q3 = np.quantile(data_NONAN, 0.75)
    data_min = np.min(data_NONAN)
    data_max = np.max(data_NONAN)
    data_outliers = []
    for idx, drift in enumerate(data_NONAN):
        if drift < data_min or drift > data_max:
            data_outliers.append(drift)
    return data_NONAN, data_m, data_q1, data_q3, data_min, data_max, data_outliers

def compute_fab_results(out_dir : str) -> None:
    """
        This function outputs the overview of the entire sequence based on the results computed

        Args:
            out_dir (str): the directory where the results are saved.
            sess_name (str): the name of the session
    """
    csv_paths = io_stream.get_subseq_metrics_csv(out_dir)

    _load_tools(csv_paths)

    for id, res in TOOLS.items():
        if res.drift_position_mean.__len__() == 0:
            print(f"NO DATA FOR {id}")
            continue

        (drift_position_mean_NONAN,
        drift_position_mean_m,
        drift_position_mean_q1,
        drift_position_mean_q3,
        drift_position_mean_min,
        drift_position_mean_max,
        drift_position_mean_outliers) = _get_distribution_data(res.drift_position_mean)
        (drift_rotation_mean_NONAN,
        drift_rotation_mean_m,
        drift_rotation_mean_q1,
        drift_rotation_mean_q3,
        drift_rotation_mean_min,
        drift_rotation_mean_max,
        drift_rotation_mean_outliers) = _get_distribution_data(res.drift_rotation_mean)
        (tags_NONAN,
        tags_m,
        tags_q1,
        tags_q3,
        tags_min,
        tags_max,
        tags_outliers) = _get_distribution_data(res.tags)

        TEMP_seqres = SequenceRes()
        TEMP_seqres.nbr_operations = res.nbr_operations
        TEMP_seqres.mean_drift_position_NONAN = drift_position_mean_NONAN
        TEMP_seqres.mean_drift_position_m = drift_position_mean_m
        TEMP_seqres.mean_drift_position_q1 = drift_position_mean_q1
        TEMP_seqres.mean_drift_position_q3 = drift_position_mean_q3
        TEMP_seqres.mean_drift_position_min = drift_position_mean_min
        TEMP_seqres.mean_drift_position_max = drift_position_mean_max
        TEMP_seqres.mean_drift_position_outliers = drift_position_mean_outliers
        TEMP_seqres.mean_drift_rotation_NONAN = drift_rotation_mean_NONAN
        TEMP_seqres.mean_drift_rotation_m = drift_rotation_mean_m
        TEMP_seqres.mean_drift_rotation_q1 = drift_rotation_mean_q1
        TEMP_seqres.mean_drift_rotation_q3 = drift_rotation_mean_q3
        TEMP_seqres.mean_drift_rotation_min = drift_rotation_mean_min
        TEMP_seqres.mean_drift_rotation_max = drift_rotation_mean_max
        TEMP_seqres.mean_drift_rotation_outliers = drift_rotation_mean_outliers
        TEMP_seqres.tags_NONAN = tags_NONAN
        TEMP_seqres.tags_m = tags_m
        TEMP_seqres.tags_q1 = tags_q1
        TEMP_seqres.tags_q3 = tags_q3
        TEMP_seqres.tags_min = tags_min
        TEMP_seqres.tags_max = tags_max
        TEMP_seqres.tags_outliers = tags_outliers
        TEMP_seqres.coverages = res.coverage
        TEMP_seqres.coverage_m = compute_coverage(res.coverage)
        
        TOOLS[id] = TEMP_seqres

        print(f"====================== {id} =====================")
        print(f"nbr_operations: {res.nbr_operations}")
        print("--------------------------------------------------")
        print(f"drift_position_mean_m: {drift_position_mean_m}")
        print(f"drift_position_mean_q1: {drift_position_mean_q1}")
        print(f"drift_position_mean_q3: {drift_position_mean_q3}")
        print(f"drift_position_mean_min: {drift_position_mean_min}")
        print(f"drift_position_mean_max: {drift_position_mean_max}")
        print(f"drift_position_mean_outliers: {drift_position_mean_outliers}")
        print("--------------------------------------------------")
        print(f"drift_rotation_mean_m: {drift_rotation_mean_m}")
        print(f"drift_rotation_mean_q1: {drift_rotation_mean_q1}")
        print(f"drift_rotation_mean_q3: {drift_rotation_mean_q3}")
        print(f"drift_rotation_mean_min: {drift_rotation_mean_min}")
        print(f"drift_rotation_mean_max: {drift_rotation_mean_max}")
        print(f"drift_rotation_mean_outliers: {drift_rotation_mean_outliers}")
        print("--------------------------------------------------")
        print(f"tags_m: {tags_m}")
        print(f"tags_q1: {tags_q1}")
        print(f"tags_q3: {tags_q3}")
        print(f"tags_min: {tags_min}")
        print(f"tags_max: {tags_max}")
        print(f"tags_outliers: {tags_outliers}")
        print("--------------------------------------------------")
        print(f"coverage: {compute_coverage(res.coverage)}")
        print("--------------------------------------------------")

