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
    def __init__(self):
        # ----------- operations/time -------------
        self.nbr_operations : int = 0
        self._frames_operations_start_end = DefaultList()  # <----
        self._idx_operations_start_end = DefaultList()     # <----
        self.average_time_per_operation : float = 0.0
        # ----------- drift position -------------
        self._mean_drift_position_NONAN = DefaultList()    # <----
        self.mean_drift_position_m : float = 0.0
        self.mean_drift_position_q1 : float = 0.0
        self.mean_drift_position_q3 : float = 0.0
        self.mean_drift_position_min : float = 0.0
        self.mean_drift_position_max : float = 0.0
        self.mean_drift_position_outliers = DefaultList()  # <----
        # ----------- drift rotation -------------
        self._mean_drift_rotation_NONAN = DefaultList()    # <----
        self.mean_drift_rotation_m : float = 0.0
        self.mean_drift_rotation_q1 : float = 0.0
        self.mean_drift_rotation_q3 : float = 0.0
        self.mean_drift_rotation_min : float = 0.0
        self.mean_drift_rotation_max : float = 0.0
        self.mean_drift_rotation_outliers = DefaultList()  # <----
        # ----------- tags -------------
        self._tags_NONAN = DefaultList()                   # <----
        self.tags_m : float = 0.0
        self.tags_q1 : float = 0.0
        self.tags_q3 : float = 0.0
        self.tags_min : float = 0.0
        self.tags_max : float = 0.0
        self.tags_outliers = DefaultList()                 # <----
        # ----------- coverage -------------
        # the coverages values of all the frames of all sub-sequences per type of tool (0 or 1)
        self._coverages = DefaultList()                    # <----
        # the median value in percentage (e.g. 0.12)
        self.coverage_m : float = 0.0
        # quintiles mean coverage percentage values for each sub-sequence
        self._coverage_mean_per_fabrication_quintiles = DefaultList()  # <----
        # quintiles percentage mean values considering each sub-sequence as a single value and then meaned
        self.mean_coverage_perc_quintiles = DefaultList()

@dataclass
class SubSequenceRes():
    def __init__(self):
        self.nbr_operations = 0
        self._frames_operations_start_end = DefaultList()
        self._idx_operations_start_end = DefaultList()
        self.drift_position_mean = DefaultList()
        self.drift_rotation_mean = DefaultList()
        self.tags = DefaultList()
        self.coverage = DefaultList()
        self.frames = DefaultList()

TOOLS = {                          # early results    mid results
    "circular_sawblade_140" :      [SubSequenceRes(), SequenceRes()],
    "saber_sawblade_t1" :          [SubSequenceRes(), SequenceRes()],
    "drill_hinge_cutter_bit_50" :  [SubSequenceRes(), SequenceRes()],
    "drill_auger_bit_20_200" :     [SubSequenceRes(), SequenceRes()],
    "drill_auger_bit_25_500" :     [SubSequenceRes(), SequenceRes()],
    "drill_oblique_hole_bit_40" :  [SubSequenceRes(), SequenceRes()],
    "st_screw_120" :               [SubSequenceRes(), SequenceRes()],
    "st_screw_100" :               [SubSequenceRes(), SequenceRes()],
    "st_screw_80" :                [SubSequenceRes(), SequenceRes()],
    "st_screw_45" :                [SubSequenceRes(), SequenceRes()]
}

def _load_tools(csv_paths : str) -> None:
    """ Loads the csv sub-sequence results into memory of tools dict """
    for csv_path in tqdm(csv_paths, total=len(csv_paths)):
        total_csv_rows : int = -1
        with open(csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            next(csv_reader)
            for row in csv_reader:
                total_csv_rows += 1

        with open(csv_path, 'r') as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            next(csv_reader)
            drift_poss_mean_IDX = 6
            drift_rots_mean_IDX = 10
            tags_IDX = 2
            coverage_IDX = 1
            frames_IDX = 0

            TEMP_seqres = SubSequenceRes()
            TEMP_seqres.nbr_operations = 1

            for key, item in TOOLS.items():
                if os.path.basename(csv_path) == f"{key}.csv":
                    row_counter = 0
                    for current_rowR, next_rowR in util.pairwise(csv_reader):
                        row = current_rowR[0].split(";")
                        if row_counter == 0:
                            first_frame = int(row[frames_IDX])
                            ifrst_idx = 0

                        try:
                            row_next = next_rowR[0].split(";")
                            if (abs(int(row[frames_IDX]) - int(row_next[frames_IDX])) >= 2):
                                TEMP_seqres.nbr_operations += 1
                                TEMP_seqres._frames_operations_start_end.append((first_frame,
                                                                                int(row[frames_IDX])))
                                first_frame = int(row_next[frames_IDX])
                                TEMP_seqres._idx_operations_start_end.append((ifrst_idx,
                                                                              row_counter+1))
                                ifrst_idx = row_counter+1
                            if row_counter+1 == total_csv_rows:
                                TEMP_seqres._frames_operations_start_end.append((first_frame,
                                                                                int(row[frames_IDX])))
                                TEMP_seqres._idx_operations_start_end.append((ifrst_idx,
                                                                              row_counter+1))
                        except StopIteration:
                            pass
                        # get the last csv row

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

                        row_counter += 1
                    
                    TOOLS[key][0] = TEMP_seqres

def _get_distribution_data(data : np.array, is_tag=False) -> tuple[float, float, float, float, list[float], list[float]]:
    """
        This function computes the 5 values for the box plot:
        - mean drift position
        - the 2 quartiles (q1, q3)
        - the min and ma
        - the outliers
        - the cleaned out data without "nans"

        Args:
            data (np.array): the data to compute the distribution
            is_tag (bool): if the data is a tag or not, the min data will be set to 0 if negative

        Returns:
            tuple[float, float, float, float, list[float]]: the 5 values for the box plot + the clean out data
    """
    # excluse all the "nan" values from the analysis
    data_NONAN = util.pop_nans(data)

    data_m = np.mean(data_NONAN)
    data_q1 = np.quantile(data_NONAN, 0.25)
    data_q3 = np.quantile(data_NONAN, 0.75)
    iqr = data_q3 - data_q1
    data_min = data_q1 - (1.5 * iqr)
    data_max = data_q3 + (1.5 * iqr)

    # in case of tags we do not have negative values
    if data_min < 0:
        data_min = 0

    data_outliers = []
    for idx, drift in enumerate(data_NONAN):
        if drift < data_min or drift > data_max:
            data_outliers.append(drift)
    return data_NONAN, data_m, data_q1, data_q3, data_min, data_max, data_outliers

def _get_average_time_per_operation(frames_operations_start_end : list[tuple[int, int]],
                                    is_round : bool = True) -> float:
    """ It computes the average time of the operations by knowing that the video was 30fps"""
    total_time = 0
    for start, end in frames_operations_start_end:
        f_range = end - start
        for i in range(f_range):
            total_time += 1/30
    if is_round:
        return round(total_time, 2)
    return total_time

def _get_coverage_quintiles(coverages : np.array,
                            operations_start_end : np.array) -> list[float, float, float, float, float]:
    """
        This function computes the coverage quintiles for each sub-sequence.
        The coverage quintiles are computed by dividing the sub-sequence in 5 parts and then
        computing the coverage for each part and then meaning the coverage for each quintile.

        Args:
            coverages (np.array): the coverage of the entire sequence
            operations_start_end (np.array): the start and end of each operation to resubdivide the coverages

        Returns:
            list[float, float, float, float, float]: the coverage quintiles
    """
    # get the coverage in percentage for each frame divided in 5 parts
    coverage_per_subsequences = []

    # based on the _frames_operations_start_end compute the quintiles
    for start, end in operations_start_end:
        # get the coverage for the current sub-sequence
        coverage_per_subsequences.append(coverages[start:end])
    
    # get the coverage in percentage for each frame divided in 5 parts
    coverage_perc_qt1 = []
    coverage_perc_qt2 = []
    coverage_perc_qt3 = []
    coverage_perc_qt4 = []
    coverage_perc_qt5 = []

    # coverage mean per sub-sequence
    coverage_mean_per_fabrication_quintiles = []

    # for each sub-sequence
    for coverage_per_subsequence in coverage_per_subsequences:
        # get one fifth of the array TOOL[id]._coverages
        quintiles = np.array_split(coverage_per_subsequence, 5)
        qt1 = quintiles[0]
        qt2 = quintiles[1]
        qt3 = quintiles[2]
        qt4 = quintiles[3]
        qt5 = quintiles[4]

        # get the coverage in percentage for each frame divided in 5 parts
        coverage_perc_qt1.append(compute_coverage(qt1))
        coverage_perc_qt2.append(compute_coverage(qt2))
        coverage_perc_qt3.append(compute_coverage(qt3))
        coverage_perc_qt4.append(compute_coverage(qt4))
        coverage_perc_qt5.append(compute_coverage(qt5))

        coverage_mean_per_fabrication_quintiles.append([compute_coverage(qt1),
                                                        compute_coverage(qt2),
                                                        compute_coverage(qt3),
                                                        compute_coverage(qt4),
                                                        compute_coverage(qt5)])

    # mean the coverage for each quintile
    mean_coverage_perc_qt1 = np.mean(coverage_perc_qt1)
    mean_coverage_perc_qt2 = np.mean(coverage_perc_qt2)
    mean_coverage_perc_qt3 = np.mean(coverage_perc_qt3)
    mean_coverage_perc_qt4 = np.mean(coverage_perc_qt4)
    mean_coverage_perc_qt5 = np.mean(coverage_perc_qt5)

    # store the values in the sequence res
    return [mean_coverage_perc_qt1,
            mean_coverage_perc_qt2,
            mean_coverage_perc_qt3,
            mean_coverage_perc_qt4,
            mean_coverage_perc_qt5], coverage_mean_per_fabrication_quintiles

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
        if res[0].drift_position_mean.__len__() == 0:
            print(f"NO DATA FOR {id}")
            continue

        (drift_position_mean_NONAN,
        drift_position_mean_m,
        drift_position_mean_q1,
        drift_position_mean_q3,
        drift_position_mean_min,
        drift_position_mean_max,
        drift_position_mean_outliers) = _get_distribution_data(res[0].drift_position_mean)
        (drift_rotation_mean_NONAN,
        drift_rotation_mean_m,
        drift_rotation_mean_q1,
        drift_rotation_mean_q3,
        drift_rotation_mean_min,
        drift_rotation_mean_max,
        drift_rotation_mean_outliers) = _get_distribution_data(res[0].drift_rotation_mean)
        (tags_NONAN,
        tags_m,
        tags_q1,
        tags_q3,
        tags_min,
        tags_max,
        tags_outliers) = _get_distribution_data(res[0].tags, is_tag=True)

        TEMP_seqres = SequenceRes()
        TEMP_seqres.nbr_operations = res[0].nbr_operations
        TEMP_seqres._frames_operations_start_end = res[0]._frames_operations_start_end
        TEMP_seqres._idx_operations_start_end = res[0]._idx_operations_start_end
        TEMP_seqres.average_time_per_operation = _get_average_time_per_operation(res[0]._frames_operations_start_end)
        TEMP_seqres._mean_drift_position_NONAN = drift_position_mean_NONAN
        TEMP_seqres.mean_drift_position_m = drift_position_mean_m
        TEMP_seqres.mean_drift_position_q1 = drift_position_mean_q1
        TEMP_seqres.mean_drift_position_q3 = drift_position_mean_q3
        TEMP_seqres.mean_drift_position_min = drift_position_mean_min
        TEMP_seqres.mean_drift_position_max = drift_position_mean_max
        TEMP_seqres.mean_drift_position_outliers = drift_position_mean_outliers
        TEMP_seqres._mean_drift_rotation_NONAN = drift_rotation_mean_NONAN
        TEMP_seqres.mean_drift_rotation_m = drift_rotation_mean_m
        TEMP_seqres.mean_drift_rotation_q1 = drift_rotation_mean_q1
        TEMP_seqres.mean_drift_rotation_q3 = drift_rotation_mean_q3
        TEMP_seqres.mean_drift_rotation_min = drift_rotation_mean_min
        TEMP_seqres.mean_drift_rotation_max = drift_rotation_mean_max
        TEMP_seqres.mean_drift_rotation_outliers = drift_rotation_mean_outliers
        TEMP_seqres._tags_NONAN = tags_NONAN
        TEMP_seqres.tags_m = tags_m
        TEMP_seqres.tags_q1 = tags_q1
        TEMP_seqres.tags_q3 = tags_q3
        TEMP_seqres.tags_min = tags_min
        TEMP_seqres.tags_max = tags_max
        TEMP_seqres.tags_outliers = tags_outliers
        TEMP_seqres._coverages = np.array([1 if cov == "True" else 0 for cov in res[0].coverage])
        TEMP_seqres.coverage_m = compute_coverage(TEMP_seqres._coverages)
        TEMP_seqres.mean_coverage_perc_quintiles, TEMP_seqres._coverage_mean_per_fabrication_quintiles = _get_coverage_quintiles(TEMP_seqres._coverages,
                                                                           TEMP_seqres._idx_operations_start_end)

        TOOLS[id][1] = TEMP_seqres

        print(f"====================== {id} =====================")
        print(f"nbr_operations [number]: {TOOLS[id][1].nbr_operations}")
        print(f"average_time_per_operation [s]: {TOOLS[id][1].average_time_per_operation}")
        print(f"frames_operations_start_end [length]: {len(TOOLS[id][1]._frames_operations_start_end)}")
        print(f"frames_operations_start_end [number]: {TOOLS[id][1]._frames_operations_start_end}")
        print(f"idx_operations_start_end [length]: {len(TOOLS[id][1]._idx_operations_start_end)}")
        print(f"idx_operations_start_end [number]: {TOOLS[id][1]._idx_operations_start_end}")
        print("--------------------------------------------------")
        print(f"drift_position_mean_m [m]: {TOOLS[id][1].mean_drift_position_m}")
        print(f"drift_position_mean_q1 [m]: {TOOLS[id][1].mean_drift_rotation_q1}")
        print(f"drift_position_mean_q3 [m]: {TOOLS[id][1].mean_drift_rotation_q3}")
        print(f"drift_position_mean_min [m]: {TOOLS[id][1].mean_drift_rotation_min}")
        print(f"drift_position_mean_max [m]: {TOOLS[id][1].mean_drift_rotation_max}")
        print(f"drift_position_mean_outliers [number]: {len(TOOLS[id][1].mean_drift_rotation_outliers)}")
        print("--------------------------------------------------")
        print(f"drift_rotation_mean_m [degrees]: {TOOLS[id][1].mean_drift_rotation_m}")
        print(f"drift_rotation_mean_q1 [degrees]: {TOOLS[id][1].mean_drift_rotation_q1}")
        print(f"drift_rotation_mean_q3 [degrees]: {TOOLS[id][1].mean_drift_rotation_q3}")
        print(f"drift_rotation_mean_min [degrees]: {TOOLS[id][1].mean_drift_rotation_min}")
        print(f"drift_rotation_mean_max [degrees]: {TOOLS[id][1].mean_drift_rotation_max}")
        print(f"drift_rotation_mean_outliers [number]: {len(TOOLS[id][1].mean_drift_rotation_outliers)}")
        print("--------------------------------------------------")
        print(f"tags_m [number]: {TOOLS[id][1].tags_m}")
        print(f"tags_q1: {TOOLS[id][1].tags_q1}")
        print(f"tags_q3: {TOOLS[id][1].tags_q3}")
        print(f"tags_min: {TOOLS[id][1].tags_min}")
        print(f"tags_max: {TOOLS[id][1].tags_max}")
        print(f"tags_outliers: {len(TOOLS[id][1].tags_outliers)}")
        print("--------------------------------------------------")
        print(f"coverage [percentage]: {TOOLS[id][1].coverage_m}")
        print(f"coverage_mean_per_fab [nbr]: {len(TOOLS[id][1]._coverage_mean_per_fabrication_quintiles)}")
        print(f"mean_coverage_perc_quintiles1 [percentage]: {TOOLS[id][1].mean_coverage_perc_quintiles[0]}")
        print(f"mean_coverage_perc_quintiles2 [percentage]: {TOOLS[id][1].mean_coverage_perc_quintiles[1]}")
        print(f"mean_coverage_perc_quintiles3 [percentage]: {TOOLS[id][1].mean_coverage_perc_quintiles[2]}")
        print(f"mean_coverage_perc_quintiles4 [percentage]: {TOOLS[id][1].mean_coverage_perc_quintiles[3]}")
        print(f"mean_coverage_perc_quintiles5 [percentage]: {TOOLS[id][1].mean_coverage_perc_quintiles[4]}")