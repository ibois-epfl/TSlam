import os
import sys
import numpy as np
import transformations as tfm
import util
from dataclasses import dataclass

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

@dataclass
class ToolResults():
    # wether or not the tool is present in the video fabrication recording
    is_present : bool = False
    # the number of times the tool is used in the video fabrication recording
    nbr_occurences : int = 0
    # the mean coverage percentage of the tool
    mean_coverage_percentage : float = 0.0
    # the mean number of tags detected by the tslam
    mean_tags_mean : float = 0.0
    # the mean drift of the position of the tslam with its ground truth
    mean_drift_position_mean : float = 0.0
    # the mean drift of the rotation of the tslam with its ground truth
    mean_drift_rotation_mean : float = 0.0
    # the number of times where reallignement was not possible
    number_reallignement_not_possible : int = 0

TOOLS = {
    "circular_sawblade_140": ToolResults(),
    "saber_sawblade_t1": ToolResults(),
    "drill_hinge_cutter_bit_50": ToolResults(),
    "drill_auger_bit_20_200": ToolResults(),
    "drill_auger_bit_25_500": ToolResults(),
    "drill_oblique_hole_bit_40": ToolResults(),
    "st_screw_120": ToolResults(),
    "st_screw_100": ToolResults(),
    "st_screw_80": ToolResults(),
    "st_screw_45": ToolResults()
}

def _get_benchmark_files(out_dir : str) -> tuple([list[str], list[str], list[str]]):
    """
        Get benchmark results from out_dir in a creation time order.

        Args:
            out_dir (str): out_dir

        Returns:
            list(str): list of benchmark results sorted by name of the:
                - x1 overview in txt format
                - x2 the drft for each pose in csv format for position and rotation
                - x1 the number of files indicating a failed reallignement
    """
    files = [os.path.join(out_dir, f) for f in os.listdir(out_dir) if os.path.isfile(os.path.join(out_dir, f))]
    files.sort(key=lambda x: os.path.getmtime(x))

    bench_overview_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_overview_bench.txt")]
    bench_drift_poss_drift_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_drift_poss_xyz_bench.csv")]
    bench_drift_rots_drift_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_drift_rots_xyz_bench.csv")]
    bench_failed_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_bench_NOALLIGNEMENT.txt")]
    return (bench_overview_paths,
            bench_drift_poss_drift_paths,
            bench_drift_rots_drift_paths,
            bench_failed_paths)

def _get_commit_hash_from_file(overview_path : str) -> str:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "git_commit_name:" in l:
                return l.split(":")[1].strip()
def _get_frame_start_from_file(overview_path : str) -> int:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "frame_start:" in l:
                return int(l.split(":")[1].strip())
def _get_frame_end_from_file(overview_path : str) -> int:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "frame_end:" in l:
                return int(l.split(":")[1].strip())
def _get_coverage_percentage_from_file(overview_path : str) -> float:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "coverage_perc:" in l:
                return float(l.split(":")[1].strip())
def _get_tags_mean_from_file(overview_path : str) -> float:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "tags_mean:" in l:
                return float(l.split(":")[1].strip())
def _get_drift_position_mean_from_file(overview_path : str) -> float:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "drift_position_mean [meters]:" in l:
                return float(l.split(":")[1].strip())
def _get_drift_rotation_mean_from_file(overview_path : str) -> float:
    with open(overview_path, "r") as f:
        lines = f.readlines()
        for l in lines:
            if "drift_rotation_mean [degrees]:" in l:
                return float(l.split(":")[1].strip())

def compute_fab_results(out_dir : str,
                        sess_name : str) -> None:
    """
        This function outputs the overview of the entire sequence based on the results computed
        for each individual sub-sequence. 
        - for each type of toolhead:
            - mean_coverage_percentage
            - mean_tags_mean
            - mean_drift_position_mean
            - mean_drift_rotation_mean
            - number where reallignement was not possible
            (schemes like the one in drilling with standard deviation)

        - Across all tools:
            - total_mean_coverage_percentage
            - total_mean_tags_mean
            - total_mean_drift_position_mean
            - total_mean_drift_rotation_mean
            - total number where reallignement was not possible

        Args:
            out_dir (str): the directory where the results are saved.
            sess_name (str): the name of the session
    """
    circular_sawblade_140_RES = ToolResults()
    saber_sawblade_t1_RES = ToolResults()
    drill_hinge_cutter_bit_50_RES = ToolResults()
    drill_auger_bit_20_200_RES = ToolResults()
    drill_auger_bit_25_500_RES = ToolResults()
    drill_oblique_hole_bit_40_RES = ToolResults()
    st_screw_120_RES = ToolResults()
    st_screw_100_RES = ToolResults()
    st_screw_80_RES = ToolResults()
    st_screw_45_RES = ToolResults()

    overview_paths, poss_drift_paths, rots_drift_paths, bench_failed_paths = _get_benchmark_files(out_dir=out_dir)

    circular_sawblade_140_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[0] in path]
    circular_sawblade_140_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[0] in path]
    circular_sawblade_140_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[0] in path]
    saber_sawblade_t1_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[1] in path]
    saber_sawblade_t1_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[1] in path]
    saber_sawblade_t1_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[1] in path]
    drill_hinge_cutter_bit_50_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[2] in path] 
    drill_hinge_cutter_bit_50_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[2] in path]
    drill_hinge_cutter_bit_50_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[2] in path]
    drill_auger_bit_20_200_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[3] in path]
    drill_auger_bit_20_200_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[3] in path]
    drill_auger_bit_20_200_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[3] in path]
    drill_auger_bit_25_500_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[4] in path]
    drill_auger_bit_25_500_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[4] in path]
    drill_auger_bit_25_500_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[4] in path]
    drill_oblique_hole_bit_40_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[5] in path]
    drill_oblique_hole_bit_40_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[5] in path]
    drill_oblique_hole_bit_40_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[5] in path]
    st_screw_120_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[6] in path]
    st_screw_120_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[6] in path]
    st_screw_120_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[6] in path]
    st_screw_100_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[7] in path]
    st_screw_100_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[7] in path]
    st_screw_100_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[7] in path]
    st_screw_80_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[8] in path]
    st_screw_80_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[8] in path]
    st_screw_80_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[8] in path]
    st_screw_45_overview_paths = [path for path in overview_paths if list(TOOLS.keys())[9] in path]
    st_screw_45_poss_drift_paths = [path for path in poss_drift_paths if list(TOOLS.keys())[9] in path]
    st_screw_45_rots_drift_paths = [path for path in rots_drift_paths if list(TOOLS.keys())[9] in path]
    circular_sawblade_140_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    circular_sawblade_140_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    circular_sawblade_140_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    saber_sawblade_t1_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    saber_sawblade_t1_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    saber_sawblade_t1_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_hinge_cutter_bit_50_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_hinge_cutter_bit_50_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_hinge_cutter_bit_50_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_20_200_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_20_200_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_20_200_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_25_500_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_25_500_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_auger_bit_25_500_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_oblique_hole_bit_40_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_oblique_hole_bit_40_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    drill_oblique_hole_bit_40_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_120_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_120_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_120_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_100_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_100_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_100_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_80_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_80_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_80_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_45_overview_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_45_poss_drift_paths.sort(key=lambda x: os.path.getmtime(x))
    st_screw_45_rots_drift_paths.sort(key=lambda x: os.path.getmtime(x))


    if circular_sawblade_140_overview_paths.__len__() != 0:
        TOOL["circular_sawblade_140"].is_present = True
        TOOL["circular_sawblade_140"].nbr_occurences = circular_sawblade_140_overview_paths.__len__()


        # TOOL["circular_sawblade_140"].mean_coverage_percentage = np.mean([_get_coverage_percentage_from_file(path) for path in circular_sawblade_140_overview_paths])
        # TOOL["circular_sawblade_140"].mean_tags_mean = np.mean([_get_tags_mean_from_file(path) for path in circular_sawblade_140_overview_paths])
        # TOOL["circular_sawblade_140"].mean_drift_position_mean = np.mean([_get_drift_position_mean_from_file(path) for path in circular_sawblade_140_overview_paths])
        # TOOL["circular_sawblade_140"].mean_drift_rotation_mean = np.mean([_get_drift_rotation_mean(_from_filepath) for path in circular_sawblade_140_overview_paths])
        # TOOL["circular_sawblade_140"].number_reallignement_not_possible = np.sum([1 for path in circular_sawblade_140_overview_paths if _get_coverage_percentage_from_file(path) == 100.0])







    if saber_sawblade_t1_overview_paths.__len__() != 0:
        TOOL["saber_sawblade_t1"].is_present = True
    if drill_hinge_cutter_bit_50_overview_paths.__len__() != 0:
        TOOL["drill_hinge_cutter_bit_50"].is_present = True
    if drill_auger_bit_20_200_overview_paths.__len__() != 0:
        TOOL["drill_auger_bit_20_200"].is_present = True
    if drill_auger_bit_25_500_overview_paths.__len__() != 0:
        TOOL["drill_auger_bit_25_500"].is_present = True
    if drill_oblique_hole_bit_40_overview_paths.__len__() != 0:
        TOOL["drill_oblique_hole_bit_40"].is_present = True
    if st_screw_120_overview_paths.__len__() != 0:
        TOOL["st_screw_120"].is_present = True
    if st_screw_100_overview_paths.__len__() != 0:
        TOOL["st_screw_100"].is_present = True
    if st_screw_80_overview_paths.__len__() != 0:
        TOOL["st_screw_80"].is_present = True
    if st_screw_45_overview_paths.__len__() != 0:
        TOOL["st_screw_45"].is_present = True

    

