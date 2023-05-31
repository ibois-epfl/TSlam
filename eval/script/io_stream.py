import numpy as np
import csv
from scipy.spatial.transform import Rotation
import math
import os
import transformations as tfm
import sys
from datetime import datetime
import pickle

import matplotlib
matplotlib.use('TkAgg')  # <-- to avoid memory leak
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import pandas as pd

import visuals
import metrics

# TODO: test debug
import gc  # for collecting buffer memory

#===============================================================================
# optitrack and tslam streams
#===============================================================================

def __set_coverage_values(camera_pos : np.array,
                          camera_rot_vec : np.array,
                          ts_poss : np.array,
                          frame_number : int,
                          is_only_tag : bool,
                          ts_tag) -> bool:
    """
        Check if the data is sane by checking if poses are valid (no drift, no lost tracking, no tags detected etc).
        This will define the coverage value for the evaluation.

        Args:
            camera_pos (np.array): the position of the camera
            camera_rot_vec (np.array): the rotation of the camera as a rotation vector

        Returns:
            bool: True if the data is sane, False otherwise
    """
    # check for untrack value (0,0,0,0,0,0,0)
    if (np.linalg.norm(camera_pos) == 0 or np.linalg.norm(camera_rot_vec) == 0):
        is_not_detected = True
        return False

    # check if it is nan
    if math.isnan(camera_pos[0]) or math.isnan(camera_pos[1]) or math.isnan(camera_pos[2]) or math.isnan(camera_rot_vec[0]) or math.isnan(camera_rot_vec[1]) or math.isnan(camera_rot_vec[2]):
        is_corrupted = True
        camera_pos = 0
        camera_rot_vec = 0
        return False

    # if the current position is away from the previous position by more than 5cm, then replace it with the previous known pose
    if ts_poss.__len__() != 0 and np.linalg.norm(camera_pos - ts_poss[-1]) > 0.05 and frame_number != 1:
        is_drifted = True
        return False

    # if the tag is 0 the coverage is lost
    if ts_tag == 0 and is_only_tag:
        is_not_detected = True
        return False

    return True

def __replace_invalid_ts_pose_with_closest_valid_pose(ts_poss : np.array,
                                                      ts_vec_rots : np.array,
                                                      ts_coverages : np.array) -> np.array:
    """
        The function cleans up the data from the tracking system by replacing invalid poses with the closest valid pose.
        This is done just for visualizations and invalid poses are not considered in the evaluation.

        Args:
            ts_poss (np.array): the positions of the camera
            ts_vec_rots (np.array): the rotations of the camera as rotation vectors
            ts_coverages (np.array): the coverage of the tracking system (0: good, 1: corrupted(lost/drifted/undetected))

        Returns:
            np.array: the positions of the camera
            np.array: the rotations of the camera as rotation vectors
    """
    for idx, coverage in enumerate(ts_coverages):
        if coverage == 0:
            next_idx = idx + 1
            while next_idx < len(ts_coverages):
                if ts_coverages[next_idx] != 0 and np.linalg.norm(ts_poss[next_idx]) != 0 and np.linalg.norm(ts_vec_rots[next_idx]) != 0:
                    break
                next_idx += 1
            if next_idx < len(ts_coverages):
                ts_poss[idx] = ts_poss[next_idx]
                ts_vec_rots[idx] = ts_vec_rots[next_idx]
            else:
                ts_poss[idx] = ts_poss[idx-1]
                ts_vec_rots[idx] = ts_vec_rots[idx-1]
    return ts_poss, ts_vec_rots

def process_opti_timber_data(gt_path : str,
                             frame_start : int = None,
                             frame_end : int = None) -> np.array:
    """ Process the data from the optitrack of the timber piece """
    opti_poss = []
    opti_vec_rots = []
    with open(gt_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        next(reader) # skip header
        for data in reader:
            frame_number = int(data[0])
            if frame_start != None or frame_end != None:
                if frame_number < frame_start and frame_number > frame_end:
                    continue
            timestamp = float(data[1])
            timber_pos = np.array([float(data[9]), float(data[10]), float(data[11])])
            timber_rot = np.array([float(data[12]), float(data[13]), float(data[14]), float(data[15])])
            timber_rot_vec = tfm.quaternion_to_rotation_vector(timber_rot)

            opti_poss.append(timber_pos)
            opti_vec_rots.append(timber_rot_vec)
    opti_poss = np.array(opti_poss)
    opti_vec_rots = np.array(opti_vec_rots)
    return opti_poss, opti_vec_rots

def process_opti_camera_data(gt_path : str,
                             frame_start : int = None,
                             frame_end : int = None) -> np.array:
    """
        Process the data from the optitrack.

        Args:
            gt_path (str): path to the ground truth data
            frame_start (int): start frame
            frame_end (int): end frame

        Returns:
            np.array: positions, the positions of the camera
            np.array: rotations, the rotations of the camera as rotation vectors (from quaternion)
            np.array: dists, the travelled total distance at each pose
    """
    opti_poss = []
    opti_vec_rots = []
    with open(gt_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        next(reader) # skip header
        for data in reader:
            frame_number = int(data[0])
            if frame_start != None and frame_end != None:
                if frame_number < frame_start or frame_number > frame_end:
                    continue
            timestamp = float(data[1])
            camera_pos = np.array([float(data[2]), float(data[3]), float(data[4])])
            camera_rot = np.array([float(data[5]), float(data[6]), float(data[7]), float(data[8])])
            camera_rot_vec = tfm.quaternion_to_rotation_vector(camera_rot)

            opti_poss.append(camera_pos)
            opti_vec_rots.append(camera_rot_vec)
    opti_poss = np.array(opti_poss)
    opti_vec_rots = np.array(opti_vec_rots)

    distances = []
    dist_temp = 0
    for idx, pos in enumerate(opti_poss):
        if idx == 0:
            distances.append(0)
        else:
            dist_temp += np.linalg.norm(pos - opti_poss[idx-1])
            distances.append(dist_temp)

    return opti_poss, opti_vec_rots, distances

def process_ts_data(ts_path : str,
                    is_only_tag : bool,
                    scale_f : float = 0.02,
                    frame_start : int = None,
                    frame_end : int = None) -> np.array:
    """
        Process the data from the tracking system.

        Args:
            ts_path (str): path to the tracking system data
            is_only_tag (bool): True if the tracking system is only tracking the tag, False otherwise we consider feature points also
            scale_f (float): scale factor to apply to the poses of the tslam.
                             It corresponds to the size of the tag (e.g. 2cm)
            frame_start (int): start frame
            frame_end (int): end frame
        Returns:
            np.array: positions,the positions of the camera
            np.array: rotations, the rotations of the camera as rotation vectors (from quaternion)
            np.array: tags, the tags of the camera
            np.array: coverages, the coverage of the tracking system (0: good, 1: corrupted(lost/drifted/undetected))
    """
    ts_poss = []
    ts_vec_rots = []
    ts_tags = []
    ts_coverages = []  # per pose, quality of the signal 0: good, 1:bad (detected drift)

    with open(ts_path, "r") as f:
        lines = f.readlines()
        lines = lines[1:]  # skip header
        for idx, l in enumerate(lines):
            is_not_detected : bool = False
            is_drifted : bool = False
            is_corrupted : bool = False

            l = l.split()
            
            frame_number = int(l[0])
            if frame_start != None and frame_end != None:
                if frame_number < frame_start or frame_number > frame_end:
                    continue

            time_stamp = float(l[1])
            camera_pos = np.array([float(l[2]), float(l[3]), float(l[4])])
            camera_pos = camera_pos * scale_f
            camera_rot = np.array([float(l[5]), float(l[6]), float(l[7]), float(l[8])])
            camera_rot_vec = tfm.quaternion_to_rotation_vector(camera_rot)
            ts_tag = int(l[9])

            ts_cover = __set_coverage_values(camera_pos,
                                             camera_rot_vec,
                                             ts_poss,
                                             frame_number,
                                             ts_tag,
                                             is_only_tag)
            ts_poss.append(camera_pos)
            ts_vec_rots.append(camera_rot_vec)
            ts_tags.append(ts_tag)
            ts_coverages.append(ts_cover)

    ts_poss, ts_vec_rots = __replace_invalid_ts_pose_with_closest_valid_pose(ts_poss,
                                                                             ts_vec_rots,
                                                                             ts_coverages)

    ts_poss = np.array(ts_poss)
    ts_vec_rots = np.array(ts_vec_rots)
    ts_tags = np.array(ts_tags)
    ts_coverages = np.array(ts_coverages)

    return ts_poss, ts_vec_rots, ts_tags, ts_coverages

def run_checks(opti_poss : np.array,
               ts_poss : np.array,
               opti_vec_rots : np.array,
               ts_vec_rots : np.array,
               ts_tags : np.array,
               ts_coverages : np.array):
    assert len(opti_poss) == len(ts_poss), "[ERROR]: The number of frames in the ground truth and the tslam positions do not match: {} vs {}".format(len(opti_poss), len(ts_poss))
    assert len(opti_vec_rots) == len(ts_vec_rots), "[ERROR]: The number of frames in the ground truth and the tslam orientations do not match: {} vs {}".format(len(opti_rots), len(ts_rots))
    assert len(ts_tags) == len(ts_poss), "[ERROR]: The number of frames in the tslam positions and the tslam marks do not match: {} vs {}".format(len(ts_poss), len(ts_tags))
    assert len(ts_tags) == len(ts_coverages), "[ERROR]: The number of frames in the tslam marks and the tslam coverages do not match: {} vs {}".format(len(ts_tags), len(ts_coverages))

#===============================================================================
# sub-sequence metrics results output
#===============================================================================

def dump_subsequence_results(out_dir : str,
                 id : str,
                 frames : np.array(int),
                 coverage_perc : float,
                 coverages : np.array(int),
                 tags_mean : float,
                 tags : np.array(int),
                 drift_poss_mean : float,
                 drift_rots_mean : float,
                 drift_poss_xyz : np.array(float),
                 drift_rots_xyz : np.array(float),
                 drift_poss_mean_xyz : np.array(float),
                 drift_rots_mean_xyz : np.array(float),
                 toolhead_name : str,
                 frame_start : int,
                 frame_end : int) -> None:
    """ The function save the results of the evaluation to local. """
    overview_path : str = f"{out_dir}/{id}_overview_bench.txt"
    metrics_csv_path : str = f"{out_dir}/{id}_bench.csv"

    with open(metrics_csv_path, "w") as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow([
                         "frames",
                         "coverages",
                         "tags",
                         "drift_poss_x",
                         "drift_poss_y",
                         "drift_poss_z",
                         "drift_poss_mean",
                         "drift_rots_x",
                         "drift_rots_y",
                         "drift_rots_z",
                         "drift_poss_mean",
                         ])
        idx_candidate = 0
        for idx, frame in enumerate(frames):
            if coverages[idx] == 1:
                writer.writerow([
                                 frame,
                                 coverages[idx],
                                 tags[idx_candidate],
                                 drift_poss_xyz[idx_candidate][0],
                                 drift_poss_xyz[idx_candidate][1],
                                 drift_poss_xyz[idx_candidate][2],
                                 drift_poss_mean_xyz[idx_candidate],
                                 drift_rots_xyz[idx_candidate][0],
                                 drift_rots_xyz[idx_candidate][1],
                                 drift_rots_xyz[idx_candidate][2],
                                 drift_rots_mean_xyz[idx_candidate],
                                 ])
                idx_candidate += 1
            else:
                writer.writerow([
                                 frame,
                                 coverages[idx],
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                 "nan",
                                ])

    with open(overview_path, "w") as f:
        f.write("---- git commit id\n")
        git_commit_name = os.popen('git rev-parse --short HEAD').read()
        f.write(f"git_commit_name: {git_commit_name}\n")

        f.write("---- info sub-sequence\n")
        f.write(f"frame_start: {frame_start}\n")
        f.write(f"frame_end: {frame_end}\n")
        f.write(f"toolhead_name: {toolhead_name}\n")

        f.write("---- overview metrics\n")
        f.write(f"coverage_perc: {coverage_perc}\n")
        f.write(f"tags_mean: {tags_mean}\n")
        f.write(f"drift_poss_mean [meters]: {drift_poss_mean}\n")
        f.write(f"drift_rots_mean [degrees]: {drift_rots_mean}\n")
        f.write("-----------------------------------------------------\n")

#===============================================================================
# sub-sequence metrics cleaning and reorganization
#===============================================================================

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
    bench_metrics_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_bench.csv")]
    bench_failed_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_bench_NOALLIGNEMENT.txt")]
    graph_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_graph.png")]
    video_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith("_video.mp4")]
    
    return (bench_overview_paths,
            bench_metrics_paths,
            bench_failed_paths,
            graph_paths,
            video_paths)

def _merge_metrics_csv(paths_2_merge : list[str], csv_path : str) -> None:
    """
        The function merges the csv files in paths_2_merge into a single csv file in csv_path.
    """
    with open(csv_path, "w") as f:
        with open(paths_2_merge[0], "r") as f_first:
            f.write(f_first.readline())
        for c2m in paths_2_merge:
            with open(c2m, "r") as f_csv:
                f_csv.readline()
                for idx, line in enumerate(f_csv):
                    if idx != 0:
                        f.write(line)
    for c in paths_2_merge:
        os.remove(c)

def clean_subsequence_results(out_dir : str) -> None:
    """
        The function cleans the sub-sequences by:
            - merging all the csv per tool
            - merging all the overview
            - putting the images (if any) in a folder subsequence > img
            - putting the video (if any) in a folder subsequence > video
            - putting the csv grouped by tools and merged from multiple sub-sequences in a folder subsequence > benchmark
            - putting all the overview in a folder subsequence > benchmark
    """
    subsequence_path = os.path.join(out_dir, "subsequences")
    os.makedirs(subsequence_path, exist_ok=True)
    subsequence_graph_path = os.path.join(subsequence_path, "graph")
    os.makedirs(subsequence_graph_path, exist_ok=True)
    subsequence_benchmark_path = os.path.join(subsequence_path, "benchmark")
    os.makedirs(subsequence_benchmark_path, exist_ok=True)
    subsequence_benchmark_overview_file = os.path.join(subsequence_benchmark_path, "overview.txt")

    bench_overview_paths, bench_metrics_paths, bench_failed_paths, graph_paths, video_paths =_get_benchmark_files(out_dir)

    with open(subsequence_benchmark_overview_file, "w") as f:
        f.write(f"NOALIGNEMENT: {len(bench_failed_paths)}\n")
        f.write("List of failed sub-sequences (if any):\n")
        for failed_path in bench_failed_paths:
            f.write(f"{failed_path}\n")
        f.write("\n")
        f.write("-----------------------------------------------------\n")
        f.write("List of sub-sequences:\n")
        f.write("\n")
        for overview_path in bench_overview_paths:
            with open(overview_path, "r") as f_overview:
                f.write(f_overview.read())
    for overview_path in bench_overview_paths:
        os.remove(overview_path)
    for failed_path in bench_failed_paths:
        os.remove(failed_path)

    for graph_path in graph_paths:
        os.rename(graph_path, os.path.join(subsequence_graph_path, os.path.basename(graph_path)))

    for video_path in video_paths:
        os.rename(video_path, os.path.join(subsequence_path, os.path.basename(video_path)))
    
    circular_sawblade_140_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[0] in path]
    saber_sawblade_t1_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[1] in path]
    drill_hinge_cutter_bit_50_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[2] in path] 
    drill_auger_bit_20_200_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[3] in path]
    drill_auger_bit_25_500_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[4] in path]
    drill_oblique_hole_bit_40_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[5] in path]
    st_screw_120_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[6] in path]
    st_screw_100_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[7] in path]
    st_screw_80_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[8] in path]
    st_screw_45_csv_paths = [path for path in bench_metrics_paths if list(metrics.TOOLS.keys())[9] in path]

    circular_sawblade_140_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    saber_sawblade_t1_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    drill_hinge_cutter_bit_50_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    drill_auger_bit_20_200_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    drill_auger_bit_25_500_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    drill_oblique_hole_bit_40_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    st_screw_120_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    st_screw_100_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    st_screw_80_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))
    st_screw_45_csv_paths.sort(key=lambda x: int(os.path.basename(x).split("_")[1]))

    if len(circular_sawblade_140_csv_paths) != 0:
        _merge_metrics_csv(circular_sawblade_140_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[0]}.csv"))
    if len(saber_sawblade_t1_csv_paths) != 0:
        _merge_metrics_csv(saber_sawblade_t1_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[1]}.csv"))
    if len(drill_hinge_cutter_bit_50_csv_paths) != 0:
        _merge_metrics_csv(drill_hinge_cutter_bit_50_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[2]}.csv"))
    if len(drill_auger_bit_20_200_csv_paths) != 0:
        _merge_metrics_csv(drill_auger_bit_20_200_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[3]}.csv"))
    if len(drill_auger_bit_25_500_csv_paths) != 0:
        _merge_metrics_csv(drill_auger_bit_25_500_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[4]}.csv"))
    if len(drill_oblique_hole_bit_40_csv_paths) != 0:
        _merge_metrics_csv(drill_oblique_hole_bit_40_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[5]}.csv"))
    if len(st_screw_120_csv_paths) != 0:
        _merge_metrics_csv(st_screw_120_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[6]}.csv"))
    if len(st_screw_100_csv_paths) != 0:
        _merge_metrics_csv(st_screw_100_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[7]}.csv"))
    if len(st_screw_80_csv_paths) != 0:
        _merge_metrics_csv(st_screw_80_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[8]}.csv"))
    if len(st_screw_45_csv_paths) != 0:
        _merge_metrics_csv(st_screw_45_csv_paths, os.path.join(subsequence_benchmark_path, f"{list(metrics.TOOLS.keys())[9]}.csv"))

#===============================================================================
# img/graphs/animation output
#===============================================================================

def dump_imgs(out_dir : str,
              id : str,
              fig_3d : plt.figure,
              fig_2d_poss_drift : plt.figure,
              fig_2d_rots_drift : plt.figure) -> None:
    """ The function saves the graphs of the evaluation to local """
    allign_3d_path : str = f"{out_dir}/{id}_allign_3d_graph.png"
    error_poss_xyz_path : str = f"{out_dir}/{id}_error_poss_xyz_graph.png"
    error_rots_xyz_path : str = f"{out_dir}/{id}_error_rots_xyz_graph.png"

    fig_3d.savefig(allign_3d_path)
    fig_2d_poss_drift.savefig(error_poss_xyz_path)
    fig_2d_rots_drift.savefig(error_rots_xyz_path)

def dump_animation(est_pos,
                   est_rot,
                   gt_pos,
                   gt_rot,
                   total_frames : int,
                   out_dir : str,
                   id : str,
                   video_path : str = None,
                   is_draw_rot_vec : bool = False,
                   ) -> None:
    """ Create a side-by-side animation with the video of the sequence of the trajectory """
    temp_dir = os.path.join(out_dir, "temp")
    temp_video_dir = os.path.join(temp_dir, "video")

    os.makedirs(temp_dir, exist_ok=True)
    os.makedirs(temp_video_dir, exist_ok=True)

    video_filename = f"{id}_animation3d_video.mp4"

    # extract each frame of the video in the temp_video_dir between frame_start and frame_end
    os.system(f"ffmpeg -y -i {video_path} -r 30 {temp_video_dir}/%d.png")

    fig = plt.figure()
    height_mm = 160 # 160
    width_mm = 380 # 380
    height = int(height_mm / 25.4)
    width = int(width_mm / 25.4)
    fig.set_size_inches(width, height)
    plt.subplots_adjust(left=0.0, right=1.00, top=1.00, bottom=0.01)
    ax = fig.add_subplot(1, 2, 2, projection='3d')
    ax.grid(False)

    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.view_init(elev=30, azim=45)

    CLR_GT = "magenta"
    CLR_EST = "darkcyan"
    FONT_SIZE = 10
    XYZ_LIM = 0.10
    SCALE_AXIS = 0.02
    idx = 0

    img = plt.imread(os.path.join(temp_video_dir, f"{idx+1}.png"))
    ax2 = fig.add_subplot(1, 2, 1)
    ax2.imshow(img)
    ax2.axis('off')

    def update(idx):
        ax.clear()
        ax2.clear()

        ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
        ax.grid(False)
        center = np.mean(gt_pos, axis=0)
        ax.set_xlim3d(center[0] - XYZ_LIM, center[0] + XYZ_LIM)
        ax.set_ylim3d(center[1] - XYZ_LIM, center[1] + XYZ_LIM)
        ax.set_zlim3d(center[2] - XYZ_LIM, center[2] + XYZ_LIM)
        ax.set_xlabel('X [m]')
        ax.set_ylabel('Y [m]')
        ax.set_zlabel('Z [m]')
        ax.text2D(0.00, 0.03, "Ground Truth (gt)", transform=ax.transAxes, color=CLR_GT, fontsize=FONT_SIZE)
        ax.text2D(0.00, 0.00, "Tslam (ts)", transform=ax.transAxes, color=CLR_EST, fontsize=FONT_SIZE)
        ax.set_position([0.3, 0.1, 0.9, 0.9])  # reduce the top border of the graph

        ax.plot(gt_pos[:idx, 0], gt_pos[:idx, 1], gt_pos[:idx, 2], color=CLR_GT)
        ax.plot(est_pos[:idx, 0], est_pos[:idx, 1], est_pos[:idx, 2], color=CLR_EST, alpha=0.5)

        visuals.__draw_local_axis_pose(ax, est_pos[idx], est_rot[idx],
                                    scale_f=SCALE_AXIS, alpha=0.25, linewidth=2)
        visuals.__draw_local_axis_pose(ax, gt_pos[idx], gt_rot[idx],
                                scale_f=SCALE_AXIS, alpha=1, linewidth=1)

        img = plt.imread(os.path.join(temp_video_dir, f"{idx+1}.png"))
        ax2.imshow(img)
        ax2.axis('off')

        idx += 1

        plt.close(fig)
        gc.collect()  # for collecting buffer memory

    # create the animation
    ani = animation.FuncAnimation(fig, update,
                                  frames=total_frames,
                                  interval=1000/30,
                                  save_count=0)  # 30fps equivalent in ms
    ani.save(os.path.join(out_dir, video_filename), writer='ffmpeg', fps=30, dpi=300)

    # erasse the temp folder
    os.system(f"rm -rf {temp_dir}")

#===============================================================================
# sequence results i/o
#===============================================================================

def get_subseq_metrics_csv(out_dir : str) -> None:
    out_dir = os.path.join(out_dir, "subsequences/benchmark")
    files = [os.path.join(out_dir, f) for f in os.listdir(out_dir) if os.path.isfile(os.path.join(out_dir, f))]
    csv_paths = [os.path.join(out_dir,f) for f in os.listdir(out_dir) if f.endswith(".csv")]
    return csv_paths

    tool_csv_lst = [
                    [path for path in files if list(metrics.TOOLS.keys())[0] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[1] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[2] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[3] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[4] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[5] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[6] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[7] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[8] in path],
                    [path for path in files if list(metrics.TOOLS.keys())[9] in path],
    ]
    return tool_csv_lst

def dump_sequence_results(out_dir : str) -> str:
    """ Save the analysis of the sequence to a local csv file """
    sequence_summary_csv_path = os.path.join(out_dir, "sequence")
    os.makedirs(sequence_summary_csv_path, exist_ok=True)
    sequence_benchmark_path = os.path.join(sequence_summary_csv_path, "benchmark")
    os.makedirs(sequence_benchmark_path, exist_ok=True)

    # save all the metrics per tool as a py pickle
    for tool_id, res in metrics.TOOLS.items():
        with open(os.path.join(sequence_benchmark_path, f"{tool_id}_bench.pickle"), 'wb') as f:
            pickle.dump(res, f)

    # save a summary as a csv
    csv_overview_path = os.path.join(sequence_benchmark_path, "summarys_sequence_bench.csv")
    with open(csv_overview_path, "w") as f:
        writer = csv.writer(f, delimiter=',')
        writer.writerow([
            "tool_id",

            "nbr_operations",
            "average_time_per_operation",

            "mean_drift_position_m",
            "mean_drift_position_q1",
            "mean_drift_position_q3",
            "mean_drift_position_min",
            "mean_drift_position_max",

            "mean_drift_rotation_m",
            "mean_drift_rotation_q1",
            "mean_drift_rotation_q3",
            "mean_drift_rotation_min",
            "mean_drift_rotation_max",

            "tags_m",
            "tags_q1",
            "tags_q3",
            "tags_min",
            "tags_max",

            "coverage_m",
            "mean_coverage_perc_quintile1",
            "mean_coverage_perc_quintile2",
            "mean_coverage_perc_quintile3",
            "mean_coverage_perc_quintile4",
            "mean_coverage_perc_quintile5"
                         ])
        for tool_id, res in metrics.TOOLS.items():
            mean_coverage_perc_quintile1 = res[1].mean_coverage_perc_quintiles[0] if len(res[1].mean_coverage_perc_quintiles) > 0 else 0
            mean_coverage_perc_quintile2 = res[1].mean_coverage_perc_quintiles[1] if len(res[1].mean_coverage_perc_quintiles) > 1 else 0
            mean_coverage_perc_quintile3 = res[1].mean_coverage_perc_quintiles[2] if len(res[1].mean_coverage_perc_quintiles) > 2 else 0
            mean_coverage_perc_quintile4 = res[1].mean_coverage_perc_quintiles[3] if len(res[1].mean_coverage_perc_quintiles) > 3 else 0
            mean_coverage_perc_quintile5 = res[1].mean_coverage_perc_quintiles[4] if len(res[1].mean_coverage_perc_quintiles) > 4 else 0

            writer.writerow([
                tool_id,

                res[1].nbr_operations,
                res[1].average_time_per_operation,

                res[1].mean_drift_position_m,
                res[1].mean_drift_position_q1,
                res[1].mean_drift_position_q3,
                res[1].mean_drift_position_min,
                res[1].mean_drift_position_max,

                res[1].mean_drift_rotation_m,
                res[1].mean_drift_rotation_q1,
                res[1].mean_drift_rotation_q3,
                res[1].mean_drift_rotation_min,
                res[1].mean_drift_rotation_max,

                res[1].tags_m,
                res[1].tags_q1,
                res[1].tags_q3,
                res[1].tags_min,
                res[1].tags_max,

                res[1].coverage_m,
                mean_coverage_perc_quintile1,
                mean_coverage_perc_quintile2,
                mean_coverage_perc_quintile3,
                mean_coverage_perc_quintile4,
                mean_coverage_perc_quintile5
                ])

    return csv_overview_path

def dump_summary_as_tex_table(out_dir : str,
                              summary_csv_path : str) -> None:
    """ Output a table of the results from the csv summary of the entire sequence """
    table_dir = os.path.join(out_dir, "sequence", "table")
    if not os.path.exists(table_dir):
        os.makedirs(table_dir)

    # read the summary csv and store the data make a table
    df = pd.read_csv(summary_csv_path)

    # save as a latex table
    table_path = os.path.join(table_dir, "table.tex")
    with open(table_path, 'w') as tf:
        tf.write(df.to_latex(index=False))