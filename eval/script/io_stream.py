import numpy as np
import csv
from scipy.spatial.transform import Rotation
import math
import os
import transformations as tfm
import sys

import matplotlib
matplotlib.use('TkAgg')  # <-- to avoid memory leak
import matplotlib.pyplot as plt
import matplotlib.animation as animation

import visuals

# TODO: test debug
import gc  # for collecting buffer memory

#===============================================================================
# optitrack and tslam streams
#===============================================================================

def __set_coverage_values(camera_pos : np.array,
                          camera_rot_vec : np.array,
                          ts_poss : np.array,
                          frame_number : int) -> bool:
    """
        Check if the data is sane by checking if poses are valid (no drift, no lost tracking, etc).
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
                    scale_f : float = 0.02,
                    frame_start : int = None,
                    frame_end : int = None) -> np.array:
    """
        Process the data from the tracking system.

        Args:
            ts_path (str): path to the tracking system data
            scale_f (float): scale factor to apply to the poses of the tslam.
                             It corresponds to the size of the tag (e.g. 2cm)

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
                                             frame_number)
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
# metrics results output
#===============================================================================

def dump_results(out_dir : str,
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
    coverages_path : str = f"{out_dir}/{id}_coverages_bench.csv"
    metrics_csv_path : str = f"{out_dir}/{id}_bench.csv"

    assert len(frames) == len(coverages)
    with open(metrics_csv_path, "w") as f:
        writer = csv.writer(f, delimiter=';')
        writer.writerow([
                         "tags"
                         "drift_poss_x",
                         "drift_poss_y",
                         "drift_poss_z",
                         "drift_poss_mean",
                         "drift_rots_x",
                         "drift_rots_y",
                         "drift_rots_z",
                         "drift_poss_mean",
                         ])
        for idx, tag in enumerate(tags):
            writer.writerow([
                             tag,
                             drift_poss_xyz[idx][0],
                             drift_poss_xyz[idx][1],
                             drift_poss_xyz[idx][2],
                             drift_poss_mean_xyz[idx],
                             drift_rots_xyz[idx][0],
                             drift_rots_xyz[idx][1],
                             drift_rots_xyz[idx][2],
                             drift_rots_mean_xyz[idx],
                             ])

    with open(coverages_path, "w") as f:
        # use csv module python
        writer = csv.writer(f, delimiter=';')
        writer.writerow([
                         "frames",
                         "coverages",
                         ])
        for idx, frame in enumerate(frames):
            writer.writerow([
                             frame,
                             coverages[idx],
                             ])

    # np.savetxt(frames_path, frames, delimiter=",")
    # np.savetxt(drift_poss_path, drift_poss_xyz, delimiter=",")
    # np.savetxt(drift_poss_path, drift_poss_xyz, delimiter=",")
    # np.savetxt(drift_rots_path, drift_rots_xyz, delimiter=",")
    # np.savetxt(drift_poss_mean_xyz_path, drift_poss_mean_xyz, delimiter=",")
    # np.savetxt(drift_rots_mean_xyz_path, drift_rots_mean_xyz, delimiter=",")
    # np.savetxt(tags_path, tags, delimiter=",")
    # # merge all the csv files into one
    # os.system(f"paste -d ',' {drift_poss_path} {drift_rots_path} {drift_poss_mean_xyz_path} {drift_rots_mean_xyz_path} {tags_path} > {out_dir}/{id}_bench.csv")
    # # add an header to the csv file
    # os.system(f"sed -i '1s/^/drift_poss_x,drift_poss_y,drift_poss_z,drift_rots_x,drift_rots_y,drift_rots_z,drift_poss_mean_x,drift_poss_mean_y,drift_poss_mean_z,drift_rots_mean_x,drift_rots_mean_y,drift_rots_mean_z,tags\n/' {out_dir}/{id}_bench.csv")




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
