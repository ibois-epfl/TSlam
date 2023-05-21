import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime
import transformations as tfm
import util

import numpy as np
import matplotlib.pyplot as plt
import csv

from scipy.signal import savgol_filter
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation


def _process_opti_data(gt_path : str,
                       frame_start : int,
                       frame_end : int) -> np.array:
    opti_poss = []
    opti_rots = []
    opti_vec_rots = []
    with open(gt_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        next(reader) # skip header
        for data in reader:
            frame_number = int(data[0])
            if frame_number >= frame_start and frame_number <= frame_end:
                timestamp = float(data[1])
                camera_pos = np.array([float(data[2]), float(data[3]), float(data[4])])
                camera_rot = np.array([float(data[5]), float(data[6]), float(data[7]), float(data[8])])
                c_R = tfm.quat_to_euler(camera_rot)
                camera_rot_vec = Rotation.from_matrix(c_R).as_rotvec()

                opti_poss.append(camera_pos)
                opti_rots.append(c_R)
                opti_vec_rots.append(camera_rot_vec)
    opti_poss = np.array(opti_poss)
    opti_rots = np.array(opti_rots)
    opti_vec_rots = np.array(opti_vec_rots)
    return opti_poss, opti_rots, opti_vec_rots

# FIXME: work needed
def _process_ts_data(ts_path : str) -> np.array:
    ts_poss = []
    ts_rots = []
    ts_vec_rots = []
    ts_tags = []
    ts_idx_candidates = []
    ts_coverages = []  # per pose, quality of the signal 0: good, 1:bad (detected drift)

    with open(ts_path, "r") as f:
        lines = f.readlines()
        lines = lines[1:]  # skip header

        last_valid_pos = np.array([0, 0, 0])
        last_valid_rot = np.array([0, 0, 0, 0])
        is_not_detected : bool = False
        is_drifted : bool = False

        for idx, l in enumerate(lines):
            l = l.split()
            frame_number = int(l[0])
            time_stamp = float(l[1])
            camera_pos = np.array([float(l[2]), float(l[3]), float(l[4])])
            camera_pos = camera_pos * 0.02
            camera_rot = np.array([float(l[5]), float(l[6]), float(l[7]), float(l[8])])
            c_R = tfm.quat_to_euler(camera_rot)
            camera_rot_vec = Rotation.from_matrix(c_R).as_rotvec()
            # change sign of the rotation around the x axis

            ts_tag = int(l[9])
            ts_cover = 0

            # >>>>>>> Post-process checks >>>>>>>
            # correct the drifting by replacing the 0 0 0 0 0 0 0 with the previous known pose (undetected)
            if np.linalg.norm(camera_pos) == 0 and frame_number != 1 and len(ts_poss) > 0:
                # last_valid_pos = ts_poss[-1]
                # last_valid_rot = ts_rots[-1]
                is_not_detected = True
                camera_pos = ts_poss[-1]
                camera_rot = ts_rots[-1]
                ts_cover = 1
            
            # FIXME: fix the mechanism otherwise if relocalization happens it is lost
            # if the current position is away from the previous position by more than 5cm, then replace it with the previous known pose
            if ts_poss.__len__() != 0 and np.linalg.norm(camera_pos - ts_poss[-1]) > 0.05 and frame_number != 1:
                is_drifted = True
                last_valid_pos = ts_poss[-1]
                last_valid_rot = ts_rots[-1]
                # camera_pos = ts_poss[-1]
                # camera_rot = ts_rots[-1]
                ts_cover = 1
            # <<<<<<<< Post-process checks <<<<<<<<

            ts_poss.append(camera_pos)
            ts_rots.append(c_R)
            ts_vec_rots.append(camera_rot_vec)
            ts_tags.append(ts_tag)
            ts_coverages.append(ts_cover)

    # >>>>>>

    ts_poss = np.array(ts_poss)
    ts_rots = np.array(ts_rots)
    ts_vec_rots = np.array(ts_vec_rots)
    ts_tags = np.array(ts_tags)
    ts_idx_candidates = np.array(ts_idx_candidates)
    ts_coverages = np.array(ts_coverages)

    return ts_poss, ts_rots, ts_vec_rots, ts_tags, ts_idx_candidates, ts_coverages

# FIXME: refine mechanism for thrshold
def _select_best_idx(_tag_threshold,
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
        if tag >= _tag_threshold and _ts_coverages[idx] == 0:
            ts_idx_candidates.append(idx)
    
    if len(ts_idx_candidates) < 2:
        ts_idx_candidates = []

    return np.array(ts_idx_candidates)



def _align_trajectories(source,
                        target,
                        src_rot,
                        tgt_rot,
                        src_rot_vec,
                        tgt_rot_vec,
                        est_idx_candidates,
                        est_coverage,
                        est_tags) -> np.array:
    if len(est_idx_candidates) != 0:
        source_candidate = source.copy()[est_idx_candidates]
        target_candidate = target.copy()[est_idx_candidates]
        src_rot_candidate = src_rot.copy()[est_idx_candidates]
        tgt_rot_candidate = tgt_rot.copy()[est_idx_candidates]
        est_tags_candidate = est_tags.copy()[est_idx_candidates]
        src_rot_vec_candidate = src_rot_vec.copy()[est_idx_candidates]
        tgt_rot_vec_candidate = tgt_rot_vec.copy()[est_idx_candidates]
    else:
        source_candidate = source.copy()
        target_candidate = target.copy()
        src_rot_candidate = src_rot.copy()
        tgt_rot_candidate = tgt_rot.copy()
        est_tags_candidate = est_tags.copy()
        src_rot_vec_candidate = src_rot_vec.copy()
        tgt_rot_vec_candidate = tgt_rot_vec.copy()


    ##############################################################
    # WORKING-SEMI METHOD: trans + rot
    # rotation A
    vec_A = target_candidate[-1] - target_candidate[0]
    vec_B = source_candidate[-1] - source_candidate[0]
    rot = tfm.rotation_matrix_from_vectors(vec_B, vec_A)

    aligned_source_r = np.dot(rot.as_matrix(), source.copy().T).T
    source_candidate_r = np.dot(rot.as_matrix(), source_candidate.copy().T).T
    src_rot_candidate_r = np.dot(rot.as_matrix(), src_rot_candidate.copy().T).T  # TEST
    src_rot_vec_candidate_r = np.dot(rot.as_matrix(), src_rot_vec_candidate.copy().T).T  # TEST

    # translation
    point_start = source_candidate_r[0]
    point_end = target_candidate[0]
    trans = point_end - point_start
    aligned_source_r_t = aligned_source_r.copy() + trans
    source_candidate_r_t = source_candidate_r.copy() + trans
    # src_rot_candidate_r_t = src_rot_candidate_r.copy() + trans
    src_rot_vec_candidate_r_t = src_rot_vec_candidate_r.copy() + trans

    # rotation B
    # we need to rotate the source_candidate_r_t to match the first its orientation with the target
    # to do so we need to:
    # 1. find the angle between the source vector and the target vector
    # 2. the axis of rotation is the vector source_candidate_r_t[-1] - source_candidate_r_t[0]
    # 3. rotate the source_candidate_r_t around the axis by the angle

    # rotation axis
    axis_rot = source_candidate_r_t[-1] - source_candidate_r_t[0]

    # rotation angle
    # target_axis = tgt_rot_candidate[0]
    # source_axis = src_rot_candidate_r[0]
    target_axis = tgt_rot_vec_candidate[0]
    source_axis = src_rot_vec_candidate_r_t[0]

    angle_rot = tfm.angle_between_vectors(source_axis, target_axis)
    print(f"angle_rot: {angle_rot}")

    rot = tfm.rotate_around_axis(axis_rot, angle_rot)
    aligned_source_r_t_r = np.dot(rot.as_matrix(), aligned_source_r_t.copy().T).T
    source_candidate_r_t_r = np.dot(rot.as_matrix(), source_candidate_r_t.copy().T).T
    src_rot_vec_candidate_r_t_r = np.dot(rot.as_matrix(), src_rot_vec_candidate_r_t.copy().T).T

    # translation B
    point_start = source_candidate_r_t_r[0]
    point_end = target_candidate[0]
    trans = point_end - point_start
    aligned_source_r_t_r_t = aligned_source_r_t_r.copy() + trans
    source_candidate_r_t_r_t = source_candidate_r_t_r.copy() + trans
    src_rot_vec_candidate_r_t_r_t = src_rot_vec_candidate_r_t_r.copy() + trans




    ##############################################################
    # # # >> CHECKS for deformation
    assert util.verify_trajectories_distortion(source, aligned_source_r, verbose=False), "[ERROR]: The trajectories are distorted r"
    assert util.verify_trajectories_distortion(source, aligned_source_r_t, verbose=False), "[ERROR]: The trajectories are distorted rxt"
    assert util.verify_trajectories_distortion(source, aligned_source_r_t_r, verbose=False), "[ERROR]: The trajectories are distorted rxtxr"
    assert util.verify_trajectories_distortion(source, aligned_source_r_t_r_t, verbose=False), "[ERROR]: The trajectories are distorted rxtxrt"

    # DEBUG >>>>>>>>>>>>>>>>>>>>
    # VISUALS
    fig = plt.figure()
    fig.set_size_inches(18.5, 10.5)
    ax = fig.add_subplot(projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.view_init(elev=30, azim=-45)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    ax.plot(target[:, 0], target[:, 1], target[:, 2], color='black', alpha=0.5)
    ax.text(target[0, 0], target[0, 1], target[0, 2], str("target"), color='black', fontsize=8)

    ax.scatter(source_candidate[:, 0], source_candidate[:, 1], source_candidate[:, 2], color='red', s=6)
    ax.plot(source[:, 0], source[:, 1], source[:, 2], color='blue', alpha=0.5)
    ax.text(source[0, 0], source[0, 1], source[0, 2], str("source ori"), color='blue', fontsize=8)

    # ax.plot(aligned_source_r[:, 0], aligned_source_r[:, 1], aligned_source_r[:, 2], color='red', alpha=0.5)
    # ax.text(aligned_source_r[0, 0], aligned_source_r[0, 1], aligned_source_r[0, 2], str("transform r"), color='red', fontsize=8)

    ax.plot(aligned_source_r_t[:, 0], aligned_source_r_t[:, 1], aligned_source_r_t[:, 2], color='green', alpha=0.5)
    ax.text(aligned_source_r_t[0, 0], aligned_source_r_t[0, 1], aligned_source_r_t[0, 2], str("transform r"), color='green', fontsize=8)

    # ax.plot(aligned_source_r_t_r[:, 0], aligned_source_r_t_r[:, 1], aligned_source_r_t_r[:, 2], color='orange', alpha=0.5)
    # ax.text(aligned_source_r_t_r[0, 0], aligned_source_r_t_r[0, 1], aligned_source_r_t_r[0, 2], str("transform rxtxr"), color='orange', fontsize=8)

    ax.plot(aligned_source_r_t_r_t[:, 0], aligned_source_r_t_r_t[:, 1], aligned_source_r_t_r_t[:, 2], color='purple', alpha=0.5)
    ax.text(aligned_source_r_t_r_t[0, 0], aligned_source_r_t_r_t[0, 1], aligned_source_r_t_r_t[0, 2], str("transform rxtxrt"), color='purple', fontsize=8)

    # TEST
    axis_rot = np.array([source_candidate_r_t[0], source_candidate_r_t[-1]])
    ax.plot(axis_rot[:, 0], axis_rot[:, 1], axis_rot[:, 2], color='red', alpha=0.5)
    
    axis_tgt = np.array([target_candidate[0], target_candidate[-1] + target_axis*0.1])
    ax.plot(axis_tgt[:, 0], axis_tgt[:, 1], axis_tgt[:, 2], color='cyan', alpha=0.5)

    # source_axis = Rotation.from_matrix(src_rot_candidate_r[0]).as_rotvec()
    axis_src = np.array([source_candidate_r_t[0], source_candidate_r_t[-1] + source_axis*0.1])
    ax.plot(axis_src[:, 0], axis_src[:, 1], axis_src[:, 2], color='yellow', alpha=0.5)

    # ------------ ORIENTATION VISUALS ------------
    # show the rotation vectors for the target
    for idx, rot in enumerate(tgt_rot_vec_candidate):
        axis_tgt = np.array([target_candidate[idx], target_candidate[idx] + rot*0.005])
        ax.plot(axis_tgt[:, 0], axis_tgt[:, 1], axis_tgt[:, 2], color='red', alpha=0.5)

    # show the rotation vectors for the source
    for idx, rot in enumerate(src_rot_vec_candidate):
        axis_src = np.array([source[idx], source[idx] + rot*0.015])
        ax.plot(axis_src[:, 0], axis_src[:, 1], axis_src[:, 2], color='blue', alpha=0.5)

    # show the rotation vectors for the aligned_source_r_t
    for idx, rot in enumerate(src_rot_vec_candidate_r_t):
        axis_src = np.array([aligned_source_r_t[idx], aligned_source_r_t[idx] + rot*0.005])
        ax.plot(axis_src[:, 0], axis_src[:, 1], axis_src[:, 2], color='green', alpha=0.5)

    # show the rotation vectors for the aligned_source_r_t_r_t
    for idx, rot in enumerate(src_rot_vec_candidate_r_t_r_t):
        axis_src = np.array([aligned_source_r_t_r_t[idx], aligned_source_r_t_r_t[idx] + rot*0.005])
        ax.plot(axis_src[:, 0], axis_src[:, 1], axis_src[:, 2], color='purple', alpha=0.5)


    axis_src_final = np.array([source_candidate_r_t_r_t[0], source_candidate_r_t_r_t[-1] + source_axis*0.1])



    plt.show()
    # sys.exit()
    # DEBUG >>>>>>>>>>>>>>>>>>>>

    return aligned_source_r_t_r_t

def _visualize_trajectories(est_pos,
                            est_rot,
                            gt_pos,
                            gt_rot,
                            est_idx_candidates,
                            ts_poss_ori,
                            ts_coverages,
                            out_dir : str = "./",
                            is_show : bool = False) -> None:
    """ Visualize the trajectories in a 3D-axis plot """

    fig = plt.figure()
    fig.set_size_inches(18.5, 10.5)
    ax = fig.add_subplot(projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.view_init(elev=30, azim=-45)


    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], color='grey', alpha=0.5)
    # ax.scatter(gt_pos[0, 0], gt_pos[0, 1], gt_pos[0, 2], color='green', s=1)
    # ax.scatter(gt_pos[-1, 0], gt_pos[-1, 1], gt_pos[-1, 2], color='black', s=1)
    if len(est_idx_candidates) != 0:
        gt_pos_candidates = gt_pos[est_idx_candidates]
        ax.scatter(gt_pos_candidates[:, 0], gt_pos_candidates[:, 1], gt_pos_candidates[:, 2], color='red', s=4)


    # TODO: debug get rid
    if len(est_idx_candidates) != 0:
        est_pos_candidates = est_pos[est_idx_candidates]
        ax.scatter(est_pos_candidates[:, 0], est_pos_candidates[:, 1], est_pos_candidates[:, 2], color='green', s=4)



    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color='red', alpha=0.5)
    ax.text(est_pos[0, 0], est_pos[0, 1], est_pos[0, 2], str("transform ts"), color='red', fontsize=8)

    # color the drifted poses
    unvalid_ts_poss_ori = ts_poss_ori.copy()
    unvalid_ts_poss_ori[ts_coverages == 0] = np.nan
    ax.plot(unvalid_ts_poss_ori[:, 0], unvalid_ts_poss_ori[:, 1], unvalid_ts_poss_ori[:, 2], color='red', alpha=0.5)
    valid_ts_poss_ori = ts_poss_ori.copy()
    valid_ts_poss_ori[ts_coverages == 1] = np.nan
    ax.plot(valid_ts_poss_ori[:, 0], valid_ts_poss_ori[:, 1], valid_ts_poss_ori[:, 2], color='green', alpha=0.5)




    # ax.scatter(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color='green', s=1)
    # ax.scatter(est_pos[-1, 0], est_pos[-1, 1], est_pos[-1, 2], color='black', s=1)

    # save img
    # timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # TODO: debug
    # name_file = out_dir + f"/traj_{timestamp}.png"
    # plt.savefig(name_file, dpi=300, bbox_inches='tight', pad_inches=0)

    ax.plot(ts_poss_ori[:, 0], ts_poss_ori[:, 1], ts_poss_ori[:, 2], color='blue', alpha=0.5)
    ax.text(ts_poss_ori[0, 0], ts_poss_ori[0, 1], ts_poss_ori[0, 2], str("original ts"), color='blue', fontsize=8)

    # # color the drifted poses
    # unvalid_ts_poss_ori = ts_poss_ori.copy()
    # unvalid_ts_poss_ori[ts_coverages == 0] = np.nan
    # ax.plot(unvalid_ts_poss_ori[:, 0], unvalid_ts_poss_ori[:, 1], unvalid_ts_poss_ori[:, 2], color='red', alpha=0.5)
    # valid_ts_poss_ori = ts_poss_ori.copy()
    # valid_ts_poss_ori[ts_coverages == 1] = np.nan
    # ax.plot(valid_ts_poss_ori[:, 0], valid_ts_poss_ori[:, 1], valid_ts_poss_ori[:, 2], color='green', alpha=0.5)



    if is_show:
        plt.show()


def main(gt_path : str,
         ts_path : str,
         out_dir : str,
         is_show_plot : bool) -> None:

    ##################
    # process_data

    frame_start = int(ts_path.split('/')[-1].split('_')[0])
    frame_end = int(ts_path.split('/')[-1].split('_')[1].split('.')[0])

    opti_poss, opti_rots, opti_vec_rots = _process_opti_data(gt_path, frame_start, frame_end)
    ts_poss, ts_rots, ts_vec_rots, ts_tags, ts_idx_candidates, ts_coverages = _process_ts_data(ts_path)

    ##################
    # select candidate idx
    # find the indexes of the 3 biggest tags and retrieve the corresponding tslam positions
    ts_idx_candidates = _select_best_idx(_ts_tags=ts_tags,
                                         _ts_coverages=ts_coverages,
                                         _tag_threshold=2)

    ##################

    assert len(opti_poss) == len(ts_poss), "[ERROR]: The number of frames in the ground truth and the tslam positions do not match: {} vs {}".format(len(opti_poss), len(ts_poss))
    assert len(opti_rots) == len(ts_rots), "[ERROR]: The number of frames in the ground truth and the tslam orientations do not match: {} vs {}".format(len(opti_rots), len(ts_rots))
    assert len(ts_tags) == len(ts_poss), "[ERROR]: The number of frames in the tslam positions and the tslam marks do not match: {} vs {}".format(len(ts_poss), len(ts_tags))

    # #######################################################
    # # Analytics
    # #######################################################

    # print pourcentage of coverage in % based on ts_coverages 0 = good, 1 = bad
    coverage_perc : float = (np.sum(ts_coverages == 0) / len(ts_coverages) * 100)
    print(f">>>>>>>>>>Coverage: {coverage_perc.round(1)} %")

    # #######################################################
    # # Extra
    # #######################################################

    # # apply a smoothing filter to the tslam trajectory
    filter_size = 21  # FIXME: is this scientifically sound?
    # opti_poss = savgol_filter(opti_poss, filter_size, 3, axis=0)  # FIXME: understand if it is correct to do it and what is the impact
    ts_poss = savgol_filter(ts_poss, filter_size, 3, axis=0)  # FIXME: understand if it is correct to do it and what is the impact

    # #######################################################
    # # Trajectory registration
    # #######################################################

    ts_poss_ori = ts_poss.copy()
    if coverage_perc > 1:  # FIXME: find a better mechanism
        ts_poss = _align_trajectories(ts_poss,
                                      opti_poss,
                                      ts_rots,
                                      opti_rots,
                                      ts_vec_rots,
                                      opti_vec_rots,
                                      ts_idx_candidates,
                                      ts_coverages,
                                      ts_tags
                                      )

    # #######################################################
    # # Visualization
    # #######################################################

    # _visualize_trajectories(ts_poss,
    #                         ts_rots,
    #                         opti_poss,
    #                         opti_rots,
    #                         ts_idx_candidates,
    #                         ts_poss_ori,
    #                         ts_coverages,
    #                         out_dir,
    #                         is_show=is_show_plot)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the metrics for tslam.')
    parser.add_argument('--name', type=str, default="noname", help='Name of the sequence.')
    parser.add_argument('--gt', type=str, help='Path to the ground truth trajectory file (refined trajectory).')
    parser.add_argument('--ts', type=str, help='Path to the tslam trajectory file.')
    parser.add_argument('--showPlot', action='store_true', help='If true, it will show the plot.')
    parser.add_argument('--singleMode', action='store_true', help='If true, it will process only one file provided in --ts.')
    parser.add_argument('--out', type=str, help='Path to the output result files.')

    args = parser.parse_args()

    if not args.gt.endswith('.csv'):
        print("\033[91m[ERROR]: --gt must be in format .csv\n\033[0m")
        os.makedirs(args.output)
    if not os.path.exists(args.out):
        print("\033[93m[WARNING]: --out folder does not exist, creating one\n\033[0m")
        os.makedirs(args.out)
    # _out_subdir : str = f"{args.out}/{args.name}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    # os.system(f"mkdir {_out_subdir}")

    if args.singleMode:
        main(gt_path=args.gt,
            ts_path=args.ts,
            out_dir=args.out,
            is_show_plot=args.showPlot)
    else:
        ts_files : str = "/home/as/TSlam/eval/script/test/outposes/01_2023-05-19_19-29-01"  # TODO: replace with arg
        files = [os.path.join(ts_files, f) for f in os.listdir(ts_files) if os.path.isfile(os.path.join(ts_files, f))]
        files.sort(key=lambda x: os.path.getmtime(x))

        for idx, file in tqdm(enumerate(files)):
            if file.endswith(".txt") and not "running_log" in file:
                file_path = os.path.join(ts_files, file)
                print(file)
                main(gt_path=args.gt,
                    ts_path=file_path,
                    out_dir=args.out,
                    is_show_plot=args.showPlot)
                # if idx == 6:
                #     break


