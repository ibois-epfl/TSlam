import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime

import numpy as np
import matplotlib.pyplot as plt
import csv

from scipy.signal import savgol_filter
from scipy.spatial import KDTree
from scipy.spatial.transform import Rotation
from scipy.linalg import orthogonal_procrustes

# def umeyama(src, dst, estimate_scale):
#     """Estimate N-D similarity transformation with or without scaling.
#     Parameters
#     ----------
#     src : (M, N) array
#         Source coordinates.
#     dst : (M, N) array
#         Destination coordinates.
#     estimate_scale : bool
#         Whether to estimate scaling factor.
#     Returns
#     -------
#     T : (N + 1, N + 1)
#         The homogeneous similarity transformation matrix. The matrix contains
#         NaN values only if the problem is not well-conditioned.
#     References
#     ----------
#     .. [1] "Least-squares estimation of transformation parameters between two
#             point patterns", Shinji Umeyama, PAMI 1991, DOI: 10.1109/34.88573
#     """

#     num = src.shape[0]
#     dim = src.shape[1]

#     # Compute mean of src and dst.
#     src_mean = src.mean(axis=0)
#     dst_mean = dst.mean(axis=0)

#     # Subtract mean from src and dst.
#     src_demean = src - src_mean
#     dst_demean = dst - dst_mean

#     # Eq. (38).
#     A = np.dot(dst_demean.T, src_demean) / num

#     # Eq. (39).
#     d = np.ones((dim,), dtype=np.double)
#     if np.linalg.det(A) < 0:
#         d[dim - 1] = -1

#     T = np.eye(dim + 1, dtype=np.double)

#     U, S, V = np.linalg.svd(A)

#     # Eq. (40) and (43).
#     rank = np.linalg.matrix_rank(A)
#     if rank == 0:
#         return np.nan * T
#     elif rank == dim - 1:
#         if np.linalg.det(U) * np.linalg.det(V) > 0:
#             T[:dim, :dim] = np.dot(U, V)
#         else:
#             s = d[dim - 1]
#             d[dim - 1] = -1
#             T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V))
#             d[dim - 1] = s
#     else:
#         T[:dim, :dim] = np.dot(U, np.dot(np.diag(d), V.T))

#     if estimate_scale:
#         # Eq. (41) and (42).
#         scale = 1.0 / src_demean.var(axis=0).sum() * np.dot(S, d)
#     else:
#         scale = 1.0

#     T[:dim, dim] = dst_mean - scale * np.dot(T[:dim, :dim], src_mean.T)
#     T[:dim, :dim] *= scale

#     return T


def _quat_to_euler(rot):
    q = rot
    q = q / np.linalg.norm(q)
    R = np.array([[1-2*q[2]**2-2*q[3]**2, 2*q[1]*q[2]-2*q[0]*q[3], 2*q[0]*q[2]+2*q[1]*q[3]],
                  [2*q[1]*q[2]+2*q[0]*q[3], 1-2*q[1]**2-2*q[3]**2, 2*q[2]*q[3]-2*q[0]*q[1]],
                  [2*q[1]*q[3]-2*q[0]*q[2], 2*q[0]*q[1]+2*q[2]*q[3], 1-2*q[1]**2-2*q[2]**2]])
    return R

# FIXME: more refined selection method has to be worked out
def _select_best_idx(_ts_tags : np.array,
                     _ts_coverages : np.array) -> np.array:
    """ It selects the idxs of poses with the highest score of detected tags"""
    ts_idx_candidates = []

    # METHOD 0
    TAG_THRESHOLD = 3
    # DOMAINE_BOUND = 20
    for idx, tag in enumerate(_ts_tags):
        if tag >= TAG_THRESHOLD and _ts_coverages[idx] == 0:
            ts_idx_candidates.append(idx)
    # keep only maximum a domaine of 10 frames around the lowest and highest idx_candidate
    if len(ts_idx_candidates) >= 2:
        ts_idx_candidates = [ts_idx_candidates[0], ts_idx_candidates[-1]]
    else:
        ts_idx_candidates = []


    # # # METHOD 1
    # ts_idx_candidates = np.where(_ts_tags >= 3)[0]
    # print(f"ts_idx_candidates: {ts_idx_candidates}")
    # # get the last and first idx

    # ts_idx_candidates = [ts_idx_candidates[0], ts_idx_candidates[-1]]

    # ts_idx_candidates = ts_idx_candidates[np.argsort(_ts_tags[ts_idx_candidates])[-1:]]

    # # METHOD 2
    # # select all the idxs with at least 3 tag detected
    # for idx, tag in enumerate(_ts_tags):
    #     if tag >= 4:  # <----- value to adjust FIXME: understand if it is correct
    #         if _ts_coverages[idx] == 0:
    #             ts_idx_candidates.append(idx)
    
    # # keep only the first and the last
    # if len(ts_idx_candidates) > 2:
    #     upper_bound = ts_idx_candidates[-1]
    #     mid_bound = ts_idx_candidates[int(len(ts_idx_candidates)/2)]
    #     lower_bound = ts_idx_candidates[0]
    #     ts_idx_candidates = [lower_bound,
    #                         #  mid_bound,
    #                          upper_bound]
    # else:
    #     ts_idx_candidates = []

    return np.array(ts_idx_candidates)

def rigid_rotation_matrix_from_vectors(vec1, vec2):
    """ Find a rigid rotation matrix that aligns vec1 to vec2 """

    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)

    # the rotation axis is the cross product between the two vectors
    axis = np.cross(vec1, vec2)
    axis = axis / np.linalg.norm(axis)

    # the rotation angle is the angle between the two vectors
    angle = np.arccos(np.dot(vec1, vec2))

    # the rotation matrix
    rot = Rotation.from_rotvec(angle * axis)


    return rot

def _align_trajectories(source, target, est_idx_candidates):
    # custom method
    # # Compute the centroids of the trajectories
    # FIXME: THE PROBLEM IS THAT IT'S A NOT-RIGID TRANSFORMATION (try the Umeyama)
    if len(est_idx_candidates) != 0:
        source_candidate = source.copy()[est_idx_candidates]
        target_candidate = target.copy()[est_idx_candidates]
    else:
        source_candidate = source.copy()
        target_candidate = target.copy()

    # CUSTOM METHOD

    # find the rotation matrix to align the two vectors and rotate the source with a pivot in the point source_candidate[0]
    vec_A = target_candidate[-1] - target_candidate[0]
    vec_B = source_candidate[-1] - source_candidate[0]
    rot = rigid_rotation_matrix_from_vectors(vec_B, vec_A)

    aligned_source_r = np.dot(rot.as_matrix(), source.T).T
    source_candidate_r = np.dot(rot.as_matrix(), source_candidate.T).T

    # print(f"distance between the first and the last point ORIGINAL: {np.linalg.norm(source_candidate[-1] - source_candidate[0])}")
    # print(f"distance between the first and the last point ROTATION: {np.linalg.norm(source_candidate_r[-1] - source_candidate_r[0])}")
    # assert np.linalg.norm(source_candidate[-1] - source_candidate[0]) == np.linalg.norm(source_candidate_r[-1] - source_candidate_r[0]), "[ERROR]: The distance between the first and the last point is not the same"

    # translation
    point_start = source_candidate_r[0]
    point_end = target_candidate[0]
    trans = point_end - point_start
    aligned_source_r_t = aligned_source_r.copy() + trans
    source_candidate_r_t = source_candidate_r + trans

    # print(f"distance between the first and the last point ORIGINAL: {np.linalg.norm(source_candidate[-1] - source_candidate[0])}")
    # print(f"distance between the first and the last point TRANSLATION+ROTATION: {np.linalg.norm(source_candidate_r_t[-1] - source_candidate_r_t[0])}")
    # assert np.linalg.norm(source_candidate[-1] - source_candidate[0]) == np.linalg.norm(source_candidate_r_t[-1] - source_candidate_r_t[0]), "[ERROR]: The distance between the first and the last point is not the same"


    aligned_source = aligned_source_r_t



    return aligned_source

def _visualize_trajectories(est_pos,
                            est_rot,
                            gt_pos,
                            gt_rot,
                            est_idx_candidates,
                            ts_poss_ori,
                            out_dir : str = "./",
                            is_show : bool = True) -> None:
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

    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color='red', alpha=0.5)
    ax.text(est_pos[0, 0], est_pos[0, 1], est_pos[0, 2], str("transform ts"), color='red', fontsize=8)

    # TODO: debug get rid
    if len(est_idx_candidates) != 0:
        est_pos_candidates = est_pos[est_idx_candidates]
        ax.scatter(est_pos_candidates[:, 0], est_pos_candidates[:, 1], est_pos_candidates[:, 2], color='green', s=4)

    # ax.scatter(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color='green', s=1)
    # ax.scatter(est_pos[-1, 0], est_pos[-1, 1], est_pos[-1, 2], color='black', s=1)

    # save img
    # timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")  # TODO: debug
    # name_file = out_dir + f"/traj_{timestamp}.png"
    # plt.savefig(name_file, dpi=300, bbox_inches='tight', pad_inches=0)

    # TODO: debug original first trajectory ts
    ax.plot(ts_poss_ori[:, 0], ts_poss_ori[:, 1], ts_poss_ori[:, 2], color='blue', alpha=0.5)
    ax.text(ts_poss_ori[0, 0], ts_poss_ori[0, 1], ts_poss_ori[0, 2], str("original ts"), color='blue', fontsize=8)


    if is_show:
        plt.show()


def main(gt_path : str,
         ts_path : str,
         out_dir : str) -> None:

    frame_start = int(ts_path.split('/')[-1].split('_')[0])
    frame_end = int(ts_path.split('/')[-1].split('_')[1].split('.')[0])

    opti_poss = []
    opti_rots = []
    with open(gt_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        next(reader) # skip header
        for data in reader:
            frame_number = int(data[0])
            if frame_number >= frame_start and frame_number <= frame_end:
                timestamp = float(data[1])
                camera_pos = np.array([float(data[2]), float(data[3]), float(data[4])])
                camera_rot = np.array([float(data[5]), float(data[6]), float(data[7]), float(data[8])])
                c_R = _quat_to_euler(camera_rot)
                opti_poss.append(camera_pos)
                opti_rots.append(c_R)
    opti_poss = np.array(opti_poss)
    opti_rots = np.array(opti_rots)

    ts_poss = []
    ts_rots = []
    ts_tags = []
    ts_idx_candidates = []
    ts_coverages = []  # per pose, quality of the signal 0: good, 1:bad (detected drift)
    with open(ts_path, "r") as f:
        lines = f.readlines()
        lines = lines[1:]  # skip header
        for l in lines:
            l = l.split()
            frame_number = int(l[0])
            time_stamp = float(l[1])
            camera_pos = np.array([float(l[2]), float(l[3]), float(l[4])])
            camera_pos = camera_pos * 0.02  # FIXME: scale to meters
            camera_rot = np.array([float(l[5]), float(l[6]), float(l[7]), float(l[8])])
            c_R = _quat_to_euler(camera_rot)
            ts_tag = int(l[9])
            ts_cover = 0

            # >>>>>>> Post-process checks >>>>>>>
            # correct the drifting by replacing the 0 0 0 0 0 0 0 with the previous known pose
            if np.linalg.norm(camera_pos) == 0 and frame_number != 1 and len(ts_poss) > 0:
                camera_pos = ts_poss[-1]
                camera_rot = ts_rots[-1]
                ts_cover = 1
            
            # FIXME: fix the mechanism otherwise if relocalization happens it is lost
            # if the current position is away from the previous position by more than 5cm, then replace it with the previous known pose
            if ts_poss.__len__() != 0 and np.linalg.norm(camera_pos - ts_poss[-1]) > 0.05 and frame_number != 1:
                camera_pos = ts_poss[-1]
                camera_rot = ts_rots[-1]
                ts_cover = 1
            # <<<<<<<< Post-process checks <<<<<<<<

            ts_poss.append(camera_pos)
            ts_rots.append(c_R)
            ts_tags.append(ts_tag)
            ts_coverages.append(ts_cover)

    ts_poss = np.array(ts_poss)
    ts_rots = np.array(ts_rots)
    ts_tags = np.array(ts_tags)

    # print(f"ts_tags: {ts_tags}")
    print(f"ts_coverages: {ts_coverages}")


    ##################
    # select candidate idx

    # find the indexes of the 3 biggest tags and retrieve the corresponding tslam positions
    ts_idx_candidates = _select_best_idx(ts_tags,
                                         ts_coverages)

    ##################

    assert len(opti_poss) == len(ts_poss), "[ERROR]: The number of frames in the ground truth and the tslam positions do not match: {} vs {}".format(len(opti_poss), len(ts_poss))
    assert len(opti_rots) == len(ts_rots), "[ERROR]: The number of frames in the ground truth and the tslam orientations do not match: {} vs {}".format(len(opti_rots), len(ts_rots))
    assert len(ts_tags) == len(ts_poss), "[ERROR]: The number of frames in the tslam positions and the tslam marks do not match: {} vs {}".format(len(ts_poss), len(ts_tags))

    # #######################################################
    # # Select candidate faces
    # #######################################################


    # #######################################################
    # # Extra
    # #######################################################

    # # apply a smoothing filter to the tslam trajectory
    filter_size = 21  # FIXME: is this scientifically sound?
    opti_poss = savgol_filter(opti_poss, filter_size, 3, axis=0)  # FIXME: understand if it is correct to do it and what is the impact
    ts_poss = savgol_filter(ts_poss, filter_size, 3, axis=0)  # FIXME: understand if it is correct to do it and what is the impact

    # #######################################################
    # # Trajectory registration
    # #######################################################

    ts_poss_ori = ts_poss.copy()
    ts_poss = _align_trajectories(ts_poss,
                                  opti_poss,
                                  ts_idx_candidates
                                  )

    # #######################################################
    # # Visualization
    # #######################################################

    _visualize_trajectories(ts_poss,
                            ts_rots,
                            opti_poss,
                            opti_rots,
                            ts_idx_candidates,
                            ts_poss_ori,
                            out_dir)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the metrics for tslam.')
    parser.add_argument('--name', type=str, default="noname", help='Name of the sequence.')
    parser.add_argument('--gt', type=str, help='Path to the ground truth trajectory file (refined trajectory).')
    parser.add_argument('--ts', type=str, help='Path to the tslam trajectory file.')
    # parser.add_argument('--start', type=str, help='The start frame of the sequence.')
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

    # SINGLE MODE METHOD
    # main(gt_path=args.gt,
    #      ts_path=args.ts,
    #      out_dir=args.out)

    # BATCH METHOD
    ts_files : str = "/home/as/TSlam/eval/script/test/outposes/01_2023-05-19_19-29-01"  # TODO: replace with arg

    # reorder file by modification time
    files = [os.path.join(ts_files, f) for f in os.listdir(ts_files) if os.path.isfile(os.path.join(ts_files, f))]
    files.sort(key=lambda x: os.path.getmtime(x))


    for idx, file in tqdm(enumerate(files)):
        if file.endswith(".txt") and not "running_log" in file:
            file_path = os.path.join(ts_files, file)
            print(file)
            main(gt_path=args.gt,
                ts_path=file_path,
                out_dir=args.out)
            # if idx == 6:
            #     break


