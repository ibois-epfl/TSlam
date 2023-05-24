import numpy as np
import csv
from scipy.spatial.transform import Rotation
import math

import transformations as tfm

# FIXME: check the ground truth, it seems to be a bit early
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
    return opti_poss, opti_vec_rots

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
            # ts_cover : int = 1
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
            # camera_rot = tfm.cvt_quat_to_ROSformat(camera_rot)
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