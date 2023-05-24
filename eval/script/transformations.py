import math
import quaternion
import sys

import numpy as np
from scipy.spatial.transform import Rotation

def cvt_quat_to_ROSformat(quat):
    """ Convert an OptiTrack quaternion to a ROS quaternion """
    return np.array([quat[3], quat[0], quat[1], quat[2]])

def quaternion_to_rotation_vector(q):
    # if 0 return an a 0,0,0 vector or nan
    if np.linalg.norm(q) == 0:
        return np.zeros(3)
    if math.isnan(q[0]) or math.isnan(q[1]) or math.isnan(q[2]) or math.isnan(q[3]):
        return np.zeros(3)
    # convert quaternion in format x,y,z,w to rotation vector
    rotation_vector = Rotation.from_quat(np.array([q[0], q[1], q[2], q[3]])).as_rotvec()

    return rotation_vector

def rotation_vector_to_matrix(rot_vec):
    """ Convert a rotation vector to a rotation matrix """
    rot = Rotation.from_rotvec(rot_vec)
    return rot.as_matrix()

def rotation_matrix_from_vectors(vec1, vec2):
    """ Find a rigid rotation matrix that aligns vec1 to vec2 """

    vec1 = vec1 / np.linalg.norm(vec1)
    vec2 = vec2 / np.linalg.norm(vec2)

    # the rotation axis is the cross product between the two vectors
    axis = np.cross(vec1, vec2)
    axis = axis / np.linalg.norm(axis)

    # the rotation angle is the angle between the two vectors
    angle = np.arccos(np.dot(vec1, vec2))

    # the rigid rotation matrix
    rot = Rotation.from_rotvec(angle * axis) # Rodrigues formula

    return rot

def get_perpendicular_plane_to_vector(vec):
    """ Find a plane perpendicular to a vector """
    
    # find a vector perpendicular to the input vector
    vec_perp = np.cross(vec, np.array([1, 0, 0]))
    if np.linalg.norm(vec_perp) == 0:
        vec_perp = np.cross(vec, np.array([0, 1, 0]))
    vec_perp = vec_perp / np.linalg.norm(vec_perp)

    # find a second vector perpendicular to the input vector and the first vector
    vec_perp2 = np.cross(vec, vec_perp)
    vec_perp2 = vec_perp2 / np.linalg.norm(vec_perp2)

    # find the plane perpendicular to the input vector
    plane = np.cross(vec_perp, vec_perp2)
    plane = plane / np.linalg.norm(plane)

    return plane

def angle_between_vectors(vec1, vec2, plane):
    """ Find the angle between two vectors on a plane"""

    # check the vectors are on the plane
    if np.dot(vec1, plane) != 0 or np.dot(vec2, plane) != 0:
        vec1 = vec1 - np.dot(vec1, plane) * plane
        vec2 = vec2 - np.dot(vec2, plane) * plane
    
    # find the angle between the two vectors
    angle = np.arccos(np.dot(vec1, vec2) / (np.linalg.norm(vec1) * np.linalg.norm(vec2)))

    return angle

def rotate_around_axis(axis, angle):
    """ Rotate a vector around an axis by an angle """

    axis_orig = axis
    axis = axis / np.linalg.norm(axis)
    rot = Rotation.from_rotvec(angle * axis)

    return rot

def rotate_matrix_from_euler(mat : np.array,
                             roll : float, 
                             pitch : float,
                             yaw : float):
    """ Convert euler angles to a rotation matrix in xyz global frame"""
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw])
    mat = np.dot(rot, mat.T).T
    return mat
