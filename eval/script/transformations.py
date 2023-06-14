import math
import quaternion
import sys

import numpy as np
from scipy.spatial.transform import Rotation

# https://math.stackexchange.com/questions/40164/how-do-you-rotate-a-vector-by-a-unit-quaternion
def quaternion_multiply(quaternion1, quaternion0):
    w0, x0, y0, z0 = quaternion0
    w1, x1, y1, z1 = quaternion1
    return np.array([-x1 * x0 - y1 * y0 - z1 * z0 + w1 * w0,
                    x1 * w0 + y1 * z0 - z1 * y0 + w1 * x0,
                    -x1 * z0 + y1 * w0 + z1 * x0 + w1 * y0,
                    x1 * y0 - y1 * x0 + z1 * w0 + w1 * z0], dtype=np.float64)

def quaternion_inv(quaternion):
    return np.array([quaternion[0], -quaternion[1], -quaternion[2], -quaternion[3]])

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

def get_rigid_trans_mat_umeyama(src, target, use_scale=False):
    """
        Rigid alignment of two sets of points in 3-dimensional
        Euclidean space. Given two sets of points in
        correspondence, this function computes the scaling,
        rotation, and translation that define the transform TR
        that minimizes the sum of squared errors between TR(src)
        and its corresponding points in target.  This routine takes
        O(n 3^3)-time.
        source: https://gist.github.com/CarloNicolini/7118015

        Args:
            np.array: src, (n, 3) matrix, n points in 3-dimensional space
            np.array: target, (n, 3) matrix of n points in 3-dimensional space

        Returns: 
            trans_mat - transformation matrix (4, 4) that transforms src to target
    """

    src = src.T
    target = target.T

    m, n = src.shape

    mx = src.mean(1)
    my = target.mean(1)
    Xc =  src - np.tile(mx, (n, 1)).T
    Yc =  target - np.tile(my, (n, 1)).T

    sx = np.mean(np.sum(Xc*Xc, 0))

    Sxy = np.dot(Yc, Xc.T) / n

    U,D,V = np.linalg.svd(Sxy,full_matrices=True,compute_uv=True)
    V=V.T.copy()

    S = np.eye(m)

    R = np.dot( np.dot(U, S ), V.T)

    c = 1
    if use_scale:
        c = np.trace(np.dot(np.diag(D), S)) / sx
    
    t = my - c * np.dot(R, mx)
    R = c * R
    
    trans_mat = np.array([
        [R[0,0], R[0,1], R[0,2], t[0]],
        [R[1,0], R[1,1], R[1,2], t[1]],
        [R[2,0], R[2,1], R[2,2], t[2]],
        [0     , 0     , 0     , 1   ]
    ])

    return trans_mat

def transform(trans_mat, points):
    """
    Transform points with transformation matrix

    Args:
        np.array: trans_mat, (4, 4) transformation matrix
        np.array: points, (n, 3) matrix of n points in 3-dimensional space

    Returns:
        np.array: reproj_points, (n, 3) matrix of n points in 3-dimensional space
    """
    reproj_points = np.concatenate((points,np.ones((points.shape[0], 1))),axis=1)
    reproj_points = (trans_mat.dot(reproj_points.T).T)[:,:3]
    return reproj_points