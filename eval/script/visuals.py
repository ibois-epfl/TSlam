import numpy as np
import transformations as tfm
import os
from tqdm import tqdm
import sys
import matplotlib.pyplot as plt

def __draw_local_axis_pose(ax : plt.Axes,
                           origin : np.array,
                           rot : np.array,
                           scale_f : float,
                           alpha : float = 0.5,
                           linewidth : int = 1,
                           is_draw_rot_vec : bool = False,
                           color : str = None) -> None:
    """ 
        Draw the local axis axis x,y,z in red, green, blue of a given pose
        Args:
            ax (plt.Axes): the axis to draw the local axis
            origin (np.array): the origin of the pose
            rot (np.array): the rotation of the pose
            scale_f (float): the scale factor of the axis dimensions
            alpha (float): the alpha of the axis
            linewidth (int): the linewidth of the axis
            is_draw_rot_vec (bool): if True, it draws the rotation vector
            color (list[int]): the color of the axis in string name mplt
    """
    # draw the rotation vector
    if is_draw_rot_vec:
        rot_c = rot.copy()
        origin_c = origin.copy()
        rot_c_ax = np.array([origin_c, origin_c + rot_c * 0.1])
        ax.plot(rot_c_ax[:, 0], rot_c_ax[:, 1], rot_c_ax[:, 2], color='purple', alpha=alpha, linewidth=linewidth)

    # draw the rotation axis system
    # create a cordinate system
    x = np.array([1, 0, 0])
    y = np.array([0, 1, 0])
    z = np.array([0, 0, 1])

    # rotate the cordinate system with the rotation vector
    x = tfm.rotation_vector_to_matrix(rot) @ x
    y = tfm.rotation_vector_to_matrix(rot) @ y
    z = tfm.rotation_vector_to_matrix(rot) @ z

    # # normalize the axis
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    z = z / np.linalg.norm(z)

    # draw the local axis
    axis_x = np.array([origin, origin + x*scale_f])
    axis_y = np.array([origin, origin + y*scale_f])
    axis_z = np.array([origin, origin + z*scale_f])

    clr_axis_x = "red"
    clr_axis_y = "green"
    clr_axis_z = "blue"
    if color is not None:
        clr_axis_x = color
        clr_axis_y = color
        clr_axis_z = color
    
    ax.plot(axis_x[:, 0], axis_x[:, 1], axis_x[:, 2], color=clr_axis_x,   alpha=alpha, linewidth=linewidth)
    ax.plot(axis_y[:, 0], axis_y[:, 1], axis_y[:, 2], color=clr_axis_y, alpha=alpha, linewidth=linewidth)
    ax.plot(axis_z[:, 0], axis_z[:, 1], axis_z[:, 2], color=clr_axis_z,  alpha=alpha, linewidth=linewidth)

    ax.scatter(origin[0], origin[1], origin[2], color='black', alpha=alpha, s=1, linewidth=linewidth, marker='s')

def visualize_trajectories_3d(est_pos,
                              est_rot,
                              gt_pos,
                              gt_rot,
                              coverages,
                              is_show : bool = False,
                              is_draw_dist_error : bool = False,
                              is_draw_rot_vec : bool = False,
                              title=None,
                              ) -> plt.figure:
    """ Visualize the trajectories in a 3D-axis plot """
    fig = plt.figure()
    fig.set_size_inches(8., 8.)
    ax = fig.add_subplot(projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.view_init(elev=30, azim=45)

    CLR_GT = "magenta"
    CLR_EST = "darkcyan"
    CLR_DIST = "yellow"

    if title is not None:
        ax.set_title(title, fontsize=10)

    FONT_SIZE = 7
    ax.text2D(0.02, 0.04, "Computed error (ce)", transform=ax.transAxes, color=CLR_DIST, fontsize=FONT_SIZE)
    ax.text2D(0.02, 0.02, "Ground Truth (gt)", transform=ax.transAxes, color=CLR_GT, fontsize=FONT_SIZE)
    ax.text2D(0.02, 0.00, "Tslam (ts)", transform=ax.transAxes, color=CLR_EST, fontsize=FONT_SIZE)

    center = np.mean(gt_pos, axis=0)
    XYZ_LIM = 0.10
    ax.set_xlim3d(center[0] - XYZ_LIM, center[0] + XYZ_LIM)
    ax.set_ylim3d(center[1] - XYZ_LIM, center[1] + XYZ_LIM)
    ax.set_zlim3d(center[2] - XYZ_LIM, center[2] + XYZ_LIM)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], color=CLR_GT)
    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color=CLR_EST, alpha=0.5)

    for idx, _ in enumerate(gt_pos):
        if coverages[idx] == True:
            ax.plot([gt_pos[idx, 0], est_pos[idx, 0]],
                    [gt_pos[idx, 1], est_pos[idx, 1]],
                    [gt_pos[idx, 2], est_pos[idx, 2]],
                    color=CLR_DIST, alpha=0.5, linewidth=1)
        if is_draw_rot_vec:
            if coverages[idx] == True:
                __draw_local_axis_pose(ax, est_pos[idx], est_rot[idx],
                                    scale_f=0.01, alpha=0.25, linewidth=2)
            __draw_local_axis_pose(ax, gt_pos[idx], gt_rot[idx],
                                   scale_f=0.01, alpha=1, linewidth=1)

    if is_show:
        plt.show()
    plt.close()

    return fig

def visualize_2d_drift(drift_xyz : np.array,
                       distances : np.array,
                       unit : str = "m",
                       title : str = None,
                       is_show : bool = False
                       ) -> plt.figure:
    """ Visualize the drift in a 2D-axis plot """
    fig = plt.figure()
    fig.set_size_inches(10., 4.)
    ax = fig.add_subplot()
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)

    if title is not None:
        ax.set_title(title, fontsize=10)

    ax.set_xlabel('Distance [m]')
    ax.set_ylabel(f'Error [{unit}]')

    ax.plot(distances, drift_xyz[:, 0], color='red', label='x')
    ax.plot(distances, drift_xyz[:, 1], color='green', label='y')
    ax.plot(distances, drift_xyz[:, 2], color='blue', label='z')
    ax.legend()

    if is_show:
        plt.show()
    plt.close()

    return fig

# TODO: fill me in
def visualized_box_plots() -> None:
    pass