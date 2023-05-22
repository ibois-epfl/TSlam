import matplotlib.pyplot as plt
import numpy as np
import transformations as tfm


def __draw_local_axis_pose(ax : plt.Axes,
                           origin : np.array,
                           rot : np.array,
                           scale_f : float,
                           alpha : float = 0.5,
                           linewidth : int = 1) -> None:
    """ 
        Draw the local axis axis x,y,z in red, green, blue of a given pose
        Args:
            ax (plt.Axes): the axis to draw the local axis
            origin (np.array): the origin of the pose
            rot (np.array): the rotation of the pose
            scale_f (float): the scale factor of the axis dimensions
            alpha (float): the alpha of the axis
            linewidth (int): the linewidth of the axis
    """
    rot_c = rot.copy()
    origin_c = origin.copy()
    rot_c_ax = np.array([origin_c, origin_c + rot_c * 0.1])
    ax.plot(rot_c_ax[:, 0], rot_c_ax[:, 1], rot_c_ax[:, 2], color='purple', alpha=alpha, linewidth=linewidth)



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

    ax.plot(axis_x[:, 0], axis_x[:, 1], axis_x[:, 2], color='red',   alpha=alpha, linewidth=linewidth)
    ax.plot(axis_y[:, 0], axis_y[:, 1], axis_y[:, 2], color='green', alpha=alpha, linewidth=linewidth)
    ax.plot(axis_z[:, 0], axis_z[:, 1], axis_z[:, 2], color='blue',  alpha=alpha, linewidth=linewidth)

    ax.scatter(origin[0], origin[1], origin[2], color='black', alpha=alpha, s=1, linewidth=linewidth, marker='s')

def visualize_trajectories(est_pos,
                            est_rot,
                            gt_pos,
                            gt_rot,
                            est_idx_candidates,
                            out_dir : str = "./",
                            is_show : bool = False) -> None:
    """ Visualize the trajectories in a 3D-axis plot """

    # VISUALS
    fig = plt.figure()
    fig.set_size_inches(18.5, 10.5)
    ax = fig.add_subplot(projection='3d')
    ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.view_init(elev=30, azim=45)

    # add legend on the bottom right
    ax.text2D(0.05, 0.92, "Ground Truth (gt)", transform=ax.transAxes, color='black', fontsize=8)
    ax.text2D(0.05, 0.90, "Tslam (ts)", transform=ax.transAxes, color='grey', fontsize=8)

    # adjust he distorsion of the plot to the center of the trajectories
    center = np.mean(gt_pos, axis=0)
    XYZ_LIM = 0.10
    ax.set_xlim3d(center[0] - XYZ_LIM, center[0] + XYZ_LIM)
    ax.set_ylim3d(center[1] - XYZ_LIM, center[1] + XYZ_LIM)
    ax.set_zlim3d(center[2] - XYZ_LIM, center[2] + XYZ_LIM)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    # positions
    ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], color='black', alpha=0.5)
    # ax.text(gt_pos[0, 0], gt_pos[0, 1], gt_pos[0, 2], str("   gt"), color='black', fontsize=8)

    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color='grey', alpha=0.5)
    # ax.text(est_pos[0, 0], est_pos[0, 1], est_pos[0, 2], str("   ts"), color='grey', fontsize=8)

    # rotations
    # __draw_local_axis_pose(ax, est_pos[0], est_rot[0],
    #                         scale_f=0.01, alpha=0.5, linewidth=1)
    # __draw_local_axis_pose(ax, gt_pos[0], gt_rot[0],
    #                         scale_f=0.01, alpha=1, linewidth=2)
    for idx in est_idx_candidates:
        if idx % 30 == 0:
            __draw_local_axis_pose(ax, est_pos[idx], est_rot[idx],
                                   scale_f=0.01, alpha=0.5, linewidth=1)
            __draw_local_axis_pose(ax, gt_pos[idx], gt_rot[idx],
                                   scale_f=0.01, alpha=1, linewidth=1)

    # if args.showPlot:
    if is_show:
        plt.show()