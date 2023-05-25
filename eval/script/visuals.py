import matplotlib.pyplot as plt
import numpy as np
import transformations as tfm
import os
from tqdm import tqdm
import sys

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
                              est_idx_candidates,
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
    ax.text2D(0.02, 0.04, "Distance error (de)", transform=ax.transAxes, color=CLR_DIST, fontsize=FONT_SIZE)
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

    for idx in est_idx_candidates:
        ax.plot([gt_pos[idx, 0], est_pos[idx, 0]],
                [gt_pos[idx, 1], est_pos[idx, 1]],
                [gt_pos[idx, 2], est_pos[idx, 2]],
                color=CLR_DIST, alpha=0.5, linewidth=1)
        if is_draw_rot_vec:
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

def animate_trajectoriy_3d(est_pos,
                           est_rot,
                           gt_pos,
                           gt_rot,
                           est_idx_candidates,
                           total_frames : int,
                           out_dir : str,
                           frame_start : int,
                           frame_end : int,
                           video_path : str = None,
                           is_draw_rot_vec : bool = False,
                           ) -> None:
    """ Create a side-by-side animation with the video of the sequence of the trajectory """
    # create a folder to save the animation gif
    anim_out_dir = os.path.join(out_dir, "animation")
    temp_dir = os.path.join(anim_out_dir, "temp")
    temp_video_dir = os.path.join(temp_dir, "video")
    temp_graph_dir = os.path.join(temp_dir, "graph")

    os.makedirs(anim_out_dir, exist_ok=True)
    os.makedirs(temp_dir, exist_ok=True)
    os.makedirs(temp_video_dir, exist_ok=True)
    os.makedirs(temp_graph_dir, exist_ok=True)

    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    # extract each frame of the video in the temp_video_dir between frame_start and frame_end
    os.system(f"ffmpeg -y -i {video_path} -r 30 {temp_video_dir}/%d.png")

    #>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>

    fig = plt.figure()
    height_mm = 200
    width_mm = 400
    # convert in inches
    height = int(height_mm / 25.4)
    width = int(width_mm / 25.4)
    fig.set_size_inches(width, height)
    # reduce borders
    plt.subplots_adjust(left=0.0, right=1.0, top=1.0, bottom=0.0)

    # place one graph on the right and by side an image on the left with same size
    # ax = fig.add_subplot(projection='3d')
    # add a sublot on the right
    ax = fig.add_subplot(1, 2, 2, projection='3d')
                        #  position=[1., 100., 1000.5, 2]) # [xmin,ymin,dx,dy]
    # # ax.set_position([1, 2, 1, 0])
    ax.grid(False)

    CLR_GT = "magenta"
    CLR_EST = "darkcyan"

    # animate the trajectories
    for idx in tqdm(range(total_frames)):
        if idx % 30 != 0:
            continue
        ax.w_xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.w_zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
        ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
        ax.grid(False)
        ax.view_init(elev=30, azim=45)

        ax.set_title("trajectories", fontsize=10)
        # move the title down
        ax.title.set_position([.5, 1.05])

        FONT_SIZE = 7
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

        ax.plot(gt_pos[:idx, 0], gt_pos[:idx, 1], gt_pos[:idx, 2], color=CLR_GT)
        ax.plot(est_pos[:idx, 0], est_pos[:idx, 1], est_pos[:idx, 2], color=CLR_EST, alpha=0.5)

        SCALE_AXIS = 0.02
        __draw_local_axis_pose(ax, est_pos[idx], est_rot[idx],
                                    scale_f=SCALE_AXIS, alpha=0.25, linewidth=2)
        __draw_local_axis_pose(ax, gt_pos[idx], gt_rot[idx],
                                scale_f=SCALE_AXIS, alpha=1, linewidth=1)


        #>>>>>>>>>>>>>>>>>>>>>>>>

        # add a png on the left side of the plot
        img = plt.imread(os.path.join(temp_video_dir, f"{idx+1}.png"))
        ax2 = fig.add_subplot(1, 2, 1)

        ax2.imshow(img)
        ax2.axis('off')

        fig.savefig(os.path.join(temp_graph_dir, f"{idx}.png"))

        # # TODO: debug
        # plt.show()
        # plt.close()
        # sys.exit()
        
        
        ax.clear()

    # get all the images in the temp folder and make a gif lightweight and loopable with ffmpeg with width 400px and height 200px
    os.system(f"ffmpeg -y -f image2 -r 30 -i {temp_graph_dir}/%d.png -vcodec libx264 -crf 25 -pix_fmt yuv420p {anim_out_dir}/animation.mp4")

    # make a gif with ffmpeg with high quality
    # os.system(f"ffmpeg -y -i {anim_out_dir}/animation.mp4 -vf fps=30 {anim_out_dir}/animation.gif")

    # erasse the temp folder
    # os.system(f"rm -rf {temp_dir}")