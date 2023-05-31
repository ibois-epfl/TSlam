import numpy as np
import transformations as tfm
import os
from tqdm import tqdm
import sys
import matplotlib.pyplot as plt
import metrics

#===============================================================================
# sub-sequence visuals
#===============================================================================

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

#===============================================================================
# sequence visuals
#===============================================================================

# TODO: we could add:
# - on the top of the graph write the median, mean, std, min, max of the drift
# - the mean time for each operation
def _draw_boxplot(data : np.array,
                  labels : np.array,
                  ylabel : str) -> plt.figure:
    fig, ax = plt.subplots()
    fig.set_size_inches(10., 5.)
    ax.set_xlabel("Tools' names (nbr of operations)")
    ax.set_ylabel(ylabel)
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(True)
    ax.spines['bottom'].set_visible(True)

    boxplot = ax.boxplot(data, labels=labels,
        #     patch_artist = True,
        #    boxprops = dict(facecolor = "lightgray", color = "black"),
        notch=False, sym='+', vert=True, whis=1.5,
        positions=None, widths=None,
        bootstrap=None, usermedians=None, conf_intervals=None)
    for whisker in boxplot['whiskers']:
        whisker.set(color='black', linewidth=1)
    for cap in boxplot['caps']:
        cap.set(color='black', linewidth=2)
    for median in boxplot['medians']:
        cybergreen = '#2DDE98'
        median.set(color=cybergreen, linewidth=2)
    for flier in boxplot['fliers']:
        flier.set(marker='+', color="black", alpha=0.5)

    fig.autofmt_xdate()  # to avoid xlabels overlapping

    fig.tight_layout()
    return fig

def _get_clean_empty_tool_results() -> dict:
    """ Return a copy of the tool dictionary without the empty results """
    TOOLS_clean = {}
    for key, value in metrics.TOOLS.items():
        if len(value[1]._mean_drift_position_NONAN) == 0:
            continue
        TOOLS_clean[key] = value
    return TOOLS_clean

def visualize_box_plots(out_dir : str) -> None:
    """ Visualize the box plots """
    graph_dir = os.path.join(out_dir, "sequence", "graph")
    os.makedirs(graph_dir, exist_ok=True)

    TOOLS_clean = _get_clean_empty_tool_results()
    
    boxplot_labels = np.array([f"{key}\n({TOOLS_clean[key][1].nbr_operations})" for key in TOOLS_clean.keys()])
    boxplot_body_position = np.array([TOOLS_clean[key][1]._mean_drift_position_NONAN for key in TOOLS_clean.keys()])
    boxplot_outliers_position = np.array([TOOLS_clean[key][1].mean_drift_position_outliers for key in TOOLS_clean.keys()])
    boxplot_body_orientation = np.array([TOOLS_clean[key][1]._mean_drift_rotation_NONAN for key in TOOLS_clean.keys()])
    boxplot_outliers_orientation = np.array([TOOLS_clean[key][1].mean_drift_rotation_outliers for key in TOOLS_clean.keys()])
    boxplot_body_tags = np.array([TOOLS_clean[key][1]._tags_NONAN for key in TOOLS_clean.keys()])
    boxplot_outliers_tags = np.array([TOOLS_clean[key][1].tags_outliers for key in TOOLS_clean.keys()])

    fig_position = _draw_boxplot(boxplot_body_position, boxplot_labels, "Error position [m]")
    fig_rotation = _draw_boxplot(boxplot_body_orientation, boxplot_labels, "Error orientation [deg]")
    fig_tags = _draw_boxplot(boxplot_body_tags, boxplot_labels, "Detected tags [nbr]")

    fig_position.savefig(os.path.join(graph_dir, "boxplot_position_graph.png"), dpi=300)
    fig_rotation.savefig(os.path.join(graph_dir, "boxplot_rotation_graph.png"), dpi=300)
    fig_tags.savefig(os.path.join(graph_dir, "boxplot_tags_graph.png"), dpi=300)

    plt.close()

# TODO: we can just indicate the median value here for each curve
def visualize_quintiles_plot(out_dir : str) -> None:
    """ Visualize the quintiles plot to inform about the coverage distribution during the fabrication """
    graph_dir = os.path.join(out_dir, "sequence", "graph")
    if not os.path.exists(graph_dir):
        os.makedirs(graph_dir)
    graph_dir = os.path.join(graph_dir, "quintiles_graph.png")

    TOOLS_clean = _get_clean_empty_tool_results()

    # draw the coverage quartiles as a graph line of different color, each for the tool
    fig, ax = plt.subplots()
    fig.set_size_inches(10., 5.)
    ax.set_title(f"Median coverage per fabrications' quintiles")
    ax.set_xlabel("Quintiles")
    ax.set_ylabel("Coverage [%]")
    ax.set_ylim(0, 100)
    ax.set_xlim(0, 5)

    # get rid of right axis border
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(True)
    ax.spines['bottom'].set_visible(True)
    
    # set manually the xticks
    ax.set_xticks([1, 2, 3, 4, 5])

    # draw one vertical dashed vertical line per quintile 1 and 4
    ax.axvline(x=1, color='black', linestyle='--', linewidth=1)
    ax.axvline(x=4, color='black', linestyle='--', linewidth=1)

    # set ticks style to be in the middle of the axis
    ax.tick_params(axis='x', which='major', pad=10)


    x_values = np.array([0, 1.5, 2.5, 3.5, 5])
    # apply a filter and multiply the x values

    # get a list of cold variety of colors
    clrs = plt.cm.tab20(np.linspace(0, 1, len(TOOLS_clean.keys())))
    from scipy.signal import savgol_filter

    counter : int = 0
    for tool_id, res in TOOLS_clean.items():
        clr = clrs[counter]

        y_values = res[1].mean_coverage_perc_quintiles

        # window_size = 5  # Adjust the window size as needed
        # order = 5  # Adjust the order of the polynomial fit as needed
        # y_smooth = savgol_filter(y_values, window_size, 2)
        # ax.plot(x_values, y_smooth, label=tool_id, color=rndm_clr)

        ax.plot(x_values, y_values, label=tool_id, color=clr)

        counter += 1
    







    # VER1: all the sub-sequences
    # for tool_id, res in TOOLS_clean.items():
    #     rndm_clr = np.random.rand(3,)
    #     for coverage_means_quintiles_fab in res[1]._coverage_mean_per_fabrication_quintiles:
    #         ax.plot(xvalues, coverage_means_quintiles_fab, label=tool_id, color=rndm_clr)

    # VER2: only median of the sub-sequences confondu
    # for tool_id, res in TOOLS_clean.items():
        # rndm_clr = np.random.rand(3,)
        # ax.plot(xvalues, res[1].mean_coverage_perc_quintiles, label=tool_id, color=rndm_clr)


    plt.show()
    
    fig.savefig(graph_dir, dpi=300)

    plt.close()





