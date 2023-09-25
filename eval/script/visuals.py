import numpy as np
import transformations as tfm
import os
from tqdm import tqdm
import cv2
import sys
import open3d as o3d
import matplotlib
import matplotlib.pyplot as plt
import metrics

CYBERGREEN = '#2DDE98'

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
                              idx_candidates,
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

    FONT_SIZE = 10
    # ax.text2D(0.02, 0.04, "Computed error (ce)", transform=ax.transAxes, color=CLR_DIST, fontsize=FONT_SIZE)
    # ax.text2D(0.02, 0.02, "Ground Truth (gt)", transform=ax.transAxes, color=CLR_GT, fontsize=FONT_SIZE)
    # ax.text2D(0.02, 0.00, "Tslam (ts)", transform=ax.transAxes, color=CLR_EST, fontsize=FONT_SIZE)

    center = np.mean(gt_pos, axis=0)
    XYZ_LIM = 0.10
    ax.set_xlim3d(center[0] - XYZ_LIM, center[0] + XYZ_LIM)
    ax.set_ylim3d(center[1] - XYZ_LIM, center[1] + XYZ_LIM)
    ax.set_zlim3d(center[2] - XYZ_LIM, center[2] + XYZ_LIM)

    ax.set_xlabel('X [m]')
    ax.set_ylabel('Y [m]')
    ax.set_zlabel('Z [m]')

    ax.plot(gt_pos[:, 0], gt_pos[:, 1], gt_pos[:, 2], color=CLR_GT, label="Ground Truth")
    ax.plot(est_pos[:, 0], est_pos[:, 1], est_pos[:, 2], color=CLR_EST, alpha=0.5, label="Tslam")
    ax.plot([], [], [], color=CLR_DIST, alpha=0.5, linewidth=1, label="Distance error")

    ax.legend(loc='lower right', fontsize=FONT_SIZE)

    for idx, _ in enumerate(gt_pos):
        # if coverages[idx] == True:
        #     ax.plot([gt_pos[idx, 0], est_pos[idx, 0]],
        #             [gt_pos[idx, 1], est_pos[idx, 1]],
        #             [gt_pos[idx, 2], est_pos[idx, 2]],
        #             color=CLR_DIST, alpha=0.5, linewidth=1, label="Distance error")
        if is_draw_rot_vec:
            if coverages[idx] == True:
                __draw_local_axis_pose(ax, est_pos[idx], est_rot[idx],
                                    scale_f=0.01, alpha=0.25, linewidth=2)
            __draw_local_axis_pose(ax, gt_pos[idx], gt_rot[idx],
                                   scale_f=0.01, alpha=1, linewidth=1)
    
    # draw sphere for each candidate
    # print(np.average(np.sqrt((gt_pos - est_pos) ** 2).sum(axis=1))) # Average distance error

    for idx in idx_candidates:
        if idx % 5 != 0:
            continue
        ax.plot([gt_pos[idx, 0], est_pos[idx, 0]],
                [gt_pos[idx, 1], est_pos[idx, 1]],
                [gt_pos[idx, 2], est_pos[idx, 2]],
                color=CLR_DIST, alpha=1, linewidth=1, label="Distance error")

        ax.plot(gt_pos[idx, 0], gt_pos[idx, 1], gt_pos[idx, 2], color=CLR_GT, alpha=1, linewidth=1, marker='o', markersize=5)
        ax.plot(est_pos[idx, 0], est_pos[idx, 1], est_pos[idx, 2], color=CLR_EST, alpha=1, linewidth=1, marker='o', markersize=5)
        

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

def _draw_boxplot(data : np.array,
                  labels : np.array,
                  metrics_info : list[str],
                  ylabel : str) -> plt.figure:
    fig, ax = plt.subplots()
    fig.set_size_inches(10., 6.)
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
    for median in boxplot['medians']:  # <--- FIXME: check for mean
        median.set(color=CYBERGREEN, linewidth=2)
    for flier in boxplot['fliers']:
        flier.set(marker='+', color="black", alpha=0.5)

    fig.autofmt_xdate()  # to avoid xlabels overlapping

    metrics_info_str = []
    for idx, info in enumerate(metrics_info):
        i_txt_O = f"O:{info[0]}"
        i_txt_M = f"M:{info[1]}"
        i_txt_Q1 = f"q1:{info[2]}"
        i_txt_Q3 = f"q3:{info[3]}"
        i_txt_MIN = f"mn:{info[4]}"
        i_txt_MAX = f"Mx:{info[5]}"

        i_txt_OM = f"{i_txt_O} {i_txt_M}"
        i_txt_Q1Q3 = f"{i_txt_Q1} {i_txt_Q3}"
        i_txt_MINMAX = f"{i_txt_MIN} {i_txt_MAX}"
        metrics_info_str.append(f"{i_txt_OM}\n{i_txt_Q1Q3}\n{i_txt_MINMAX}")

    # find emplacement for info text
    xlabels_loc = ax.get_xticks()
    max_y_lst = []
    for d in data:
        max_y_lst.append(np.max(d))
    MAX_y = np.max(np.array(max_y_lst))
    max_y_lst = []
    for d in data:
        max_y_extra = np.max(d) + 0.1 * MAX_y
        max_y_lst.append(max_y_extra)
    for idx, xloc in enumerate(xlabels_loc):
        ax.text(x=xloc, y=max_y_lst[idx], s=metrics_info_str[idx], fontsize=8, horizontalalignment='center')

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
    
    boxplot_labels = np.array([f"{key}" for key in TOOLS_clean.keys()])
    
    boxplot_body_position = np.array([TOOLS_clean[key][1]._mean_drift_position_NONAN for key in TOOLS_clean.keys()])
    boxplot_metrics_position_info = [
        [
            f"{TOOLS_clean[key][1].nbr_operations}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_position_m, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_position_q1, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_position_q3, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_position_min, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_position_max, 3)}\n",
        ] for key in TOOLS_clean.keys()
    ]

    boxplot_body_rotation = np.array([TOOLS_clean[key][1]._mean_drift_rotation_NONAN for key in TOOLS_clean.keys()])
    boxplot_metrics_rotation_info = [
        [
            f"{round(TOOLS_clean[key][1].nbr_operations, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_rotation_m, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_rotation_q1, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_rotation_q3, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_rotation_min, 3)}\n",
            f"{round(TOOLS_clean[key][1].mean_drift_rotation_max, 3)}\n",
        ] for key in TOOLS_clean.keys()
    ]

    boxplot_body_tags = np.array([TOOLS_clean[key][1]._tags_NONAN for key in TOOLS_clean.keys()])
    boxplot_metrics_tag_info = [
        [
            f"{TOOLS_clean[key][1].nbr_operations}\n",
            f"{round(TOOLS_clean[key][1].tags_m, 1)}\n",
            f"{round(TOOLS_clean[key][1].tags_q1, 1)}\n",
            f"{round(TOOLS_clean[key][1].tags_q3, 1)}\n",
            f"{round(TOOLS_clean[key][1].tags_min, 1)}\n",
            f"{round(TOOLS_clean[key][1].tags_max, 1)}\n",
        ] for key in TOOLS_clean.keys()
    ]

    fig_position = _draw_boxplot(data=boxplot_body_position,
                                 labels=boxplot_labels,
                                 metrics_info=boxplot_metrics_position_info,
                                 ylabel="Error position [m]")
    fig_rotation = _draw_boxplot(data=boxplot_body_rotation,
                                 labels=boxplot_labels,
                                 metrics_info=boxplot_metrics_rotation_info,
                                 ylabel="Error orientation [deg]")
    fig_tags = _draw_boxplot(data=boxplot_body_tags,
                             labels=boxplot_labels,
                             metrics_info=boxplot_metrics_tag_info,
                             ylabel="Detected tags [nbr]")

    fig_position.savefig(os.path.join(graph_dir, "boxplot_position_graph.png"), dpi=300)
    fig_rotation.savefig(os.path.join(graph_dir, "boxplot_rotation_graph.png"), dpi=300)
    fig_tags.savefig(os.path.join(graph_dir, "boxplot_tags_graph.png"), dpi=300)

    plt.close()

def visualize_quintiles_plot(out_dir : str) -> None:
    """ Visualize the quintiles plot to inform about the coverage distribution during the fabrication """
    graph_dir = os.path.join(out_dir, "sequence", "graph")
    if not os.path.exists(graph_dir):
        os.makedirs(graph_dir)
    graph_dir = os.path.join(graph_dir, "quintiles_graph.png")

    TOOLS_clean = _get_clean_empty_tool_results()

    # draw the coverage quartiles as a graph line of different color, each for the tool
    fig, ax = plt.subplots()
    fig.set_size_inches(10., 4.)
    ax.set_title(f"Mean coverage")
    ax.set_xlabel("Fabrication timeline [quintiles]")
    ax.set_ylabel("Coverage [%]")
    ax.set_ylim(0, 105)
    ax.set_xlim(0, 5)

    # get rid of right axis border
    ax.spines['right'].set_visible(False)
    ax.spines['top'].set_visible(False)
    ax.spines['left'].set_visible(True)
    ax.spines['bottom'].set_visible(True)
    
    # set manually the xticks
    ax.set_xticks([1, 2, 3, 4, 5])
    ax.set_yticks([0, 20, 40, 60, 80, 100])
    ax.labels = ["Qt1", "Qt2", "Qt3", "Qt4", "Qt5"]
    ax.set_xticklabels(ax.labels)

    # draw one vertical dashed vertical line per quintile 1 and 4
    CLR_AXVLN = "gray"
    ax.axvline(x=1, color=CLR_AXVLN, linewidth=1)
    ax.axvline(x=4, color=CLR_AXVLN, linewidth=1)

    # add text to tick 0.5 and y=10
    H_Y_TEXT = 2.5
    ax.text(0.5, H_Y_TEXT, "S(0%-20%)", fontsize=10, horizontalalignment='center')
    ax.text(2.5, H_Y_TEXT, "B(20%-80%)", fontsize=10, horizontalalignment='center')
    ax.text(4.5, H_Y_TEXT, "E(80%-100%)", fontsize=10, horizontalalignment='center')

    # line points style
    #                0    1    2    3    4    5    6    7    8    9   10   11   12   14   15   16   17   18   19   20
    ln_pts_style = ['o', 'v', 's', 'P', 'X', 'D', 'p', 'h', 'H', '8', '*', 'd', 'x', '1', '2', '3', '4', '+', '|', '_']
    keys = list(metrics.TOOLS.keys())
    ln_pt_dict = {
        keys[0] : ln_pts_style[6],
        keys[1] : ln_pts_style[0],
        keys[2] : ln_pts_style[5],
        keys[3] : ln_pts_style[19],
        keys[4] : ln_pts_style[17],
        keys[5] : ln_pts_style[12],
        keys[6] : ln_pts_style[1],
        keys[7] : ln_pts_style[14],
        keys[8] : ln_pts_style[2],
        keys[9] : ln_pts_style[10],
    }

    x_values = np.array([0.5, 1.5, 2.5, 3.5, 4.5])

    counter : int = 0
    for tool_id, res in TOOLS_clean.items():
        y_values = res[1].mean_coverage_perc_quintiles
        ax.plot(x_values, y_values, label=tool_id, color="black", marker=ln_pt_dict[tool_id], markersize=5, linewidth=1, linestyle='--')
        counter += 1

    # draw in green a plot of the mean coverage among all the tools for each quintile
    mean_coverage_perc_quintiles = np.array([TOOLS_clean[key][1].mean_coverage_perc_quintiles for key in TOOLS_clean.keys()])
    mean_coverage_perc_quintiles = np.mean(mean_coverage_perc_quintiles, axis=0)
    ax.plot(x_values, mean_coverage_perc_quintiles, label="mean", color=CYBERGREEN, marker='x', markersize=5, linewidth=2, linestyle='-')
    
    # add a legend and show
    plt.legend(loc='upper right', bbox_to_anchor=(1.15, 1), ncol=1, 
               fontsize=8, frameon=False)
    fig.tight_layout()

    fig.savefig(graph_dir, dpi=300)

    plt.close()



######################################
### reconstructed model evaluation ###
######################################

def draw_model_comp_result_o3d(point_cloud, save_path, to_show=False):
    """ Draw the reconstructed 3D model on open3d and export it as png.
        it's faster for preview, but it has some issues with the camera position
        and exporting to png. """
    
    """ Draw the reconstructed 3D model on open3d and export it as png """

    # bbox = o3d.geometry.OrientedBoundingBox.create_from_points(objs[0].points)
    # o3d.visualization.draw_geometries(objs,
    #                                   zoom=0.4459,
    #                                   front=[0.9288, -0.2951, -0.2242],
    #                                   lookat=bbox.get_center(),
    #                                   up=[-0.3402, -0.9189, -0.1996])

    save_path = os.path.join(save_path, f"{id}_3d_error_map_o3d.png")
    vis = o3d.visualization.Visualizer()
    vis.create_window(visible=True)
    view_ctr = vis.get_view_control()
    view_ctr.set_lookat([0, 0, 0])
    view_ctr.camera_local_translate(0.5, -0.5, 0.25)
    # view_ctr.change_field_of_view(step=50)
    # view_ctr.set_front([0, 0, -1])
    # view_ctr.set_up([0.5, -0.5, 0.25])

    vis.add_geometry(point_cloud)
    vis.update_geometry(point_cloud)
    vis.poll_events()
    vis.update_renderer()
    vis.capture_screen_image(save_path)
    vis.destroy_window()

def draw_model_comp_result(id, point_cloud, save_path, to_show=False):
    """ Draw the reconstructed 3D model on matplotlib and export to png """

    pts = np.asarray(point_cloud.points)
    colors = np.asarray(point_cloud.colors)

    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(projection='3d')
    ax.set_aspect("equal")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], color=colors, marker='.', s=0.05)
    ax.set_xlim(-1.2, 1.2)
    ax.set_ylim(-1.2, 1.2)
    ax.set_zlim(-1.2, 1.2)
    ax.xaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.yaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.zaxis.set_pane_color((1.0, 1.0, 1.0, 0.0))
    ax.set_facecolor((1.0, 1.0, 1.0, 0.0))
    ax.grid(False)
    ax.axis('off')
    ax.view_init(elev=15, azim=315)

    save_path = os.path.join(save_path, f"{id}_3d_error_map.png")
    plt.savefig(save_path, bbox_inches='tight', dpi=300)
    
    # center crop image
    img = cv2.imread(save_path)
    img = img[750:1101, 600:1401]
    cv2.imwrite(save_path, img)

    if to_show:
        plt.show()
    plt.close()

def draw_model_metric_histogram(data, bins, id, save_path, range = None, to_show = False):
    """ Draw the histogram of the metric """

    fig = plt.figure(figsize=(8, 8))
    ax = plt.axes()
    plt.ylabel("Point Amount")
    values, base, _ = plt.hist(data, bins=bins, alpha=0.2, color=CYBERGREEN, range=range, label= "Histogram")
    ax_bis = ax.twinx()
    values = np.append(values,0)
    ax_bis.plot(base, np.cumsum(values)/np.cumsum(values)[-1], color='darkcyan', marker='.', linestyle='-', markersize = 1, label = "Cumulative Histogram" )
    plt.ylabel("Proportion")
    ax.set_xlabel("Distance (m)")
    ax.legend()
    ax_bis.legend()
    save_path = os.path.join(save_path, f"{id}_histogram.png")
    plt.savefig(save_path, bbox_inches='tight', dpi=300)

    if to_show:
        plt.show()
    plt.close()

    return

def draw_combined_model_metric_histogram(batch_data, bins, title, save_path, x_range = None, to_show = False):
    init_stat_disc = ["0 joinery", "1 joinery", "2 joinery", "3 joinery", "4 joinery"]
    
    fig = plt.figure(figsize=(8, 8))

    ax = plt.axes()

    all_values = []
    ax.set_ylabel("Point Amount")
    for i, data in enumerate(batch_data):
        data = np.asarray(data)
        values, base, _ = ax.hist(data, bins=bins, alpha=0.2, color=CYBERGREEN, range=x_range)
        values = np.append(values,0)
        all_values.append(values)
    
    ax_twinx = ax.twinx()
    for i, values in enumerate(all_values):
        max_propotion = (np.asarray(batch_data[i]) <= x_range[1]).sum() / len(batch_data[i])
        cum_max_value = np.cumsum(values)[-1]
        ax_twinx.plot(base, (np.cumsum(values)/cum_max_value) * max_propotion, alpha=0.2 + 0.15*i, color="purple", marker='.', linestyle='-', markersize = 1, label = init_stat_disc[i])
        

    ax_twinx.set_ylabel("Proportion")
    ax.set_xlabel("Distance (m)")

    hist_handles = matplotlib.patches.Rectangle((0,0),1,1,color=CYBERGREEN, alpha=0.2)
    hist_labels = "Distance Distribution"
    line, label = ax_twinx.get_legend_handles_labels()
    
    line.append(hist_handles)
    label.append(hist_labels)

    ax.legend(line, label, loc='center right')

    plt.title(title)
    filename = title.replace(",","").replace(" ","_")
    save_path = os.path.join(save_path, f"{filename}_histogram.png")
    plt.savefig(save_path, bbox_inches='tight', dpi=300)

    if to_show:
        plt.show()
    plt.close()

    return
