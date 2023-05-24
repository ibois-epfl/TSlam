import visuals
import io_stream
import transformations as tfm

import os
import sys

import numpy as np
import matplotlib.pyplot as plt

path_gt_csv : str = "/home/as/TSlam/eval/script/test/refined_stream_01.csv"
path_ts_txt : str = "/home/as/TSlam/eval/script/test/01_4000_9000_tslam"

frame_start = 4500
frame_end = 8100

t_pos, t_ros = io_stream.process_opti_timber_data(path_gt_csv, frame_start, frame_end)
cam_pos, cam_ros = io_stream.process_opti_camera_data(path_gt_csv, frame_start, frame_end)
ts_pos, ts_ros, ts_tags, ts_coverages = io_stream.process_ts_data(path_ts_txt)


# # rotate the camera positions and rotations of 90 degrees around the x axis
# rot = tfm.rotate_matrix_from_euler(90, 0, 0)
# cam_pos = np.dot(rot, cam_pos.T).T
# cam_ros = np.dot(rot, cam_ros.T).T

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
ax.set_xlabel('X [m]')
ax.set_ylabel('Y [m]')
ax.set_zlabel('Z [m]')

# positions
# draw a straight line with as points as the elements in t_pos
# straight_ln = []
# coord = 0
# step = 0.0000001
# for idx, pos in enumerate(t_pos):
#     pt = np.array([coord, coord, coord])
#     coord += step
#     straight_ln.append(pt)
# straight_ln = np.array(straight_ln)


# ax.plot(straight_ln[:, 0], straight_ln[:, 1], straight_ln[:, 2], color='black', alpha=0.5)
# ax.plot(cam_pos[:, 0], cam_pos[:, 1], cam_pos[:, 2], color='grey', alpha=0.5)
ax.plot(ts_pos[:, 0], ts_pos[:, 1], ts_pos[:, 2], color='grey', alpha=0.5)



# visuals.__draw_local_axis_pose(ax, t_ros[0], t_ros[0],
#                                 scale_f=0.001, alpha=0.25, linewidth=2)

# visuals.__draw_local_axis_pose(ax, t_ros[100], t_ros[100],
#                                 scale_f=0.001, alpha=0.25, linewidth=2)

# # rotations
for idx, pos in enumerate(cam_pos):
    if idx % 30 == 0:
        # visuals.__draw_local_axis_pose(ax, cam_pos[idx], cam_ros[idx],
        #                         scale_f=0.02, alpha=0.5, linewidth=2)
        visuals.__draw_local_axis_pose(ax, ts_pos[idx], ts_ros[idx],
                                scale_f=0.02, alpha=0.5, linewidth=2)

plt.show()