import enum
from re import M
from tkinter.messagebox import NO
import cv2
from scipy.fft import dst
import yaml
import numpy as np

exported_tag_yml = [
    '/home/tpp/UCOSlam-IBOIS/build/utils/sticker_6_1_tag.yml',   
    '/home/tpp/UCOSlam-IBOIS/build/utils/sticker_6_2_tag.yml'   
]

"""
https://carlonicolini.github.io/sections/tech/2016/10/06/A-code-for-absolute-orientation-problem-with-Umeyama-algorithm-in-Python.html
"""

marker_ids = [[], []]
marker_corners = [{}, {}]

for map_id, filename in enumerate(exported_tag_yml):
    with open(filename, "r") as stream:
        stream = stream.read().replace('d:', 'd: ').replace("%YAML:1.0\n---","")
        map_data = yaml.safe_load(stream)

        for marker in map_data["aruco_bc_markers"]:
            marker_ids[map_id].append(marker["id"])
            marker_corners[map_id][marker["id"]] = np.array(marker["corners"])

ref_ids = list(set(marker_ids[0]).intersection(marker_ids[1]))
print_id = ref_ids[0]
print("Reference markers:", ref_ids)

# src_points = []
# dst_points = []

src_points = None
dst_points = None

for marker_id in marker_ids[0]:
    if marker_id in marker_ids[1]:
        # src_points.append(marker_corners[0][marker_id])
        # dst_points.append(marker_corners[1][marker_id])

        if src_points is None:
            src_points = marker_corners[0][marker_id].flatten()
            dst_points = marker_corners[1][marker_id].flatten()
        else:
            src_points = np.concatenate([src_points, marker_corners[0][marker_id].flatten()])
            dst_points = np.concatenate([dst_points, marker_corners[1][marker_id].flatten()])
        

# src_points = np.array(src_points)
# dst_points = np.array(dst_points)

src_points = src_points.reshape(-1, 3)
dst_points = dst_points.reshape(-1, 3)

print(src_points.shape)
print(dst_points.shape)

_, affine_matrix, mask = cv2.estimateAffine3D(src_points, dst_points)

print("Estimated Affine Matrix: ")
print(affine_matrix)

t = affine_matrix[:, 3]
affine_matrix = affine_matrix[:, :3]

print("Decompose: ")
print("Translation")
print(t)
print("Transformation")
print(affine_matrix)

print("---")

print("Orinal position:")
print(marker_corners[0][print_id])

for id in marker_ids[0]:
    for i in range(4):
        marker_corners[0][id][i] = (affine_matrix @ marker_corners[0][id][i]) + t

print("Reprojected position:")
print(marker_corners[0][print_id])
print("Destination position")
print(marker_corners[1][print_id])

print("---")

###################
# Combine the Map #
###################

# marker_ids_merged = list(set().union(marker_ids[0], marker_ids[1]))
marker_corners_merged = marker_corners[0].copy()

# check if marker in map[1] is also shown in map[0]
# if it does, update with average;
# else, just append into the dictionary
for id in marker_ids[1]:
    if id in marker_ids[0]:
        marker_corners_merged[id] = 0.5 * (marker_corners[0][id] + marker_corners[1][id])
    else:
        marker_corners_merged[id] = marker_corners[1][id]


###################
# Export yml file #
###################
yaml_data = {}
yaml_data["aruco_bc_dict"] = "m.dict_info"
yaml_data["aruco_bc_nmarkers"] = len(marker_corners_merged)
yaml_data["aruco_bc_mInfoType"] = 1
yaml_data["aruco_bc_markers"] = []
for id in marker_corners_merged.keys():
    yaml_data["aruco_bc_markers"].append(
        {
            "id": id,
            "corners": marker_corners_merged[id].tolist()
        }
    )

with open("merged_map.yml", "w") as f:
    yaml.dump(yaml_data, f, default_flow_style=True)