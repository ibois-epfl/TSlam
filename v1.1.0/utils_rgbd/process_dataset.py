#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import os

num_args=len(sys.argv)

if num_args<4:
    print("Args: <dataset_dir> <ref_marker_num> <marker_size> [--tracking-calib <calib_path>] [--reconstruction-calib <calib_path>] [--neighbours-only] [--all-clouds] [--point2plane] [--orig] [--nokeypoints]")
    sys.exit(-1)

dir = sys.argv[1]+"/"
dir_files = os.listdir(dir)

ref_marker_num = sys.argv[2]
marker_size = sys.argv[3]

all_clouds_str=""
point2plane_str=""
tracking_calib_str=""
reconstruction_calib_str=""
orig_str=""
neighbours_only_str=""
nokeypoints_str=""
monocular=False

for i in range(4,num_args):
    option=sys.argv[i]
    if option=="--all-clouds":
        all_clouds_str=option
    elif option=="--point2plane":
        point2plane_str=option
    elif option=="--orig":
        orig_str=option
    elif option=="--neighbours-only":
        neighbours_only_str=option
    elif option=="--tracking-calib" and i+1<num_args:
        tracking_calib_str=sys.argv[i+1]
    elif option=="--reconstruction-calib" and i+1<num_args:
        reconstruction_calib_str=sys.argv[i+1]
    elif option=="--nokeypoints":
        nokeypoints_str="-nokeypoints"
    elif option=="--monocular":
        monocular=True

subdirs = []
for file_name in dir_files:
    if os.path.isdir(dir+file_name):
        subdirs.append(dir+file_name+"/")

subdirs.sort()

print_commands = False
slam_tracking = False
rigid_registration = True
cloud_matching = True
transform_finding = False

#ucoslam tracking
if slam_tracking:
    for subdir in subdirs:
        command = "./rgbd_slam "+subdir+"cut.oni oni "+" -out "+subdir+"cut "+"-size "+marker_size+" --calib "+tracking_calib_str+" "+nokeypoints_str
        if monocular:
            command = "../utils/ucoslam_monocular "+subdir+"cut.avi "+tracking_calib_str+" -out "+subdir+"cut "+"-aruco-markerSize "+marker_size+" "+nokeypoints_str+" -KFMinConfidence 0.1"
        if print_commands:
            print command
        else:
            os.system(command)

#scene reconstruction
if rigid_registration:
    for subdir in subdirs:
        command = "./rigid_registration "+subdir+"cut.map "+subdir+"cut.oni oni "+ref_marker_num+" "+marker_size+" --calib "+reconstruction_calib_str+" "+all_clouds_str+" "+point2plane_str+" "+orig_str+" "+neighbours_only_str
        if print_commands:
            print command
        else:
            os.system(command)

#matching the scenes
if cloud_matching:
    for subdir in subdirs:
        if(subdir == subdirs[0]):
            command = "./cloud_matcher "+subdir+"cut_results "+ref_marker_num+" "+marker_size
        else:
            command = "./cloud_matcher "+subdirs[0]+"cut_results "+ref_marker_num+" "+marker_size+" "+subdir+"cut_results"
        if print_commands:
            print command
        else:
            os.system(command)

#calculating transformations
if transform_finding:
    for subdir in subdirs:
        if(subdir != subdirs[0]):
            command = "./transform_finder "+subdirs[0]+"cut_results "+subdir+"cut_results"+" --with-icp"
            if print_commands:
                print command
            else:
                os.system(command)


