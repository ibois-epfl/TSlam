#!/bin/bash


# get args of folder directory where videos are located
__video_dir__="/home/as/TSlam/eval/util/video/test"
__output_dir__="/home/as/TSlam/eval/util/video/test/output"
mkdir -p $__output_dir__

# go through each folder 01, 02, 03,
folders=$(ls $__video_dir__)
for folder in $folders
do
    name=$(echo $folder | cut -c 1-2)
    echo $name
    vid_path=$__video_dir__/$folder/${name}_video.mp4
    vid_path_out=$__output_dir__/${name}_video_lbl_frames.mp4
    echo $vid_path

    ffmpeg -i ${vid_path} \
           -vf "drawtext=fontfile=Arial.ttf: text=%{n}: x=(w-tw)/2: y=h-(2*lh): fontcolor=white: box=1: boxcolor=0x00000099" \
           -y ${vid_path_out}
done



