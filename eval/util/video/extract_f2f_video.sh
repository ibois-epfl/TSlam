#!/bin/bash

# write a script bash that extracts a video sequence given:
# - the video path
# - the start frame
# - the end frame

# example:
# ./extract_f2f_video.sh /path/to/video.mp4 100 200

# set 3 variables for the 3 arguments
video=$1
start_frame=$2
end_frame=$3

# extract the video sequence using ffmpeg
ffmpeg -i $video -vf "select=between(n\,$start_frame\,$end_frame)" -vsync 0 -an -y -q:v 2 temp.mp4

# rename the file to the original video name with the start and end frame
mv temp.mp4 $video.$start_frame.$end_frame.mp4