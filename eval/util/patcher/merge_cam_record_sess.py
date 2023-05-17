#!/usr/bin/env python

#! python3
# -*- coding: utf-8 -*-

import os
import glob
import shutil
import argparse
import sys
from tqdm import tqdm

__dir__ = os.path.abspath(os.path.dirname(__file__))
sys.path.append(__dir__)


def main(_sessions : list[str],
         _output : str) -> None:
    print("\033[93m [WARN]: Copying camera_settings... \033[0m")
    shutil.copy(os.path.join(_sessions[0], "camera_settings.txt"), _output)
    fps : int = 0
    with open(os.path.join(_output, "camera_settings.txt"), "r") as cam_settings:
        line = cam_settings.readlines()[0]
        fps = int(line.split("fps:")[1].split(",")[0].strip())
    print(f"\033[92m [SUCCESS]: Copied camera_settings with fps: {fps}. \033[0m")
    print("\033[92m [SUCCESS]: Copied camera_settings. \033[0m")

    print("\033[93m [WARN]: Merging csv files... \033[0m")
    csv_path = os.path.join(_output, "time_stamps.csv")
    with open(csv_path, "w") as csv_file:
        for idx, s in tqdm(enumerate(sessions_ordered_by_time)):
            if idx == 0:
                with open(os.path.join(s, "time_stamps.csv"), "r") as s_csv:
                    csv_file.write(s_csv.read())
            else:
                with open(os.path.join(s, "time_stamps.csv"), "r") as s_csv:
                    csv_file.write(s_csv.read().split("\n", 1)[1])
    
    csv_path_reorded = os.path.join(_output, "time_stamps.csv")
    with open(csv_path, "r") as csv_file:
        csv_lines = csv_file.readlines()
    lines_len = len(csv_lines)

    frame_count : int = 1
    frame_nbr : list[str] = []
    frame_timestamp : list[float] = []
    for idx, line in enumerate(csv_lines):
        if idx == 0:
            continue
        line_frame = line.split(",")[0]
        frame_nbr.append(line_frame)
        line_tmstamp = line.split(",")[1:]
        frame_timestamp.append(float(line_tmstamp[0].strip()))

    frame_nbr_reorded = [i for i in range(1, lines_len)]

    tmstamp_cumulative : float = 0
    frame_timestamp_reordered : list[float] = []
    for idx_tmstmp, tmstmp in enumerate(frame_timestamp):
        if idx_tmstmp == 0:
            frame_timestamp_reordered.append(tmstmp)
            continue
        if tmstmp < frame_timestamp[idx_tmstmp - 1]:
            tmstamp_cumulative += frame_timestamp[idx_tmstmp - 1] - tmstmp
            frame_timestamp_reordered.append(tmstmp + tmstamp_cumulative)
        else:
            frame_timestamp_reordered.append(tmstmp + tmstamp_cumulative)

    with open(csv_path_reorded, "w") as csv_file:
        csv_file.write("frame_number,timestamp\n")
        for idx, frame in enumerate(frame_nbr_reorded):
            csv_file.write(f"{frame},{frame_timestamp_reordered[idx]}\n")
    print("\033[92m [SUCCESS]: Merged csv files. \033[0m")

    print("\033[93m [WARN]: Merging total time log... \033[0m")
    total_time : float = 0
    total_time_path = os.path.join(_output, "total_time.txt")
    with open(total_time_path, "w") as f:
        for idx, s in tqdm(enumerate(sessions_ordered_by_time)):
            with open(os.path.join(s, "total_time.txt"), "r") as s_total_time:
                line = s_total_time.readlines()[0]
                time_tmp = float(line.split("Total time: ")[1].split(" ")[0].strip())
                total_time += time_tmp
        f.write(f"Total time: {total_time} minutes\n")
    print("\033[92m [SUCCESS]: Merged total time log. \033[0m")

    print("\033[93m [WARN]: Merging videos... \033[0m")
    video_path_out = os.path.join(_output, "video.mp4")
    # in each session directory see the format of the file "video"
    format : str = ".mp4"
    for idx, s in enumerate(_sessions):
        if os.path.exists(os.path.join(s, "video.mp4")):
            format = ".mp4"
            break
        elif os.path.exists(os.path.join(s, "video.avi")):
            format = ".avi"
            break
    _videos = [os.path.join(s, f"video{format}") for s in _sessions]

    for idx, video_path in enumerate(_videos):
        with open("temp.txt", "a") as temp_file:
            temp_file.write(f"file '{video_path}'\n")
    ffmpeg_cmd_tt = f"ffmpeg -safe 0 -f concat -i temp.txt -c copy \"{video_path_out}\""
    os.system(ffmpeg_cmd_tt)
    os.remove("temp.txt")
    print("\033[92m [SUCCESS]: Merged videos. \033[0m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=("This script merge multiple recording session together (metadata + videos)" +
                                                  " from the camera recording in case there was an interruption" +
                                                  " in the recording."),
                                                  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                                  epilog="Example: python merge_cam_record_sess.py -s session1 session2 -o output_dir")
    parser.add_argument("-s", "--sessions", nargs="+", required=True,
                        help="The sessions directories to merge")
    parser.add_argument("-o", "--output", default=__dir__,
                        help="The output directory")
    args = parser.parse_args()


    print("\033[93m [WARN]: Checking for sessions' dir health... \033[0m")
    sessions : list[str] = [os.path.abspath(s) for s in args.sessions]
    for s in sessions:
        if not os.path.isdir(s):
            raise ValueError("\033[91m [ERROR]: Resolution format is not correct. \033[0m")
        print(f"[INFO]: Session: {s}")
    print("\033[92m [SUCCESS]: Valid sessions' dirs. \033[0m")
    print("\033[93m [WARN]: Reordering sessions by time stamp... \033[0m")
    sessions_ordered_by_time : list[str] = sorted(sessions, key=lambda s: s.split("_")[-3:])
    for s in sessions_ordered_by_time:
        print(f"[INFO]: Session reordered: {s}")
    print("\033[92m [SUCCESS]: Sessions reordered by time stamp. \033[0m")

    print("\033[93m [WARN]: Checking for output dir health... \033[0m")
    if not os.path.isdir(args.output):
        raise ValueError("\033[91m [ERROR]: Output dir is not correct. \033[0m")
    output : str = os.path.join(args.output, f"{sessions_ordered_by_time[0].split(os.sep)[-1]}_{sessions_ordered_by_time[-1].split(os.sep)[-1]}")
    if os.path.isdir(output):
        print("\033[91m [ERROR]: Output dir already exists. \033[0m")
        # ask the user if you want to overwrite the output dir
        is_delete = input(f"\033[93m [WARN]: Output dir already exists. Do you want to delete it? [y/n] \033[0m")
        if is_delete == "y":
            shutil.rmtree(output)
        else:
            raise ValueError("\033[91m [ERROR]: Not deleting the existing dir. \033[0m")
    os.makedirs(output)
    print(f"\033[92m [SUCCESS]: Valid output dir: {args.output}. \033[0m")

    main(_sessions=sessions_ordered_by_time,
         _output=output)