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

    print("\033[93m [WARN]: Merging csv files... \033[0m")
    csv_path = os.path.join(_output, "refined_stream.csv")
    with open(csv_path, "w") as csv_file:
        for idx, s in tqdm(enumerate(_sessions)):
            if idx == 0:
                with open(os.path.join(s, "refined_stream.csv"), "r") as s_csv:
                    csv_file.write(s_csv.read())
            else:
                with open(os.path.join(s, "refined_stream.csv"), "r") as s_csv:
                    csv_file.write(s_csv.read().split("\n", 1)[1])

    csv_path_reorded = os.path.join(_output, "refined_stream.csv")
    with open(csv_path, "r") as csv_file:
        csv_lines = csv_file.readlines()
    lines_len = len(csv_lines)

    frame_count : int = 0
    with open(csv_path_reorded, "w") as csv_file_r:
        for i in tqdm(range(lines_len)):
            if i == 0:
                csv_file_r.write(csv_lines[i])
            else:
                line = csv_lines[i]
                line_idx = line.split(';')[0]
                line_rest = line.split(';')[1:]
                line_new_idx = str(frame_count)
                recomposed_line = line_new_idx + ";" + ";".join(line_rest)
                csv_file_r.write(recomposed_line)
                frame_count += 1
    print("\033[92m [SUCCESS]: Merged csv files. \033[0m")

    print("\033[93m [WARN]: Merging videos... \033[0m")
    video_path_out = os.path.join(_output, "video.mp4")
    _videos = [os.path.join(s, "video.mp4") for s in _sessions]
    for idx, video_path in enumerate(_videos):
        with open("temp.txt", "a") as temp_file:
            temp_file.write(f"file '{video_path}'\n")
    ffmpeg_cmd_tt = f"ffmpeg -safe 0 -f concat -i temp.txt -c copy \"{video_path_out}\""
    os.system(ffmpeg_cmd_tt)
    os.remove("temp.txt")
    print("\033[92m [SUCCESS]: Merged videos. \033[0m")

    print("\033[93m [WARN]: Copying the ffmpeg video recording log files... \033[0m")
    for s in _sessions:
        shutil.copy(os.path.join(s, "ffmpeg_capture.log"), os.path.join(_output, f"ffmpeg_capture_{os.path.basename(s)}.log"))
    print("\033[92m [SUCCESS]: Copied ffmpeg logs. \033[0m")

    print("=" * max([len(s) for s in ["Merged session: ", "Frames: ", "Video: ", "[SUCCESS]: Merging process over."]]))
    print(f"[INFO]: Merged session: {output}")
    # print(f"[INFO]: Frames: {len(os.listdir(frames_dir))}")
    print(f"[INFO]: Video: {os.path.getsize(video_path)} bytes")
    print("\033[92m [SUCCESS]: Merging process over. \033[0m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=("This script merge multiple session together (frames + csv + videos)" +
                                                  " from the optitrack recording in case there was an interruption" +
                                                  " in the recording."),
                                                  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                                  epilog="Example: python merge_sess.py -s session1 session2 -o output_dir")
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

