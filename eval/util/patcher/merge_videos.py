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

def main(_videos : list[str],
         _output : str) -> None:
    video_path_out = _output
    print("\033[93m [WARN]: Merging videos... \033[0m")

    for idx, video_path in enumerate(_videos):
        with open("temp.txt", "a") as temp_file:
            temp_file.write(f"file '{video_path}'\n")
    ffmpeg_cmd_tt = f"ffmpeg -safe 0 -f concat -i temp.txt -c copy \"{video_path_out}\""

    os.system(ffmpeg_cmd_tt)
    os.remove("temp.txt")
    print("\033[92m [SUCCESS]: Merged videos. \033[0m")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=("This script merge multiple videos" +
                                                  " in the recording."),
                                                  formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                                  epilog="Example: python merge_cam_record_sess.py -s video1 video2 -o output_dir")
    parser.add_argument("-v", "--videos", nargs="+", required=True,
                        help="The videos paths to merge")
    parser.add_argument("-o", "--output", default=__dir__,
                        help="The output directory")
    args = parser.parse_args()

    print("\033[93m [WARN]: Checking for output dir health... \033[0m")
    if os.path.isfile(args.output):
        print("\033[91m [ERROR]: Output file already exists. \033[0m")
        is_delete = input(f"\033[93m [WARN]: Output dir already exists. Do you want to delete it? [y/n] \033[0m")
        if is_delete == "y":
            os.remove(args.output)
        else:
            raise ValueError("\033[91m [ERROR]: Not deleting the existing file. \033[0m")
    print(f"\033[92m [SUCCESS]: Valid output path: {args.output}. \033[0m")

    main(_videos=args.videos,
         _output=args.output)