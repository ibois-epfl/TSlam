#!/bin/bash

"""
This file is part of TSLAM and distributed under the terms of the GPLv3.

Copyright (C) 2023 Andrea Settimi <andrea dot settimi at epfl dot ch>, Hong-Bin Yang <hong dash bin dot yang at epfl dot ch> (IBOIS, EPFL)

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Andrea Settimi and Hong-Bin Yang ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied Andrea Settimi and Hong-Bin Yang.
"""


import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime

def _parse_frames(_input_frames_path : str) -> list[tuple]:
    """
        Parse the frames file and return a list of tuples conataining:
        - first frame start
        - last frame end
        - tool used for the sequence

        Args:
            _input_frames_path (str): path to the frames file

        Returns:
            list[tuple]: list of tuples containing the frames sequence
    """
    frames : list[tuple] = []
    with open(_input_frames_path, 'r') as f:
        lines = f.readlines()
        if len(lines) == 0:
            sys.stderr("\033[91m [ERROR]: --frames file is empty\n\033[0m")
            sys.flush()
            sys.exit(1)
        lines = [line.strip() for line in lines]
        for line in lines[20:]:
            if len(line) == 0:
                continue
            spltline = line.split('-')
            frame_start : int = int(spltline[0])
            frame_end : int = int(spltline[1])
            tool : str = spltline[2]
            frames.append((frame_start, frame_end, tool))
    return frames

def _compute_poses(_frames : list[tuple],
                   _input_monoexe_path : str,
                   _input_calib_path : str,
                   _input_map_path : str,
                   _input_vid_path : str,
                   _out_dir : str,
                   _is_only_tag : bool,
                   _is_video_export : bool) -> None:
    """
        Compute the poses for the video sequences and frames start and end with tslam_monocular

        Args:
            _frames (list[tuple]): list of tuples containing the frames sequence
            _input_monoexe_path (str): path to the tslam_monocular executable
            _input_calib_path (str): path to the calibration file
            _input_map_path (str): path to the map file
            _input_vid_path (str): path to the video file
            _out_dir (str): path to the output directory
    """
    tee_path : str = f"{_out_dir}/running_log_{_frames[0][0]}_{_frames[-1][1]}.log"
    timestamp_start : str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    os.system(f"echo -----start process time: {timestamp_start}----- | tee -a {tee_path}")
    os.system(f"git show -s | tee -a {tee_path}")

    for f in tqdm(_frames):
        out_pose_name_base : str = f"{f[0]}_{f[1]}_{f[2]}"

        out_name_mod : str = "onlytag" if _is_only_tag else "keypoints"
        param_no_keypoints : str = "-noKeyPoints" if _is_only_tag else ""
        param_video_record : str = f"-exportVideo {_out_dir}/{out_pose_name_base}" if _is_video_export else ""

        out_pose_name : str = f"{out_pose_name_base}-{out_name_mod}.txt"
        cmd : str = f"{_input_monoexe_path} " \
                                f"{_input_vid_path} " \
                                f"{_input_calib_path} " \
                                f"-map {_input_map_path} " \
                                f"-startFrameID {f[0]} " \
                                f"-endFrameID {f[1]} " \
                                f"-localizeOnly " \
                                f"-noX " \
                                f"{param_no_keypoints} " \
                                f"-outCamPose {_out_dir}/{out_pose_name} " \
                                f"{param_video_record} " \
                                f"-f -s" \
                                f"| tee -a {tee_path}"
        os.system(f"echo -----command only_tags: {cmd}----- | tee -a {tee_path}")
        os.system(f"echo -----start recording only_tags {out_pose_name_base}----- | tee -a {tee_path}")
        os.system(cmd)

    timestamp_end : str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    time_process : str = datetime.strptime(timestamp_end, "%d/%m/%Y %H:%M:%S") - datetime.strptime(timestamp_start, "%d/%m/%Y %H:%M:%S")
    os.system(f"echo -----end process time: {timestamp_end}----- | tee -a {tee_path}")
    os.system(f"echo -----TOTAL TIME: {time_process}----- | tee -a {tee_path}")

def main(input_frames_path : str,
         input_monoexe_path : str,
         input_calib_path : str,
         input_map_path : str,
         input_vid_path : str,
         out_dir : str,
         export_video : bool,
         is_only_tags : bool) -> None:

    print("[INFO]: Parsing the notated frames sequences ..")
    frames : list[tuple] = _parse_frames(input_frames_path)
    print("\033[92m[INFO]: Parsed frames file\033[0m")

    print("[INFO]: Running tslam_monocular for video sequences..")
    _compute_poses(frames,
                   input_monoexe_path,
                   input_calib_path,
                   input_map_path,
                   input_vid_path,
                   out_dir,
                   is_only_tags,
                   export_video)
    print("\n\033[92m[INFO]: Tslam output poses out\033[0m")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Parsing the manual noted frames sequences.',
                                     formatter_class=argparse.ArgumentDefaultsHelpFormatter,
                                     conflict_handler='resolve')
    parser.add_argument('--name', type=str, default="noname", help='name of the processed video of the dataset')
    parser.add_argument('--frames', type=str, help='frames .txt file')
    parser.add_argument('--monoexe', type=str, help='tslam_monocular executable path')
    parser.add_argument('--calib', type=str, help='camera calibration file in .yml format')
    parser.add_argument('--map', type=str, help='tslam map file of the piece')
    parser.add_argument('--video', type=str, help='frames video file')
    parser.add_argument('--output', type=str, help='output folder where to save the poses')
    parser.add_argument('--onlyTags', action='store_true', help='run the tslam_monocular only with tags and no keypoints')
    parser.add_argument('--exportVideo', action='store_true', help='export the video raw and processed')
    args = parser.parse_args()

    if not args.frames.endswith('.txt'):
        print("\033[91m[ERROR]: --frames file must be a .txt file\n\033[0m")
        sys.exit(1)
    if not os.path.exists(args.monoexe):
        print("\033[93m[WARN]: --monoexe file does not exist\n\033[0m")
        sys.exit(1)
    if not args.calib.endswith('.yml'):
        print("\033[91m[ERROR]: --calib file must be a .yml file\n\033[0m")
        sys.exit(1)
    if not os.path.exists(args.map) and not args.map.endswith('.map'):
        print("\033[91m[ERROR]: --map file does not exist\n\033[0m")
        sys.exit(1)
    if not args.video.endswith('.mp4') and not args.video.endswith('.avi'):
        print("\033[91m[ERROR]: --video file must be a .mp4, .avi file\n\033[0m")
        sys.exit(1)
    if not os.path.exists(args.output):
        print("\033[93m[WARNING]: --output folder does not exist, creating one\n\033[0m")
        os.makedirs(args.output)
    output_dir : str = f"{args.output}/{args.name}"
    # check if the subfolder exists, if it is so erase it
    if os.path.exists(output_dir):
        print("\033[93m[WARNING]: --output folder already exists, erasing it\n\033[0m")
        shutil.rmtree(output_dir)
    os.makedirs(output_dir)

    os.system(f"mkdir {output_dir}")

    main(input_frames_path=args.frames,
         input_monoexe_path=args.monoexe,
         input_calib_path=args.calib,
         input_map_path=args.map,
         input_vid_path=args.video,
         out_dir=output_dir,
         export_video=args.exportVideo,
         is_only_tags=args.onlyTags)
    sys.exit(0)