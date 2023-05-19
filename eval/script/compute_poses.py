#!/bin/bash

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
                   _out_dir : str) -> None:
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
    tee_path : str = f"{_out_dir}/running_log_{_frames[0][0]}_{_frames[-1][1]}.txt"
    timestamp_start : str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    os.system(f"echo -----start process time: {timestamp_start}----- | tee -a {tee_path}")
    os.system(f"git show -s | tee -a {tee_path}")

    for f in tqdm(_frames):
        out_pose_name_base : str = f"{f[0]}_{f[1]}"
        out_pose_name_keypoints : str = f"{out_pose_name_base}_keypoints.txt"
        out_pose_name_onlytag : str = f"{out_pose_name_base}_onlytag.txt"
        cmd_featurept : str = f"{_input_monoexe_path} " \
                                f"{_input_vid_path} " \
                                f"{_input_calib_path} " \
                                f"-map {_input_map_path} " \
                                f"-startFrameID {f[0]} " \
                                f"-endFrameID {f[1]} " \
                                f"-localizeOnly " \
                                f"-noX " \
                                f"-outCamPose {_out_dir}/{out_pose_name_keypoints} " \
                                f"| tee -a {tee_path}"
        cmd_only_tags : str = f"{_input_monoexe_path} " \
                                f"{_input_vid_path} " \
                                f"{_input_calib_path} " \
                                f"-map {_input_map_path} " \
                                f"-startFrameID {f[0]} " \
                                f"-endFrameID {f[1]} " \
                                f"-localizeOnly " \
                                f"-noX " \
                                f"-nokeypoints " \
                                f"-outCamPose {_out_dir}/{out_pose_name_onlytag} " \
                                f"| tee -a {tee_path}"
        os.system(f"echo -----start recording no_keypoints {out_pose_name_base}----- | tee -a {tee_path}")
        os.system(cmd_featurept)
        os.system(f"echo -----start recording only_tags {out_pose_name_base}----- | tee -a {tee_path}")
        os.system(cmd_only_tags)

    timestamp_end : str = datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    time_process : str = datetime.strptime(timestamp_end, "%d/%m/%Y %H:%M:%S") - datetime.strptime(timestamp_start, "%d/%m/%Y %H:%M:%S")
    os.system(f"echo -----end process time: {timestamp_end}----- | tee -a {tee_path}")
    os.system(f"echo -----TOTAL TIME: {time_process}----- | tee -a {tee_path}")


def main(input_frames_path : str,
         input_monoexe_path : str,
         input_calib_path : str,
         input_map_path : str,
         input_vid_path : str,
         out_dir : str) -> None:

    print("[INFO]: Parsing the notated frames sequences ..")
    frames : list[tuple] = _parse_frames(input_frames_path)
    print("\033[92m[INFO]: Parsed frames file\033[0m")

    print("[INFO]: Running tslam_monocular for video sequences..")
    _compute_poses(frames,
                   input_monoexe_path,
                   input_calib_path,
                   input_map_path,
                   input_vid_path,
                   out_dir)
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
    args = parser.parse_args()

    if not args.frames.endswith('.txt'):
        print("\033[91m[ERROR]: --frames file must be a .txt file\n\033[0m")
        sys.exit(1)
    if not os.path.exists(args.monoexe):
        # print in yellow
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
    output_dir : str = f"{args.output}/{args.name}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    if not os.path.exists(output_dir):
        print("\033[93m[WARNING]: --output folder does not exist, creating one\n\033[0m")
    os.system(f"mkdir {output_dir}")

    
    main(input_frames_path=args.frames,
         input_monoexe_path=args.monoexe,
         input_calib_path=args.calib,
         input_map_path=args.map,
         input_vid_path=args.video,
         out_dir=output_dir)
    sys.exit(0)