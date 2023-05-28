import sys
import os
import shutil
from tqdm import tqdm
import csv

import argparse

import numpy as np



def refine_poses(csv_path : str,
                 img_log_path : str,
                 out_path : str
                 ) -> None:
    #######################################################################
    # get all the timestamps
    #######################################################################

    start_opti_world_timestamp : float = 0.0
    end_opti_world_timestamp : float = 0.0
    start_frame_world_timestamp : float = 0.0
    end_frame_world_timestamp : float = 0.0

    with open(csv_path, "r") as f:
      lines = f.readlines()
      start_opti_world_timestamp = float(lines[1].split(";")[1])
      end_opti_world_timestamp = float(lines[-1].split(";")[1])
      print(f"start_world_timestamp: {start_opti_world_timestamp}")
      print(f"end_opti_world_timestamp: {end_opti_world_timestamp}")

    with open(img_log_path, "r") as f:
      lines = f.readlines()
      for l in lines:
          if ">>>>>>>>>>>>>>>>>>>>> World START time:" in l:
              start_frame_world_timestamp = float(l.split(":")[1])
              print(f"start_frame_world_timestamp: {start_frame_world_timestamp}")
          if ">>>>>>>>>>>>>>>>>>>>> World END time:" in l:
              end_frame_world_timestamp = float(l.split(":")[1])
              print(f"end_frame_world_timestamp: {end_frame_world_timestamp}")

    assert start_opti_world_timestamp < start_frame_world_timestamp, f"[ERROR]: the start timestamp of the optitrack is greater than the start timestamp of the frames"
    assert end_opti_world_timestamp > end_frame_world_timestamp, f"[ERROR]: the end timestamp of the optitrack is smaller than the end timestamp of the frames"

    #######################################################################
    # parse the ffmpeg log to get the timestamps and batches of frames
    #######################################################################

    frames_cap_timestamps : list[float] = []
    frame_single_timestamps : list[float] = []
    frames_batches_size : list[int] = []
    batch_size : int = 0
    total_frame_size : int = 0

    batch_size_temp_prev : int = 0

    with open(img_log_path, "r") as f:
      lines = f.readlines()

      for line in lines:
        if "frame=" in line:
          tstamp : str = line.split("time=")[1][:11]
          h = int(tstamp[:2])
          m = int(tstamp[3:5])
          s = int(tstamp[6:8])
          ds = int(tstamp[9:11])
          tstamp_sd = (h * 3600 + m * 60 + s) + ds / 100.0
          frames_cap_timestamps.append(tstamp_sd)

          batch_size_temp = int(line.split("fps=")[0].split("=")[1])
          batch_size = batch_size_temp - batch_size_temp_prev
          frames_batches_size.append(batch_size)
          batch_size_temp_prev = batch_size_temp

      for i in range(len(frames_batches_size)):
        total_frame_size += frames_batches_size[i]

    #######################################################################
    # compute timestamps for each frame
    #######################################################################

    current_frame_world_timestamp : float = start_frame_world_timestamp

    for idx, batch_size in enumerate(frames_batches_size):
      current_frame_cap_timestamp = frames_cap_timestamps[idx]
      timestamp_diff = current_frame_cap_timestamp - current_frame_world_timestamp
      for i in range(batch_size):
        frame_single_timestamps.append(current_frame_world_timestamp)
        current_frame_world_timestamp += timestamp_diff / batch_size
        # ADD THE START GLOBAL TIMESTAMP TO THE FRAME TIMESTAMP
    
    # TODO: test me
    # to each frame timestamp, add the start global timestamp
    frame_single_timestamps = [t + start_frame_world_timestamp for t in frame_single_timestamps]

    # TODO: test to solve the error frame redistribution considering that the video was at 30fps perfectly
    time_lapse : float = end_frame_world_timestamp - start_frame_world_timestamp
    single_timestamp = time_lapse / total_frame_size
    frame_single_timestamps = [start_frame_world_timestamp + single_timestamp * i for i in range(total_frame_size)]

    #######################################################################
    # copy the raw csv file by changing its name in mod_csv
    #######################################################################

    mod_csv_path : str = out_path + "/refined_stream.csv"
    if os.path.exists(mod_csv_path):
      os.remove(mod_csv_path)
    shutil.copyfile(csv_path, mod_csv_path)

    #######################################################################
    # for each timestamp in the list, find the closest pose in the csv file and delete the others
    #######################################################################

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> V1
    gt_total_frame_size : int = 0
    with open(csv_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        for idx, data in enumerate(reader):
            gt_total_frame_size += 1

    with open(csv_path, 'r') as f:
        reader = csv.reader(f, delimiter=';')
        line_count = 0
        refined_line_count = 1
        with open(mod_csv_path, 'w') as f2:
            writer = csv.writer(f2, delimiter=';')
            for idx, data in tqdm(enumerate(reader), total=gt_total_frame_size, desc="Processing"):
                if line_count == 0:
                    writer.writerow(data)
                    line_count += 1
                    continue
                
                raw_csv_tstamp : float = float(data[1])
                
                # find the closest timestamp to the frame timestamp
                if raw_csv_tstamp >= start_frame_world_timestamp and raw_csv_tstamp <= end_frame_world_timestamp:
                    for idx, t_frame in enumerate(frame_single_timestamps):
                        if raw_csv_tstamp >= t_frame:
                            frame_single_timestamps.pop(idx)
                            data[0] = str(refined_line_count)
                            writer.writerow(data)
                            refined_line_count += 1
                            break
                
                line_count += 1

    # >>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>> V2



    assert refined_line_count == total_frame_size, f"[ERROR]: the number of refined timestamps are not the same as frames: gt: {refined_line_count} != {total_frame_size}"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Refine the optitrack poses.')
    parser.add_argument('--raw', type=str, default=None, help='path to the csv file')
    parser.add_argument('--log', type=str, default=None, help='path to the ffmpeg log file')
    parser.add_argument('--out', type=str, default=None, help='path to the refined output csv file')
    args = parser.parse_args()

    # FIXME: debug
    # if the out directory exists, delete it and create a new one
    if os.path.exists(args.out):
      shutil.rmtree(args.out)
    os.makedirs(args.out)

    refine_poses(csv_path=args.raw,
                 img_log_path=args.log,
                 out_path=args.out)