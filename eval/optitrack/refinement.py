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
      print(f"start_opti_world_timestamp: {start_opti_world_timestamp}")
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
    frames_single_timestamps : list[float] = []
    frames_batches_size : list[int] = []
    batch_size : int = 0
    total_frame_size : int = 0

    prev_batch_start_frame_id : int = 0

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

          # frame id
          batch_start_frame_id = int(line.split("fps=")[0].split("=")[1])
          batch_size = batch_start_frame_id - prev_batch_start_frame_id
          frames_batches_size.append(batch_size)
          prev_batch_start_frame_id = batch_start_frame_id

      total_frame_size = prev_batch_start_frame_id

    #######################################################################
    # compute timestamps for each frame
    #######################################################################

    for idx in range(1, len(frames_cap_timestamps)):
      batch_start_ts = frames_cap_timestamps[idx - 1]
      batch_end_ts = frames_cap_timestamps[idx]
      batch_size = frames_batches_size[idx]
  
      frame_ts_diff = (batch_end_ts - batch_start_ts) / batch_size
      current_ts = batch_start_ts
      for _ in range(batch_size):
        frames_single_timestamps.append(current_ts)
        current_ts += frame_ts_diff

    # add the final frame
    frames_single_timestamps.append(frames_cap_timestamps[-1])
    frames_single_timestamps = [t + start_frame_world_timestamp for t in frames_single_timestamps]

    #######################################################################
    # for each timestamp in the list, find the closest pose in the csv file and delete the others
    #######################################################################

    frame_id = 0
    refined_line_count = 0

    out_path = os.path.join(out_path, "refined_stream.csv")

    with open(out_path, 'w') as wf:
      writer = csv.writer(wf, delimiter=';')
    
      with open(csv_path, 'r') as rf:
        reader = csv.reader(rf, delimiter=';')
        best_dif = 1e9
        current_ts = frames_single_timestamps[frame_id]

        reader = list(reader)
        idx = 0
        while idx < len(reader):
          data = reader[idx]
          if idx == 0:
            data.insert(1, "frame_timestamp")
            data.insert(2, "opti_frame_id")
            writer.writerow(data)
            idx += 1
            continue
          raw_csv_tstamp : float = float(data[1])
          dif = abs(raw_csv_tstamp - current_ts)
          
          if dif <= best_dif:
            best_dif = dif
            best_data = data.copy()
            idx += 1
          else:
            # when dif > best_dif, meaning that it starts to go away, write the best data in the refined csv
            opti_frame_id = int(best_data[0])
            best_data.insert(1, current_ts)
            best_data.insert(2, opti_frame_id)

            best_data[0] = frame_id + 1
            writer.writerow(best_data)
            refined_line_count += 1

            # reset the best dif and best data
            frame_id += 1
            if frame_id == len(frames_single_timestamps):
              break

            current_ts = frames_single_timestamps[frame_id]
            best_dif = 1e9
            if idx <= 11:
              idx = 1
            else:
              idx -= 10

    assert refined_line_count == total_frame_size, f"[ERROR]: the number of refined timestamps are not the same as frames: gt: {refined_line_count} != {total_frame_size}"


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Refine the optitrack poses.')
    parser.add_argument('--raw', type=str, default=None, help='path to the csv file')
    parser.add_argument('--log', type=str, default=None, help='path to the ffmpeg log file')
    parser.add_argument('--out', type=str, default=None, help='path to the refined output csv file')
    args = parser.parse_args()

    # FIXME: debug
    # if the out directory exists, delete it and create a new one
    # if os.path.exists(args.out):
    #   shutil.rmtree(args.out)
    # os.makedirs(args.out)

    refine_poses(csv_path=args.raw,
                 img_log_path=args.log,
                 out_path=args.out)