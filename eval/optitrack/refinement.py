import sys
import os
import shutil
from tqdm import tqdm

import argparse

import numpy as np


def find_closest_value(source_list, target_list):
    """
    Find the closest value in source_list for each value in target_list.

    :param source_list: list of values to search for (the ffmpeg timestamps)
    :param target_list: list of values to search from (the optitrack timestamps)

    :return: list of closest values in source_list for each value in target_list
    """

    closest_values = []
    closest_pool = []

    tol_pool : int = 2  # tolerance in seconds for searching the closest value pool ori: 1
    for s in tqdm(source_list):
        # get first the seconds same group
        s_seconds = int(str(s).split(".")[0])  # without ms
        for t in target_list:
            t_seconds = int(str(t).split(".")[0])
            t_seconds_up = t_seconds + tol_pool
            t_seconds_down = t_seconds - tol_pool

            if t_seconds_down <= s_seconds <= t_seconds_up:
                closest_pool.append(t)
            if t_seconds > s_seconds:
                break
        
        # get all the diffs between the source and the closest values
        clst_c = 0
        diff_pool : list[float] = []
        for c in closest_pool:
          diff = abs(c - s)
          diff_pool.append(diff)

        if len(diff_pool) > 0:
          min_diff = min(diff_pool)
          clst_c = closest_pool[diff_pool.index(min_diff)]

          if clst_c not in closest_values:
            if len(closest_values) == 0:
              closest_values.append(clst_c)
            if clst_c > closest_values[-1]:
              closest_values.append(clst_c)
        else:
          with open("./debug.txt", "w") as f:
            f.write(f"s: {s} \n \n")
            f.write(f"closest_pool: {closest_pool} \n \n")
            f.write(f"diff_pool: {diff_pool}")
            f.write(f"min_diff: {min_diff} \n \n")
            f.write(f"clst_c: {clst_c} \n \n")
            f.write(f"closest_values: {closest_values} \n \n")
          print(f"[ERROR]: The diff list is empty. It means that the closest value is not found for the given tolerance of +- {tol_pool} seconds. \n")
          sys.exit(0)

        # clean out memory for the temp pool
        closest_pool.clear()

        # remove all the values in target_list smaller than the last value in closest_values
        if len(closest_values) > 0:
            target_list = [t for t in target_list if t > closest_values[-1]]
    
    return closest_values

def refine_poses(csv_path : str,
                 img_path : str,
                 img_log_path : str) -> None:
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

    img_path_size : int = len(os.listdir(img_path))
    print(f"img_path_size: {img_path_size}")

    frames_cap_timestamps : list[float] = []
    frame_single_timestamps : list[float] = []
    frames_batches_size : list[int] = []
    batch_size : int = 0

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

      total_batch_sum = 0
      for i in range(len(frames_batches_size)):
        total_batch_sum += frames_batches_size[i]
      assert img_path_size == total_batch_sum , f"[ERROR]: the number of frames and the number of batches are not the same: gt: {img_path_size} != {total_batch_sum}"

    #######################################################################
    # compute timestamps for each frame
    #######################################################################

    current_frame_world_timestamp : float = start_frame_world_timestamp
    timestamp_diff : float = 0.0

    for idx, batch_size in enumerate(frames_batches_size):

      current_frame_cap_timestamp = frames_cap_timestamps[idx]

      if idx == 0:
        current_frame_cap_timestamp = 1/30
        prev_frame_cap_timestamp = 0.0
      else:
        prev_frame_cap_timestamp = frames_cap_timestamps[idx - 1]
      timestamp_diff = current_frame_cap_timestamp - prev_frame_cap_timestamp

      for i in range(batch_size):
        frame_single_timestamps.append(current_frame_world_timestamp)
        current_frame_world_timestamp += timestamp_diff / batch_size
    
    assert(len(frame_single_timestamps) == img_path_size), f"[ERROR]: the number of frames and the number of timestamps are not the same: gt: {img_path_size} != {len(frame_single_timestamps)}"
    assert(len(frame_single_timestamps) == len(set(frame_single_timestamps))), f"[ERROR]: there are duplicates in the list of corresponding timestamps"
    for i in range(len(frame_single_timestamps) - 1):
          assert(frame_single_timestamps[i] < frame_single_timestamps[i + 1]), f"[ERROR]: the frame timestamps are not in a growing order"

    #######################################################################
    # copy the raw csv file by changing its name in mod_csv
    #######################################################################

    mod_csv_path = csv_path.replace("raw_stream.csv", "refined_stream.csv")
    if os.path.exists(mod_csv_path):
      os.remove(mod_csv_path)
    shutil.copyfile(csv_path, mod_csv_path)

    #######################################################################
    # for each timestamp in the list, find the closest pose in the csv file and delete the others
    #######################################################################

    raw_csv_timestamps : list[float] = []
    with open(mod_csv_path, "r") as f:
      lines = f.readlines()
      for idx_l, l in enumerate(lines):
        if idx_l == 0:
          continue
        raw_csv_tstamp = float(l.split(";")[1])
        raw_csv_timestamps.append(raw_csv_tstamp)
    
    mod_csv_closest_timestamps : list[float] = []
    mod_csv_closest_timestamps = find_closest_value(frame_single_timestamps,
                                                    raw_csv_timestamps)

    assert(len(raw_csv_timestamps) == len(set(raw_csv_timestamps))), f"[ERROR]: there are duplicates in the list of raw timestamps"
    assert(len(frame_single_timestamps) == len(mod_csv_closest_timestamps)), f"[ERROR]: the number of refined timestamps are not the same: gt: {len(frame_single_timestamps)} != {len(mod_csv_closest_timestamps)}"
    assert(len(mod_csv_closest_timestamps) == len(set(mod_csv_closest_timestamps))), f"[ERROR]: there are duplicates in the list of refined timestamps"
    for i in range(len(mod_csv_closest_timestamps) - 1):
      assert(mod_csv_closest_timestamps[i] < mod_csv_closest_timestamps[i + 1]), f"[ERROR]: the timestamps are not in a growing order"

    #######################################################################
    # delete the lines in the csv file that are not in the list and rename the frames from 1 to n
    #######################################################################

    i_counter : int = 1
    with open(mod_csv_path, "r") as f:
      lines = f.readlines()
      for idx_l, l in enumerate(lines):
        if idx_l == 0:
          continue
        mod_csv_tstamp = float(l.split(";")[1])
        if mod_csv_tstamp not in mod_csv_closest_timestamps:
          lines[idx_l] = ""
        else:
          keep_data = l.split(";")[1:]
          lines[idx_l] = f"{i_counter};{';'.join(keep_data)}"
          i_counter += 1
    with open(mod_csv_path, "w") as f:
      # rename the frames from 1 to n
      for idx_l, l in enumerate(lines):
        if idx_l == 0:
              continue
        if l == "":
          continue
        l = l.replace(str(idx_l), str(idx_l - 1))
        lines[idx_l] = l
      f.writelines(lines)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Refine the optitrack poses.')
    parser.add_argument('--fpath', type=str, default=None,
                        help='path to the csv file')
    args = parser.parse_args()

    if args.fpath is None:
      print("[ERROR]: please specify the folder path")
      exit(1)

    __CSV_PATH = args.fpath + "/raw_stream.csv"
    __IMG_PATH = args.fpath + "/img"
    __IMG_LOG_PATH = args.fpath + "/ffmpeg_capture.log"
    
    refine_poses(csv_path=__CSV_PATH,
                 img_path=__IMG_PATH,
                 img_log_path=__IMG_LOG_PATH)