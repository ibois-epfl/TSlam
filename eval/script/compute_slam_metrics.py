import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime
import transformations as tfm
import math

import io_stream
import util
import postpro
import visuals
import metrics

import numpy as np

from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation


def main(gt_path : str,
         ts_path : str,
         out_dir : str,
         is_img_saved : bool,
         is_show_plot : bool,
         is_img_save : bool = False,
         is_rescale : bool = False
         ) -> None:

    # #######################################################
    # # Import/clean streams
    # #######################################################

    toolhead_name = '_'.join(ts_path.split('/')[-1].split('_')[2:]).split('.')[0]
    frame_start = int(ts_path.split('/')[-1].split('_')[0])
    frame_end = int(ts_path.split('/')[-1].split('_')[1].split('.')[0])

    opti_poss, opti_vec_rots, distances = io_stream.process_opti_camera_data(gt_path, frame_start, frame_end)
    ts_poss, ts_vec_rots, ts_tags, ts_coverages = io_stream.process_ts_data(ts_path)
    io_stream.run_checks(opti_poss, opti_vec_rots, ts_poss, ts_vec_rots, ts_tags, ts_coverages)

    # #######################################################
    # # Filtering
    # #######################################################

    # apply a smoothing filter to the tslam trajectory
    filter_size = 21
    ts_poss = savgol_filter(ts_poss, filter_size, 2, axis=0)

    # #######################################################
    # # Trajectory registration
    # #######################################################

    ts_poss, ts_vec_rots, ts_idx_candidates = postpro.align_trajectories(ts_poss,
                                                                         opti_poss,
                                                                         ts_vec_rots,
                                                                         opti_vec_rots,
                                                                         ts_coverages,
                                                                         ts_tags,
                                                                         coverage_threshold=20,
                                                                         tag_threshold=3,
                                                                         is_rescale=is_rescale
                                                                         )
    
    # #######################################################
    # # Analytics
    # #######################################################

    coverage_perc : float = metrics.compute_coverage(ts_coverages)
    tags_mean : float = metrics.compute_tags_nbr_mean(ts_tags)
    drift_poss_mean, poss_xyz = metrics.compute_position_drift(opti_poss, ts_poss)
    drift_rots_mean, rots_xyz = metrics.compute_rotation_drift(opti_vec_rots, ts_vec_rots)

    io_stream.dump_results(out_dir,
                           coverage_perc,
                           tags_mean,
                           drift_poss_mean,
                           drift_rots_mean,
                           poss_xyz,
                           rots_xyz,
                           toolhead_name,
                           frame_start,
                           frame_end)

    # #######################################################
    # # Visualization
    # #######################################################

    fig_3d = visuals.visualize_trajectories3d(ts_poss,
                                              ts_vec_rots,
                                              opti_poss,
                                              opti_vec_rots,
                                              ts_idx_candidates,
                                              is_draw_dist_error=True,
                                              is_show=is_show_plot)
    fig_2d_poss_drift = visuals.visualize_2d_drift(poss_xyz,
                                                   distances,
                                                   unit='m',
                                                   title='Position drift',
                                                   is_show=is_show_plot)
    fig_2d_rots_drift = visuals.visualize_2d_drift(rots_xyz,
                                                  distances,
                                                  unit='deg',
                                                  title='Rotation drift',
                                                  is_show=is_show_plot)
    
    # #######################################################
    # save img
    # #######################################################
    
    io_stream.dump_imgs(out_dir=out_dir,
                        fig_3d=fig_3d,
                        fig_2d_poss_drift=fig_2d_poss_drift,
                        fig_2d_rots_drift=fig_2d_rots_drift,
                        is_img_save=is_img_saved)
    

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the metrics for tslam.')
    parser.add_argument('--name', type=str, default="noname", help='Name of the sequence.')
    parser.add_argument('--gt', type=str, help='Path to the ground truth trajectory file (refined trajectory).')
    parser.add_argument('--ts', type=str, help='Path to the tslam trajectory file.')
    parser.add_argument('--showPlot', action='store_true', help='If true, it will show the plot.')
    parser.add_argument('--singleMode', action='store_true', help='If true, it will process only one file provided in --ts.')
    parser.add_argument('--saveImg', action='store_true', help='If true, it will save the plots.')
    parser.add_argument('--rescale', action='store_true', help='If true, it will rescale the trajectory to the ground truth trajectory during umeyama.')
    parser.add_argument('--out', type=str, help='Path to the output result files.')

    args = parser.parse_args()

    if not args.gt.endswith('.csv'):
        print("\033[91m[ERROR]: --gt must be in format .csv\n\033[0m")
        os.makedirs(args.output)

    if not os.path.exists(args.out):
        print("\033[93m[WARNING]: --out folder does not exist, creating one\n\033[0m")
        os.makedirs(args.out)
    # print(f"[INFO]: creating sub-directory for current sequence ..")
    name_file : str = args.ts.split('/')[-1].split('.')[0]
    time_stamp : str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _out_subdir : str = f"{args.out}/{args.name}_{time_stamp}"
    os.system(f"mkdir {_out_subdir}")
    # print(f"\033[92m[INFO]: output folder: {_out_subdir}\n\033[0m")


    if args.singleMode:
        main(gt_path=args.gt,
             ts_path=args.ts,
             out_dir=_out_subdir,
             is_show_plot=args.showPlot,
             is_img_saved=args.saveImg,
             is_rescale=args.rescale)
    else:
        ts_files : str = "/home/as/TSlam/eval/script/test/outposes/01_2023-05-19_19-29-01"  # TODO: replace with arg
        files = [os.path.join(ts_files, f) for f in os.listdir(ts_files) if os.path.isfile(os.path.join(ts_files, f))]
        files.sort(key=lambda x: os.path.getmtime(x))

        for idx, file in tqdm(enumerate(files)):
            if file.endswith(".txt") and not "running_log" in file:
                file_path = os.path.join(ts_files, file)
                print(file)
                main(gt_path=args.gt,
                    ts_path=file_path,
                    out_dir=_out_subdir,
                    is_show_plot=args.showPlot,
                    is_img_saved=args.saveImg,
                    is_rescale=args.rescale)
                # if idx == 6:
                #     break


