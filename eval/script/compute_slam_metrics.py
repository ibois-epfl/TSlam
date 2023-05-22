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

import numpy as np

from scipy.signal import savgol_filter
from scipy.spatial.transform import Rotation


def main(gt_path : str,
         ts_path : str,
         out_dir : str,
         is_show_plot : bool,
         is_img_save : bool = False
         ) -> None:

    # #######################################################
    # # Import/clean streams
    # #######################################################

    frame_start = int(ts_path.split('/')[-1].split('_')[0])
    frame_end = int(ts_path.split('/')[-1].split('_')[1].split('.')[0])

    opti_poss, opti_vec_rots = io_stream.process_opti_data(gt_path, frame_start, frame_end)
    ts_poss, ts_vec_rots, ts_tags, ts_coverages = io_stream.process_ts_data(ts_path)
    io_stream.run_checks(opti_poss, opti_vec_rots, ts_poss, ts_vec_rots, ts_tags, ts_coverages)

    # # print pourcentage of coverage in % based on ts_coverages 0 = good, 1 = bad
    # coverage_perc : float = (np.sum(ts_coverages == 1) / len(ts_coverages) * 100)
    # print(f">>>>>>>>>>Coverage: {coverage_perc.round(1)} %")

    # #######################################################
    # # Filtering
    # #######################################################

    # # apply a smoothing filter to the tslam trajectory
    filter_size = 21  # FIXME: is this scientifically sound?
    # opti_poss = savgol_filter(opti_poss, filter_size, 3, axis=0)  # FIXME: understand if it is correct to do it and what is the impact
    ts_poss = savgol_filter(ts_poss, filter_size, 2, axis=0)  # FIXME: understand if it is correct to do it and what is the impact

    # #######################################################
    # # Trajectory registration
    # #######################################################

    ts_poss_ori = ts_poss.copy()
    ts_vec_rots_ori = ts_vec_rots.copy()
    ts_poss, ts_vec_rots, ts_idx_candidates = postpro.align_trajectories(ts_poss,
                                            opti_poss,
                                            ts_vec_rots,
                                            opti_vec_rots,
                                            ts_coverages,
                                            20,
                                            2,
                                            ts_tags
                                            )
    
    # #######################################################
    # # Analytics
    # #######################################################

    # #######################################################
    # # Visualization
    # #######################################################

    # FIXME: we need to visualize the poses used for the alignment
    visuals.visualize_trajectories(ts_poss,
                                   ts_vec_rots,
                                   opti_poss,
                                   opti_vec_rots,
                                   ts_idx_candidates,
                                   out_dir,
                                   is_show=is_show_plot)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the metrics for tslam.')
    parser.add_argument('--name', type=str, default="noname", help='Name of the sequence.')
    parser.add_argument('--gt', type=str, help='Path to the ground truth trajectory file (refined trajectory).')
    parser.add_argument('--ts', type=str, help='Path to the tslam trajectory file.')
    parser.add_argument('--showPlot', action='store_true', help='If true, it will show the plot.')
    parser.add_argument('--singleMode', action='store_true', help='If true, it will process only one file provided in --ts.')
    parser.add_argument('--saveImg', action='store_true', help='If true, it will save the plots.')
    parser.add_argument('--out', type=str, help='Path to the output result files.')

    args = parser.parse_args()

    if not args.gt.endswith('.csv'):
        print("\033[91m[ERROR]: --gt must be in format .csv\n\033[0m")
        os.makedirs(args.output)
    if not os.path.exists(args.out):
        print("\033[93m[WARNING]: --out folder does not exist, creating one\n\033[0m")
        os.makedirs(args.out)
    # _out_subdir : str = f"{args.out}/{args.name}_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}"
    # os.system(f"mkdir {_out_subdir}")

    if args.singleMode:
        main(gt_path=args.gt,
            ts_path=args.ts,
            out_dir=args.out,
            is_show_plot=args.showPlot,
            is_img_save=args.saveImg)
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
                    out_dir=args.out,
                    is_show_plot=args.showPlot,
                    is_img_save=args.saveImg)
                # if idx == 6:
                #     break


