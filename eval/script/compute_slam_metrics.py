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


# TODO: get rid of all the subfolders while outputing files
# FIXME: the path to the video of the animation needs to be adjusted for patching
# TODO: add an overview of the video (what metrics?based on tools?) at the end of the video when batched

def main(gt_path : str,
         ts_path : str,
         out_dir : str,
         video_path : str,
         id : str,
         is_img_saved : bool,
         is_show_plot : bool,
         is_img_save : bool = False,
         is_rescale : bool = False,
         is_make_animation : bool = False
         ) -> None:
    print("==============================================================================================")
    print(f"--- Import/clean streams processing ..")
    """ We extract the name of the toolhead and the frame start and end from the tslam file name. """
    toolhead_name = '_'.join(ts_path.split('/')[-1].split('_')[2:]).split('.')[0]
    frame_start = int(ts_path.split('/')[-1].split('_')[0])
    frame_end = int(ts_path.split('/')[-1].split('_')[1].split('.')[0])
    total_frames = frame_end - frame_start
    """
        We import the data from the ground truth and the tslam and we output the following data:
        - opti_poss: the positions of the camera from the ground truth optitrack
        - opti_vec_rots: the gt rotations (converted from quat to rotation vector) of the camera from the optitrack
        - ts_poss: the positions of the camera from the tslam
        - ts_vec_rots: the tslam rotations (converted from quat to rotation vector) of the camera from the tslam
        - ts_tags: the number of detected tags for each pose
        - distances: the real total distance effectueted at each pose from the start of the trajectory
        - ts_coverages: the coverage of the tslam (0: good, 1: corrupted(lost/drifted/undetected)).
                        The coverage is defined as the percentage of poses which are not:
                        - corrupted (unreadable data)
                        - lost (no data)
                        - drifted (drifted data when the signal is lost)
                        - undetected (no data)
    """
    opti_poss, opti_vec_rots, distances = io_stream.process_opti_camera_data(gt_path, frame_start, frame_end)
    opti_poss_p30, opti_vec_rots_p30, distances_p30 = io_stream.process_opti_camera_data(gt_path,
                                                                                         (frame_start+30),
                                                                                         (frame_end+30))
    ts_poss, ts_vec_rots, ts_tags, ts_coverages = io_stream.process_ts_data(ts_path)
    io_stream.run_checks(opti_poss, opti_vec_rots, ts_poss, ts_vec_rots, ts_tags, ts_coverages)
    print(f"Streams processed succesfully.")
    print("==============================================================================================")
    print(f"--- Filtering ..")
    """ We apply a savgol filter to the positions of the tslam to smooth the trajectory to mean the traj."""
    filter_size = 21
    ts_poss = savgol_filter(ts_poss, filter_size, 2, axis=0)
    print(f"Filtering applied.")
    print("==============================================================================================")
    print(f"--- Alignement and benchmarking ..")
    """
        We reallign and compute the metrcis. For more details about these two steps, please open the  function
        align_and_benchmark() in the file postpro.py.
        To solve the problem of some corrupted misalignement for the ground truth with the video frames, we 
        compute this phase twice: once with the original ground truth and once with the ground truth shifted
        of 30 frames. Finally we record only the version with the best results for mean position drift.

        The main function responsible for register the tslam trajectory to the gt trajectory based
        on umeyama. We only take the poses with a good coverage and a minimum number of a given
        threshold of detected tags. If the candidates are less than 3, we consider the transformation
        not to be possible.
    """
    TAG_THRESH = 2
    print(f"Number of minimum tag detected per pose: {TAG_THRESH}")
    # FIXME: if the realignement is failing we need to report it in files or do something
    results = postpro.align_and_benchmark(src_poss=ts_poss,
                                          tgt_poss=opti_poss,
                                          src_rot_vec=ts_vec_rots,
                                          tgt_rot_vec=opti_vec_rots,
                                          est_coverage=ts_coverages,
                                          est_tags=ts_tags,
                                          tag_threshold=TAG_THRESH,
                                          is_rescale=is_rescale,
                                          distances=distances,
                                          )
    results_p30 = postpro.align_and_benchmark(src_poss=ts_poss,
                                          tgt_poss=opti_poss_p30,
                                          src_rot_vec=ts_vec_rots,
                                          tgt_rot_vec=opti_vec_rots_p30,
                                          est_coverage=ts_coverages,
                                          est_tags=ts_tags,
                                          tag_threshold=TAG_THRESH,
                                          is_rescale=is_rescale,
                                          distances=distances_p30,
                                          )
    # if results is empty, we return
    if results is None:
        print(f"--- No alignment possible, exiting..")
        os.system(f"touch {out_dir}/NOALLIGNEMENT.txt")
        return

    if results["drift_position_mean"] > results_p30["drift_position_mean"]:
        results = results_p30
        opti_poss = opti_poss_p30
        opti_vec_rots = opti_vec_rots_p30
        distances = distances_p30
    print(f"Alignement and benchmarking terminated.")
    print("==============================================================================================")
    print(f"--- Dumping results to local file..")
    io_stream.dump_results(out_dir,
                           id,
                           results["coverage_percentage"],
                           results["tags_mean"],
                           results["drift_position_mean"],
                           results["drift_rotation_mean"],
                           results["drift_poss_xyz"],
                           results["drift_rots_xyz"],
                           toolhead_name,
                           frame_start,
                           frame_end)
    print(f"Results dumped.")
    print("==============================================================================================")
    print(f"--- Graph creation ..")
    """
        For the visualizations we use the following graphs:
        - 3D trajectories: the trajectories of the tslam and the ground truth in 3D
        - (x2) 2D drift: the drift of the position and rotation of the tslam with its ground truth in 2D
    """
    if is_show_plot or is_img_saved:
        fig_3d = visuals.visualize_trajectories_3d(est_pos=results["ts_position"],
                                                est_rot=results["ts_rotation_vec"],
                                                gt_pos=opti_poss,
                                                gt_rot=opti_vec_rots,
                                                coverages=ts_coverages,
                                                is_draw_dist_error=True,
                                                is_show=is_show_plot)
        fig_2d_poss_drift = visuals.visualize_2d_drift(results["drift_poss_xyz"],
                                                       results["distances_4_metrics"],
                                                       unit='m',
                                                       title='Position drift',
                                                       is_show=is_show_plot)
        fig_2d_rots_drift = visuals.visualize_2d_drift(results["drift_rots_xyz"],
                                                       results["distances_4_metrics"],
                                                       unit='deg',
                                                       title='Rotation drift',
                                                       is_show=is_show_plot)
    print(f"Graphs created.")
    print("==============================================================================================")
    print(f"--- Saving graph locally in {out_dir}/graph ..")
    """ The images are saved in the output folder. """
    if is_img_saved:
        io_stream.dump_imgs(out_dir=out_dir,
                            fig_3d=fig_3d,
                            fig_2d_poss_drift=fig_2d_poss_drift,
                            fig_2d_rots_drift=fig_2d_rots_drift)
    print(f"Graphs saved.")
    print("==============================================================================================")
    print(f"--- Computing animation ..")
    """
        If enabled, we create and save an animation of a comparison side-by-side of the trajectory (gt and est)
        with the corresponding video frames.
    """
    if is_make_animation:
        io_stream.dump_animation(results["ts_position"],
                                 results["ts_rotation_vec"],
                                 opti_poss,
                                 opti_vec_rots,
                                 video_path=video_path,
                                 out_dir=out_dir,
                                 total_frames=total_frames,
                                 is_draw_rot_vec=True)
    print(f"Animation completed and save locally.")
    print("==============================================================================================")
    print("The program exited correctly")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the metrics for tslam.')
    parser.add_argument('--name', type=str, default="noname", help='Name of the sequence.')
    parser.add_argument('--gt', type=str, help='Path to the ground truth trajectory file (refined trajectory).')
    parser.add_argument('--ts', type=str, help='Path to the tslam trajectory directory containing all the ts subsequences poses.')
    parser.add_argument('--showPlot', action='store_true', help='If true, it will show the plot.')
    parser.add_argument('--singleMode', type=str, default="", help='Provides the path to the sub-sequence ts file to benchmark.' \
                                                                    'This provides the chance to cherry pick and analyse just one')
    parser.add_argument('--saveImg', action='store_true', help='If true, it will save the plots.')
    parser.add_argument('--rescale', action='store_true', help='If true, it will rescale the trajectory to the ground truth trajectory during umeyama.')
    parser.add_argument('--makeAnimation', type=str, default="", help='Provides the path to the video sub-sequence with the TSlam interface \
                                                                       (not the entire video) if you want to output animations (extra time).')
    parser.add_argument('--out', type=str, help='Path to the output result files.')
    parser.add_argument('--debug', action='store_false', help='If called, it will print debug information on console rather than saving.')

    args = parser.parse_args()

    if not args.gt.endswith('.csv'):
        print("\033[91m[ERROR]: --gt must be in format .csv\n\033[0m")
        os.makedirs(args.output)

    if not os.path.exists(args.out):
        print("\033[93m[WARNING]: --out folder does not exist, creating one\n\033[0m")
        os.makedirs(args.out)

    name_file : str = args.ts.split('/')[-1].split('.')[0]
    time_stamp : str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _out_subdir : str = f"{args.out}/{args.name}_{time_stamp}"
    os.system(f"mkdir {_out_subdir}")

    _is_make_animation : bool = False
    _video_path = ""
    if args.makeAnimation != "":
        _is_make_animation = True
        _video_path = args.makeAnimation

    _is_single_mode : bool = False
    if args.singleMode != "":
        _is_single_mode = True

    if args.debug:
        sys.stdout = open(f"{_out_subdir}/running_log.txt", "w")

    print("==============================================================================================")
    print(f"Benchmarking of subsequence file: {args.ts}")
    print(f"IS in single mode: {_is_single_mode}")
    print(f"Processing: {args.ts}")

    if _is_single_mode:
        _id = args.singleMode.split('/')[-1].split('.')[0]
        main(gt_path=args.gt,
             ts_path=args.singleMode,
             out_dir=_out_subdir,
             video_path=_video_path,
             id=_id,
             is_make_animation=_is_make_animation,
             is_show_plot=args.showPlot,
             is_img_saved=args.saveImg,
             is_rescale=args.rescale)

        sys.stdout.close()
        return

    ts_files : str = args.ts
    files = [os.path.join(ts_files, f) for f in os.listdir(ts_files) if os.path.isfile(os.path.join(ts_files, f))]
    files.sort(key=lambda x: os.path.getmtime(x))
    total_nbr_files : int = len(files)

    for idx, file in tqdm(enumerate(files), total=total_nbr_files, desc="Benchmarking sequences"):
        _id = file.split('/')[-1].split('.')[0]
        _out_sec_subdir : str = f"{_out_subdir}/bench_{_id}"
        os.system(f"mkdir {_out_sec_subdir}")
        print(f"_out_sec_subdir: {_out_sec_subdir}")

        if not file.endswith(".txt") and "running_log" in file:
            continue

        file_path = os.path.join(ts_files, file)
        print(f"processing file: {file} ..")

        main(gt_path=args.gt,
            ts_path=file_path,
            out_dir=_out_sec_subdir,
            video_path=_video_path,
            id=_id,
            is_make_animation=_is_make_animation,
            is_show_plot=args.showPlot,
            is_img_saved=args.saveImg,
            is_rescale=args.rescale)

    sys.stdout.close()

