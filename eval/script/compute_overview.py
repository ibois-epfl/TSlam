#!/bin/bash

import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime

import numpy as np

import metrics
import io_stream
import visuals
import postpro


__SEQUENCES_MAP__ = {
    # name sequence : [density (0=low, 1=high), layout(0=stripe, 1=ring)]
    "sequence_1" :  [0,0],
    "sequence_2" :  [1,0],
    "sequence_3" :  [0,1],
    "sequence_4" :  [1,1],
    "sequence_5" :  [0,0],
    "sequence_6" :  [1,0],
    "sequence_7" :  [0,1],
    "sequence_8" :  [1,1],
    "sequence_9" :  [0,0],
    "sequence_10" : [1,0],
    "sequence_11" : [0,1],
    "sequence_12" : [1,1],
    "sequence_13" : [0,0],
    "sequence_14" : [1,0],
    "sequence_15" : [0,1],
    "sequence_16" : [1,1],
    "sequence_17" : [0,0],
    "sequence_18" : [1,0],
    "sequence_19" : [0,1],
    "sequence_20" : [1,1]
}
__SEQUENCES_MAP_LOWD_STRIPE__ = [1,5,9,13,17]
__SEQUENCES_MAP_LOWD_RING__ = [2,6,10,14,18]
__SEQUENCES_MAP_HIGHD_STRIPE__ = [3,7,11,15,19]
__SEQUENCES_MAP_HIGHD_RING__ = [4,8,12,16,20]


def main(out_subdir : str,
         csv_sequ_paths : list[str],
         csv_subsequ_paths : list[list[str]]) -> None:
    """
        In this part we produce the summary results.
        We use the averaged results of the per-sequence (20 sequences) averages per tool.
        These averages are averaged together since we consider the entire fabrication sequence, 
        all tools EXCEPT THE SABERSAW BECAUSE WE KNOW THE TRACKING IS NOT WORKING.
        We produce the following graphs:
        - A) the position drift (m) for the 4 groups (low/high density, stripe/ring layout)
        - B) the rotation drift (deg) for the 4 groups (low/high density, stripe/ring layout)
        - C) the tags detection (m) for the 4 groups (low/high density, stripe/ring layout)
        - D) the time (s) for the 4 groups (low/high density, stripe/ring layout) as column graph
    """
    # mean and merge all csv based on the density/layout/stripe/ring matrix (20 csv -> 4 csv)
    csv_sequ_paths_lowD_stripe : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    csv_sequ_paths_lowD_ring : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    csv_sequ_paths_highD_stripe : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    csv_sequ_paths_highD_ring : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]

    # convert csv to data
    # NB.:! we are skipping the saber_sawblade if it exists because its performance is too bad and we know
    data_lowD_stripe = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_lowD_stripe)
    data_lowD_ring = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_lowD_ring)
    data_highD_stripe = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_highD_stripe)
    data_highD_ring = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_highD_ring)

    # time parsing
    # # get the preparation times from videos and retain only the not pre-fabricated piece
    # # in-fact the comparison will not equal with the manual timing due to the fact that
    # # with more joinery the sticking time would be longer and the manual timing is not.
    # # we consider and show in the time graph the standard situation where the piece
    # # is not pre-fabricated (i.e. box shape).
    vid_manual_mark_paths, vid_mapping_paths, vid_tag_paths  = io_stream.get_video_path()

    path_time_manual_lowD_stripe : float = [x for x in vid_manual_mark_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    path_time_manual_lowD_ring : float = [x for x in vid_manual_mark_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    path_time_manual_highD_stripe : float = [x for x in vid_manual_mark_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    path_time_manual_highD_ring : float = [x for x in vid_manual_mark_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]
    path_time_tag_lowD_stripe : float = [x for x in vid_tag_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    path_time_tag_lowD_ring : float = [x for x in vid_tag_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    path_time_tag_highD_stripe : float = [x for x in vid_tag_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    path_time_tag_highD_ring : float = [x for x in vid_tag_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]
    path_time_mapping_lowD_stripe : float = [x for x in vid_mapping_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    path_time_mapping_lowD_ring : float = [x for x in vid_mapping_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    path_time_mapping_highD_stripe : float = [x for x in vid_mapping_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    path_time_mapping_highD_ring : float = [x for x in vid_mapping_paths if int(x.split("/")[-3].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]

    time_manual_lowD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_manual_lowD_stripe)
    time_manual_lowD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_manual_lowD_ring)
    time_manual_highD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_manual_highD_stripe)
    time_manual_highD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_manual_highD_ring)
    time_tag_lowD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_tag_lowD_stripe)
    time_tag_lowD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_tag_lowD_ring)
    time_tag_highD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_tag_highD_stripe)
    time_tag_highD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_tag_highD_ring)
    time_mapping_lowD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_mapping_lowD_stripe)
    time_mapping_lowD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_mapping_lowD_ring)
    time_mapping_highD_stripe : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_mapping_highD_stripe)
    time_mapping_highD_ring : list[float] = postpro.cvt_video_2_time(vid_paths=path_time_mapping_highD_ring)

    mean_time_manual_lowD_stripe : float = np.mean(time_manual_lowD_stripe)
    mean_time_manual_lowD_ring : float = np.mean(time_manual_lowD_ring)
    mean_time_manual_highD_stripe : float = np.mean(time_manual_highD_stripe)
    mean_time_manual_highD_ring : float = np.mean(time_manual_highD_ring)
    mean_time_tag_lowD_stripe : float = np.mean(time_tag_lowD_stripe)
    mean_time_tag_lowD_ring : float = np.mean(time_tag_lowD_ring)
    mean_time_tag_highD_stripe : float = np.mean(time_tag_highD_stripe)
    mean_time_tag_highD_ring : float = np.mean(time_tag_highD_ring)
    mean_time_mapping_lowD_stripe : float = np.mean(time_mapping_lowD_stripe)
    mean_time_mapping_lowD_ring : float = np.mean(time_mapping_lowD_ring)
    mean_time_mapping_highD_stripe : float = np.mean(time_mapping_highD_stripe)
    mean_time_mapping_highD_ring : float = np.mean(time_mapping_highD_ring)


    # # ==========================================
    # # visualize and dump
    # # time prep
    graph_time = visuals.draw_time_graph(data_a=np.array([mean_time_tag_lowD_stripe,
                                                 mean_time_tag_highD_stripe,
                                                 mean_time_tag_lowD_ring,
                                                 mean_time_tag_highD_ring], dtype=object),
                                         data_b=np.array([mean_time_manual_lowD_stripe,
                                                 mean_time_manual_highD_stripe,
                                                 mean_time_manual_lowD_ring,
                                                 mean_time_manual_highD_ring], dtype=object))

    # NB: in green is the median!
    # position drift
    data_lowD_stripe_pos_np = np.array(data_lowD_stripe[2], dtype=float)
    data_highD_stripe_pos_np = np.array(data_highD_stripe[2], dtype=float)
    data_lowD_ring_pos_np = np.array(data_lowD_ring[2], dtype=float)
    data_highD_ring_pos_np = np.array(data_highD_ring[2], dtype=float)

    pair_pos_stripe = list([data_lowD_stripe_pos_np, data_highD_stripe_pos_np])
    pair_pos_ring = list([data_lowD_ring_pos_np, data_highD_ring_pos_np])

    graph_pos = visuals.draw_double_boxplot(data_a=pair_pos_stripe,
                                            data_b=pair_pos_ring,
                                            ytitle="Position drift (m)",
                                            xthick=0.001)

    
    # # rotation drift
    data_lowD_stripe_rot_np = np.array(data_lowD_stripe[7], dtype=float)
    data_highD_stripe_rot_np = np.array(data_highD_stripe[7], dtype=float)
    data_lowD_ring_rot_np = np.array(data_lowD_ring[7], dtype=float)
    data_highD_ring_rot_np = np.array(data_highD_ring[7], dtype=float)

    pair_rot_stripe = list([data_lowD_stripe_rot_np, data_highD_stripe_rot_np])
    pair_rot_ring = list([data_lowD_ring_rot_np, data_highD_ring_rot_np])

    graph_rot = visuals.draw_double_boxplot(data_a=pair_rot_stripe,
                                            data_b=pair_rot_ring,
                                            ytitle="Rotation drift (deg)")

    # # tags detection
    data_lowD_stripe_tags_np = np.array(data_lowD_stripe[12], dtype=float)
    data_highD_stripe_tags_np = np.array(data_highD_stripe[12], dtype=float)
    data_lowD_ring_tags_np = np.array(data_lowD_ring[12], dtype=float)
    data_highD_ring_tags_np = np.array(data_highD_ring[12], dtype=float)

    pair_tags_stripe = list([data_lowD_stripe_tags_np, data_highD_stripe_tags_np])
    pair_tags_ring = list([data_lowD_ring_tags_np, data_highD_ring_tags_np])

    graph_tags = visuals.draw_double_boxplot(data_a=pair_tags_stripe,
                                data_b=pair_tags_ring,
                                ytitle="Tags detection (m)",
                                xthick=1)

    # # save the graphs
    io_stream.save_graph(graph=graph_pos, path=f"{out_subdir}/summary_position_drift.png")
    io_stream.save_graph(graph=graph_rot, path=f"{out_subdir}/summary_rotation_drift.png")
    io_stream.save_graph(graph=graph_tags, path=f"{out_subdir}/summary_tags_detection.png")
    io_stream.save_graph(graph=graph_time, path=f"{out_subdir}/summary_time.png")

    # #===========================================
    # #TODO:
    # # print csv + latex table (at least to have the main values like mean, median, std, min, max)
    # # - get also the total number of operations
    # # metrics.compute_summary_table

    total_nbr_operations : int = 0
    # do a mass addition of the total number of operations
    for data in [data_lowD_stripe, data_highD_stripe, data_lowD_ring, data_highD_ring]:
        total_nbr_operations += int(data[0])
    print(f"total_nbr_operations: {total_nbr_operations}")

    # create a new csv file in the out_subdir
    csv_summary_path : str = f"{out_subdir}/summary.csv"
    with open(csv_summary_path, 'w') as f:
        f.write("total_nbr_operations

    return None


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Compute the summary and output the graphs for TSlam analysis.')
    parser.add_argument('--inDir', type=str, default="", help='Path where the analysis output folders are stored.')
    parser.add_argument('--outDir', type=str, default="./", help='Path to the output directory to dump results/graphs.')

    if len(sys.argv) == 1:
        parser.print_help()
        sys.exit()
    args = parser.parse_args()

    if args.inDir == "":
        print("\033[91m[ERROR]: --inDir is empty\n\033[0m")
        sys.exit()

    if not os.path.exists(args.outDir):
        print("\033[93m[WARNING]: --out folder does not exist, creating one\n\033[0m")
        os.makedirs(args.outDir)
    time_stamp : str = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    _out_subdir : str = f"{args.outDir}/{time_stamp}"
    os.system(f"mkdir {_out_subdir}")

    _csv_sequ_paths : list[str] = []
    for root, dirs, files in os.walk(args.inDir):
        for file in files:
            if file.endswith("summarys_sequence_bench.csv"):
                _csv_sequ_paths.append(os.path.join(root, file))
    if _csv_sequ_paths.__len__() != 20:
        print(f"\033[91m[ERROR]:Found {_csv_sequ_paths.__len__()} csv in {args.inDir}\n\033[0m")
    # assert _csv_sequ_paths.__len__() == 20, f"Wrong number of csv found in {args.inDir}"
    _csv_sequ_paths_nbr : list[str] = []
    for csv in _csv_sequ_paths:
        _csv_sequ_paths_nbr.append(int(csv.split("/")[-6].split("_")[0]))
    _csv_sequ_paths = [x for _,x in sorted(zip(_csv_sequ_paths_nbr,_csv_sequ_paths))]
    # check if there are all 0 summary values
    for path in _csv_sequ_paths:
        with open(path, 'r') as f:
            lns = f.readlines()
            lns.pop(0)
            all_zero : bool = True
            for ln in lns:
                ln = ln.split(",")
                for i in range(1,ln.__len__()):
                    if float(ln[i]) != 0.0:
                        all_zero = False
                        break
                if not all_zero:
                    break
            if all_zero:
                _csv_sequ_paths.remove(path)
                print(f"\033[93m[WARNING]: all values are 0 in {path}, removing it from the list\n\033[0m")

    _csv_subsequ_paths : list[list[str]] = []
    _in_subdirs = os.listdir(args.inDir)
    for subd in _in_subdirs:
        _csv_lst_temp : list[str] = []
        for root, dirs, files in os.walk(f"{args.inDir}/{subd}"):
            for file in files:
                if file.endswith("circular_sawblade_140.csv") or \
                   file.endswith("drill_auger_bit_20_200.csv") or \
                   file.endswith("drill_auger_bit_25_500.csv") or \
                   file.endswith("drill_hinge_cutter_bit_50.csv") or \
                   file.endswith("drill_oblique_hole_bit_40.csv") or \
                   file.endswith("saber_sawblade_t1.csv") or \
                   file.endswith("st_screw_45.csv") or \
                   file.endswith("st_screw_80.csv") or \
                   file.endswith("st_screw_100.csv") or \
                   file.endswith("st_screw_120.csv"):
                    _csv_lst_temp.append(os.path.join(root, file))
        if _csv_lst_temp.__len__() == 0:
            _csv_subsequ_paths.append(_csv_lst_temp)
            print(f"\033[93m[WARNING]: no subsequences found in {subd}\n\033[0m")
            continue
        _csv_subsequ_paths.append(_csv_lst_temp)

    main(out_subdir=_out_subdir,
         csv_sequ_paths=_csv_sequ_paths,
         csv_subsequ_paths=_csv_subsequ_paths)