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
__SEQUENCES_MAP_LOWD_STRIPE__ = [0,4,8,12,16]
__SEQUENCES_MAP_LOWD_RING__ = [1,5,9,13,17]
__SEQUENCES_MAP_HIGHD_STRIPE__ = [2,6,10,14,18]
__SEQUENCES_MAP_HIGHD_RING__ = [3,7,11,15,19]


def main(out_subdir : str,
         csv_sequ_paths : list[str],
         csv_subsequ_paths : list[list[str]]) -> None:
    """
        We need to regroup the results of boxplots in the categories of tag params (density/layout):
        - a) low/high density
        - b) stripe/ring layout
        First we will give an overview by keeping evident each tool results based on criteria `a` and `b`.
        Second we will average all the tools and provide an overview based uniquely on the mean for à`and `b`.

        Here's the final graphs/results to output:
        # ----------------------- part 1 - per tool summary(SUSPENDED) -----------------------
        # - 1.A) position boxplot - high/low density for stripe layout
        # - 1.B) position boxplot - high/low density for ring layout

        # - 2.A) rotation boxplot - high/low density for stripe layout
        # - 2.B) rotation boxplot - high/low density for ring layout

        # - 3.A) tags detection boxplot - high/low density for stripe layout
        # - 3.B) tags detection boxplot - high/low density for ring layout

        # - 4.A) coverage distribution - high density for stripe layout
        # - 4.B) coverage distribution - high density for ring layout
        # - 4.C) coverage distribution - low density for stripe layout
        # - 4.D) coverage distribution - low density for ring layout

        ----------------------- part 2 - fabrication summary --------------------
        - 5) preparation time columns - mean high/ mean low density for stripe and ring layout
        - 6) position boxplot - mean high/ mean low density for stripe and ring layout
        - 7) rotation boxplot - mean high/ mean low density for stripe and ring layout
        - 8) tags detection boxplot - mean high/ mean low density for stripe and ring layout
        - 9) coverage distribution - mean high/ mean low density for stripe and ring layout (4 lines)

        :param out_subdir: path to the output directory to dump results/graphs
        :param csv_sequ_paths: list of list of paths to the csv files containing the results of the analysis per sequence
    """
    # #================================================
    # ## Part 1
    # #================================================

    # # parse the csv by the 4 groups and merge them by category
    # sequences_map_group : list[list[str]] = [[],[],[],[]]

    # csv_subsequ_paths_lowD_stripe : list[str] = []
    # for i in tqdm(__SEQUENCES_MAP_LOWD_STRIPE__):
    #     csv_subsequ_paths_lowD_stripe.extend(csv_subsequ_paths[i])
    # csv_subsequ_paths_lowD_ring : list[str] = []
    # for i in tqdm(__SEQUENCES_MAP_LOWD_RING__):
    #     csv_subsequ_paths_lowD_ring.extend(csv_subsequ_paths[i])
    # csv_subsequ_paths_highD_stripe : list[str] = []
    # for i in tqdm(__SEQUENCES_MAP_HIGHD_STRIPE__):
    #     csv_subsequ_paths_highD_stripe.extend(csv_subsequ_paths[i])
    # csv_subsequ_paths_highD_ring : list[str] = []
    # for i in tqdm(__SEQUENCES_MAP_HIGHD_RING__):
    #     csv_subsequ_paths_highD_ring.extend(csv_subsequ_paths[i])

    # sequences_map_group[0] = csv_subsequ_paths_lowD_stripe
    # sequences_map_group[1] = csv_subsequ_paths_lowD_ring
    # sequences_map_group[2] = csv_subsequ_paths_highD_stripe
    # sequences_map_group[3] = csv_subsequ_paths_highD_ring

    # #================================================
    # # for each group:
    # # 1. check if one typology exists
    # # 2. if other exists, merge all the same csv into one
    # sequence_map_data : list[list[list[str]]] = [[],[],[],[]]

    # for idx, path_lst in enumerate(tqdm(sequences_map_group)):
    #     sequence_map_data[idx] = io_stream.merge_csv_by_categ(csv_paths=path_lst)

    # #================================================
    # # prepare the data and labels for the boxplots









    #================================================
    ## Part 2
    #================================================
    #TODO: explain well what we are doing

    # mean and merge all csv based on the density/layout/stripe/ring matrix (20 csv -> 4 csv)
    csv_sequ_paths_lowD_stripe : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    csv_sequ_paths_lowD_ring : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    csv_sequ_paths_highD_stripe : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    csv_sequ_paths_highD_ring : list[str] = [x for x in csv_sequ_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]

    # #================================================
    # ## 6.A) LOWDENSITY STRIPE
    # # merge all values by line in csv by averaging them
    # tool_id : int = 0  x
    # nbr_operations : int = 0
    # average_time_per_operation : float = 0.0
    # mean_drift_position_m : float = 0.0
    # mean_drift_position_q1 : float = 0.0
    # mean_drift_position_q3 : float = 0.0
    # mean_drift_position_min : float = 0.0
    # mean_drift_position_max : float = 0.0
    # mean_drift_rotation_m : float = 0.0
    # mean_drift_rotation_q1 : float = 0.0
    # mean_drift_rotation_q3 : float = 0.0
    # mean_drift_rotation_min : float = 0.0
    # mean_drift_rotation_max : float = 0.0
    # tags_m : float = 0.0
    # tags_q1 : float = 0.0
    # tags_q3 : float = 0.0
    # tags_min : float = 0.0
    # tags_max : float = 0.0
    # coverage_m : float = 0.0
    # mean_coverage_perc_quintile1 : float = 0.0
    # mean_coverage_perc_quintile2 : float = 0.0
    # mean_coverage_perc_quintile3 : float = 0.0
    # mean_coverage_perc_quintile4 : float = 0.0
    # mean_coverage_perc_quintile5 : float = 0.0

    #================================================
    # convert csv to data
    # NB.:! we are skipping the saber_sawblade if it exists because its performance is too bad and we know
    data_lowD_stripe = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_lowD_stripe)
    data_lowD_ring = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_lowD_ring)
    data_highD_stripe = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_highD_stripe)
    data_highD_ring = io_stream.cvt_csv_summary_to_data(csv_paths=csv_sequ_paths_highD_ring)

    # get the avrages of each values in the summary groups
    avr_data_lowD_stripe = metrics.compute_average_summary_values(data=data_lowD_stripe)
    avr_data_lowD_ring = metrics.compute_average_summary_values(data=data_lowD_ring)
    avr_data_highD_stripe = metrics.compute_average_summary_values(data=data_highD_stripe)
    avr_data_highD_ring = metrics.compute_average_summary_values(data=data_highD_ring)

    #================================================
    # visualize and dump
    # NB: in green is the median!
    # position drift
    pair_pos_stripe = np.array(([data_lowD_stripe[2], data_highD_stripe[2]]), dtype=object)
    pair_pos_ring = np.array(([data_lowD_ring[2], data_highD_ring[2]]), dtype=object)
    graph_pos = visuals.draw_double_boxplot(data_a=pair_pos_stripe,
                                data_b=pair_pos_ring,
                                ytitle="Position drift (m)",
                                xthick=0.001)
    
    # rotation drift
    pair_rot_stripe = np.array(([data_lowD_stripe[7], data_highD_stripe[7]]), dtype=object)
    pair_rot_ring = np.array(([data_lowD_ring[7], data_highD_ring[7]]), dtype=object)
    graph_rot = visuals.draw_double_boxplot(data_a=pair_rot_stripe,
                                data_b=pair_rot_ring,
                                ytitle="Rotation drift (deg)")

    # tags detection
    pair_tags_stripe = np.array(([data_lowD_stripe[12], data_highD_stripe[12]]), dtype=object)
    pair_tags_ring = np.array(([data_lowD_ring[12], data_highD_ring[12]]), dtype=object)
    graph_tags = visuals.draw_double_boxplot(data_a=pair_tags_stripe,
                                data_b=pair_tags_ring,
                                ytitle="Tags detection (m)",
                                xthick=1)

    #TODO: time graph

    # save the graphs
    io_stream.save_graph(graph=graph_pos, path=f"{out_subdir}/summary_position_drift.png")
    io_stream.save_graph(graph=graph_rot, path=f"{out_subdir}/summary_rotation_drift.png")
    io_stream.save_graph(graph=graph_tags, path=f"{out_subdir}/summary_tags_detection.png")

    #================================================
    # print csv + latex table
    #TODO: output tables



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
    assert _csv_sequ_paths.__len__() == 20, f"Wrong number of csv found in {args.inDir}"
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