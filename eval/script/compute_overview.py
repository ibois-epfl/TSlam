#!/bin/bash

import argparse
import os
import sys
from tqdm import tqdm
from datetime import datetime

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
         csv_paths : list[str]) -> None:
    """
        We need to regroup the results of boxplots in the categories of tag params (density/layout):
        - a) low/high density
        - b) stripe/ring layout
        First we will give an overview by keeping evident each tool results based on criteria `a` and `b`.
        Second we will average all the tools and provide an overview based uniquely on the mean for à`and `b`.

        Here's the final graphs/results to output:
        ----------------------- part 1 - per tool summary -----------------------
        - 1.A) position boxplot - high/low density for stripe layout
        - 1.B) position boxplot - high/low density for ring layout

        - 2.A) rotation boxplot - high/low density for stripe layout
        - 2.B) rotation boxplot - high/low density for ring layout

        - 3.A) tags detection boxplot - high/low density for stripe layout
        - 3.B) tags detection boxplot - high/low density for ring layout

        - 4.A) coverage distribution - high density for stripe layout
        - 4.B) coverage distribution - high density for ring layout
        - 4.C) coverage distribution - low density for stripe layout
        - 4.D) coverage distribution - low density for ring layout

        ----------------------- part 2 - fabrication summary --------------------
        - 5) preparation time columns - mean high/ mean low density for stripe and ring layout
        - 6) position boxplot - mean high/ mean low density for stripe and ring layout
        - 7) rotation boxplot - mean high/ mean low density for stripe and ring layout
        - 8) tags detection boxplot - mean high/ mean low density for stripe and ring layout
        - 9) coverage distribution - mean high/ mean low density for stripe and ring layout (4 lines)

        :param out_subdir: path to the output directory to dump results/graphs
        :param csv_paths: list of paths to the csv files containing the results of the analysis per sequence
    """

    # mean and merge all csv based on the density/layout/stripe/ring matrix (20 csv -> 4 csv)
    csv_paths_lowD_stripe : list[str] = [x for x in csv_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_STRIPE__]
    csv_paths_lowD_ring : list[str] = [x for x in csv_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_LOWD_RING__]
    csv_paths_highD_stripe : list[str] = [x for x in csv_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_STRIPE__]
    csv_paths_highD_ring : list[str] = [x for x in csv_paths if int(x.split("/")[-6].split("_")[0]) in __SEQUENCES_MAP_HIGHD_RING__]



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

    _csv_paths : list[str] = []
    for root, dirs, files in os.walk(args.inDir):
        for file in files:
            if file.endswith("summarys_sequence_bench.csv"):
                _csv_paths.append(os.path.join(root, file))
    assert _csv_paths.__len__() == 20, f"Wrong number of csv found in {args.inDir}"
    _csv_paths_nbr : list[str] = []
    for csv in _csv_paths:
        _csv_paths_nbr.append(int(csv.split("/")[-6].split("_")[0]))
    _csv_paths = [x for _,x in sorted(zip(_csv_paths_nbr,_csv_paths))]

    main(out_subdir=_out_subdir,
         csv_paths=_csv_paths)