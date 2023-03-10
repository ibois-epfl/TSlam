#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt
import os
import pandas as pd


def main():
    """Main function."""
    # variying parameters of the matrix

    ## timber dimensions
    timber_dim : list = ["14x14x2000"]  # <-- dimensions fixed

    ## initial number of joints per timber piece
    init_joints_num : list = ["0", "1", "2", "3", "4"]

    ## type of joints
    joints : list = ["1xscar",
                     "1xhalf-lap",
                     "1xfull-lap",
                     "1xspliced"]

    ## type of tag distribution
    tags_dist : list = ["stripe layout",
                        "ring layout"]
    
    ## type of tag density
    tags_den : list = ["low density",
                       "medium density",
                       "high density"]

    ## type of holes
    holes : list = ["10xspiral20",
                    "3xspiral30",
                    "2xwasher"]

    # create a list of all possible combinations
    combi : list = []
    combi_temp_cuts : list = []
    combi_temp_drills : list = []

    for i in timber_dim:  # <-- dimensions fixed
        for j in init_joints_num:
            for idx_k, k in enumerate(joints):
                idx_k = idx_k + 1
                if (idx_k > int(j)):
                    break
                combi_temp_cuts.append(k)

            for h in holes:
                combi_temp_drills.append(h)

            for l in tags_dist:
                for m in tags_den:
                    combi.append([i, combi_temp_cuts, combi_temp_drills, l, m])
            combi_temp_cuts = []
            combi_temp_drills = []

    # print("List of all possible combinations:")
    # print(f"length: {combi.__len__()}")
    # for i in combi:
    #     print(i)

    df = pd.DataFrame(combi)

    # add speecimen index number
    df.insert(0, "index", range(0, len(df)))

    list_title_row = ["speciment index",
                      "timber dimensions",
                      "cuts itype/number in initial state",
                      "drills type/number to execute",
                      "tags distribution",
                      "tags density"]
    df.loc[0] = list_title_row

    # save the csv/html file
    current_dir : str = os.path.dirname(os.path.realpath(__file__))
    file_path_csv : str = os.path.join(current_dir, "combi.csv")
    file_path_html : str = os.path.join(current_dir, "combi.html")
    df.to_csv(file_path_csv, index=False, header=False)
    df.to_html(file_path_html, index=False, header=False)


if __name__ == "__main__":
    main()