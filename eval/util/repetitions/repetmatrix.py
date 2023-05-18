#!/usr/bin/env python3

import numpy as np
import os
import pandas as pd


def main():
    """Main function."""
    # variying parameters of the matrix

    ## timber dimensions
    timber_dim : list = ["14x14x200"]  # <-- dimensions fixed

    ## initial number of joints per timber piece
    init_joints_num : list = ["0", "1", "2", "3", "4"]

    ## type of joints
    joints_init : list = ["1xscar",
                     "1xhalf-lap",
                     "1xfull-lap",
                     "1xspliced"]
    
    ## type of joints
    joints_final : list = ["1xscar",
                     "1xhalf-lap",
                     "1xfull-lap",
                     "1xspliced",
                     "2xspliced-45"]

    ## type of tag distribution
    tags_dist : list = ["stripe layout",
                        "ring layout"]
    
    ## type of tag density
    tags_den : list = ["low density",
                       "medium density"]

    ## type of holes
    holes : list = ["18xspiralShort20",
                    "2xspiralLong25",
                    "2xspiralOblique35"
                    "6xwasher50",
                    "8xscrews120",
                    "8xscrews100",
                    "8xscrews80",
                    "8xscrews45"
                    ]

    # create a list of all possible combinations
    combi : list = []
    combi_temp_init_cuts : list = []
    combi_temp_final_cuts : list = []
    combi_temp_drills : list = []

    for i in timber_dim:  # <-- dimensions fixed
        for j in init_joints_num:
            for idx_k, k in enumerate(joints_init):
                idx_k = idx_k + 1
                if (idx_k > int(j)):
                    break
                combi_temp_init_cuts.append(k)

            for jf in joints_final:
                combi_temp_final_cuts.append(jf)

            for h in holes:
                combi_temp_drills.append(h)

            for l in tags_dist:
                for m in tags_den:
                    combi.append([i, combi_temp_init_cuts, combi_temp_final_cuts, combi_temp_drills, l, m])
            combi_temp_init_cuts = []
            combi_temp_final_cuts = []
            combi_temp_drills = []
    
    df = pd.DataFrame(combi)

    # add speecimen index number without counting the title row
    df.insert(0, "index", range(1, len(df)+1))

    list_title_row = ["specimen index",
                      "timber dimensions [cm]",
                      "cuts type/number in initial state",
                      "cuts type/number in final state",
                      "drills type/number to execute",
                      "tags distribution",
                      "tags density"]
    # insert the titles before the first row
    # df.loc[0] = list_title_row
    # insert title row at the top
    df = pd.concat([pd.DataFrame([list_title_row], columns=df.columns), df], ignore_index=True)

    # save the csv/html file
    current_dir : str = os.path.dirname(os.path.realpath(__file__))
    file_path_csv : str = os.path.join(current_dir, "combi.csv")
    file_path_html : str = os.path.join(current_dir, "combi.html")
    df.to_csv(file_path_csv, index=False, header=False)
    df.to_html(file_path_html, index=False, header=False)


    print(df)


if __name__ == "__main__":
    main()