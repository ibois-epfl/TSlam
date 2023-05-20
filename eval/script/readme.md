TODO:
There are two evaluations:
- the slam eval
- the reconstruction eval

- [x] write a bash script that parse a txt file to get two frames values for each + the name of the tool and store them in a variable
- [x] for all the couples of frames, run the tslam-monocular and output the pose results in a folder by renaming with: '01_outpose_framstart_frameend_tooltype'

- [ ] run the analysis on the poses:
    - [ ] read the poses from frames (gt, ts)
    - [ ] allign the trajectories
    - [ ] visualize trajectories
    - [ ] we need a metric: COVERAGE: amount of frames without drifting and tags VS total frames

- [ ] create a batch script to process all the videos and frame sequences at once